import argparse
import json
import math
import os
from datetime import datetime

import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed

import torch
from botorch.utils.sampling import SobolEngine

"""
Global Sobol sampling for PullTest.

This script mirrors the CLI of PullTest_BO.py (minus batches/state),
generates `--sobol` Sobol sequence samples (seed=0) over the same
integer parameter bounds, and evaluates them using the same parallel
GPU worker pattern. Results are written in the same CSV schema as
PullTest_BO.py so existing analysis scripts continue to work.
"""


def compute_int_bounds(lower_m, upper_m, spacing):
    lb = int(math.ceil(lower_m / spacing))
    ub = int(math.floor(upper_m / spacing))
    if lb > ub:
        raise ValueError(f"Invalid bounds after scaling: [{lb}, {ub}] for spacing={spacing}")
    return lb, ub


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def _discover_gpus(user_gpus_arg=None):
    """Return a list of GPU ids to use.
    Priority: explicit CLI list -> CUDA_VISIBLE_DEVICES -> [0]
    """
    if user_gpus_arg:
        return [int(x) for x in user_gpus_arg.split(",")]

    cvd = os.environ.get("CUDA_VISIBLE_DEVICES")
    if cvd:
        try:
            return [int(x) for x in cvd.split(",") if x.strip() != ""]
        except Exception:
            pass
    return [0]


def map_params_to_sim(param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed):
    from PullTest_sim import Params, SimParams

    p = Params()
    p.particle_spacing = particle_spacing
    p.rad = int(param_dict["rad"])
    p.width = int(param_dict["width"])
    p.g_height = int(param_dict["g_height"])
    p.g_density = int(param_dict["g_density"])
    p.fan_theta_deg = int(param_dict.get("fan_theta_deg", 0))

    sp = SimParams()
    sp.density = density
    sp.cohesion = cohesion
    sp.friction = friction
    sp.max_force = max_force
    sp.target_speed = target_speed

    return p, sp


def _worker_run_trial(trial_index, param_dict, particle_spacing, assigned_gpu, failed_log,
                      density, cohesion, friction, max_force, target_speed):
    """
    Execute a single simulation in an isolated process on a specific GPU.
    Returns: (trial_index, metric, total_time_to_reach, average_power, sim_failed, error_message)
    """
    try:
        os.environ["CUDA_VISIBLE_DEVICES"] = str(assigned_gpu)

        from PullTest_sim import sim

        params_for_sim, sim_params = map_params_to_sim(
            param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed
        )
        metric, total_time_to_reach, _, average_power, sim_failed = sim(params_for_sim, sim_params)
        return trial_index, metric, total_time_to_reach, average_power, bool(sim_failed), None
    except Exception as e:
        try:
            with open(failed_log, "a") as flog:
                flog.write(json.dumps({
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "trial_index": trial_index,
                    "parameters": param_dict,
                    "error": str(e),
                }) + "\n")
        except Exception:
            pass
        return trial_index, None, None, None, True, str(e)


def _int_from_unit(u, lo, hi):
    """Map a unit sample u in [0,1) to an integer in [lo, hi] inclusive uniformly.
    """
    return int(min(hi, math.floor(lo + (hi - lo + 1) * max(0.0, min(0.999999999, float(u))))))


def generate_sobol_param_dicts(n, particle_spacing):
    """Generate n Sobol samples mapped to the parameter dictionary space.
    Parameters follow PullTest_BO.py bounds.
    """
    # Compute integer bounds derived from meters and spacing
    rad_lb, rad_ub = compute_int_bounds(0.06, 0.12, particle_spacing)
    width_lb, width_ub = compute_int_bounds(0.05, 0.15, particle_spacing)
    gh_lb, gh_ub = compute_int_bounds(0.02, 0.05, particle_spacing)
    gd_lb, gd_ub = 2, 16
    th_lb, th_ub = 45, 135

    # Dimension d=5: [rad, width, g_height, g_density, fan_theta_deg]
    engine = SobolEngine(dimension=5, scramble=True, seed=0)
    U = engine.draw(n)  # tensor shape [n, 5], values in [0,1)

    params = []
    for i in range(n):
        u = U[i].tolist()
        pdict = {
            "rad": _int_from_unit(u[0], rad_lb, rad_ub),
            "width": _int_from_unit(u[1], width_lb, width_ub),
            "g_height": _int_from_unit(u[2], gh_lb, gh_ub),
            "g_density": _int_from_unit(u[3], gd_lb, gd_ub),
            "fan_theta_deg": _int_from_unit(u[4], th_lb, th_ub),
        }
        params.append(pdict)
    return params


def main():
    parser = argparse.ArgumentParser(description="Global Sobol sampling for wheel parameters (evaluate with PullTest_sim)")
    parser.add_argument("--particle_spacing", type=float, default=0.01, help="Particle spacing (meters)")
    parser.add_argument("--q", type=int, default=16, help="Max parallel suggestions evaluated at once")
    parser.add_argument("--sobol", type=int, default=64, help="Number of Sobol samples to evaluate")
    parser.add_argument("--out_dir", type=str, default=os.path.join(os.getcwd(), "pull_Sobol"), help="Output directory for logs")
    parser.add_argument("--per_gpu_concurrency", type=int, default=4, help="Max concurrent sims per GPU")
    parser.add_argument("--gpus", type=str, default=None, help="Comma-separated GPU ids to use (default: respect CUDA_VISIBLE_DEVICES or [0])")
    parser.add_argument("--density", type=float, default=1700, help="Material density")
    parser.add_argument("--cohesion", type=float, default=0, help="Material cohesion")
    parser.add_argument("--friction", type=float, default=0.8, help="Material friction coefficient")
    parser.add_argument("--max_force", type=float, default=20, help="Maximum pulling force")
    parser.add_argument("--target_speed", type=float, default=2.0, help="Target vehicle speed")
    args = parser.parse_args()

    particle_spacing = args.particle_spacing
    q = max(1, int(args.q))
    n_samples = max(1, int(args.sobol))
    out_dir = args.out_dir
    ensure_dir(out_dir)
    trials_csv = os.path.join(out_dir, "trials.csv")
    failed_log = os.path.join(out_dir, "failed_trials.jsonl")

    # Initialize CSV header if needed
    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write("timestamp,trial_index,metric,total_time_to_reach,average_power,rad,width,g_height,g_density,fan_theta_deg,particle_spacing\n")

    # Prepare Sobol samples
    param_dicts = generate_sobol_param_dicts(n_samples, particle_spacing)

    # Parallel execution configuration
    try:
        mp.set_start_method("spawn", force=True)
    except RuntimeError:
        pass

    gpu_ids = _discover_gpus(args.gpus)
    per_gpu_conc = max(1, int(args.per_gpu_concurrency))
    max_workers = min(q, len(gpu_ids) * per_gpu_conc)

    density = args.density
    cohesion = args.cohesion
    friction = args.friction
    max_force = args.max_force
    target_speed = args.target_speed

    # Assign each sample to a GPU in round-robin fashion
    assignments = []
    for idx, param_dict in enumerate(param_dicts):
        assigned_gpu = gpu_ids[idx % len(gpu_ids)]
        assignments.append((idx, param_dict, particle_spacing, assigned_gpu, failed_log,
                            density, cohesion, friction, max_force, target_speed))

    # Submit all tasks; executor enforces concurrency <= max_workers
    with ProcessPoolExecutor(max_workers=max_workers, mp_context=mp.get_context("spawn")) as ex:
        futures = [ex.submit(_worker_run_trial, *args_tuple) for args_tuple in assignments]
        for fut in as_completed(futures):
            trial_index, metric, total_time_to_reach, average_power, sim_failed, err = fut.result()
            param_dict = param_dicts[trial_index]

            if sim_failed or metric is None:
                # Log failure and continue
                with open(failed_log, "a") as flog:
                    flog.write(json.dumps({
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "trial_index": trial_index,
                        "parameters": param_dict,
                        "error": err or "sim_failed",
                    }) + "\n")
                print(f"Trial {trial_index}: FAILED")
                continue

            # Success: append to CSV
            with open(trials_csv, "a") as f:
                f.write(
                    f"{datetime.utcnow().isoformat()}Z,{trial_index},{metric:.4f},{total_time_to_reach:.4f},{average_power:.2f},"
                    f"{param_dict['rad']},{param_dict['width']},{param_dict['g_height']},{param_dict['g_density']},{param_dict['fan_theta_deg']},{particle_spacing}\n"
                )
            print(
                f"Trial {trial_index}: metric={metric:.4f}, time={total_time_to_reach:.4f}s, power={average_power:.2f}W"
            )


if __name__ == "__main__":
    main()

