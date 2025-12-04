import argparse
import json
import math
import os
from datetime import datetime

from botorch.utils.sampling import SobolEngine

"""
Simplified single-threaded Sobol sampling for PullTest.

This script generates Sobol sequence samples and evaluates them sequentially,
one at a time. No multiprocessing or GPU management - just simple, straightforward
execution. Results are written in the same CSV schema as PullTest_BO.py so
existing analysis scripts continue to work.
"""


def compute_int_bounds(lower_m, upper_m, spacing):
    lb = int(math.ceil(lower_m / spacing))
    ub = int(math.floor(upper_m / spacing))
    if lb > ub:
        raise ValueError(f"Invalid bounds after scaling: [{lb}, {ub}] for spacing={spacing}")
    return lb, ub


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def map_params_to_sim(param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed):
    from PullTest_sim import Params, SimParams

    p = Params()
    p.particle_spacing = particle_spacing
    # New parametrization inputs
    rad_outer = int(param_dict["rad"])  # multiples of particle spacing
    w_by_r = float(param_dict["w_by_r"]) 
    pct_g = float(param_dict["what_percent_is_grouser"]) 

    # Derived discrete geometry
    g_height_int = int(round(rad_outer * pct_g))
    g_height_int = max(0, g_height_int)
    rad_inner_int = int(rad_outer - g_height_int)
    width_int = int(round(w_by_r * rad_outer))

    p.rad = rad_inner_int
    p.width = width_int
    p.g_height = g_height_int
    p.g_density = int(param_dict["g_density"])
    p.fan_theta_deg = int(param_dict.get("fan_theta_deg", 0))

    sp = SimParams()
    sp.density = density
    sp.cohesion = cohesion
    sp.friction = friction
    sp.max_force = max_force
    sp.target_speed = target_speed

    return p, sp


def _int_from_unit(u, lo, hi):
    """Map a unit sample u in [0,1) to an integer in [lo, hi] inclusive uniformly."""
    return int(min(hi, math.floor(lo + (hi - lo + 1) * max(0.0, min(0.999999999, float(u))))))

def _float_from_unit(u, lo, hi):
    """Map a unit sample u in [0,1) to a float in [lo, hi]."""
    uu = max(0.0, min(0.999999999, float(u)))
    return lo + uu * (hi - lo)


def generate_sobol_param_dicts(n, particle_spacing):
    """Generate n Sobol samples mapped to the parameter dictionary space.
    Parameters follow PullTest_BO.py bounds.
    """
    # Compute integer bounds derived from meters and spacing
    rad_lb, rad_ub = compute_int_bounds(0.05, 0.14, particle_spacing)
    gd_lb, gd_ub = 2, 16
    th_lb, th_ub = 45, 135

    # Dimension d=5: [rad_outer, w_by_r, what_percent_is_grouser, g_density, fan_theta_deg]
    engine = SobolEngine(dimension=5, scramble=True, seed=0)
    U = engine.draw(n)  # tensor shape [n, 5], values in [0,1)

    params = []
    for i in range(n):
        u = U[i].tolist()
        pdict = {
            "rad": _int_from_unit(u[0], rad_lb, rad_ub),
            "w_by_r": _float_from_unit(u[1], 0.7, 1.4),
            "what_percent_is_grouser": _float_from_unit(u[2], 0.0, 0.3),
            "g_density": _int_from_unit(u[3], gd_lb, gd_ub),
            "fan_theta_deg": _int_from_unit(u[4], th_lb, th_ub),
        }
        params.append(pdict)
    return params


def main():
    parser = argparse.ArgumentParser(description="Single-threaded Sobol sampling for wheel parameters")
    parser.add_argument("--particle_spacing", type=float, default=0.01, help="Particle spacing (meters)")
    parser.add_argument("--sobol", type=int, default=64, help="Number of Sobol samples to evaluate")
    parser.add_argument("--out_dir", type=str, default=os.path.join(os.getcwd(), "pull_Sobol"), help="Output directory for logs")
    parser.add_argument("--density", type=float, default=1700, help="Material density")
    parser.add_argument("--cohesion", type=float, default=0, help="Material cohesion")
    parser.add_argument("--friction", type=float, default=0.8, help="Material friction coefficient")
    parser.add_argument("--max_force", type=float, default=20, help="Maximum pulling force")
    parser.add_argument("--target_speed", type=float, default=2.0, help="Target vehicle speed")
    parser.add_argument("--resume", action="store_true", help="Skip trials already recorded in out_dir/trials.csv")
    parser.add_argument("--retry_failed", action="store_true", help="When used with --resume, also retry trials listed in failed_trials.jsonl")
    args = parser.parse_args()

    particle_spacing = args.particle_spacing
    n_samples = max(1, int(args.sobol))
    out_dir = args.out_dir
    ensure_dir(out_dir)
    trials_csv = os.path.join(out_dir, "trials.csv")
    failed_log = os.path.join(out_dir, "failed_trials.jsonl")

    # Initialize CSV header if needed
    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write("timestamp,trial_index,metric,total_time_to_reach,average_power,rad_outer,w_by_r,what_percent_is_grouser,g_density,fan_theta_deg,particle_spacing\n")

    # Prepare Sobol samples
    param_dicts = generate_sobol_param_dicts(n_samples, particle_spacing)

    # If resuming, detect which indices are already done/failed
    remaining_indices = list(range(n_samples))
    if args.resume:
        done = set()
        failed = set()
        if os.path.isfile(trials_csv):
            try:
                with open(trials_csv, "r") as f:
                    for i, line in enumerate(f):
                        if i == 0:
                            continue  # header
                        parts = line.strip().split(",")
                        if len(parts) >= 3:
                            try:
                                done.add(int(parts[1]))
                            except Exception:
                                pass
            except Exception:
                pass
        if os.path.isfile(failed_log):
            try:
                with open(failed_log, "r") as f:
                    for line in f:
                        try:
                            rec = json.loads(line)
                            if "trial_index" in rec:
                                failed.add(int(rec["trial_index"]))
                        except Exception:
                            pass
            except Exception:
                pass

        if args.retry_failed:
            remaining_indices = [i for i in remaining_indices if i not in done]
        else:
            remaining_indices = [i for i in remaining_indices if i not in done and i not in failed]

        if not remaining_indices:
            print("Nothing to do: all requested Sobol samples already processed.")
            return

    # Import sim function
    from PullTest_sim import sim

    # Execute trials sequentially
    for idx in remaining_indices:
        param_dict = param_dicts[idx]
        print("Sampling: \n")
        print(param_dict)
        try:
            params_for_sim, sim_params = map_params_to_sim(
                param_dict, particle_spacing, args.density, args.cohesion,
                args.friction, args.max_force, args.target_speed
            )
            metric, total_time_to_reach, _, average_power, sim_failed = sim(params_for_sim, sim_params)

            if sim_failed or metric is None:
                # Log failure and continue
                with open(failed_log, "a") as flog:
                    flog.write(json.dumps({
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "trial_index": idx,
                        "parameters": param_dict,
                        "error": "sim_failed" if sim_failed else "metric is None",
                    }) + "\n")
                print(f"Trial {idx}: FAILED")
                continue

            # Success: append to CSV
            with open(trials_csv, "a") as f:
                f.write(
                    f"{datetime.utcnow().isoformat()}Z,{idx},{metric:.4f},{total_time_to_reach:.4f},{average_power:.2f},"
                    f"{param_dict['rad']},{param_dict['w_by_r']:.6f},{param_dict['what_percent_is_grouser']:.6f},{param_dict['g_density']},{param_dict['fan_theta_deg']},{particle_spacing}\n"
                )
            print(
                f"Trial {idx}: metric={metric:.4f}, time={total_time_to_reach:.4f}s, power={average_power:.2f}W"
            )
        except KeyboardInterrupt:
            print(f"\n\nKeyboard interrupt received. Stopping at trial {idx}.")
            break
        except Exception as e:
            # Log exception and continue
            try:
                with open(failed_log, "a") as flog:
                    flog.write(json.dumps({
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "trial_index": idx,
                        "parameters": param_dict,
                        "error": str(e),
                    }) + "\n")
            except Exception:
                pass
            print(f"Trial {idx}: EXCEPTION - {str(e)}")
            continue


if __name__ == "__main__":
    main()
