import argparse
import json
import math
import os
from datetime import datetime

from ax.service.ax_client import AxClient, ObjectiveProperties
from ax.generation_strategy.center_generation_node import CenterGenerationNode
from ax.generation_strategy.generation_node import GenerationNode
from ax.generation_strategy.generation_strategy import GenerationStrategy
from ax.generation_strategy.generator_spec import GeneratorSpec
from ax.generation_strategy.transition_criterion import MinTrials
from ax.adapter.registry import Generators
from botorch.acquisition.logei import qLogExpectedImprovement

import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed

"""
Parallel trial execution notes:
- We avoid importing PullTest_sim at module import time to ensure each worker
  process can set CUDA_VISIBLE_DEVICES before any CUDA context is created.
- Workers import PullTest_sim lazily after selecting their assigned GPU.
"""


def compute_int_bounds(lower_m, upper_m, spacing):
    lb = int(math.ceil(lower_m / spacing))
    ub = int(math.floor(upper_m / spacing))
    if lb > ub:
        raise ValueError(f"Invalid bounds after scaling: [{lb}, {ub}] for spacing={spacing}")
    return lb, ub


def build_ax_client(particle_spacing, sobol_trials, max_parallelism, state_path, resume):
    """
    Create an AxClient configured to:
      - run `sobol_trials` quasi-random Sobol initial points,
      - then switch to Modular BoTorch Bayesian optimization using qEI,
    """
    botorch_node = GenerationNode(
        node_name="BoTorch",
        generator_specs=[
            GeneratorSpec(
                generator_enum=Generators.BOTORCH_MODULAR,
                model_kwargs={
                    "botorch_acqf_class": qLogExpectedImprovement,
                },
            )
        ],
    )

    sobol_node = GenerationNode(
        node_name="Sobol",
        generator_specs=[
            GeneratorSpec(
                generator_enum=Generators.SOBOL,
                model_kwargs={"seed": 0},
            )
        ],
        transition_criteria=[
            MinTrials(
                threshold=max(1, int(sobol_trials)),
                transition_to=botorch_node.node_name,
                use_all_trials_in_exp=True,
            )
        ],
    )

    center_node = CenterGenerationNode(next_node_name=sobol_node.node_name)

    gs = GenerationStrategy(
        name="Center+Sobol+BoTorch(qEI)",
        nodes=[center_node, sobol_node, botorch_node],
    )
        
    if resume and os.path.isfile(state_path):
        ax_client = AxClient.load_from_json_file(filepath=state_path)
        best_params, best_vals = ax_client.get_best_parameters()
        print(f"Best params: {best_params}")
        print(f"Best metric: {best_vals}")
        return ax_client

    ax_client = AxClient(generation_strategy=gs)

    # Outer radius (including grousers) bounds in meters
    rad_lb, rad_ub = compute_int_bounds(0.05, 0.14, particle_spacing)

    ax_client.create_experiment(
        name="wheel_optimization",
        parameters=[
            {"name": "rad", "type": "range", "bounds": [rad_lb, rad_ub], "value_type": "int"},
            {"name": "w_by_r", "type": "range", "bounds": [0.7, 1.3], "value_type": "float"},
            {"name": "what_percent_is_grouser", "type": "range", "bounds": [0.0, 0.3], "value_type": "float"},
            {"name": "g_density", "type": "range", "bounds": [2, 16], "value_type": "int"},
            {"name": "fan_theta_deg", "type": "range", "bounds": [45, 135], "value_type": "int"},
        ],
        objectives={
            "metric": ObjectiveProperties(minimize=True)
        },
    )

    return ax_client


def map_params_to_sim(param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed):
    from PullTest_sim import Params, SimParams

    p = Params()
    p.particle_spacing = particle_spacing
    # New parametrization inputs
    rad_outer = int(param_dict["rad"])  # in multiples of particle spacing
    w_by_r = float(param_dict["w_by_r"])
    pct_g = float(param_dict["what_percent_is_grouser"])

    # Derived discrete geometry (in multiples of particle spacing)
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


def _worker_run_trial(trial_index, param_dict, particle_spacing, assigned_gpu, failed_log, density, cohesion, friction, max_force, target_speed):
    """
    Execute a single simulation in an isolated process on a specific GPU.
    Returns: (trial_index, metric, total_time_to_reach, average_power, sim_failed, error_message)
    """
    try:
        os.environ["CUDA_VISIBLE_DEVICES"] = str(assigned_gpu)

        from PullTest_sim import sim

        params_for_sim, sim_params = map_params_to_sim(param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed)
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


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def main():
    parser = argparse.ArgumentParser(description="Bayesian Optimization for wheel parameters using Ax + BoTorch (qEI)")
    parser.add_argument("--particle_spacing", type=float, default=0.01, help="Particle spacing (meters)")
    parser.add_argument("--batches", type=int, default=100, help="Number of outer iterations (batches)")    
    parser.add_argument("--q", type=int, default=16, help="Batch size per iteration (number of parallel suggestions)")
    parser.add_argument("--sobol", type=int, default=64, help="Number of Sobol warmup trials before BO")
    parser.add_argument("--out_dir", type=str, default=os.path.join(os.getcwd(), "pull_BO"), help="Output directory for logs and state")
    parser.add_argument("--state_path", type=str, default=None, help="Path to save/load Ax state JSON (defaults to out_dir/ax_state.json)")
    parser.add_argument("--resume", action="store_true", help="Resume from existing Ax state JSON if present")
    parser.add_argument("--per_gpu_concurrency", type=int, default=4, help="Max concurrent sims per GPU")
    parser.add_argument("--gpus", type=str, default=None, help="Comma-separated GPU ids to use (default: respect CUDA_VISIBLE_DEVICES or [0])")
    parser.add_argument("--density", type=float, default=1700, help="Material density")
    parser.add_argument("--cohesion", type=float, default=0, help="Material cohesion")
    parser.add_argument("--friction", type=float, default=0.8, help="Material friction coefficient")
    parser.add_argument("--max_force", type=float, default=20, help="Maximum pulling force")
    parser.add_argument("--target_speed", type=float, default=2.0, help="Target vehicle speed")
    args = parser.parse_args()

    particle_spacing = args.particle_spacing
    batches = args.batches
    q = args.q
    sobol_trials = args.sobol
    out_dir = args.out_dir
    ensure_dir(out_dir)
    state_path = args.state_path or os.path.join(out_dir, "ax_state.json")
    trials_csv = os.path.join(out_dir, "trials.csv")
    failed_log = os.path.join(out_dir, "failed_trials.jsonl")

    ax_client = build_ax_client(
        particle_spacing=particle_spacing,
        sobol_trials=sobol_trials,
        max_parallelism=q,
        state_path=state_path,
        resume=args.resume,
    )

    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write("timestamp,trial_index,metric,total_time_to_reach,average_power,rad_outer,w_by_r,what_percent_is_grouser,g_density,fan_theta_deg,particle_spacing\n")

    try:
        mp.set_start_method("spawn", force=True)
    except RuntimeError:
        pass

    gpu_ids = _discover_gpus(args.gpus)
    per_gpu_conc = max(1, int(args.per_gpu_concurrency))
    
    density = args.density
    cohesion = args.cohesion
    friction = args.friction
    max_force = args.max_force
    target_speed = args.target_speed

    for batch_idx in range(batches):
        print(f"Batch {batch_idx + 1}/{batches}")
        suggestions, is_available = ax_client.get_next_trials(max_trials=q)

        trial_items = list(suggestions.items())
        assignments = []
        for idx, (trial_index, param_dict) in enumerate(trial_items):
            assigned_gpu = gpu_ids[(idx // 1) % len(gpu_ids)]
            assignments.append((trial_index, param_dict, particle_spacing, assigned_gpu, failed_log, density, cohesion, friction, max_force, target_speed))

        with ProcessPoolExecutor(max_workers=min(q, len(gpu_ids) * per_gpu_conc), mp_context=mp.get_context("spawn")) as ex:
            futures = [ex.submit(_worker_run_trial, *args_tuple) for args_tuple in assignments]
            for fut in as_completed(futures):
                trial_index, metric, total_time_to_reach, average_power, sim_failed, err = fut.result()
                param_dict = suggestions[trial_index]

                if sim_failed or metric is None:
                    ax_client.abandon_trial(trial_index=trial_index)
                    with open(failed_log, "a") as flog:
                        flog.write(json.dumps({
                            "timestamp": datetime.utcnow().isoformat() + "Z",
                            "trial_index": trial_index,
                            "parameters": param_dict,
                            "error": err or "sim_failed",
                        }) + "\n")
                    print(f"  Trial {trial_index}: FAILED")
                    continue

                ax_client.complete_trial(trial_index=trial_index, raw_data={"metric": float(metric)})
                with open(trials_csv, "a") as f:
                    f.write(
                        f"{datetime.utcnow().isoformat()}Z,{trial_index},{metric:.4f},{total_time_to_reach:.4f},{average_power:.2f},"
                        f"{param_dict['rad']},{param_dict['w_by_r']:.6f},{param_dict['what_percent_is_grouser']:.6f},{param_dict['g_density']},"
                        f"{param_dict['fan_theta_deg']},{particle_spacing}\n"
                    )
                print(f"  Trial {trial_index}: metric={metric:.4f}, time={total_time_to_reach:.4f}s, power={average_power:.2f}W")

        ax_client.save_to_json_file(filepath=state_path)

    try:
        best_params, best_vals = ax_client.get_best_parameters()
        print(f"\nOptimization complete!")
        print(f"Best params: {best_params}")
        print(f"Best metric: {best_vals}")
    except Exception:
        pass


if __name__ == "__main__":
    main()
