import argparse
import json
import math
import os
import sys
from datetime import datetime

from ax.service.ax_client import AxClient, ObjectiveProperties

from ax.generation_strategy.center_generation_node import CenterGenerationNode
from ax.generation_strategy.generation_node import GenerationNode
from ax.generation_strategy.generation_strategy import GenerationStrategy
from ax.generation_strategy.generator_spec import GeneratorSpec
from ax.generation_strategy.transition_criterion import MinTrials
from ax.adapter.registry import Generators

from botorch.acquisition.logei import qLogExpectedImprovement

"""
Parallel trial execution notes
 - We avoid importing PullTest_sim at module import time to ensure each worker
   process can set CUDA_VISIBLE_DEVICES before any CUDA context is created.
 - Workers import PullTest_sim lazily after selecting their assigned GPU.
"""

import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed


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
    # --- Build GenerationStrategy: Center -> Sobol -> BoTorch ---
    # BoTorch (modular) node with explicit qEI:
    botorch_node = GenerationNode(
        node_name="BoTorch",
        generator_specs=[
            GeneratorSpec(
                generator_enum=Generators.BOTORCH_MODULAR,
                model_kwargs={
                    # CRITICAL: hand the BoTorch class object, not a string
                    "botorch_acqf_class": qLogExpectedImprovement,
                },
                # Optional: tune acquisition/optimizer budgets via model_gen_kwargs
                # model_gen_kwargs={
                #     "model_gen_options": {
                #         "optimizer_kwargs": {
                #             "num_restarts": 20,
                #             "sequential": False,
                #             "options": {"batch_limit": 5, "maxiter": 200},
                #         }
                #     }
                # }
            )
        ],
    )

    # Sobol node: transition after `sobol_trials` total trials in the experiment
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

    # Center-of-search-space warm start (single point) before Sobol
    center_node = CenterGenerationNode(next_node_name=sobol_node.node_name)

    gs = GenerationStrategy(
        name="Center+Sobol+BoTorch(qEI)",
        nodes=[center_node, sobol_node, botorch_node],
    )
        
    if resume and os.path.isfile(state_path):
        ax_client = AxClient.load_from_json_file(filepath=state_path)
        return ax_client

    ax_client = AxClient(generation_strategy=gs)

    # Integer bounds derived from physical limits (meters) and particle spacing
    rad_lb, rad_ub = compute_int_bounds(0.06, 0.12, particle_spacing)
    width_lb, width_ub = compute_int_bounds(0.05, 0.15, particle_spacing)
    gh_lb, gh_ub = compute_int_bounds(0.02, 0.08, particle_spacing)
    gw_lb, gw_ub = compute_int_bounds(0.02, 0.05, particle_spacing)

    ax_client.create_experiment(
        name="wheel_optimization",
        parameters=[
            {"name": "rad", "type": "range", "bounds": [rad_lb, rad_ub], "value_type": "int"},
            {"name": "width", "type": "range", "bounds": [width_lb, width_ub], "value_type": "int"},
            {"name": "g_height", "type": "range", "bounds": [gh_lb, gh_ub], "value_type": "int"},
            {"name": "g_width", "type": "range", "bounds": [gw_lb, gw_ub], "value_type": "int"},
            {"name": "g_density", "type": "range", "bounds": [2, 16], "value_type": "int"},
            {"name": "grouser_type", "type": "choice", "values": [0, 1], "value_type": "int", "is_ordered": True, "sort_values": True},
            {"name": "fan_theta_deg", "type": "range", "bounds": [45, 135], "value_type": "int"},
        ],
        objectives={
            "total_time_to_reach": ObjectiveProperties(minimize=True)
        },
    )

    return ax_client


def map_params_to_sim(param_dict, particle_spacing):
    # Lazy import so workers can set CUDA environment before any CUDA init
    from PullTest_sim import Params

    p = Params()
    p.particle_spacing = particle_spacing
    p.rad = int(param_dict["rad"])  # multiples of spacing
    p.width = int(param_dict["width"])  # multiples of spacing
    p.g_height = int(param_dict["g_height"])  # multiples of spacing
    p.g_width = int(param_dict["g_width"])  # multiples of spacing
    p.g_density = int(param_dict["g_density"])  # count
    p.grouser_type = int(param_dict["grouser_type"])  # 0: straight, 1: semi_circle
    p.fan_theta_deg = int(param_dict.get("fan_theta_deg", 0)) if p.grouser_type == 0 else int(param_dict.get("fan_theta_deg", 0))
    return p


def _worker_run_trial(trial_index, param_dict, particle_spacing, assigned_gpu, failed_log):
    """
    Execute a single simulation in an isolated process on a specific GPU.
    Returns: (trial_index, total_time_to_reach, sim_failed, error_message)
    """
    try:
        # Restrict this process to the assigned GPU (single device id)
        os.environ["CUDA_VISIBLE_DEVICES"] = str(assigned_gpu)

        # Import after setting device visibility so CUDA contexts are created on the right GPU
        from PullTest_sim import sim  # noqa: WPS433 (intentional local import)

        params_for_sim = map_params_to_sim(param_dict, particle_spacing)
        total_time_to_reach, sim_failed = sim(params_for_sim)
        return trial_index, total_time_to_reach, bool(sim_failed), None
    except Exception as e:
        # Persist failure detail here as well (workers have isolated memory)
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
        return trial_index, None, True, str(e)


def _discover_gpus(user_gpus_arg=None):
    """Return a list of GPU ids to use.
    Priority: explicit CLI list -> CUDA_VISIBLE_DEVICES -> [0]
    """
    if user_gpus_arg:
        return [int(x) for x in user_gpus_arg.split(",")]

    cvd = os.environ.get("CUDA_VISIBLE_DEVICES")
    if cvd:
        # CUDA_VISIBLE_DEVICES may be a comma-separated map of physical ids; use index order
        try:
            return [int(x) for x in cvd.split(",") if x.strip() != ""]
        except Exception:
            pass
    return [0]


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def main():
    parser = argparse.ArgumentParser(description="Bayesian Optimization for wheel parameters using Ax + BoTorch (qEI)")
    parser.add_argument("--particle_spacing", type=float, default=0.01, help="Particle spacing (meters), used as scale unit for integer geometry params")
    parser.add_argument("--batches", type=int, default=100, help="Number of outer iterations (batches)")    
    parser.add_argument("--q", type=int, default=16, help="Batch size per iteration (number of parallel suggestions)")
    parser.add_argument("--sobol", type=int, default=64, help="Number of Sobol warmup trials before BO")
    parser.add_argument("--out_dir", type=str, default=os.path.join(os.getcwd(), "ARTcar_PullTest_BO"), help="Output directory for logs and state")
    parser.add_argument("--state_path", type=str, default=None, help="Path to save/load Ax state JSON (defaults to out_dir/ax_state.json)")
    parser.add_argument("--resume", action="store_true", help="Resume from existing Ax state JSON if present")
    parser.add_argument("--per_gpu_concurrency", type=int, default=4, help="Max concurrent sims per GPU")
    parser.add_argument("--gpus", type=str, default=None, help="Comma-separated GPU ids to use (default: respect CUDA_VISIBLE_DEVICES or [0])")
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

    # Write CSV header if new file
    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write(
                "timestamp,trial_index,total_time_to_reach,rad,width,g_height,g_width,g_density,grouser_type,fan_theta_deg,particle_spacing\n"
            )

    # Ensure clean worker processes without inheriting CUDA contexts
    try:
        mp.set_start_method("spawn", force=True)
    except RuntimeError:
        pass

    gpu_ids = _discover_gpus(args.gpus)
    per_gpu_conc = max(1, int(args.per_gpu_concurrency))

    for batch_idx in range(batches):
        suggestions, is_available = ax_client.get_next_trials(max_trials=q)

        # Build assignments: round-robin GPUs, respecting per-GPU concurrency
        trial_items = list(suggestions.items())
        total_workers = min(len(trial_items), len(gpu_ids) * per_gpu_conc)
        if total_workers < len(trial_items):
            # We still queue all tasks; the executor will limit concurrent workers to q
            pass

        # Assign GPU for each trial deterministically
        assignments = []  # list of tuples for worker args
        for idx, (trial_index, param_dict) in enumerate(trial_items):
            assigned_gpu = gpu_ids[(idx // 1) % len(gpu_ids)]  # repeated round-robin
            assignments.append((trial_index, param_dict, particle_spacing, assigned_gpu, failed_log))

        # Execute in parallel
        with ProcessPoolExecutor(max_workers=min(q, len(gpu_ids) * per_gpu_conc), mp_context=mp.get_context("spawn")) as ex:
            futures = [ex.submit(_worker_run_trial, *args_tuple) for args_tuple in assignments]
            for fut in as_completed(futures):
                trial_index, total_time_to_reach, sim_failed, err = fut.result()
                param_dict = suggestions[trial_index]

                if sim_failed or total_time_to_reach is None:
                    ax_client.abandon_trial(trial_index=trial_index)
                    with open(failed_log, "a") as flog:
                        flog.write(json.dumps({
                            "timestamp": datetime.utcnow().isoformat() + "Z",
                            "trial_index": trial_index,
                            "parameters": param_dict,
                            "error": err or "sim_failed",
                        }) + "\n")
                    continue

                ax_client.complete_trial(trial_index=trial_index, raw_data={"total_time_to_reach": float(total_time_to_reach)})
                with open(trials_csv, "a") as f:
                    f.write(
                        f"{datetime.utcnow().isoformat()}Z,{trial_index},{total_time_to_reach},{param_dict['rad']},{param_dict['width']},{param_dict['g_height']},{param_dict['g_width']},{param_dict['g_density']},{param_dict['grouser_type']},{param_dict['fan_theta_deg']},{particle_spacing}\n"
                    )

        # Persist Ax state after each batch
        ax_client.save_to_json_file(filepath=state_path)

    # Optionally report best found
    try:
        best_params, best_vals = ax_client.get_best_parameters()
        print(f"Best params: {best_params}")
        print(f"Best total_time_to_reach: {best_vals}")
    except Exception:
        pass


if __name__ == "__main__":
    main()