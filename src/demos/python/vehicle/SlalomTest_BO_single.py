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
Single GPU version of SlalomTest_BO.py
- No multiprocessing, runs trials sequentially on a single GPU
- Simplified execution model for easier debugging and single-GPU setups
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
        best_params, best_vals = ax_client.get_best_parameters()
        print(f"Best params: {best_params}")
        print(f"Best composite_metric: {best_vals}")
        return ax_client

    ax_client = AxClient(generation_strategy=gs)

    # Integer bounds derived from physical limits (meters) and particle spacing
    rad_lb, rad_ub = compute_int_bounds(0.06, 0.12, particle_spacing)
    width_lb, width_ub = compute_int_bounds(0.05, 0.15, particle_spacing)
    gh_lb, gh_ub = compute_int_bounds(0.02, 0.05, particle_spacing)
    gw_lb, gw_ub = compute_int_bounds(0.02, 0.05, particle_spacing)

    ax_client.create_experiment(
        name="slalom_wheel_controller_optimization",
        parameters=[
            {"name": "rad", "type": "range", "bounds": [rad_lb, rad_ub], "value_type": "int"},
            {"name": "width", "type": "range", "bounds": [width_lb, width_ub], "value_type": "int"},
            {"name": "g_height", "type": "range", "bounds": [gh_lb, gh_ub], "value_type": "int"},
            {"name": "g_width", "type": "range", "bounds": [gw_lb, gw_ub], "value_type": "int"},
            {"name": "g_density", "type": "range", "bounds": [2, 16], "value_type": "int"},
            {"name": "grouser_type", "type": "choice", "values": [0, 1], "value_type": "int", "is_ordered": True, "sort_values": True},
            {"name": "fan_theta_deg", "type": "range", "bounds": [45, 135], "value_type": "int"},
            # Controller parameters
            {"name": "steering_kp", "type": "range", "bounds": [0.1, 20.0], "value_type": "float"},
            {"name": "steering_kd", "type": "range", "bounds": [0.0, 5.0], "value_type": "float"},
            {"name": "speed_kp", "type": "range", "bounds": [0.1, 5.0], "value_type": "float"},
            {"name": "speed_kd", "type": "range", "bounds": [0.0, 1.0], "value_type": "float"},
        ],
        objectives={
            "composite_metric": ObjectiveProperties(minimize=True)
        },
    )

    return ax_client


def map_params_to_sim(param_dict, particle_spacing):
    from SlalomTest_sim import Params

    p = Params()
    p.particle_spacing = particle_spacing
    p.rad = int(param_dict["rad"])  # multiples of spacing
    p.width = int(param_dict["width"])  # multiples of spacing
    p.g_height = int(param_dict["g_height"])  # multiples of spacing
    p.g_width = int(param_dict["g_width"])  # multiples of spacing
    p.g_density = int(param_dict["g_density"])  # count
    p.grouser_type = int(param_dict["grouser_type"])  # 0: straight, 1: semi_circle
    p.fan_theta_deg = int(param_dict.get("fan_theta_deg", 0)) if p.grouser_type == 0 else int(param_dict.get("fan_theta_deg", 0))
    # Controller parameters
    p.steering_kp = float(param_dict["steering_kp"])
    p.steering_kd = float(param_dict["steering_kd"])
    p.speed_kp = float(param_dict["speed_kp"])
    p.speed_kd = float(param_dict["speed_kd"])
    return p


def run_trial(trial_index, param_dict, particle_spacing, weight_speed, weight_power, failed_log):
    """
    Execute a single simulation on the specified GPU.
    Returns: (trial_index, metric, t_elapsed, rms_error, sim_failed, error_message)
    """
    try:
        # CRITICAL: Clear any previous CUDA errors before starting trial
        from cuda_error_checker import clear_cuda_error, check_cuda_error
        clear_cuda_error()
        
        # Import simulation module
        from SlalomTest_sim import sim

        params_for_sim = map_params_to_sim(param_dict, particle_spacing)
        try:
            metric, t_elapsed, rms_error, average_power, sim_failed = sim(params_for_sim, weight_speed=weight_speed, weight_power=weight_power)
        except Exception as e:
            print(f"Simulation error at trial {trial_index}: {type(e).__name__}: {e}")
            sim_failed = True
            metric = 50  # Penalty for simulation errors
            return trial_index, metric, t_elapsed, rms_error, 0.0, bool(sim_failed), str(e)
        
        # CRITICAL: Check for CUDA errors after simulation
        error_occurred, error_message = check_cuda_error()
        if error_occurred:
            print(f"CUDA error detected after trial {trial_index}: {error_message}")
            sim_failed = True
            if metric > 0:
                metric = 50  # Penalty for CUDA errors
            else:
                metric = -50  # Initialization failed which is weird and the sim should be discarded
        
        return trial_index, metric, t_elapsed, rms_error, average_power, bool(sim_failed), error_message if error_occurred else None
        
    except Exception as e:
        # Log failure detail
        with open(failed_log, "a") as flog:
            flog.write(json.dumps({
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "trial_index": trial_index,
                "parameters": param_dict,
                "error": str(e),
            }) + "\n")

        return trial_index, 50, None, None, 0.0, True, str(e)


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def main():
    parser = argparse.ArgumentParser(description="Single GPU Bayesian Optimization for wheel parameters using Ax + BoTorch (qEI)")
    parser.add_argument("--particle_spacing", type=float, default=0.01, help="Particle spacing (meters), used as scale unit for integer geometry params")
    parser.add_argument("--batches", type=int, default=500, help="Number of outer iterations (batches)")    
    parser.add_argument("--q", type=int, default=1, help="Batch size per iteration (number of sequential suggestions)")
    parser.add_argument("--sobol", type=int, default=64, help="Number of Sobol warmup trials before BO")
    parser.add_argument("--weight_speed", type=float, default=0.5, help="Weight for speed component in composite metric")
    parser.add_argument("--weight_power", type=float, default=0.25, help="Weight for power component in composite metric")
    parser.add_argument("--out_dir", type=str, default=os.path.join(os.getcwd(), "ARTcar_SlalomTest_BO_single"), help="Output directory for logs and state")
    parser.add_argument("--state_path", type=str, default=None, help="Path to save/load Ax state JSON (defaults to out_dir/ax_state.json)")
    parser.add_argument("--resume", action="store_true", help="Resume from existing Ax state JSON if present")
    parser.add_argument("--gpu", type=int, default=0, help="GPU id to use (default: 0)")
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
                "timestamp,trial_index,composite_metric,t_elapsed,rms_error,average_power,rad,width,g_height,g_width,g_density,grouser_type,fan_theta_deg,steering_kp,steering_kd,speed_kp,speed_kd,particle_spacing\n"
            )

    for batch_idx in range(batches):
        print(f"Starting batch {batch_idx + 1}/{batches}")
        suggestions, is_available = ax_client.get_next_trials(max_trials=q)

        # Execute trials sequentially
        for trial_index, param_dict in suggestions.items():
            print(f"  Running trial {trial_index}...")
            trial_index, metric, t_elapsed, rms_error, average_power, sim_failed, err = run_trial(
                trial_index, param_dict, particle_spacing, args.weight_speed, args.weight_power, failed_log
            )

            if sim_failed:
                if metric < 0:
                    ax_client.abandon_trial(trial_index=trial_index)
                    with open(failed_log, "a") as flog:
                        flog.write(json.dumps({
                            "timestamp": datetime.utcnow().isoformat() + "Z",
                            "trial_index": trial_index,
                            "parameters": param_dict,
                            "error": err or "sim_failed",
                        }) + "\n")
                    print(f"    Trial {trial_index} failed: {err or 'sim_failed'}")
                    continue
                else:
                    # If its not at initialization then don't abandon the trial
                    with open(failed_log, "a") as flog:
                        flog.write(json.dumps({
                            "timestamp": datetime.utcnow().isoformat() + "Z",
                            "trial_index": trial_index,
                            "parameters": param_dict,
                            "error": err or "sim_failed",
                        }) + "\n")
            ax_client.complete_trial(trial_index=trial_index, raw_data={"composite_metric": float(metric)})
            with open(trials_csv, "a") as f:
                f.write(
                    f"{datetime.utcnow().isoformat()}Z,{trial_index},{metric},{t_elapsed},{rms_error},{average_power},{param_dict['rad']},{param_dict['width']},{param_dict['g_height']},{param_dict['g_width']},{param_dict['g_density']},{param_dict['grouser_type']},{param_dict['fan_theta_deg']},{param_dict['steering_kp']},{param_dict['steering_kd']},{param_dict['speed_kp']},{param_dict['speed_kd']},{particle_spacing}\n"
                )
            print(f"    Trial {trial_index} completed: metric={metric:.4f}, time={t_elapsed:.2f}s, power={average_power:.2f}W")

        # Persist Ax state after each batch
        ax_client.save_to_json_file(filepath=state_path)
        print(f"Batch {batch_idx + 1} completed, state saved")

    # Report best found
    try:
        best_params, best_vals = ax_client.get_best_parameters()
        print(f"\nOptimization completed!")
        print(f"Best params: {best_params}")
        print(f"Best composite_metric: {best_vals}")
    except Exception as e:
        print(f"Could not retrieve best parameters: {e}")


if __name__ == "__main__":
    main()
