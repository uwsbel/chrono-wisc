import argparse
import json
import math
import os
import sys
from datetime import datetime

from ax.generation_strategy.generation_strategy import GenerationStep, GenerationStrategy
from ax.adapter.registry import Generators
from ax.service.ax_client import AxClient
from ax.service.utils.instantiation import ObjectiveProperties

from PullTest_sim import Params, sim


def compute_int_bounds(lower_m, upper_m, spacing):
    lb = int(math.ceil(lower_m / spacing))
    ub = int(math.floor(upper_m / spacing))
    if lb > ub:
        raise ValueError(f"Invalid bounds after scaling: [{lb}, {ub}] for spacing={spacing}")
    return lb, ub


def build_ax_client(particle_spacing, sobol_trials, max_parallelism, state_path, resume):
    ax_client = AxClient(generation_strategy=GenerationStrategy(steps=[
        GenerationStep(
            generator=Generators.SOBOL,
            num_trials=sobol_trials,
            max_parallelism=max_parallelism,
        ),
        GenerationStep(
            generator=Generators.BOTORCH_MODULAR,
            num_trials=-1,
            max_parallelism=max_parallelism,
            # Prefer qExpectedImprovement for single-objective batch BO
            model_kwargs={"acquisition_class": "qExpectedImprovement"},
        ),
    ]))

    if resume and os.path.isfile(state_path):
        ax_client.load_from_json_file(filepath=state_path)
        return ax_client

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
    p = Params()
    p.particle_spacing = particle_spacing
    p.rad = int(param_dict["rad"])  # multiples of spacing
    p.width = int(param_dict["width"])  # multiples of spacing
    p.g_height = int(param_dict["g_height"])  # multiples of spacing
    p.g_width = int(param_dict["g_width"])  # multiples of spacing
    p.g_density = int(param_dict["g_density"])  # count
    p.grouser_type = int(param_dict["grouser_type"])  # 0: straight, 1: semi_circle
    # fan_theta_deg is used only when grouser_type == 0 (straight); otherwise dummy
    p.fan_theta_deg = int(param_dict.get("fan_theta_deg", 0)) if p.grouser_type == 0 else int(param_dict.get("fan_theta_deg", 0))
    # cp_deviation not optimized; keep default from class if present
    return p


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

    for batch_idx in range(batches):
        suggestions, is_available = ax_client.get_next_trials(max_trials=q)
        for trial_index, param_dict in suggestions.items():
            params_for_sim = map_params_to_sim(param_dict, particle_spacing)
            try:
                total_time_to_reach, sim_failed = sim(params_for_sim)
            except Exception as e:
                sim_failed = True
                total_time_to_reach = None
                # Log exception as failed trial
                with open(failed_log, "a") as flog:
                    flog.write(json.dumps({
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "trial_index": trial_index,
                        "parameters": param_dict,
                        "error": str(e),
                    }) + "\n")

            if sim_failed or total_time_to_reach is None:
                ax_client.abandon_trial(trial_index=trial_index)
                # Also log parameters that led to failure
                with open(failed_log, "a") as flog:
                    flog.write(json.dumps({
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "trial_index": trial_index,
                        "parameters": param_dict,
                        "error": "sim_failed",
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