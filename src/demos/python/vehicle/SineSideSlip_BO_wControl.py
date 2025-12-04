import argparse
import json
import math
import os
from datetime import datetime

from ax.adapter.registry import Generators
from ax.generation_strategy.center_generation_node import CenterGenerationNode
from ax.generation_strategy.generation_node import GenerationNode
from ax.generation_strategy.generation_strategy import GenerationStrategy
from ax.generation_strategy.generator_spec import GeneratorSpec
from ax.generation_strategy.transition_criterion import MinTrials
from ax.service.ax_client import AxClient, ObjectiveProperties
from botorch.acquisition.logei import qLogNoisyExpectedImprovement


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def compute_int_bounds(lower_m: float, upper_m: float, spacing: float):
    lb = int(math.ceil(lower_m / spacing))
    ub = int(math.floor(upper_m / spacing))
    if lb > ub:
        raise ValueError(f"Invalid bounds after scaling: [{lb}, {ub}] for spacing={spacing}")
    return lb, ub


def build_ax_client(particle_spacing: float, sobol_trials: int, state_path: str, resume: bool):
    """Create or load an AxClient configured for joint wheel+controller tuning."""
    botorch_node = GenerationNode(
        name="BoTorch",
        generator_specs=[
            GeneratorSpec(
                generator_enum=Generators.BOTORCH_MODULAR,
                model_kwargs={"botorch_acqf_class": qLogNoisyExpectedImprovement},
            )
        ],
    )

    sobol_node = GenerationNode(
        name="Sobol",
        generator_specs=[GeneratorSpec(generator_enum=Generators.SOBOL, model_kwargs={"seed": 0})],
        transition_criteria=[
            MinTrials(
                threshold=max(1, int(sobol_trials)),
                transition_to=botorch_node.name,
                use_all_trials_in_exp=True,
            )
        ],
    )

    center_node = CenterGenerationNode(next_node_name=sobol_node.name)
    gs = GenerationStrategy(name="Center+Sobol+BoTorch(qLogNEI)", nodes=[center_node, sobol_node, botorch_node])

    if resume and os.path.isfile(state_path):
        try:
            return AxClient.load_from_json_file(filepath=state_path)
        except Exception as e:
            print(f"Could not load Ax state from {state_path}: {e}")

    ax_client = AxClient(generation_strategy=gs)

    rad_lb, rad_ub = compute_int_bounds(0.05, 0.14, particle_spacing)

    ax_client.create_experiment(
        name="sine_wheel_plus_controller",
        parameters=[
            {"name": "rad", "type": "range", "bounds": [rad_lb, rad_ub], "value_type": "int"},
            {"name": "w_by_r", "type": "range", "bounds": [0.7, 1.4], "value_type": "float"},
            {"name": "what_percent_is_grouser", "type": "range", "bounds": [0.0, 0.3], "value_type": "float"},
            {"name": "g_density", "type": "range", "bounds": [2, 16], "value_type": "int"},
            {"name": "fan_theta_deg", "type": "range", "bounds": [45, 135], "value_type": "int"},
            {"name": "steering_kp", "type": "range", "bounds": [1.0, 15.0], "value_type": "float"},
            {"name": "steering_ki", "type": "range", "bounds": [0.0, 4.0], "value_type": "float"},
            {"name": "steering_kd", "type": "range", "bounds": [0.0, 4.0], "value_type": "float"},
        ],
        objectives={"metric": ObjectiveProperties(minimize=True)},
    )
    return ax_client


def map_params_to_sim(param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed, terrain_length):
    from SineTestSideSlip_sim import Params, SimParams

    p = Params()
    p.particle_spacing = particle_spacing

    rad_outer = int(param_dict["rad"])
    w_by_r = float(param_dict["w_by_r"])
    pct_g = float(param_dict["what_percent_is_grouser"])

    g_height_int = int(round(rad_outer * pct_g))
    g_height_int = max(0, g_height_int)
    rad_inner_int = int(rad_outer - g_height_int)
    width_int = int(round(w_by_r * rad_outer))

    p.rad = rad_inner_int
    p.width = width_int
    p.g_height = g_height_int
    p.g_density = int(param_dict["g_density"])
    p.fan_theta_deg = int(param_dict.get("fan_theta_deg", 0))

    # Steering controller
    p.steering_kp = float(param_dict["steering_kp"])
    p.steering_ki = float(param_dict["steering_ki"])
    p.steering_kd = float(param_dict["steering_kd"])

    sp = SimParams()
    sp.density = density
    sp.cohesion = cohesion
    sp.friction = friction
    sp.max_force = max_force
    sp.target_speed = target_speed
    try:
        sp.terrain_length = float(terrain_length)
    except Exception:
        pass

    return p, sp


def main():
    parser = argparse.ArgumentParser(description="Bayesian Optimization for wheel + steering controller gains (joint).")
    parser.add_argument("--particle_spacing", type=float, default=0.005, help="Particle spacing (meters)")
    parser.add_argument("--sobol", type=int, default=1800, help="Number of Sobol warmup trials (default: 1800)")
    parser.add_argument("--max_trials", type=int, default=3300, help="Total number of trials (default: 3300 -> 1800 Sobol + 1500 BO)")
    parser.add_argument("--q", type=int, default=1, help="Parallel suggestions per batch (evaluated sequentially here)")
    parser.add_argument("--out_dir", type=str, default="SineSideSlip_wControl", help="Output directory for logs and state")
    parser.add_argument("--state_path", type=str, default=None, help="Path for Ax JSON state (default: out_dir/ax_state_wControl.json)")
    parser.add_argument("--trials_path", type=str, default=None, help="Path for trials CSV (default: out_dir/trials_wControl.csv)")
    parser.add_argument("--resume", action="store_true", help="Resume from existing Ax state if present")
    # Environment / material
    parser.add_argument("--density", type=float, default=1700, help="Material density")
    parser.add_argument("--cohesion", type=float, default=0, help="Material cohesion")
    parser.add_argument("--friction", type=float, default=0.6, help="Material friction coefficient")
    parser.add_argument("--max_force", type=float, default=25, help="Maximum pulling force")
    parser.add_argument("--target_speed", type=float, default=3.0, help="Target vehicle speed")
    parser.add_argument("--terrain_length", type=float, default=5.0, help="Terrain length in meters (e.g., 5.0 or 10.0)")
    # Sine metric weighting
    parser.add_argument("--weight_speed", type=float, default=0.4, help="Weight for speed metric component")
    parser.add_argument("--weight_power", type=float, default=0.4, help="Weight for power metric component")
    parser.add_argument("--weight_beta", type=float, default=0.0, help="Weight for side-slip (beta) metric component (default: 0 to focus on rms_error)")
    # Optional sine shape overrides
    parser.add_argument("--sine_amplitude", type=float, default=None, help="Override sine amplitude in meters (optional)")
    parser.add_argument("--num_periods", type=int, default=None, help="Override number of sine periods (optional)")

    args = parser.parse_args()

    particle_spacing = float(args.particle_spacing)
    out_dir = args.out_dir
    ensure_dir(out_dir)
    state_path = args.state_path or os.path.join(out_dir, "ax_state_wControl.json")
    trials_csv = args.trials_path or os.path.join(out_dir, "trials_wControl.csv")
    failed_log = os.path.join(out_dir, "failed_trials_wControl.jsonl")

    ax_client = build_ax_client(
        particle_spacing=particle_spacing,
        sobol_trials=args.sobol,
        state_path=state_path,
        resume=args.resume,
    )

    csv_columns = [
        "timestamp",
        "trial_index",
        "metric",
        "total_time_to_reach",
        "rms_error",
        "average_power",
        "beta_rms",
        "rad_outer",
        "w_by_r",
        "what_percent_is_grouser",
        "g_density",
        "fan_theta_deg",
        "steering_kp",
        "steering_ki",
        "steering_kd",
        "particle_spacing",
    ]

    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write(",".join(csv_columns) + "\n")

    total_limit = max(1, int(args.max_trials))

    density = args.density
    cohesion = args.cohesion
    friction = args.friction
    max_force = args.max_force
    target_speed = args.target_speed
    terrain_length = args.terrain_length

    while ax_client.experiment.num_trials < total_limit:
        remaining = total_limit - ax_client.experiment.num_trials
        request = min(max(1, int(args.q)), remaining)
        suggestions, _ = ax_client.get_next_trials(max_trials=request)
        for trial_index, param_dict in suggestions.items():
            try:
                from SineTestSideSlip_sim import sim

                params_for_sim, sim_params = map_params_to_sim(
                    param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed, terrain_length
                )

                kwargs = {}
                if args.sine_amplitude is not None:
                    kwargs["sine_amplitude"] = float(args.sine_amplitude)
                if args.num_periods is not None:
                    kwargs["num_periods"] = int(args.num_periods)

                metric, total_time_to_reach, rms_error, average_power, beta_rms, sim_failed = sim(
                    params_for_sim,
                    sim_params,
                    weight_speed=float(args.weight_speed),
                    weight_power=float(args.weight_power),
                    weight_beta=float(args.weight_beta),
                    **kwargs,
                )

                if sim_failed or metric is None or not _isfinite(metric):
                    ax_client.abandon_trial(trial_index=trial_index)
                    with open(failed_log, "a") as flog:
                        flog.write(json.dumps({
                            "timestamp": datetime.utcnow().isoformat() + "Z",
                            "trial_index": trial_index,
                            "parameters": param_dict,
                            "error": "sim_failed" if sim_failed else "metric is None",
                        }) + "\n")
                    print(f"  Trial {trial_index}: FAILED")
                    continue

                ax_client.complete_trial(trial_index=trial_index, raw_data={"metric": float(metric)})
                with open(trials_csv, "a") as f:
                    f.write(
                        f"{datetime.utcnow().isoformat()}Z,{trial_index},{metric:.4f},{total_time_to_reach:.4f},{rms_error:.6f},{average_power:.2f},{beta_rms:.6f},"
                        f"{param_dict['rad']},{param_dict['w_by_r']:.6f},{param_dict['what_percent_is_grouser']:.6f},{param_dict['g_density']},{param_dict['fan_theta_deg']},"
                        f"{param_dict['steering_kp']:.6f},{param_dict['steering_ki']:.6f},{param_dict['steering_kd']:.6f},{particle_spacing}\n"
                    )
                print(
                    f"  Trial {trial_index}: metric={metric:.4f}, time={total_time_to_reach:.4f}s, rms={rms_error:.4f}m, power={average_power:.2f}W, beta_rms={beta_rms:.4f}rad, "
                    f"rad={param_dict['rad']}, w_by_r={param_dict['w_by_r']:.3f}, pct_g={param_dict['what_percent_is_grouser']:.3f}, g_density={param_dict['g_density']}, fan_theta={param_dict['fan_theta_deg']}, "
                    f"kp={param_dict['steering_kp']:.3f}, ki={param_dict['steering_ki']:.3f}, kd={param_dict['steering_kd']:.3f}"
                )
            except KeyboardInterrupt:
                print("Keyboard interrupt received. Saving state and exiting.")
                ax_client.save_to_json_file(filepath=state_path)
                return
            except Exception as e:
                ax_client.abandon_trial(trial_index=trial_index)
                with open(failed_log, "a") as flog:
                    flog.write(json.dumps({
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "trial_index": trial_index,
                        "parameters": param_dict,
                        "error": str(e),
                    }) + "\n")
                print(f"  Trial {trial_index}: EXCEPTION - {str(e)}")

        ax_client.save_to_json_file(filepath=state_path)

    try:
        best_params, best_vals = ax_client.get_best_parameters()
        print("\nOptimization complete!")
        print(f"Best params: {best_params}")
        print(f"Best metric: {best_vals}")
    except Exception:
        pass


if __name__ == "__main__":
    main()
