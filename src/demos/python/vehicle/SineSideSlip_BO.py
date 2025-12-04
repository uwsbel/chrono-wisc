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
from botorch.acquisition.logei import qLogNoisyExpectedImprovement

"""
Single-GPU, single-process BO driver for the Sine test with side-slip metric,
aligned with SineTestSideSlip_global_single.py. Evaluates suggestions
sequentially and can seed prior completed trials from SineSideSlip_global_* runs.
"""


def _fmt_num(x):
    """Format numbers for folder names without unnecessary trailing zeros.
    Examples: 25.0 -> "25", 2.0 -> "2", 0.005 -> "0.005", 0.0 -> "0".
    """
    if isinstance(x, float):
        s = ("%f" % x).rstrip("0").rstrip(".")
        return s if s != "" else "0"
    return str(x)


def compute_int_bounds(lower_m, upper_m, spacing):
    lb = int(math.ceil(lower_m / spacing))
    ub = int(math.floor(upper_m / spacing))
    if lb > ub:
        raise ValueError(f"Invalid bounds after scaling: [{lb}, {ub}] for spacing={spacing}")
    return lb, ub


def build_ax_client(particle_spacing, sobol_trials, state_path, resume):
    """
    Create an AxClient configured to:
      - run `sobol_trials` quasi-random Sobol initial points,
      - then switch to Modular BoTorch Bayesian optimization using qLogNEI.
    """
    botorch_node = GenerationNode(
        node_name="BoTorch",
        generator_specs=[
            GeneratorSpec(
                generator_enum=Generators.BOTORCH_MODULAR,
                model_kwargs={
                    "botorch_acqf_class": qLogNoisyExpectedImprovement,
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
        name="Center+Sobol+BoTorch(qLogNEI)",
        nodes=[center_node, sobol_node, botorch_node],
    )

    if resume and os.path.isfile(state_path):
        ax_client = AxClient.load_from_json_file(filepath=state_path)
        try:
            best = ax_client.get_best_parameters()
            if best is not None:
                best_params, best_vals = best
                print(f"Best params: {best_params}")
                print(f"Best metric: {best_vals}")
            else:
                print("No completed trials in loaded Ax state yet; continuing.")
        except Exception as e:
            print(f"Could not determine best parameters on resume: {e}")
        return ax_client

    ax_client = AxClient(generation_strategy=gs)

    # Outer radius (including grousers) bounds in meters
    rad_lb, rad_ub = compute_int_bounds(0.05, 0.14, particle_spacing)

    # Note: w_by_r upper bound widened to 1.4 to match SideSlip global sampling
    ax_client.create_experiment(
        name="sine_wheel_optimization_sideslip",
        parameters=[
            {"name": "rad", "type": "range", "bounds": [rad_lb, rad_ub], "value_type": "int"},
            {"name": "w_by_r", "type": "range", "bounds": [0.7, 1.4], "value_type": "float"},
            {"name": "what_percent_is_grouser", "type": "range", "bounds": [0.0, 0.3], "value_type": "float"},
            {"name": "g_density", "type": "range", "bounds": [2, 16], "value_type": "int"},
            {"name": "fan_theta_deg", "type": "range", "bounds": [45, 135], "value_type": "int"},
        ],
        objectives={
            "metric": ObjectiveProperties(minimize=True)
        },
    )

    return ax_client


def map_params_to_sim(param_dict, particle_spacing, density, cohesion, friction, max_force, target_speed, terrain_length):
    from SineTestSideSlip_sim import Params, SimParams

    p = Params()
    p.particle_spacing = particle_spacing
    # New parametrization inputs (outer radius and ratios)
    rad_outer = int(param_dict["rad"])  # multiples of particle spacing
    w_by_r = float(param_dict["w_by_r"])  # unitless ratio
    pct_g = float(param_dict["what_percent_is_grouser"])  # fraction of outer radius

    # Derived discrete geometry in multiples of spacing
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
    try:
        sp.terrain_length = float(terrain_length)
    except Exception:
        pass

    return p, sp


def _seed_from_csv(ax_client, csv_paths, expected_particle_spacing=None):
    """Read existing completed trials from one or more CSV files and attach to Ax.
    The CSV is expected to have the schema written by SineTestSideSlip_global_single.py:
    timestamp,trial_index,metric,total_time_to_reach,rms_error,average_power,beta_rms,rad_outer,w_by_r,what_percent_is_grouser,g_density,fan_theta_deg,particle_spacing

    Parameters are mapped as:
      rad <- rad_outer
      w_by_r <- w_by_r
      what_percent_is_grouser <- what_percent_is_grouser
      g_density <- g_density
      fan_theta_deg <- fan_theta_deg

    If expected_particle_spacing is provided, rows with a different particle_spacing are skipped.
    Returns the number of trials successfully seeded.
    """
    if not csv_paths:
        return 0
    if isinstance(csv_paths, str):
        csv_paths = [csv_paths]

    attached = 0
    completed = 0
    for path in csv_paths:
        if not os.path.isfile(path):
            print(f"Seed CSV not found: {path}")
            continue
        try:
            with open(path, "r") as f:
                header = f.readline()
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split(",")
                    if len(parts) < 13:
                        continue
                    try:
                        # Columns: 0 ts, 1 trial_idx, 2 metric, 3 time, 4 rms, 5 power, 6 beta_rms,
                        # 7 rad_outer, 8 w_by_r, 9 pct_g, 10 g_density, 11 fan_theta_deg, 12 particle_spacing
                        metric = float(parts[2])
                        if not math.isfinite(metric):
                            # Skip non-finite metrics
                            continue
                        rad_outer = int(float(parts[7]))
                        w_by_r = float(parts[8])
                        pct_g = float(parts[9])
                        g_density = int(float(parts[10]))
                        fan_theta_deg = int(float(parts[11]))
                        pspace = float(parts[12])
                        if expected_particle_spacing is not None and abs(pspace - expected_particle_spacing) > 1e-12:
                            continue
                        params = {
                            "rad": rad_outer,
                            "w_by_r": w_by_r,
                            "what_percent_is_grouser": pct_g,
                            "g_density": g_density,
                            "fan_theta_deg": fan_theta_deg,
                        }
                        _, t_idx = ax_client.attach_trial(parameters=params)
                        attached += 1
                        print(f"Attached trial {t_idx} with parameters: {params}")
                        try:
                            ax_client.complete_trial(trial_index=t_idx, raw_data={"metric": metric})
                            completed += 1
                        except Exception as comp_e:
                            # If completion fails, abandon the trial to keep state clean
                            try:
                                ax_client.abandon_trial(trial_index=t_idx)
                            except Exception:
                                pass
                            print(f"  Completion failed for attached trial {t_idx}: {comp_e}")
                    except Exception:
                        continue
        except Exception as e:
            print(f"Failed to read seed CSV {path}: {e}")
            continue
    if completed > 0:
        print(f"Seeding summary: attached={attached}, completed={completed} from CSV.")
    else:
        print(f"No trials were seeded from CSV. attached={attached}, completed={completed}.")
    return completed


def _autodiscover_seed_csv(density, cohesion, friction, max_force, target_speed, terrain_length):
    """Return a candidate seed CSV path based on requested parameters.
    Pattern: SineSideSlip_global_{density}_{cohesion}_{friction}_{max_force}_{target_speed}_{terrain_length}/trials.csv
    """
    folder = (
        f"SineSideSlip_global_{_fmt_num(density)}_"
        f"{_fmt_num(cohesion)}_{_fmt_num(friction)}_"
        f"{_fmt_num(max_force)}_{_fmt_num(target_speed)}_{_fmt_num(terrain_length)}"
    )
    return os.path.join(folder, "trials.csv")


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def main():
    parser = argparse.ArgumentParser(description="Bayesian Optimization (Sine test with side-slip) for wheel parameters using Ax + BoTorch (qLogNEI)")
    parser.add_argument("--particle_spacing", type=float, default=0.005, help="Particle spacing (meters)")
    parser.add_argument("--batches", type=int, default=100, help="Number of outer iterations (batches)")
    parser.add_argument("--q", type=int, default=8, help="Suggestions evaluated per batch (sequential)")
    parser.add_argument("--sobol", type=int, default=0, help="Number of Sobol warmup trials before BO")
    # Default out_dir follows SineSideSlip_global_* template for consistency with global runs
    parser.add_argument("--out_dir", type=str, default=None, help="Output directory for logs and state (default: SineSideSlip_global_* template)")
    parser.add_argument("--state_path", type=str, default=None, help="Path to save/load Ax state JSON (defaults to out_dir/ax_state.json)")
    parser.add_argument("--resume", action="store_true", help="Resume from existing Ax state JSON if present")
    parser.add_argument("--seed", action="store_true", help="Autodiscover and seed prior trials from SineSideSlip_global_* matching the parameters")
    parser.add_argument("--seed_csv", type=str, nargs="*", default=None, help="Path(s) to trials.csv files to seed completed trials (overrides autodiscovery)")
    # Environment / material
    parser.add_argument("--density", type=float, default=1700, help="Material density")
    parser.add_argument("--cohesion", type=float, default=0, help="Material cohesion")
    parser.add_argument("--friction", type=float, default=0.8, help="Material friction coefficient")
    parser.add_argument("--max_force", type=float, default=25, help="Maximum pulling force")
    parser.add_argument("--target_speed", type=float, default=2.0, help="Target vehicle speed")
    parser.add_argument("--terrain_length", type=float, default=5.0, help="Terrain length in meters (e.g., 5.0 or 10.0)")
    # Sine metric weighting (defaults align with SineTestSideSlip_global_single)
    parser.add_argument("--weight_speed", type=float, default=0.5, help="Weight for speed metric component")
    parser.add_argument("--weight_power", type=float, default=0.0, help="Weight for power metric component")
    parser.add_argument("--weight_beta", type=float, default=0.2, help="Weight for side-slip (beta) metric component")
    # Optional sine shape overrides
    parser.add_argument("--sine_amplitude", type=float, default=None, help="Override sine amplitude in meters (optional)")
    parser.add_argument("--num_periods", type=int, default=None, help="Override number of sine periods (optional)")

    args = parser.parse_args()

    particle_spacing = args.particle_spacing
    batches = args.batches
    q = args.q

    # Default out_dir template if not provided (use normalized float formatting)
    if args.out_dir is None:
        out_dir = (
            f"SineSideSlip_global_{_fmt_num(args.density)}_"
            f"{_fmt_num(args.cohesion)}_{_fmt_num(args.friction)}_"
            f"{_fmt_num(args.max_force)}_{_fmt_num(args.target_speed)}_{_fmt_num(args.terrain_length)}"
        )
    else:
        out_dir = args.out_dir
    ensure_dir(out_dir)
    state_path = args.state_path or os.path.join(out_dir, "ax_state.json")
    trials_csv = os.path.join(out_dir, "trials.csv")
    failed_log = os.path.join(out_dir, "failed_trials.jsonl")

    # Keep Sobol warmup by default. If seeding adds enough trials, transition will skip Sobol.
    sobol_trials = args.sobol

    ax_client = build_ax_client(
        particle_spacing=particle_spacing,
        sobol_trials=sobol_trials,
        state_path=state_path,
        resume=args.resume,
    )

    # Seeding: use explicit CSV if provided; otherwise autodiscover when --seed is passed
    seeded_count = 0
    if args.seed_csv and len(args.seed_csv) > 0:
        seeded_count = _seed_from_csv(ax_client, args.seed_csv, expected_particle_spacing=particle_spacing)
    elif args.seed:
        auto_csv = _autodiscover_seed_csv(args.density, args.cohesion, args.friction, args.max_force, args.target_speed, args.terrain_length)
        seeded_count = _seed_from_csv(ax_client, [auto_csv], expected_particle_spacing=particle_spacing)
    if (args.seed or (args.seed_csv and len(args.seed_csv) > 0)) and seeded_count == 0:
        print("Seeding requested but no trials found; retaining Sobol warmup to avoid empty-data BO.")

    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write("timestamp,trial_index,metric,total_time_to_reach,rms_error,average_power,beta_rms,rad_outer,w_by_r,what_percent_is_grouser,g_density,fan_theta_deg,particle_spacing\n")

    density = args.density
    cohesion = args.cohesion
    friction = args.friction
    max_force = args.max_force
    target_speed = args.target_speed
    terrain_length = args.terrain_length

    for batch_idx in range(batches):
        print(f"Batch {batch_idx + 1}/{batches}")
        suggestions, _ = ax_client.get_next_trials(max_trials=q)

        # Evaluate each suggestion sequentially on the single GPU
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
                    params_for_sim, sim_params,
                    weight_speed=float(args.weight_speed),
                    weight_power=float(args.weight_power),
                    weight_beta=float(args.weight_beta),
                    **kwargs,
                )

                if sim_failed or metric is None:
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
                        f"{param_dict['rad']},{param_dict['w_by_r']:.6f},{param_dict['what_percent_is_grouser']:.6f},{param_dict['g_density']},"
                        f"{param_dict['fan_theta_deg']},{particle_spacing}\n"
                    )
                print(
                    f"  Trial {trial_index}: metric={metric:.4f}, time={total_time_to_reach:.4f}s, rms={rms_error:.4f}m, power={average_power:.2f}W, beta_rms={beta_rms:.4f}rad"
                )
            except KeyboardInterrupt:
                print("Keyboard interrupt received. Stopping batch early.")
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
        print(f"\nOptimization complete!")
        print(f"Best params: {best_params}")
        print(f"Best metric: {best_vals}")
    except Exception:
        pass


if __name__ == "__main__":
    main()

