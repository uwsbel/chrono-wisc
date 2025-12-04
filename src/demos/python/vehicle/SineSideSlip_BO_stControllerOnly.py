import argparse
import json
import math
import os
from datetime import datetime

import pandas as pd
from ax.adapter.registry import Generators
from ax.generation_strategy.center_generation_node import CenterGenerationNode
from ax.generation_strategy.generation_node import GenerationNode
from ax.generation_strategy.generation_strategy import GenerationStrategy
from ax.generation_strategy.generator_spec import GeneratorSpec
from ax.generation_strategy.transition_criterion import MinTrials
from ax.service.ax_client import AxClient, ObjectiveProperties
from botorch.acquisition.logei import qLogNoisyExpectedImprovement


def _isfinite(x):
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _int_from(val, default=None):
    try:
        return int(round(float(val)))
    except Exception:
        return default


def _float_from(val, default=None):
    try:
        return float(val)
    except Exception:
        return default


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def load_best_wheel(csv_path, metric_col="metric", ascending=True, rank=1):
    """Load the rank-th wheel row from a trials.csv file (rank is 1-based)."""
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Wheel trials CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)
    if metric_col not in df.columns:
        raise ValueError(f"Metric column '{metric_col}' not found in {csv_path}; columns={df.columns.tolist()}")
    df[metric_col] = pd.to_numeric(df[metric_col], errors="coerce")
    df = df[df[metric_col].notna()]
    df = df[df[metric_col].map(_isfinite)]
    if df.empty:
        raise ValueError(f"No finite '{metric_col}' values found in {csv_path}")
    if rank <= 0 or rank > len(df):
        raise ValueError(f"Requested rank {rank} is out of range (1..{len(df)}) for {csv_path}")
    df_sorted = df.sort_values(by=metric_col, ascending=ascending)
    return df_sorted.iloc[rank - 1].to_dict(), rank


def derive_wheel_geometry(row, fallback_spacing=None):
    """Return a dict with derived wheel geometry fields based on a CSV row."""
    rad_outer = _int_from(row.get("rad_outer")) or _int_from(row.get("rad"))
    w_by_r = _float_from(row.get("w_by_r"))
    pct_g = _float_from(row.get("what_percent_is_grouser"))
    g_density = _int_from(row.get("g_density"))
    fan_theta_deg = _float_from(row.get("fan_theta_deg"), 0.0)
    particle_spacing = _float_from(row.get("particle_spacing"), fallback_spacing)

    missing = []
    for key, val in [
        ("rad_outer", rad_outer),
        ("w_by_r", w_by_r),
        ("what_percent_is_grouser", pct_g),
        ("g_density", g_density),
        ("particle_spacing", particle_spacing),
    ]:
        if val is None or (isinstance(val, float) and not math.isfinite(val)):
            missing.append(key)
    if missing:
        raise ValueError(f"Missing required wheel fields in trials.csv: {missing}")

    g_height_int = int(round(rad_outer * pct_g))
    g_height_int = max(0, g_height_int)
    rad_inner_int = int(rad_outer - g_height_int)
    width_int = int(round(w_by_r * rad_outer))

    return {
        "rad_outer": rad_outer,
        "rad_inner": rad_inner_int,
        "width": width_int,
        "g_height": g_height_int,
        "g_density": g_density,
        "fan_theta_deg": fan_theta_deg,
        "particle_spacing": particle_spacing,
        "w_by_r": w_by_r,
        "pct_g": pct_g,
    }


def build_ax_client(sobol_trials, state_path, resume, enable_ki=True):
    """Create or load an AxClient configured for steering-gain tuning."""
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
    params = [
        {"name": "steering_kp", "type": "range", "bounds": [1.0, 15.0], "value_type": "float"},
        {"name": "steering_kd", "type": "range", "bounds": [0.0, 4.0], "value_type": "float"},
    ]
    if enable_ki:
        params.insert(1, {"name": "steering_ki", "type": "range", "bounds": [0.0, 4.0], "value_type": "float"})
    ax_client.create_experiment(
        name="sine_steering_tuning",
        parameters=params,
        objectives={"metric": ObjectiveProperties(minimize=True)},
    )
    return ax_client


def map_params_to_sim(param_dict, wheel_geom, density, cohesion, friction, max_force, target_speed, terrain_length):
    from SineTestSideSlip_sim import Params, SimParams

    p = Params()
    p.particle_spacing = wheel_geom["particle_spacing"]
    p.rad = wheel_geom["rad_inner"]
    p.width = wheel_geom["width"]
    p.g_height = wheel_geom["g_height"]
    p.g_density = wheel_geom["g_density"]
    p.fan_theta_deg = wheel_geom["fan_theta_deg"]

    # Steering controller overrides
    p.steering_kp = float(param_dict.get("steering_kp", 0.0))
    p.steering_ki = float(param_dict.get("steering_ki", 0.0))
    p.steering_kd = float(param_dict.get("steering_kd", 0.0))

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
    parser = argparse.ArgumentParser(description="BO for steering gains with wheel geometry frozen to best wheel.")
    parser.add_argument("wheel_trials", help="Path to wheel optimization trials.csv (used to select best wheel)")
    parser.add_argument("--metric_col", type=str, default="metric", help="Metric column to rank wheels (default: metric)")
    parser.add_argument("--descending", action="store_true", help="Set if the metric should be maximized instead of minimized")
    parser.add_argument("--rank", type=int, default=1, help="Rank of the wheel to freeze (1=best, 10=10th best, etc.)")
    parser.add_argument("--out_dir", type=str, default=None, help="Output directory (default: parent of wheel_trials)")
    parser.add_argument("--state_path", type=str, default=None, help="Path for Ax JSON state (default: out_dir/ax_state_stControllerOnly.json)")
    parser.add_argument("--trials_path", type=str, default=None, help="Path for trials CSV (default: out_dir/trials_stControllerOnly.csv)")
    parser.add_argument("--resume", action="store_true", help="Resume from existing Ax state if present")
    parser.add_argument("--sobol", type=int, default=600, help="Number of Sobol warmup trials before BO (default: 600)")
    parser.add_argument("--max_trials", type=int, default=1000, help="Total number of trials (Sobol + BO)")
    parser.add_argument("--q", type=int, default=1, help="Parallel suggestions per batch (evaluated sequentially here)")
    parser.add_argument("--disable-ki", action="store_true", help="Disable sampling of steering_ki (fix to 0)")
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
    parser.add_argument("--weight_beta", type=float, default=0.2, help="Weight for side-slip (beta) metric component")
    # Optional sine shape overrides
    parser.add_argument("--sine_amplitude", type=float, default=None, help="Override sine amplitude in meters (optional)")
    parser.add_argument("--num_periods", type=int, default=None, help="Override number of sine periods (optional)")

    args = parser.parse_args()

    wheel_csv = os.path.abspath(args.wheel_trials)
    if not os.path.isfile(wheel_csv):
        parser.error(f"Wheel trials CSV not found: {wheel_csv}")

    out_dir = args.out_dir or os.path.dirname(wheel_csv)
    ensure_dir(out_dir)
    state_path = args.state_path or os.path.join(out_dir, "ax_state_stControllerOnly.json")
    trials_csv = args.trials_path or os.path.join(out_dir, "trials_stControllerOnly.csv")
    failed_log = os.path.join(out_dir, "failed_trials_stControllerOnly.jsonl")
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

    # Load best wheel from prior optimization
    best_wheel_row, best_rank = load_best_wheel(
        wheel_csv,
        metric_col=args.metric_col,
        ascending=not args.descending,
        rank=max(1, int(args.rank)),
    )
    wheel_geom = derive_wheel_geometry(best_wheel_row)
    print(f"Freezing wheel at rank {best_rank} from previous run:")
    print(json.dumps({k: v for k, v in wheel_geom.items() if k not in {"rad_inner", "width"}}, indent=2))

    ax_client = build_ax_client(
        sobol_trials=args.sobol,
        state_path=state_path,
        resume=args.resume,
        enable_ki=not args.disable_ki,
    )

    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write(",".join(csv_columns) + "\n")
    else:
        # Upgrade legacy CSV (without steering_ki) by adding the column with default 0
        df_existing = pd.read_csv(trials_csv)
        if "steering_ki" not in df_existing.columns:
            df_existing["steering_ki"] = 0.0
            # Reorder columns if possible to keep consistent layout
            ordered_cols = [c for c in csv_columns if c in df_existing.columns]
            remaining = [c for c in df_existing.columns if c not in ordered_cols]
            df_existing = df_existing[ordered_cols + remaining]
            df_existing.to_csv(trials_csv, index=False)
            print(f"Upgraded existing trials CSV to include steering_ki -> {trials_csv}")

    total_limit = max(1, int(args.max_trials))

    while ax_client.experiment.num_trials < total_limit:
        remaining = total_limit - ax_client.experiment.num_trials
        request = min(max(1, int(args.q)), remaining)
        suggestions, _ = ax_client.get_next_trials(max_trials=request)
        for trial_index, param_dict in suggestions.items():
            try:
                from SineTestSideSlip_sim import sim

                params_for_sim, sim_params = map_params_to_sim(
                    param_dict, wheel_geom, args.density, args.cohesion, args.friction, args.max_force, args.target_speed, args.terrain_length
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
                        f"{wheel_geom['rad_outer']},{wheel_geom['w_by_r']:.6f},{wheel_geom['pct_g']:.6f},{wheel_geom['g_density']},{wheel_geom['fan_theta_deg']:.6f},"
                        f"{param_dict['steering_kp']:.6f},{param_dict.get('steering_ki', 0.0):.6f},{param_dict['steering_kd']:.6f},{wheel_geom['particle_spacing']}\n"
                    )
                print(
                    f"  Trial {trial_index}: metric={metric:.4f}, time={total_time_to_reach:.4f}s, rms={rms_error:.4f}m, power={average_power:.2f}W, beta_rms={beta_rms:.4f}rad, kp={param_dict['steering_kp']:.3f}, ki={param_dict.get('steering_ki', 0.0):.3f}, kd={param_dict['steering_kd']:.3f}"
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
        print(f"Best steering params: {best_params}")
        print(f"Best metric: {best_vals}")
    except Exception:
        pass


if __name__ == "__main__":
    main()
