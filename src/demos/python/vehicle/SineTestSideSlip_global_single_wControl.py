import argparse
import json
import math
import os
from datetime import datetime

from botorch.utils.sampling import SobolEngine

"""
Single-threaded Sobol sampling for SineTest (SideSlip) with steering control gains.

This mirrors SineTestSideSlip_global_single.py but additionally samples
steering controller gains (kp, kd) while keeping speed controller gains
at their defaults in SineTestSideSlip_sim.Params.

Results are written to trials.csv with extra columns for steering_kp
and steering_kd. The objective includes side-slip (beta) metrics.
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

    # Steering controller (override defaults)
    p.steering_kp = float(param_dict["steering_kp"])  # [0.1, 20]
    p.steering_kd = float(param_dict["steering_kd"])  # [0, 5]

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
    Parameters follow PullTest bounds for wheel geometry.
    """
    # Compute integer bounds derived from meters and spacing
    rad_lb, rad_ub = compute_int_bounds(0.05, 0.14, particle_spacing)
    gd_lb, gd_ub = 2, 16
    th_lb, th_ub = 45, 135

    # Dimension d=7: [rad_outer, w_by_r, what_percent_is_grouser, g_density, fan_theta_deg, kp, kd]
    engine = SobolEngine(dimension=7, scramble=True, seed=0)
    U = engine.draw(n)  # tensor shape [n, 7], values in [0,1)

    params = []
    for i in range(n):
        u = U[i].tolist()
        pdict = {
            "rad": _int_from_unit(u[0], rad_lb, rad_ub),
            "w_by_r": _float_from_unit(u[1], 0.7, 1.4),
            "what_percent_is_grouser": _float_from_unit(u[2], 0.0, 0.3),
            "g_density": _int_from_unit(u[3], gd_lb, gd_ub),
            "fan_theta_deg": _int_from_unit(u[4], th_lb, th_ub),
            "steering_kp": _float_from_unit(u[5], 1.0, 10.0),
            "steering_kd": _float_from_unit(u[6], 0.0, 2.0),
        }
        params.append(pdict)
    return params


def main():
    parser = argparse.ArgumentParser(description="Single-threaded Sobol sampling for SineTest wheel parameters")
    parser.add_argument("--particle_spacing", type=float, default=0.005, help="Particle spacing (meters)")
    parser.add_argument("--sobol", type=int, default=64, help="Number of Sobol samples to evaluate")
    parser.add_argument("--out_dir", type=str, default=None, help="Output directory for logs (auto-named if omitted)")
    parser.add_argument("--density", type=float, default=1700, help="Material density")
    parser.add_argument("--cohesion", type=float, default=0, help="Material cohesion")
    parser.add_argument("--friction", type=float, default=0.8, help="Material friction coefficient")
    parser.add_argument("--max_force", type=float, default=20, help="Maximum pulling force")
    parser.add_argument("--target_speed", type=float, default=2.0, help="Target vehicle speed")
    parser.add_argument("--resume", action="store_true", help="Skip trials already recorded in out_dir/trials.csv")
    parser.add_argument("--retry_failed", action="store_true", help="When used with --resume, also retry trials listed in failed_trials.jsonl")
    # Optional sine-specific knobs (kept default if not set)
    parser.add_argument("--weight_speed", type=float, default=0.5, help="Weight for speed metric")
    parser.add_argument("--weight_power", type=float, default=0.0, help="Weight for power metric")
    parser.add_argument("--weight_beta", type=float, default=0.2, help="Weight for side-slip (beta) metric")
    parser.add_argument("--sine_amplitude", type=float, default=None, help="Override sine amplitude in meters (optional)")
    parser.add_argument("--num_periods", type=int, default=None, help="Override number of sine periods (optional)")
    # Terrain configuration
    parser.add_argument("--terrain_length", type=float, default=10.0, help="Terrain length in meters (e.g., 5.0 or 10.0)")

    args = parser.parse_args()

    particle_spacing = args.particle_spacing
    n_samples = max(1, int(args.sobol))
    # Auto-name out_dir to include environment parameters like pull runs
    if args.out_dir is None:
        base = "SineSideSlip_global_wControl"
        out_dir = os.path.join(
            os.getcwd(),
            f"{base}_{args.density:g}_{args.cohesion:g}_{args.friction:g}_{args.max_force:g}_{args.target_speed:g}_{args.terrain_length:g}"
        )
    else:
        out_dir = args.out_dir
    ensure_dir(out_dir)
    trials_csv = os.path.join(out_dir, "trials.csv")
    failed_log = os.path.join(out_dir, "failed_trials.jsonl")

    # Initialize CSV header if needed
    if not os.path.isfile(trials_csv):
        with open(trials_csv, "w") as f:
            f.write("timestamp,trial_index,metric,total_time_to_reach,rms_error,average_power,beta_rms,rad_outer,w_by_r,what_percent_is_grouser,g_density,fan_theta_deg,steering_kp,steering_kd,particle_spacing\n")

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

    # Import sim function (side-slip aware)
    from SineTestSideSlip_sim import sim

    # Execute trials sequentially
    for idx in remaining_indices:
        param_dict = param_dicts[idx]
        print("Sampling:\n")
        print(param_dict)
        try:
            params_for_sim, sim_params = map_params_to_sim(
                param_dict, particle_spacing, args.density, args.cohesion,
                args.friction, args.max_force, args.target_speed
            )
            # Pass terrain length to SimParams (SineTest_sim expects it as a SimParam)
            try:
                setattr(sim_params, "terrain_length", float(args.terrain_length))
            except Exception:
                # Attribute assignment is safe even if SimParams lacks predefined field
                sim_params.terrain_length = float(args.terrain_length)

            # Run sim; preserve defaults for controller params
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
                    f"{datetime.utcnow().isoformat()}Z,{idx},{metric:.4f},{total_time_to_reach:.4f},{rms_error:.6f},{average_power:.2f},{beta_rms:.6f},"
                    f"{param_dict['rad']},{param_dict['w_by_r']:.6f},{param_dict['what_percent_is_grouser']:.6f},{param_dict['g_density']},{param_dict['fan_theta_deg']},{param_dict['steering_kp']:.6f},{param_dict['steering_kd']:.6f},{particle_spacing}\n"
                )
            print(
                f"Trial {idx}: metric={metric:.4f}, time={total_time_to_reach:.4f}s, rms={rms_error:.4f}m, power={average_power:.2f}W, beta_rms={beta_rms:.4f}rad, kp={param_dict['steering_kp']:.3f}, kd={param_dict['steering_kd']:.3f}"
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
