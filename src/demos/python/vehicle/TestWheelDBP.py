#!/usr/bin/env python3

import argparse
from pathlib import Path

import pandas as pd

from DBP100SlipTest_sim import Params as WheelParams, SimParams, sim


def _sanitize(value):
    s = str(value)
    return "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in s)


def _fmt_num(v):
    if isinstance(v, float):
        if v.is_integer():
            return str(int(v))
        return f"{v:.4g}"
    return str(v)


def load_wheel_from_rank(csv_path: Path, rank: int, metric_col: str, descending: bool):
    df = pd.read_csv(csv_path)
    if metric_col not in df.columns:
        raise ValueError(f"Metric column '{metric_col}' not found in {csv_path}")
    required_cols = ["rad_outer", "w_by_r", "what_percent_is_grouser", "g_density", "fan_theta_deg", "particle_spacing"]
    missing_cols = [c for c in required_cols if c not in df.columns]
    if missing_cols:
        raise ValueError(f"Missing required columns in {csv_path}: {missing_cols}")
    df[metric_col] = pd.to_numeric(df[metric_col], errors="coerce")
    df = df[df[metric_col].notna() & pd.notnull(df[metric_col])]
    df_sorted = df.sort_values(by=metric_col, ascending=not descending)
    if rank <= 0 or rank > len(df_sorted):
        raise ValueError(f"Rank {rank} out of range (1..{len(df_sorted)}) for {csv_path}")
    row = df_sorted.iloc[rank - 1]
    rad_outer = int(round(row["rad_outer"]))
    w_by_r = float(row["w_by_r"])
    pct_g = float(row["what_percent_is_grouser"])
    g_density = int(round(row["g_density"]))
    fan_theta_deg = float(row["fan_theta_deg"])
    particle_spacing = float(row["particle_spacing"])

    g_height = max(0, int(round(rad_outer * pct_g)))
    rad_inner = int(rad_outer - g_height)
    width = int(round(w_by_r * rad_outer))

    wheel_args = {
        "rad": rad_inner,
        "width": width,
        "g_height": g_height,
        "g_density": g_density,
        "particle_spacing": particle_spacing,
        "fan_theta_deg": fan_theta_deg,
    }
    raw = {
        "rad_outer": rad_outer,
        "w_by_r": w_by_r,
        "pct_g": pct_g,
        "g_density": g_density,
        "fan_theta_deg": fan_theta_deg,
        "particle_spacing": particle_spacing,
        "metric": row[metric_col],
    }
    return wheel_args, raw


def main():
    parser = argparse.ArgumentParser(description="Run DBP slip test for a wheel from BO trials or manual input.")
    parser.add_argument("--trials", type=Path, help="Path to trials.csv (used with --rank).")
    parser.add_argument("--rank", type=int, help="Rank (1-based) to select from trials.csv.")
    parser.add_argument("--metric-col", default="metric", help="Metric column in trials.csv (default: metric).")
    parser.add_argument("--descending", action="store_true", help="Sort trials in descending order of metric.")

    parser.add_argument("--rad", type=int, help="Inner radius (multiples of particle spacing) when using manual input.")
    parser.add_argument("--width", type=int, help="Width (multiples of particle spacing) when using manual input.")
    parser.add_argument("--g_height", type=int, help="Grouser height (multiples of particle spacing) for manual input.")
    parser.add_argument("--g_density", type=int, help="Number of grousers per revolution for manual input.")
    parser.add_argument("--particle_spacing", type=float, help="Particle spacing for manual input.")
    parser.add_argument("--fan_theta_deg", type=float, help="Fan angle (deg) for manual input.")

    parser.add_argument("--density", type=float, default=1700, help="Soil density (default: 1700).")
    parser.add_argument("--cohesion", type=float, default=1000, help="Soil cohesion (default: 0).")
    parser.add_argument("--friction", type=float, default=0.8, help="Soil friction coefficient (default: 0.8).")

    parser.add_argument("--visualize", action="store_true", help="Enable visualization.")
    parser.add_argument("--snapshots", action="store_true", help="Enable snapshots (implies visualize).")
    parser.add_argument("--output-dir", type=Path, default=Path("."), help="Directory for CSV (and snapshots, if any).")

    args = parser.parse_args()

    use_rank = args.trials is not None and args.rank is not None
    if use_rank:
        wheel_args, raw = load_wheel_from_rank(args.trials, args.rank, args.metric_col, args.descending)
    else:
        required = ["rad", "width", "g_height", "g_density", "particle_spacing", "fan_theta_deg"]
        missing = [name for name in required if getattr(args, name) is None]
        if missing:
            parser.error(f"Manual input requires: {', '.join(missing)}")
        wheel_args = {
            "rad": int(args.rad),
            "width": int(args.width),
            "g_height": int(args.g_height),
            "g_density": int(args.g_density),
            "particle_spacing": float(args.particle_spacing),
            "fan_theta_deg": float(args.fan_theta_deg),
        }
        raw = None

    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    p = WheelParams()
    p.rad = wheel_args["rad"]
    p.width = wheel_args["width"]
    p.g_height = wheel_args["g_height"]
    p.g_density = wheel_args["g_density"]
    p.particle_spacing = wheel_args["particle_spacing"]
    p.fan_theta_deg = wheel_args["fan_theta_deg"]

    sp = SimParams()
    sp.density = args.density
    sp.cohesion = args.cohesion
    sp.friction = args.friction

    logs = []

    def log_recorder(t, pull_fx, veh_x_vel, wheel_rpms, power):
        row = {"time": t, "reaction_fx": pull_fx, "veh_x_vel": veh_x_vel, "power": power}
        for idx, rpm in enumerate(wheel_rpms):
            row[f"wheel{idx}_rpm"] = rpm
        logs.append(row)

    fan_tag = _sanitize(_fmt_num(p.fan_theta_deg))
    fname = f"pull_rad{p.rad}_w{p.width}_gh{p.g_height}_gd{p.g_density}_ft{fan_tag}.csv"
    visualize = args.visualize or args.snapshots
    snapshot_dir = out_dir / Path(fname).with_suffix("") if args.snapshots else None
    if snapshot_dir is not None:
        snapshot_dir.mkdir(parents=True, exist_ok=True)

    metric, total_time, _, avg_power, sim_failed = sim(
        p,
        sp,
        snapshot_dir=snapshot_dir,
        visualize=visualize,
        log_recorder=log_recorder,
    )

    if logs:
        df_out = pd.DataFrame(logs)
        csv_path = out_dir / fname
        df_out.to_csv(csv_path, index=False)
    else:
        csv_path = None

    avg_pull = sum(abs(r["reaction_fx"]) for r in logs) / len(logs) if logs else 0.0
    power_samples = [r["power"] for r in logs if r["time"] > 0.5]
    avg_power = sum(power_samples) / len(power_samples) if power_samples else 0.0

    print("=== DBP slip run ===")
    print(f"Avg pull force: {avg_pull}")
    print(f"Simulated time: {total_time}")
    print(f"Average power: {avg_power}")
    if csv_path:
        print(f"Saved CSV: {csv_path}")
    print("Wheel parameters:")
    for key in ("rad", "width", "g_height", "g_density", "particle_spacing", "fan_theta_deg"):
        print(f"  {key}: {wheel_args[key]}")
    if raw:
        print("Source (trials.csv) raw parameters:")
        for key, val in raw.items():
            print(f"  {key}: {val}")
    print(f"Simulation failed: {sim_failed}")


if __name__ == "__main__":
    main()
