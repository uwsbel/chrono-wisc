#!/usr/bin/env python3

import argparse
import math
import os
import shlex
import subprocess
import sys
from pathlib import Path

import pandas as pd
import numpy as np


def _format_cmd(parts):
    return " ".join(shlex.quote(str(p)) for p in parts)


def _sanitize_name(value: str) -> str:
    s = str(value)
    return "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in s)


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


def detect_schema(df: pd.DataFrame) -> str:
    cols = set(df.columns)
    # Sine trials often include rms_error or steering gains
    if {"rms_error", "steering_kp", "steering_kd"} & cols:
        return "sine"
    return "pull"


def build_args_from_row(row: pd.Series, schema: str, sim_script: Path, python: str,
                        visualize: bool, snapshots: bool, output_dir: Path,
                        extra_args: list[str]) -> list[str]:
    # Geometry mapping (common between pull and sine)
    rad_outer = _int_from(row.get("rad_outer")) or _int_from(row.get("rad"))
    w_by_r = _float_from(row.get("w_by_r"))
    pct_g = _float_from(row.get("what_percent_is_grouser"))
    g_density = _int_from(row.get("g_density"))
    fan_theta_deg = _float_from(row.get("fan_theta_deg"))
    particle_spacing = _float_from(row.get("particle_spacing"))

    if None in (rad_outer, w_by_r, pct_g, g_density, fan_theta_deg, particle_spacing):
        missing = []
        for key in ["rad_outer", "w_by_r", "what_percent_is_grouser", "g_density", "fan_theta_deg", "particle_spacing"]:
            if key not in row or row.get(key) is None or (isinstance(row.get(key), float) and not math.isfinite(row.get(key))):
                missing.append(key)
        raise ValueError(f"Missing required geometry in trials.csv: {missing}")

    g_height = int(round(rad_outer * pct_g))
    g_height = max(0, g_height)
    rad_inner = int(rad_outer - g_height)
    width = int(round(w_by_r * rad_outer))

    cmd = [python, str(sim_script)]
    # Common args
    cmd += [
        "--rad", str(rad_inner),
        "--width", str(width),
        "--g_height", str(g_height),
        "--g_density", str(g_density),
        "--particle_spacing", f"{particle_spacing:.10g}",
        "--fan_theta_deg", f"{fan_theta_deg:.10g}",
    ]

    # Optional controller for sine if present
    if schema == "sine":
        for name in ("steering_kp", "steering_kd", "speed_kp", "speed_kd"):
            if name in row and pd.notna(row[name]):
                val = row[name]
                if _isfinite(val):
                    cmd += [f"--{name}", f"{float(val):.10g}"]

    # Visualization and snapshots
    if snapshots:
        cmd.append("--snapshots")
    if visualize or snapshots:
        cmd.append("--visualize")
    if output_dir is not None:
        cmd += ["--output-dir", str(output_dir)]

    # Extra args passthrough
    cmd += list(extra_args)
    return cmd


def derive_sim_params(row: dict, schema: str) -> dict:
    """Compute the exact wheel/controller parameters that will be passed to the sim.
    Returns a dict including derived rad (inner), width, g_height, plus other flags.
    """
    rad_outer = _int_from(row.get("rad_outer")) or _int_from(row.get("rad"))
    w_by_r = _float_from(row.get("w_by_r"))
    pct_g = _float_from(row.get("what_percent_is_grouser"))
    g_density = _int_from(row.get("g_density"))
    fan_theta_deg = _float_from(row.get("fan_theta_deg"))
    particle_spacing = _float_from(row.get("particle_spacing"))

    g_height = int(round(rad_outer * pct_g)) if rad_outer is not None and pct_g is not None else None
    if g_height is not None:
        g_height = max(0, g_height)
    rad_inner = int(rad_outer - g_height) if (rad_outer is not None and g_height is not None) else None
    width = int(round(w_by_r * rad_outer)) if (w_by_r is not None and rad_outer is not None) else None

    out = {
        "rad": rad_inner,
        "width": width,
        "g_height": g_height,
        "g_density": g_density,
        "particle_spacing": particle_spacing,
        "fan_theta_deg": fan_theta_deg,
    }
    if schema == "sine":
        for name in ("steering_kp", "steering_kd", "speed_kp", "speed_kd"):
            if name in row and pd.notna(row[name]) and _isfinite(row[name]):
                out[name] = float(row[name])
    return out


def main():
    parser = argparse.ArgumentParser(description="Run top-N trials using a given sim script (Pull or Sine)")
    parser.add_argument("folder", help="BO output folder containing trials.csv")
    parser.add_argument("sim_script", help="Path to the Python sim script to run")
    parser.add_argument("--python", default=sys.executable, help="Python interpreter to use")
    parser.add_argument("--schema", choices=["auto", "pull", "sine"], default="auto")
    parser.add_argument("--top", type=int, default=10, help="Number of top trials to run")
    parser.add_argument("--metric-col", type=str, default=None, help="Metric column name (default: 'metric')")
    sort_group = parser.add_mutually_exclusive_group()
    sort_group.add_argument("--ascending", action="store_true", help="Sort ascending (default)")
    sort_group.add_argument("--descending", action="store_true", help="Sort descending")
    parser.add_argument("--visualize", action="store_true", help="Enable visualization")
    parser.add_argument("--snapshots", action="store_true", help="Enable snapshots (implies visualize)")
    parser.add_argument("--rank", type=int, default=None, help="Select a specific rank (1-based) instead of top-N")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without executing")
    parser.add_argument("--extra-args", nargs=argparse.REMAINDER, help="Additional args to pass to sim script; prefix with --")

    args = parser.parse_args()

    folder = Path(args.folder).expanduser().resolve()
    if not folder.exists():
        parser.error(f"Folder not found: {folder}")
    csv_path = folder / "trials.csv"
    if not csv_path.exists():
        parser.error(f"Missing trials.csv at {csv_path}")
    sim_script = Path(args.sim_script).expanduser().resolve()
    if not sim_script.exists():
        parser.error(f"Sim script not found: {sim_script}")

    df = pd.read_csv(csv_path)

    # Metric selection defaults
    metric_col = args.metric_col or ("metric" if "metric" in df.columns else None)
    if not metric_col:
        parser.error("Could not infer metric column. Specify --metric-col.")
    if metric_col not in df.columns:
        parser.error(f"Metric column '{metric_col}' not in CSV columns: {df.columns.tolist()}")
    df[metric_col] = pd.to_numeric(df[metric_col], errors="coerce")
    df = df[df[metric_col].notna()]
    df = df[np.isfinite(df[metric_col])]

    ascending = True if args.descending is False else False
    # Note: default is ascending; args.descending toggles it
    if args.ascending:
        ascending = True

    df_sorted = df.sort_values(by=metric_col, ascending=ascending)

    if args.schema == "auto":
        schema = detect_schema(df_sorted)
    else:
        schema = args.schema

    extra_args = args.extra_args or []
    if extra_args and len(extra_args) > 0 and extra_args[0] == "--":
        extra_args = extra_args[1:]

    # Selection: either a specific rank or top-N
    if args.rank is not None:
        if args.rank <= 0 or args.rank > len(df_sorted):
            parser.error(f"--rank {args.rank} out of range (1..{len(df_sorted)})")
        selected = [df_sorted.iloc[args.rank - 1].to_dict()]
        base_index = args.rank
    else:
        selected_df = df_sorted.head(max(1, int(args.top)))
        selected = list(selected_df.to_dict(orient="records"))
        base_index = 1

    # Summarize mode when snapshots are not requested: write/print stats only
    # Always use 'total_time_to_reach' as the time column
    time_col = "total_time_to_reach"
    power_col = "average_power" if "average_power" in df_sorted.columns else ("avg_power" if "avg_power" in df_sorted.columns else None)
    rms_error_col = "rms_error" if "rms_error" in df_sorted.columns else None

    if not args.snapshots:
        # If a specific rank is requested, just print its stats
        if args.rank is not None:
            row = selected[0]
            idx = args.rank
            sim_params = derive_sim_params(row, schema)
            rad_outer = row.get("rad_outer")
            w_by_r = row.get("w_by_r")
            pct_g = row.get("what_percent_is_grouser")
            g_density = row.get("g_density")
            fan_theta_deg = row.get("fan_theta_deg")
            particle_spacing = row.get("particle_spacing")
            m = row.get(metric_col)
            t = row.get(time_col)
            pwr = row.get(power_col) if power_col else None
            rms_err = row.get(rms_error_col) if rms_error_col else None
            print(f"rank={idx}")
            print(f"  trial_index: {row.get('trial_index')}")
            print(f"  metric ({metric_col}): {m}")
            print(f"  {time_col}: {t}")
            if power_col:
                print(f"  {power_col}: {pwr}")
            if rms_error_col:
                print(f"  {rms_error_col}: {rms_err}")
            print("  wheel (raw from CSV):")
            print(f"    rad_outer: {rad_outer}")
            print(f"    w_by_r: {w_by_r}")
            print(f"    what_percent_is_grouser: {pct_g}")
            print(f"    g_density: {g_density}")
            print(f"    fan_theta_deg: {fan_theta_deg}")
            print(f"    particle_spacing: {particle_spacing}")
            # Optional controllers if present in CSV
            for name in ("steering_kp", "steering_kd", "speed_kp", "speed_kd"):
                if name in row and pd.notna(row[name]):
                    print(f"    {name}: {row[name]}")
            print("  wheel (sim args):")
            print(f"    rad: {sim_params.get('rad')}")
            print(f"    width: {sim_params.get('width')}")
            print(f"    g_height: {sim_params.get('g_height')}")
            print(f"    g_density: {sim_params.get('g_density')}")
            print(f"    particle_spacing: {sim_params.get('particle_spacing')}")
            print(f"    fan_theta_deg: {sim_params.get('fan_theta_deg')}")
            for name in ("steering_kp", "steering_kd", "speed_kp", "speed_kd"):
                if name in sim_params:
                    print(f"    {name}: {sim_params[name]}")
            return

        # Otherwise, write top-N summary file in the folder
        out_path = folder / f"top_{len(selected)}_summary.txt"
        with open(out_path, "w") as f:
            f.write(f"schema: {schema}\n")
            f.write(f"metric_col: {metric_col}, ascending: {ascending}\n")
            f.write(f"source: {csv_path}\n\n")
            for i, row in enumerate(selected, start=base_index):
                sim_params = derive_sim_params(row, schema)
                rad_outer = row.get("rad_outer")
                w_by_r = row.get("w_by_r")
                pct_g = row.get("what_percent_is_grouser")
                g_density = row.get("g_density")
                fan_theta_deg = row.get("fan_theta_deg")
                particle_spacing = row.get("particle_spacing")
                m = row.get(metric_col)
                t = row.get(time_col)
                pwr = row.get(power_col) if power_col else None
                rms_err = row.get(rms_error_col) if rms_error_col else None
                f.write(f"rank {i}:\n")
                f.write(f"  trial_index: {row.get('trial_index')}\n")
                f.write(f"  metric ({metric_col}): {m}\n")
                f.write(f"  {time_col}: {t}\n")
                if power_col:
                    f.write(f"  {power_col}: {pwr}\n")
                if rms_error_col:
                    f.write(f"  {rms_error_col}: {rms_err}\n")
                f.write("  wheel (raw from CSV):\n")
                f.write(f"    rad_outer: {rad_outer}\n")
                f.write(f"    w_by_r: {w_by_r}\n")
                f.write(f"    what_percent_is_grouser: {pct_g}\n")
                f.write(f"    g_density: {g_density}\n")
                f.write(f"    fan_theta_deg: {fan_theta_deg}\n")
                f.write(f"    particle_spacing: {particle_spacing}\n")
                # Optional controllers if present
                for name in ("steering_kp", "steering_kd", "speed_kp", "speed_kd"):
                    if name in row and pd.notna(row[name]):
                        f.write(f"    {name}: {row[name]}\n")
                f.write("  wheel (sim args):\n")
                f.write(f"    rad: {sim_params.get('rad')}\n")
                f.write(f"    width: {sim_params.get('width')}\n")
                f.write(f"    g_height: {sim_params.get('g_height')}\n")
                f.write(f"    g_density: {sim_params.get('g_density')}\n")
                f.write(f"    particle_spacing: {sim_params.get('particle_spacing')}\n")
                f.write(f"    fan_theta_deg: {sim_params.get('fan_theta_deg')}\n")
                for name in ("steering_kp", "steering_kd", "speed_kp", "speed_kd"):
                    if name in sim_params:
                        f.write(f"    {name}: {sim_params[name]}\n")
                f.write("\n")
        print(f"Wrote summary: {out_path}")
        return

    # Snapshots requested: run simulations (top-N or a specific rank)
    N = len(selected)
    for offset, row in enumerate(selected):
        idx = (base_index + offset)
        rank_label = f"rank_{idx:02d}"
        trial_idx = row.get("trial_index")
        if trial_idx is None or (isinstance(trial_idx, float) and not math.isfinite(trial_idx)):
            trial_idx = "?"
        subdir_name = f"{rank_label}"
        if trial_idx != "?":
            subdir_name += f"_trial_{_sanitize_name(trial_idx)}"
        run_output_dir = folder / subdir_name
        run_output_dir.mkdir(parents=True, exist_ok=True)

        cmd = build_args_from_row(
            row,
            schema=schema,
            sim_script=sim_script,
            python=args.python,
            visualize=True,  # snapshots implies visualize
            snapshots=True,
            output_dir=run_output_dir,
            extra_args=extra_args,
        )

        print(f"[{offset+1}/{N}] {_format_cmd(cmd)}")
        if args.dry_run:
            continue
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as exc:
            print(f"Run {idx} failed with exit code {exc.returncode}")


if __name__ == "__main__":
    main()
