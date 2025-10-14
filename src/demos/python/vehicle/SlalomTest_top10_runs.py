#!/usr/bin/env python3

import argparse
import csv
import math
import shlex
import subprocess
import sys
from pathlib import Path

DEFAULT_PARAMS = {
    "rad": "12",
    "width": "8",
    "g_height": "4",
    "g_width": "4",
    "g_density": "6",
    "particle_spacing": "0.01",
    "grouser_type": "1",
    "fan_theta_deg": "108",
    "cp_deviation": "0",
    "steering_kp": "11.538847",
    "steering_kd": "1.795085",
    "speed_kp": "1.557757",
    "speed_kd": "0.02518",
}

PARAM_KEYS = [
    "rad",
    "width",
    "g_height",
    "g_width",
    "g_density",
    "particle_spacing",
    "grouser_type",
    "fan_theta_deg",
    "cp_deviation",
    "steering_kp",
    "steering_kd",
    "speed_kp",
    "speed_kd",
]

INT_FIELDS = {"g_density"}
NUMERIC_FIELDS = {
    "rad",
    "width",
    "g_height",
    "g_width",
    "particle_spacing",
    "fan_theta_deg",
    "cp_deviation",
    "steering_kp",
    "steering_kd",
    "speed_kp",
    "speed_kd",
}


def _resolve_script(path_str: str, base_dir: Path) -> Path:
    path = Path(path_str)
    return path if path.is_absolute() else (base_dir / path)


def _clean_value(field: str, row: dict) -> str:
    raw = row.get(field)
    if raw is None:
        raw = DEFAULT_PARAMS.get(field)
    text = "" if raw is None else str(raw).strip()
    if text == "" or text.lower() == "nan":
        text = DEFAULT_PARAMS.get(field, "")
    if field == "grouser_type":
        lowered = text.lower()
        if lowered in {"straight", "semi", "semi_circle", "semicircle"}:
            return lowered
        try:
            num = float(text)
        except ValueError:
            return text
        if math.isfinite(num) and abs(num - round(num)) < 1e-9:
            return str(int(round(num)))
        return str(num)
    if field in INT_FIELDS:
        try:
            num = float(text)
        except ValueError:
            return text
        if math.isfinite(num):
            return str(int(round(num)))
        return text
    if field in NUMERIC_FIELDS:
        try:
            num = float(text)
        except ValueError:
            return text
        if not math.isfinite(num):
            return text
        if abs(num - round(num)) < 1e-9:
            return str(int(round(num)))
        return f"{num:.10g}"
    return text


def _format_cmd(parts):
    return " ".join(shlex.quote(str(part)) for part in parts)


def _sanitize_name(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in value)


def main():
    base_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(
        description="Run slalom simulations for the top 10 wheel configurations."
    )
    parser.add_argument(
        "folder",
        help="Optimization output folder containing trials.csv.",
    )
    parser.add_argument(
        "--python",
        default=sys.executable,
        help="Python interpreter to use for subprocess calls.",
    )
    parser.add_argument(
        "--top10-script",
        default="slalom_top_10_wheels.py",
        help="Path to slalom_top_10_wheels.py (relative to this script by default).",
    )
    parser.add_argument(
        "--sim-script",
        default="SlalomTest_sim_vis.py",
        help="Path to SlalomTest_sim_vis.py (relative to this script by default).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without executing simulations.",
    )
    parser.add_argument(
        "--sim-args",
        nargs=argparse.REMAINDER,
        help="Additional arguments forwarded to SlalomTest_sim_vis.py (prefix with --).",
    )
    parser.add_argument(
        "--snapshots",
        action="store_true",
        help="Enable snapshot saving for each simulation run.",
    )
    parser.add_argument(
        "--video-root",
        type=str,
        help="Optional directory where per-run snapshot subfolders are created. Defaults to '<folder>/slalom_top10_videos'.",
    )

    args = parser.parse_args()

    target_folder = Path(args.folder).expanduser().resolve()
    if not target_folder.exists():
        parser.error(f"Folder '{target_folder}' does not exist.")

    top10_script = _resolve_script(args.top10_script, base_dir).resolve()
    sim_script = _resolve_script(args.sim_script, base_dir).resolve()

    if not top10_script.exists():
        parser.error(f"Cannot find slalom_top_10_wheels script at '{top10_script}'.")
    if not sim_script.exists():
        parser.error(f"Cannot find SlalomTest_sim_vis script at '{sim_script}'.")

    top10_cmd = [args.python, str(top10_script), str(target_folder)]
    print(f"Running top-10 selection: {_format_cmd(top10_cmd)}")
    subprocess.run(top10_cmd, check=True)

    csv_path = target_folder / "top_10_best_metrics.csv"
    if not csv_path.exists():
        raise FileNotFoundError(
            f"Expected '{csv_path}' after running slalom_top_10_wheels.py."
        )

    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = [row for row in reader]

    if not rows:
        raise RuntimeError("No rows found in top_10_best_metrics.csv.")

    extra_args = args.sim_args or []
    if extra_args and extra_args[0] == "--":
        extra_args = extra_args[1:]

    if args.video_root:
        base_output_dir = Path(args.video_root).expanduser().resolve()
    else:
        base_output_dir = target_folder / "slalom_top10_videos"

    snapshot_dir_already_set = any(
        arg == "--snapshots-dir" or arg.startswith("--snapshots-dir=") for arg in extra_args
    )

    if args.snapshots and not snapshot_dir_already_set:
        base_output_dir.mkdir(parents=True, exist_ok=True)

    for idx, row in enumerate(rows[1:10], start=1):
        params = {key: _clean_value(key, row) for key in PARAM_KEYS}
        sim_cmd = [args.python, str(sim_script)]
        for key in PARAM_KEYS:
            value = params.get(key)
            if value is None or value == "":
                continue
            sim_cmd.extend([f"--{key}", value])
        trial = row.get("trial_index", "?")
        metric = row.get("composite_metric", "?")

        run_dir = None
        if args.snapshots and not snapshot_dir_already_set:
            subdir_name = f"rank_{idx:02d}"
            if trial not in (None, "", "?"):
                subdir_name += f"_trial_{_sanitize_name(str(trial))}"
            run_dir = base_output_dir / subdir_name
            run_dir.mkdir(parents=True, exist_ok=True)
            sim_cmd.extend(["--snapshots-dir", str(run_dir)])

        sim_cmd.extend(extra_args)

        print(f"\n[{idx}/10] Trial {trial} (metric={metric}): {_format_cmd(sim_cmd)}")
        if run_dir is not None:
            print(f"  Saving snapshots to: {run_dir}")
        if args.dry_run:
            continue
        try:
            subprocess.run(sim_cmd, check=True)
        except subprocess.CalledProcessError as exc:
            print(
                f"Run {idx} failed with exit code {exc.returncode}. Continuing to next run."
            )


if __name__ == "__main__":
    main()
