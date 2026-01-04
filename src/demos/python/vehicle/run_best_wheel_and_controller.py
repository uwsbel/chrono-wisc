#!/usr/bin/env python3

import argparse
import math
import os
import re
import sys
import shutil
from contextlib import redirect_stdout
from datetime import datetime
from typing import Dict, Optional, Tuple

import pandas as pd

from SineSideSlip_BO_stControllerOnly import derive_wheel_geometry, load_best_wheel

_SIM_MOD = None


def _import_sim_module():
    global _SIM_MOD
    if _SIM_MOD is None:
        import SineTestSideSlip_sim as sim_mod

        _SIM_MOD = sim_mod
    return _SIM_MOD


def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


class _TeeStdout:
    """Capture stdout while still echoing to the real stdout."""

    def __init__(self, real_stdout):
        self._real = real_stdout
        self._buf = []

    def write(self, data):
        self._real.write(data)
        self._buf.append(data)

    def flush(self):
        self._real.flush()

    def getvalue(self):
        return "".join(self._buf)


def _resolve_trials_path(path_str: str) -> str:
    """Allow a folder (expects trials.csv inside) or a direct CSV path."""
    path = os.path.abspath(path_str)
    if os.path.isdir(path):
        csv_path = os.path.join(path, "trials.csv")
    else:
        csv_path = path
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Trials CSV not found at {csv_path}")
    return csv_path


def _load_best_row(csv_path: str, metric_col: str = "metric", rank: int = 1) -> Tuple[Dict, int]:
    """Load the rank-th row (1-based) after sorting ascending by metric_col."""
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Controller trials CSV not found: {csv_path}")
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
    df_sorted = df.sort_values(by=metric_col, ascending=True)
    return df_sorted.iloc[rank - 1].to_dict(), rank


def _count_samples(csv_path: str, metric_col: str = "metric") -> int:
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Trials CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)
    if metric_col not in df.columns:
        raise ValueError(f"Metric column '{metric_col}' not found in {csv_path}; columns={df.columns.tolist()}")
    df[metric_col] = pd.to_numeric(df[metric_col], errors="coerce")
    df = df[df[metric_col].notna()]
    df = df[df[metric_col].map(_isfinite)]
    return int(len(df))


def _mode_snapshot_base(args: argparse.Namespace, wheel_csv: Optional[str]) -> Optional[str]:
    if not (args.snapshots or args.save_blender or args.save_particles):
        return None

    if args.snapshot_dir:
        base_dir = os.path.abspath(args.snapshot_dir)
    elif getattr(args, "joint_csv", None):
        base_dir = os.path.dirname(os.path.abspath(args.joint_csv))
    elif getattr(args, "controller_csv", None):
        base_dir = os.path.dirname(os.path.abspath(args.controller_csv))
    else:
        base_dir = os.path.dirname(os.path.abspath(wheel_csv)) if wheel_csv else os.getcwd()

    snapshot_base = os.path.join(base_dir, f"best_runs_snapshots_{datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')}")
    os.makedirs(snapshot_base, exist_ok=True)
    return snapshot_base


def _prepare_output_dirs(base_dir: Optional[str], label: str, args: argparse.Namespace):
    if base_dir is None:
        return None, None, None, None
    run_dir = os.path.join(base_dir, label)
    if os.path.isdir(run_dir):
        shutil.rmtree(run_dir)
    elif os.path.exists(run_dir):
        os.remove(run_dir)
    os.makedirs(run_dir, exist_ok=True)

    snapshot_dir = os.path.join(run_dir, "snapshots") if args.snapshots else None
    blender_dir = os.path.join(run_dir, "blender") if args.save_blender else None
    particle_dir = os.path.join(run_dir, "particle_files") if args.save_particles else None
    particle_sph_dir = None
    particle_fsi_dir = None

    if snapshot_dir is not None:
        os.makedirs(snapshot_dir, exist_ok=True)
    if blender_dir is not None:
        os.makedirs(blender_dir, exist_ok=True)
    if particle_dir is not None:
        particle_sph_dir = os.path.join(particle_dir, "particles")
        particle_fsi_dir = os.path.join(particle_dir, "fsi")
        os.makedirs(particle_sph_dir, exist_ok=True)
        os.makedirs(particle_fsi_dir, exist_ok=True)

    return snapshot_dir, particle_sph_dir, particle_fsi_dir, blender_dir


def _prepare_params(
    wheel_geom: Dict,
    controller_row: Optional[Dict],
    args: argparse.Namespace,
    include_controller: bool,
) -> Tuple[object, object]:
    sim_mod = _import_sim_module()
    p = sim_mod.Params()
    p.particle_spacing = wheel_geom["particle_spacing"]
    p.rad = wheel_geom["rad_inner"]
    p.width = wheel_geom["width"]
    p.g_height = wheel_geom["g_height"]
    p.g_density = wheel_geom["g_density"]
    p.fan_theta_deg = wheel_geom["fan_theta_deg"]

    if include_controller and controller_row is not None:
        missing = [k for k in ("steering_kp", "steering_kd") if k not in controller_row or not _isfinite(controller_row[k])]
        if missing:
            raise ValueError(f"Controller column(s) {missing} missing or non-finite in {args.controller_csv}")
        p.steering_kp = float(controller_row["steering_kp"])
        p.steering_kd = float(controller_row["steering_kd"])
        ki_val = controller_row.get("steering_ki", 0.0)
        p.steering_ki = float(ki_val) if _isfinite(ki_val) else 0.0

    sp = sim_mod.SimParams()
    sp.density = float(args.density)
    sp.cohesion = float(args.cohesion)
    sp.friction = float(args.friction)
    sp.max_force = float(args.max_force)
    sp.target_speed = float(args.target_speed)
    try:
        sp.terrain_length = float(args.terrain_length)
    except Exception:
        pass

    return p, sp


def _run_sim(label: str, wheel_geom: Dict, controller_row: Optional[Dict], args: argparse.Namespace,
             snapshot_dir: Optional[str], particle_sph_dir: Optional[str], particle_fsi_dir: Optional[str],
             blender_dir: Optional[str]):
    p, sp = _prepare_params(wheel_geom, controller_row, args, include_controller=controller_row is not None)
    sim_mod = _import_sim_module()

    kwargs = {
        "weight_speed": float(args.weight_speed),
        "weight_power": float(args.weight_power),
        "weight_beta": float(args.weight_beta),
        "visualize": bool(args.visualize or args.snapshots),
        "snapshot_dir": snapshot_dir,
        "particle_sph_dir": particle_sph_dir,
        "particle_fsi_dir": particle_fsi_dir,
        "blender_dir": blender_dir,
    }
    if args.sine_amplitude is not None:
        kwargs["sine_amplitude"] = float(args.sine_amplitude)
    if args.num_periods is not None:
        kwargs["num_periods"] = int(args.num_periods)

    tee = _TeeStdout(sys.stdout)
    with redirect_stdout(tee):
        metric, total_time, rms_error, avg_power, beta_rms, sim_failed = sim_mod.sim(p, sp, **kwargs)
    log_text = tee.getvalue()

    avg_speed = None
    m = re.search(r"average_speed:\\s*([0-9.eE+\\-]+)", log_text)
    if m:
        try:
            avg_speed = float(m.group(1))
        except Exception:
            avg_speed = None

    return {
        "label": label,
        "metric": metric,
        "total_time_to_reach": total_time,
        "rms_error": rms_error,
        "average_power": avg_power,
        "beta_rms": beta_rms,
        "average_speed": avg_speed,
        "sim_failed": sim_failed,
        "snapshot_dir": snapshot_dir,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Run the sine side-slip test using the best design from separate or joint Bayesian optimization runs."
    )
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument("--default", action="store_true", help="Run best wheel with default controller gains")
    mode_group.add_argument(
        "--separate", action="store_true", help="Run best wheel (wheel BO) with best controller gains (controller BO)"
    )
    mode_group.add_argument(
        "--joint-csv",
        type=str,
        default=None,
        help="Path to joint wheel+controller trials CSV (e.g., trials_wControl.csv); runs joint best only",
    )

    parser.add_argument("wheel_path", nargs="?", help="Folder containing trials.csv or path to wheel trials.csv")
    parser.add_argument("--controller-csv", default=None, help="Path to steering-controller trials CSV (separate mode)")
    parser.add_argument("--wheel-metric-col", default="metric", help="Metric column for wheel ranking (minimizes)")
    parser.add_argument("--controller-metric-col", default="metric", help="Metric column for controller ranking (minimizes)")
    parser.add_argument("--wheel-rank", type=int, default=1, help="Rank of wheel to select (1=best)")
    parser.add_argument("--controller-rank", type=int, default=1, help="Rank of controller to select (1=best)")
    parser.add_argument("--joint-rank", type=int, default=1, help="Rank of joint row to select (1=best)")
    parser.add_argument("--snapshots", action="store_true", help="Enable snapshots for the run")
    parser.add_argument("--visualize", action="store_true", help="Enable visualization (implied by --snapshots)")
    parser.add_argument("--save-blender", action="store_true", help="Enable Blender output")
    parser.add_argument("--save-particles", action="store_true", help="Enable particle output")
    parser.add_argument("--snapshot-dir", type=str, default=None, help="Base directory for snapshots (optional)")
    # Environment / material
    parser.add_argument("--density", type=float, default=1700, help="Material density")
    parser.add_argument("--cohesion", type=float, default=0, help="Material cohesion")
    parser.add_argument("--friction", type=float, default=0.6, help="Material friction coefficient")
    parser.add_argument("--max_force", type=float, default=25, help="Maximum pulling force")
    parser.add_argument("--target_speed", type=float, default=3.0, help="Target vehicle speed")
    parser.add_argument("--terrain_length", type=float, default=5.0, help="Terrain length in meters (5.0 or 10.0)")
    # Weights and sine shape
    parser.add_argument("--weight_speed", type=float, default=0.4, help="Weight for speed metric component")
    parser.add_argument("--weight_power", type=float, default=0.4, help="Weight for power metric component")
    parser.add_argument("--weight_beta", type=float, default=0.2, help="Weight for side-slip (beta) metric component")
    parser.add_argument("--sine_amplitude", type=float, default=None, help="Override sine amplitude in meters (optional)")
    parser.add_argument("--num_periods", type=int, default=None, help="Override number of sine periods (optional)")
    args = parser.parse_args()

    wheel_csv = None
    wheel_geom = None
    wheel_row = None
    controller_row = None
    controller_rank = None

    if args.joint_csv:
        joint_csv = os.path.abspath(args.joint_csv)
        joint_samples = _count_samples(joint_csv, metric_col=args.controller_metric_col)
        joint_row, joint_rank = _load_best_row(
            joint_csv,
            metric_col=args.controller_metric_col,
            rank=max(1, int(args.joint_rank)),
        )
        wheel_geom = derive_wheel_geometry(joint_row)
        controller_row = joint_row

        print(f"Selected joint wheel+controller (rank {joint_rank}) from {joint_csv}:")
        for k in [
            "rad_outer",
            "w_by_r",
            "what_percent_is_grouser",
            "g_density",
            "fan_theta_deg",
            "particle_spacing",
            args.controller_metric_col,
        ]:
            if k in joint_row:
                print(f"  {k}: {joint_row[k]}")
        print(f"Derived wheel geometry: {wheel_geom}")

        print("Selected joint controller gains:")
        for k in ["steering_kp", "steering_ki", "steering_kd", args.controller_metric_col]:
            if k in joint_row:
                print(f"  {k}: {joint_row[k]}")

        print(f"Samples: joint={joint_samples} (from {joint_csv})")

        snapshot_base = _mode_snapshot_base(args, wheel_csv=None)
        joint_snap_dir, joint_sph_dir, joint_fsi_dir, joint_blender_dir = _prepare_output_dirs(
            snapshot_base, "joint", args
        )
        print("\nRunning joint-optimized wheel+controller...")
        results = [
            _run_sim(
                "joint",
                wheel_geom,
                controller_row=controller_row,
                args=args,
                snapshot_dir=joint_snap_dir,
                particle_sph_dir=joint_sph_dir,
                particle_fsi_dir=joint_fsi_dir,
                blender_dir=joint_blender_dir,
            )
        ]

    else:
        if not args.wheel_path:
            parser.error("wheel_path is required unless --joint-csv is provided")
        wheel_csv = _resolve_trials_path(args.wheel_path)
        wheel_samples = _count_samples(wheel_csv, metric_col=args.wheel_metric_col)
        wheel_row, wheel_rank = load_best_wheel(
            wheel_csv, metric_col=args.wheel_metric_col, ascending=True, rank=max(1, int(args.wheel_rank))
        )
        wheel_geom = derive_wheel_geometry(wheel_row)

        print(f"Selected wheel (rank {wheel_rank}) from {wheel_csv}:")
        for k in ["rad_outer", "w_by_r", "what_percent_is_grouser", "g_density", "fan_theta_deg", "particle_spacing"]:
            if k in wheel_row:
                print(f"  {k}: {wheel_row[k]}")
        print(f"Derived wheel geometry: {wheel_geom}")

        snapshot_base = _mode_snapshot_base(args, wheel_csv=wheel_csv)

        if args.default:
            p_default, _sp_default = _prepare_params(wheel_geom, controller_row=None, args=args, include_controller=False)
            print("Default controller gains (from SineTestSideSlip_sim.Params):")
            print(f"  steering_kp: {getattr(p_default, 'steering_kp', None)}")
            print(f"  steering_ki: {getattr(p_default, 'steering_ki', None)}")
            print(f"  steering_kd: {getattr(p_default, 'steering_kd', None)}")
            print(f"Samples: wheel={wheel_samples} (from {wheel_csv})")

            default_snap_dir, default_sph_dir, default_fsi_dir, default_blender_dir = _prepare_output_dirs(
                snapshot_base, "default_controller", args
            )
            print("\nRunning with default controller gains...")
            results = [
                _run_sim(
                    "default_controller",
                    wheel_geom,
                    controller_row=None,
                    args=args,
                    snapshot_dir=default_snap_dir,
                    particle_sph_dir=default_sph_dir,
                    particle_fsi_dir=default_fsi_dir,
                    blender_dir=default_blender_dir,
                )
            ]
        elif args.separate:
            if not args.controller_csv:
                parser.error("--controller-csv is required for --separate")
            controller_csv = os.path.abspath(args.controller_csv)
            controller_samples = _count_samples(controller_csv, metric_col=args.controller_metric_col)
            controller_row, controller_rank = _load_best_row(
                controller_csv,
                metric_col=args.controller_metric_col,
                rank=max(1, int(args.controller_rank)),
            )

            print(f"Selected controller (rank {controller_rank}) from {args.controller_csv}:")
            for k in ["steering_kp", "steering_ki", "steering_kd", args.controller_metric_col]:
                if k in controller_row:
                    print(f"  {k}: {controller_row[k]}")

            print(
                f"Samples: wheel={wheel_samples} (from {wheel_csv}), controller={controller_samples} (from {controller_csv}), "
                f"total={wheel_samples + controller_samples}"
            )

            optimized_snap_dir, opt_sph_dir, opt_fsi_dir, opt_blender_dir = _prepare_output_dirs(
                snapshot_base, "optimized_controller", args
            )
            print("\nRunning with optimized controller gains...")
            results = [
                _run_sim(
                    "optimized_controller",
                    wheel_geom,
                    controller_row=controller_row,
                    args=args,
                    snapshot_dir=optimized_snap_dir,
                    particle_sph_dir=opt_sph_dir,
                    particle_fsi_dir=opt_fsi_dir,
                    blender_dir=opt_blender_dir,
                )
            ]
        else:
            parser.error("Must specify exactly one of --default, --separate, or --joint-csv")

    print("\nResults:")
    for res in results:
        label = res["label"]
        print(f"  [{label}] sim_failed={res['sim_failed']}")
        print(f"    metric: {res['metric']:.4f}")
        print(f"    rms_error: {res['rms_error']:.4f} m")
        print(f"    average_power: {res['average_power']:.2f} W")
        print(f"    total_time_to_reach: {res['total_time_to_reach']:.4f} s")
        if res["average_speed"] is not None:
            print(f"    average_speed: {res['average_speed']:.4f} m/s")
        print(f"    beta_rms: {res['beta_rms']:.4f} rad")
        if res["snapshot_dir"]:
            print(f"    snapshots: {res['snapshot_dir']}")


if __name__ == "__main__":
    main()
