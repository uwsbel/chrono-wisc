import argparse
import csv
import math
import os
from typing import List, Tuple


def infer_time_scale(
    rows: List[dict],
    control_delay: float,
    ideal_e_norm: float,
    orig_w_speed: float,
    orig_w_rms: float,
) -> Tuple[float, float]:
    """Infer the time-score scale factor c_t from CSV.

    We use the known structure:
        e_score = (rms_error / ideal_e_norm) * 10
        time_score = c_t * (total_time_to_reach - control_delay)
        metric ≈ orig_w_speed * time_score + orig_w_rms * e_score

    Solve for c_t in least-squares sense over all non-failed rows.
    Returns (c_t, rms_residual).
    """
    xs: List[float] = []
    ys: List[float] = []

    for row in rows:
        try:
            metric = float(row["metric"])
            total_time = float(row["total_time_to_reach"])
            rms_error = float(row["rms_error"])
        except (KeyError, ValueError):
            continue

        # Skip obvious failure/sentinel metrics (e.g., 500)
        if not math.isfinite(metric) or metric >= 499.0:
            continue

        t_elapsed = total_time - control_delay
        if t_elapsed <= 0.0:
            continue

        e_score = (rms_error / ideal_e_norm) * 10.0
        # metric = orig_w_speed * c_t * t_elapsed + orig_w_rms * e_score
        # => metric - orig_w_rms * e_score = orig_w_speed * c_t * t_elapsed
        y = metric - orig_w_rms * e_score
        x = t_elapsed
        xs.append(x)
        ys.append(y)

    if not xs:
        raise RuntimeError("No valid rows found to infer time scale.")

    # Least-squares fit for y ≈ orig_w_speed * c_t * x
    num = 0.0
    den = 0.0
    for x, y in zip(xs, ys):
        num += x * y
        den += x * x
    if den == 0.0:
        raise RuntimeError("Cannot infer time scale: zero denominator.")

    c_t = num / (orig_w_speed * den)

    # Compute RMS residual to quantify fit quality
    sq_err_sum = 0.0
    n = 0
    for x, y in zip(xs, ys):
        y_hat = orig_w_speed * c_t * x
        sq_err_sum += (y - y_hat) ** 2
        n += 1
    rms_residual = math.sqrt(sq_err_sum / max(1, n))

    return c_t, rms_residual


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Recompute sine side-slip metrics from trials.csv using the same "
            "normalized-score structure as SineTestSideSlip_sim.py."
        )
    )
    parser.add_argument(
        "--csv",
        type=str,
        required=True,
        help="Input trials CSV (e.g., SineSideSlip_noBeta_global_1700_0_0.6_25_3_5/trials.csv)",
    )
    parser.add_argument(
        "--out_csv",
        type=str,
        default=None,
        help="Output CSV path (default: trials_with_metrics.csv in same directory)",
    )
    parser.add_argument(
        "--terrain_length",
        type=float,
        default=5.0,
        help="Terrain length (used to pick ideal normalization constants, e.g., 5.0 or 10.0)",
    )
    parser.add_argument(
        "--control_delay",
        type=float,
        default=0.5,
        help="Control delay used before scoring (vehicle_init_time in the sim).",
    )
    # Original weights for verification (defaults = values you used for this dataset)
    parser.add_argument(
        "--orig_w_speed",
        type=float,
        default=0.6,
        help="Original weight for speed/time used to create the existing metric.",
    )
    parser.add_argument(
        "--orig_w_rms",
        type=float,
        default=0.4,
        help="Original weight for tracking/RMS error used to create the existing metric.",
    )
    # New weights for the additional metric column
    parser.add_argument(
        "--w_speed",
        type=float,
        required=True,
        help="Weight for time/speed component in the new metric.",
    )
    parser.add_argument(
        "--w_rms",
        type=float,
        required=True,
        help="Weight for RMS tracking error component in the new metric.",
    )
    parser.add_argument(
        "--w_power",
        type=float,
        required=True,
        help="Weight for power component in the new metric.",
    )
    parser.add_argument(
        "--w_beta",
        type=float,
        required=True,
        help="Weight for side-slip (beta) component in the new metric.",
    )

    args = parser.parse_args()

    # Pick normalization constants exactly as in SineTestSideSlip_sim.py
    if abs(args.terrain_length - 10.0) < 1e-6:
        ideal_e_norm = 0.05
        ideal_power = 200.0
        ideal_beta_rms = 0.05
    elif abs(args.terrain_length - 5.0) < 1e-6:
        ideal_e_norm = 0.03
        ideal_power = 100.0
        ideal_beta_rms = 0.05
    else:
        raise ValueError(f"Unsupported terrain_length={args.terrain_length} for normalization.")

    csv_path = args.csv
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    # Read all rows
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    required_cols = {
        "metric",
        "total_time_to_reach",
        "rms_error",
        "average_power",
        "beta_rms",
    }
    missing = required_cols - set(fieldnames)
    if missing:
        raise RuntimeError(f"Input CSV missing required columns: {sorted(missing)}")

    # Infer time-score scale factor c_t from the existing metric and known original weights
    c_t, rms_residual = infer_time_scale(
        rows,
        control_delay=args.control_delay,
        ideal_e_norm=ideal_e_norm,
        orig_w_speed=args.orig_w_speed,
        orig_w_rms=args.orig_w_rms,
    )

    ideal_time = 10.0 / c_t
    print(f"Inferred c_t (time score scale) = {c_t:.6f}")
    print(f"Inferred ideal_time = {ideal_time:.6f} s")
    print(f"RMS residual for original metric fit = {rms_residual:.6e}")

    # Verify reconstruction of existing metric with original weights
    max_abs_err = 0.0
    max_rel_err = 0.0
    n_checked = 0

    for row in rows:
        try:
            metric_orig = float(row["metric"])
            total_time = float(row["total_time_to_reach"])
            rms_error = float(row["rms_error"])
            average_power = float(row["average_power"])
            beta_rms = float(row["beta_rms"])
        except (KeyError, ValueError):
            continue

        if not math.isfinite(metric_orig) or metric_orig >= 499.0:
            continue

        t_elapsed = total_time - args.control_delay
        if t_elapsed <= 0.0:
            continue

        time_score = c_t * t_elapsed
        e_score = (rms_error / ideal_e_norm) * 10.0
        power_score = (average_power / ideal_power) * 10.0
        beta_score = (beta_rms / ideal_beta_rms) * 10.0

        metric_recon = (
            args.orig_w_speed * time_score
            + args.orig_w_rms * e_score
            # In the original run, power and beta weights were 0
        )

        abs_err = abs(metric_recon - metric_orig)
        rel_err = abs_err / max(1e-8, abs(metric_orig))
        max_abs_err = max(max_abs_err, abs_err)
        max_rel_err = max(max_rel_err, rel_err)
        n_checked += 1

    print(f"Verified {n_checked} rows against existing metric.")
    print(f"Max abs error: {max_abs_err:.6e}")
    print(f"Max rel error: {max_rel_err:.6e}")

    # Prepare new metric column name and compute new metric values
    col_name = f"metric_{args.w_speed}_{args.w_beta}_{args.w_power}_{args.w_rms}"
    if col_name in fieldnames:
        raise RuntimeError(f"Output column {col_name!r} already exists in CSV.")

    new_fieldnames = list(fieldnames) + [col_name]

    new_rows: List[dict] = []
    for row in rows:
        try:
            total_time = float(row["total_time_to_reach"])
            rms_error = float(row["rms_error"])
            average_power = float(row["average_power"])
            beta_rms = float(row["beta_rms"])
        except (KeyError, ValueError):
            # Keep row but leave metric blank
            row[col_name] = ""
            new_rows.append(row)
            continue

        t_elapsed = total_time - args.control_delay
        time_score = c_t * t_elapsed
        e_score = (rms_error / ideal_e_norm) * 10.0
        power_score = (average_power / ideal_power) * 10.0
        beta_score = (beta_rms / ideal_beta_rms) * 10.0

        metric_new = (
            args.w_speed * time_score
            + args.w_rms * e_score
            + args.w_power * power_score
            + args.w_beta * beta_score
        )
        row[col_name] = f"{metric_new:.4f}"
        new_rows.append(row)

    # Decide output path
    if args.out_csv is not None:
        out_path = args.out_csv
    else:
        base_dir = os.path.dirname(csv_path)
        out_path = os.path.join(base_dir, "trials_with_metrics.csv")

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=new_fieldnames)
        writer.writeheader()
        for row in new_rows:
            writer.writerow(row)

    print(f"Wrote output with new metric column {col_name!r} to: {out_path}")


if __name__ == "__main__":
    main()

