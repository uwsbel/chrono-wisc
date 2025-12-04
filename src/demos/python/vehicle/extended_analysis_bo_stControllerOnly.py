import argparse
import os
from typing import List, Tuple, Dict

# Prefer non-interactive backend for headless runs
try:
    import matplotlib
    matplotlib.use("Agg")
except Exception:
    pass

import numpy as np
import pandas as pd

# Reuse plotting/analysis utilities from the general extended_analysis script
from extended_analysis_bo import (
    summarize_coverage,
    plot_metrics_boxplot,
    corner_plot,
    clustering_analysis,
)


# ----- Controller-only helpers -----

BASE_FEATURES = ["steering_kp", "steering_ki", "steering_kd"]


def detect_features(df: pd.DataFrame) -> List[str]:
    """Return varying steering-gain columns present in the CSV."""
    feats: List[str] = []
    for c in BASE_FEATURES:
        if c in df.columns and df[c].notna().any():
            feats.append(c)
    return feats


def load_filtered_controller(
    csv_path: str,
    target_col: str = "metric",
    filter_penalties: bool = True,
    penalty_time: float = 12.0,
    penalty_metric: float = 500.0,
) -> Tuple[pd.DataFrame, str, List[str]]:
    """Load steering-controller-only trials and perform light cleaning."""
    df = pd.read_csv(csv_path)

    required = ["total_time_to_reach"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise SystemExit(f"CSV missing required columns: {missing}")

    if target_col not in df.columns:
        raise SystemExit(
            f"Target column '{target_col}' not found. Available: "
            f"{[c for c in ['metric','beta_rms','rms_error','average_power'] if c in df.columns]}"
        )

    df[target_col] = pd.to_numeric(df[target_col], errors="coerce")
    df["total_time_to_reach"] = pd.to_numeric(df["total_time_to_reach"], errors="coerce")

    feats = detect_features(df)
    if not feats:
        raise SystemExit("No steering-gain columns detected (expected steering_kp, steering_ki, steering_kd).")

    for c in feats:
        df[c] = pd.to_numeric(df[c], errors="coerce").astype(float)

    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.dropna(subset=feats + [target_col, "total_time_to_reach"])

    if filter_penalties and target_col == "metric":
        df = df[(df[target_col] < penalty_metric) & (df["total_time_to_reach"] != penalty_time)]

    if df.empty:
        raise SystemExit("No usable rows after filtering/cleaning.")

    return df, target_col, feats


def ensure_outdir(csv_path: str, outdir: str | None) -> str:
    if outdir:
        os.makedirs(outdir, exist_ok=True)
        return outdir
    base = os.path.dirname(os.path.abspath(csv_path))
    out = os.path.join(base, "analysis_controller", "extended")
    os.makedirs(out, exist_ok=True)
    return out


def _controller_bounds(features: List[str]) -> Dict[str, Tuple[float, float, str]]:
    bounds: Dict[str, Tuple[float, float, str]] = {}
    if "steering_kp" in features:
        bounds["steering_kp"] = (1.0, 15.0, "float")
    if "steering_ki" in features:
        bounds["steering_ki"] = (0.0, 4.0, "float")
    if "steering_kd" in features:
        bounds["steering_kd"] = (0.0, 4.0, "float")
    return bounds


# ----- Main CLI -----


def main():
    parser = argparse.ArgumentParser(
        description="Extended BO analysis for steering-controller-only runs (kp/ki/kd) after wheel optimization."
    )
    parser.add_argument("--csv", required=True, help="Path to trials_stControllerOnly.csv")
    parser.add_argument(
        "--outdir",
        default=None,
        help="Output directory (default: <csvdir>/analysis_controller/extended)",
    )
    parser.add_argument(
        "--target",
        default="metric",
        help="Target column to use for coloring/clustering (e.g., metric, beta_rms, rms_error, average_power)",
    )
    parser.add_argument(
        "--no-filter-penalties",
        action="store_true",
        help="Keep penalty trials (metric>=500 or total_time_to_reach==12.0)",
    )
    parser.add_argument("--restrict", type=int, default=None, help="Restrict to first N rows after loading")
    parser.add_argument(
        "--lower-is-better",
        action="store_true",
        help="Metric direction: lower is better (default: False -> higher is better)",
    )
    parser.add_argument("--bins-1d", type=int, default=20, help="Bins for 1D histograms")
    parser.add_argument("--bins-2d", type=int, default=20, help="Bins for 2D occupancy")
    parser.add_argument("--corner-max-params", type=int, default=4, help="Max params to include in corner plot")
    parser.add_argument("--corner-topN", type=int, default=0, help="Highlight top-N designs in corner plot")
    parser.add_argument("--corner-kde", action="store_true", help="Overlay KDE contours on corner plot")
    parser.add_argument("--cluster-k-min", type=int, default=2, help="Min clusters to consider")
    parser.add_argument("--cluster-k-max", type=int, default=4, help="Max clusters to consider")
    parser.add_argument(
        "--cluster-subsample",
        type=int,
        default=2000,
        help="Optional subsample for clustering computations",
    )
    parser.add_argument(
        "--cluster-top-quantile",
        type=float,
        default=1.0,
        help="Quantile of best points to cluster (0<q<=1). Uses metric direction.",
    )
    parser.add_argument(
        "--cluster-stability-iters",
        type=int,
        default=20,
        help="Bootstrap iterations for cluster stability (ARI)",
    )

    args = parser.parse_args()

    df, target_col, features = load_filtered_controller(
        args.csv,
        target_col=args.target,
        filter_penalties=not args.no_filter_penalties,
    )

    if args.restrict is not None:
        if args.restrict <= 0:
            raise SystemExit(f"--restrict must be positive, got {args.restrict}")
        if args.restrict < len(df):
            df = df.iloc[: args.restrict].copy()
            print(f"Restricted to first {len(df)} rows (requested {args.restrict})")

    outdir = ensure_outdir(args.csv, args.outdir)

    # Save detected columns
    with open(os.path.join(outdir, "detected_columns.txt"), "w") as f:
        f.write(f"metric_col: {target_col}\n")
        f.write("param_cols:\n")
        for c in features:
            f.write(f"- {c}\n")

    bounds = _controller_bounds(features)

    # Coverage
    cov_df = summarize_coverage(
        df,
        features,
        outdir,
        bins_1d=args.bins_1d,
        bins_2d=args.bins_2d,
        bounds=bounds,
    )

    # Metrics box plot
    plot_metrics_boxplot(df, outdir)

    # Corner plot
    corner_plot(
        df,
        features,
        target_col,
        os.path.join(outdir, "corner_plot.png"),
        max_params=args.corner_max_params,
        bounds=bounds,
        topN=(args.corner_topN if args.corner_topN and args.corner_topN > 0 else None),
        lower_is_better=args.lower_is_better,
        add_kde=args.corner_kde,
        plot_units=False,
    )

    # Optional filter to top-quantile by metric before clustering
    df_for_cluster = df.copy()
    if target_col and 0 < args.cluster_top_quantile < 1.0:
        if args.lower_is_better:
            thr = df_for_cluster[target_col].quantile(args.cluster_top_quantile)
            df_for_cluster = df_for_cluster[df_for_cluster[target_col] <= thr]
        else:
            thr = df_for_cluster[target_col].quantile(1.0 - args.cluster_top_quantile)
            df_for_cluster = df_for_cluster[df_for_cluster[target_col] >= thr]

    clust_info = clustering_analysis(
        df_for_cluster,
        features,
        target_col,
        outdir,
        lower_is_better=args.lower_is_better,
        k_min=args.cluster_k_min,
        k_max=args.cluster_k_max,
        subsample=args.cluster_subsample,
        stability_iters=args.cluster_stability_iters,
    )

    # Summary
    with open(os.path.join(outdir, "summary.txt"), "w") as f:
        f.write("Extended analysis summary (steering controller only)\n")
        f.write("====================================================\n\n")
        f.write(f"CSV: {args.csv}\n")
        f.write(f"Detected metric: {target_col}\n")
        f.write(f"Steering parameters ({len(features)}): {', '.join(features)}\n\n")
        f.write("Coverage (per-parameter)\n")
        f.write(cov_df.to_string(index=False))
        f.write("\n\n")
        f.write("Clustering\n")
        f.write(f"Best k: {clust_info.get('best_k')}\n")
        f.write(f"Best silhouette: {clust_info.get('best_silhouette')}\n")
        if "stability_ari_mean" in clust_info and not np.isnan(clust_info["stability_ari_mean"]):
            f.write(f"Stability ARI mean: {clust_info['stability_ari_mean']:.4f}\n")


if __name__ == "__main__":
    main()
