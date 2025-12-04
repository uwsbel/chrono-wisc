import argparse
import os
import json
from typing import List, Tuple, Dict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize


# def _patch_threadpoolctl_openblas_null_config() -> None:
#     """Guard threadpoolctl against null OpenBLAS config pointers."""
#     try:
#         import ctypes
#         import threadpoolctl
#     except Exception:
#         return
#     try:
#         cls = threadpoolctl._OpenBLASModule  # type: ignore[attr-defined]
#     except Exception:
#         return
#     if getattr(cls, "_null_config_patch_applied", False):
#         return

#     def _safe_get_version(self):
#         get_config = getattr(self._dynlib, "openblas_get_config", None)
#         if get_config is None:
#             return None
#         try:
#             get_config.restype = ctypes.c_char_p
#             cfg = get_config()
#             if not cfg:
#                 return None
#             cfg_parts = cfg.split()
#             if len(cfg_parts) >= 2 and cfg_parts[0] == b"OpenBLAS":
#                 return cfg_parts[1].decode("utf-8")
#         except Exception:
#             return None
#         return None

#     cls.get_version = _safe_get_version  # type: ignore[assignment]
#     cls._null_config_patch_applied = True


# _patch_threadpoolctl_openblas_null_config()

from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.metrics import silhouette_score
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import adjusted_rand_score
from scipy.stats import gaussian_kde


def _safe_numeric(df: pd.DataFrame, cols: List[str]) -> pd.DataFrame:
    out = df.copy()
    for c in cols:
        # If object dtype, map to category codes to retain discrete info
        if out[c].dtype == object:
            cat = pd.Categorical(out[c])
            out[c] = pd.Series(cat.codes, index=out.index).replace(-1, np.nan)
        else:
            out[c] = pd.to_numeric(out[c], errors="coerce")
    return out


def load_trials(csv_path: str) -> pd.DataFrame:
    """Load trials.csv and drop failed/invalid rows to avoid biasing stats.

    Rules applied:
    - If a sibling failed_trials.jsonl exists, drop rows whose trial_index appears there.
    - If a 'sim_failed' column exists, drop rows where it's truthy.
    - Drop rows where any present key metrics are non-finite: composite_metric, metric,
      rms_error, total_time_to_reach, t_elapsed, average_power.
    """
    df = pd.read_csv(csv_path)

    # Standard cleanup for numeric fields we care about
    num_candidates = [
        "metric",
        "composite_metric",
        "total_time_to_reach",
        "t_elapsed",
        "average_power",
        "beta_rms",
        "rms_error",
    ]
    for c in num_candidates:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    # Drop failed rows by consulting failed_trials.jsonl if present
    failed_path = os.path.join(os.path.dirname(os.path.abspath(csv_path)), "failed_trials.jsonl")
    failed_idx = set()
    if os.path.isfile(failed_path):
        try:
            with open(failed_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line)
                        ti = obj.get("trial_index")
                        if ti is not None:
                            failed_idx.add(int(ti))
                    except Exception:
                        continue
        except Exception:
            pass
    if failed_idx and "trial_index" in df.columns:
        df = df[~df["trial_index"].astype(int).isin(failed_idx)]

    # If an explicit sim_failed column exists, drop those too
    if "sim_failed" in df.columns:
        try:
            mask_ok = ~df["sim_failed"].astype(bool)
            df = df[mask_ok]
        except Exception:
            pass

    # Drop rows with non-finite values in key metric columns that are present
    key_metrics = [c for c in [
        "composite_metric",
        "metric",
        "rms_error",
        "total_time_to_reach",
        "t_elapsed",
        "average_power",
    ] if c in df.columns]
    if key_metrics:
        finite_mask = np.ones(len(df), dtype=bool)
        for c in key_metrics:
            vals = pd.to_numeric(df[c], errors="coerce")
            finite_mask &= np.isfinite(vals.values)
        df = df[finite_mask]

    # Drop rows missing the generic 'metric' if that is the target of interest
    if "metric" in df.columns:
        df = df.dropna(subset=["metric"])  # keep only rows with metric

    return df.reset_index(drop=True)


def plot_metrics_boxplot(df: pd.DataFrame, outdir: str) -> None:
    """Create box plots for key metrics if present.

    Generates:
    - One combined figure with all present metrics.
    - Separate one-metric-per-figure box plots.

    Metrics considered (in order): metric, total_time_to_reach, rms_error,
    average_power, and beta_rms (optional).
    """
    metric_candidates = [
        "metric",
        "composite_metric",
        "total_time_to_reach",
        "t_elapsed",
        "rms_error",
        "average_power",
        "beta_rms",
    ]

    present = [c for c in metric_candidates if c in df.columns]
    if not present:
        return

    # Collect data, coercing to numeric and dropping NaNs per-series
    data = []
    labels = []
    for c in present:
        vals = pd.to_numeric(df[c], errors="coerce").dropna().values
        if len(vals) == 0:
            continue
        data.append(vals)
        labels.append(c)

    if not data:
        return

    # Color palette reused for both combined and per-metric plots
    colors = ["#4C78A8", "#F58518", "#E45756", "#72B7B2", "#54A24B", "#EECA3B"]

    # Combined plot (kept for convenience)
    fig_c, ax_c = plt.subplots(figsize=(max(6.0, 1.6 * len(labels)), 4.8))
    bp_c = ax_c.boxplot(data, tick_labels=labels, showfliers=False, patch_artist=True)
    for patch, color in zip(bp_c['boxes'], colors * (len(labels) // len(colors) + 1)):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
        patch.set_edgecolor("#333333")
    for median in bp_c['medians']:
        median.set_color("#222222")
        median.set_linewidth(1.5)
    ax_c.set_title("Metrics distribution (box plot)")
    ax_c.set_ylabel("value")
    ax_c.grid(axis="y", alpha=0.25)
    fig_c.tight_layout()
    fig_c.savefig(os.path.join(outdir, "metrics_boxplot.png"), dpi=200)
    plt.close(fig_c)

    # Print and save summary statistics for each metric
    print("\nMetric summary statistics:")
    header = (
        f"{'metric':<26}  {'n':>6}  {'mean':>12}  {'std':>12}  "
        f"{'min':>12}  {'p5':>12}  {'p50':>12}  {'p95':>12}  {'max':>12}"
    )
    print(header)
    print("-" * len(header))
    stats_rows = []
    for lbl, vals in zip(labels, data):
        if len(vals) == 0:
            continue
        mean = float(np.nanmean(vals))
        std = float(np.nanstd(vals))
        vmin = float(np.nanmin(vals))
        vmax = float(np.nanmax(vals))
        p5, p50, p95 = [float(np.nanpercentile(vals, q)) for q in (5, 50, 95)]
        print(
            f"{lbl:<26}  {len(vals):6d}  {mean:12.6f}  {std:12.6f}  "
            f"{vmin:12.6f}  {p5:12.6f}  {p50:12.6f}  {p95:12.6f}  {vmax:12.6f}"
        )
        stats_rows.append({
            "metric": lbl,
            "n": int(len(vals)),
            "mean": mean,
            "std": std,
            "min": vmin,
            "p5": p5,
            "p50": p50,
            "p95": p95,
            "max": vmax,
        })

    if stats_rows:
        stats_df = pd.DataFrame(stats_rows)
        stats_df.to_csv(os.path.join(outdir, "metrics_stats.csv"), index=False)

    # Individual plots per metric
    for idx, (lbl, vals) in enumerate(zip(labels, data)):
        fig, ax = plt.subplots(figsize=(4.8, 4.8))
        bp = ax.boxplot([vals], showfliers=False, patch_artist=True)
        color = colors[idx % len(colors)]
        bp['boxes'][0].set_facecolor(color)
        bp['boxes'][0].set_alpha(0.7)
        bp['boxes'][0].set_edgecolor("#333333")
        bp['medians'][0].set_color("#222222")
        bp['medians'][0].set_linewidth(1.5)
        ax.set_title(f"{lbl} (box plot)")
        ax.set_ylabel(lbl)
        ax.set_xticks([])
        ax.grid(axis="y", alpha=0.25)
        fig.tight_layout()
        safe_lbl = str(lbl).replace(" ", "_")
        fig.savefig(os.path.join(outdir, f"boxplot_{safe_lbl}.png"), dpi=200)
        plt.close(fig)


def detect_param_columns(df: pd.DataFrame) -> Tuple[List[str], str]:
    # Heuristics: Exclude non-parameter columns commonly present
    ignore = {
        "timestamp",
        "trial_index",
        "arm_name",
        "arm_parameters",
        "metric",
        "total_time_to_reach",
        "average_power",
        "rms_error",
        "beta_rms",
    }

    # Include particle/control parameters only if varied (nunique > 1)
    candidates = []
    for c in df.columns:
        if c in ignore:
            continue
        if df[c].nunique(dropna=True) <= 1:
            continue
        # Keep only numeric-like for analysis; try conversion
        try:
            pd.to_numeric(df[c], errors="raise")
            candidates.append(c)
        except Exception:
            # If non-numeric but low cardinality, encode as category later
            if df[c].nunique(dropna=True) > 1:
                # Convert to category codes on the fly by caller if needed
                candidates.append(c)
    # Target metric column
    metric_col = "metric" if "metric" in df.columns else None
    # metric_col = "rms_error" if "rms_error" in df.columns else None
    return candidates, metric_col


def ensure_outdir(csv_path: str, outdir: str | None) -> str:
    if outdir:
        os.makedirs(outdir, exist_ok=True)
        return outdir
    # Default: <csv_parent>/analysis/extended
    base = os.path.dirname(os.path.abspath(csv_path))
    out = os.path.join(base, "analysis", "extended")
    os.makedirs(out, exist_ok=True)
    return out


def _pull_param_bounds(df: pd.DataFrame) -> Dict[str, Tuple[float, float, str]]:
    """Return bounds for known PullTest parameters as (low, high, kind).
    - rad_outer depends on particle_spacing per run.
    - Other bounds are constants as in PullTest_BO.py.
    If a parameter is missing, it's omitted.
    """
    bounds: Dict[str, Tuple[float, float, str]] = {}
    # particle spacing can vary across runs but in a folder it's typically constant.
    spacing = None
    if "particle_spacing" in df.columns:
        s = pd.to_numeric(df["particle_spacing"], errors="coerce").dropna()
        if len(s):
            spacing = float(s.mode().iloc[0])
    if spacing is not None:
        # rad_outer bounds derived from meters [0.05, 0.14]
        rad_lb = int(np.ceil(0.05 / spacing))
        rad_ub = int(np.floor(0.14 / spacing))
        if "rad_outer" in df.columns:
            bounds["rad_outer"] = (float(rad_lb), float(rad_ub), "int")
    # Constants from PullTest_BO.py
    if "w_by_r" in df.columns:
        # Widen upper bound to 1.4 to reflect SineSideSlip runs
        bounds["w_by_r"] = (0.7, 1.4, "float")
    if "what_percent_is_grouser" in df.columns:
        bounds["what_percent_is_grouser"] = (0.0, 0.3, "float")
    if "g_density" in df.columns:
        bounds["g_density"] = (2.0, 16.0, "int")
    if "fan_theta_deg" in df.columns:
        bounds["fan_theta_deg"] = (45.0, 135.0, "int")
    return bounds


def _get_spacing(df: pd.DataFrame) -> float | None:
    """Return modal particle spacing from the CSV if present."""
    if "particle_spacing" in df.columns:
        s = pd.to_numeric(df["particle_spacing"], errors="coerce").dropna()
        if len(s):
            return float(s.mode().iloc[0])
    return None


def summarize_coverage(df: pd.DataFrame, param_cols: List[str], outdir: str, bins_1d: int = 20, bins_2d: int = 20,
                       bounds: Dict[str, Tuple[float, float, str]] | None = None) -> pd.DataFrame:
    # Compute per-parameter coverage stats and save histograms (bounds-aware when provided)
    stats = []
    df_num = df[param_cols].copy()
    df_num = _safe_numeric(df_num, param_cols)
    df_num = df_num.dropna()

    for c in param_cols:
        series = df_num[c].values
        n = len(series)
        uniq = len(np.unique(series))
        lo_b, hi_b = (np.nanmin(series), np.nanmax(series))
        kind = None
        if bounds and c in bounds:
            lo_b, hi_b, kind = bounds[c]
        # Normalized entropy in [0,1]
        hist, _ = np.histogram(series, bins=bins_1d, range=(lo_b, hi_b))
        p = hist / (hist.sum() + 1e-12)
        ent = -np.sum(np.where(p > 0, p * np.log(p + 1e-12), 0.0))
        ent_norm = ent / (np.log(len(hist)) + 1e-12)
        # Bounds-aware ratios
        span = float(np.nanmax(series) - np.nanmin(series))
        denom = float(hi_b - lo_b) if (hi_b is not None and lo_b is not None) else np.nan
        span_ratio = float(span / denom) if denom and denom > 0 else np.nan
        unique_possible_ratio = np.nan
        if bounds and c in bounds:
            if (bounds[c][2] == "int"):
                unique_possible = int(hi_b - lo_b + 1)
                unique_possible_ratio = float(uniq / max(unique_possible, 1))
        stats.append({
            "param": c,
            "n": int(n),
            "unique": int(uniq),
            "unique_ratio": float(uniq / max(n, 1)),
            "entropy_norm": float(ent_norm),
            "bound_low": float(lo_b),
            "bound_high": float(hi_b),
            "span_ratio": float(span_ratio),
            "unique_possible_ratio": float(unique_possible_ratio) if not np.isnan(unique_possible_ratio) else np.nan,
            "min": float(np.nanmin(series)),
            "max": float(np.nanmax(series)),
            "mean": float(np.nanmean(series)),
            "std": float(np.nanstd(series)),
        })

        # Save histogram
        fig, ax = plt.subplots(figsize=(4, 3))
        ax.hist(series, bins=bins_1d, range=(lo_b, hi_b), color="#4C78A8", alpha=0.85)
        ax.set_title(f"{c} (n={n}, uniq={uniq})")
        ax.set_xlabel(c)
        ax.set_ylabel("count")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"hist_{c}.png"), dpi=200)
        plt.close(fig)

    # Pairwise occupancy ratio (fraction of 2D bins visited)
    pair_rows = []
    for i, ci in enumerate(param_cols):
        xi = df_num[ci].values
        for j, cj in enumerate(param_cols):
            if j <= i:
                continue
            xj = df_num[cj].values
            # Use bounds-aware bin edges if provided
            range_i = None
            range_j = None
            if bounds and ci in bounds:
                range_i = (bounds[ci][0], bounds[ci][1])
            if bounds and cj in bounds:
                range_j = (bounds[cj][0], bounds[cj][1])
            if range_i is None:
                range_i = (np.nanmin(xi), np.nanmax(xi))
            if range_j is None:
                range_j = (np.nanmin(xj), np.nanmax(xj))
            H, _, _ = np.histogram2d(xi, xj, bins=bins_2d, range=(range_i, range_j))
            occ = (H > 0).sum() / H.size
            pair_rows.append({"param_i": ci, "param_j": cj, "occupancy_ratio": float(occ)})

    cov_df = pd.DataFrame(stats)
    cov_df.to_csv(os.path.join(outdir, "coverage_stats.csv"), index=False)

    pair_df = pd.DataFrame(pair_rows)
    pair_df.to_csv(os.path.join(outdir, "coverage_pairwise_occupancy.csv"), index=False)

    return cov_df


def corner_plot(df: pd.DataFrame, param_cols: List[str], metric_col: str | None, outpath: str,
                max_params: int = 8, s: float = 10.0, bounds: Dict[str, Tuple[float, float, str]] | None = None,
                topN: int | None = None, lower_is_better: bool = True, add_kde: bool = False,
                plot_units: bool = True):
    cols = param_cols[:max_params]
    data = _safe_numeric(df[cols], cols).dropna()
    # Optional unit conversion (rad_outer -> meters using particle_spacing)
    spacing = _get_spacing(df) if plot_units else None
    label_overrides: Dict[str, str] = {}
    plot_bounds = dict(bounds) if bounds else {}
    if spacing is not None and "rad_outer" in cols:
        # ensure float dtype before assignment to avoid dtype warnings
        if str(data["rad_outer"].dtype) != "float64":
            data["rad_outer"] = data["rad_outer"].astype(float)
        data.loc[:, "rad_outer"] = data["rad_outer"] * spacing
        label_overrides["rad_outer"] = "rad_outer (m)"
        if plot_bounds and "rad_outer" in plot_bounds:
            lo, hi, kind = plot_bounds["rad_outer"]
            plot_bounds["rad_outer"] = (lo * spacing, hi * spacing, kind)
    mvals = None
    if metric_col and metric_col in df.columns:
        mvals = pd.to_numeric(df.loc[data.index, metric_col], errors="coerce")
        # Drop rows with NaN metric
        valid = ~mvals.isna()
        data = data.loc[valid]
        mvals = mvals.loc[valid]

    k = len(cols)
    # Provide extra width for a dedicated colorbar axis to avoid overlap
    fig_w = 2.6 * k + 1.0
    fig_h = 2.6 * k
    fig, axes = plt.subplots(k, k, figsize=(fig_w, fig_h))
    cmap = plt.cm.viridis
    norm = Normalize(vmin=float(np.nanmin(mvals)) if mvals is not None else None,
                     vmax=float(np.nanmax(mvals)) if mvals is not None else None)

    # Precompute topN indices for highlighting, if requested
    top_idx = None
    if topN and mvals is not None and len(mvals) > 0:
        order = np.argsort(mvals.values)
        if not lower_is_better:
            order = order[::-1]
        top_idx = set(data.index.values[order[: min(topN, len(order))]])

    for i in range(k):
        for j in range(k):
            ax = axes[i, j]
            if i == j:
                lo_b, hi_b = None, None
                # Bounds for hist axis (use transformed plot bounds if available)
                if plot_bounds and cols[j] in plot_bounds:
                    lo_b, hi_b = plot_bounds[cols[j]][0], plot_bounds[cols[j]][1]
                hist_kwargs = {"bins": 20, "color": "#72B7B2", "alpha": 0.9}
                if lo_b is not None and hi_b is not None:
                    hist_kwargs["range"] = (lo_b, hi_b)
                ax.hist(data.iloc[:, j].values, **hist_kwargs)
                ax.set_ylabel("count") if j == 0 else None
            elif i > j:
                x = data.iloc[:, j].values
                y = data.iloc[:, i].values
                if mvals is not None:
                    sc = ax.scatter(
                        x,
                        y,
                        c=mvals.values,
                        s=s,
                        cmap=cmap,
                        norm=norm,
                        alpha=0.8,
                        edgecolors="none",
                    )
                else:
                    ax.scatter(x, y, s=s, alpha=0.8)

                # Highlight topN
                if top_idx is not None:
                    mask = np.array([idx in top_idx for idx in data.index])
                    ax.scatter(x[mask], y[mask], s=max(s * 1.8, 14), facecolors='none', edgecolors='crimson', linewidths=1.0, alpha=0.9)

                # KDE contours (optional)
                if add_kde and len(x) > 20:
                    try:
                        xy = np.vstack([x, y])
                        kde = gaussian_kde(xy)
                        # grid from bounds if available, else from data
                        if plot_bounds and cols[j] in plot_bounds:
                            x_min, x_max = plot_bounds[cols[j]][0], plot_bounds[cols[j]][1]
                        else:
                            x_min, x_max = float(np.nanmin(x)), float(np.nanmax(x))
                        if plot_bounds and cols[i] in plot_bounds:
                            y_min, y_max = plot_bounds[cols[i]][0], plot_bounds[cols[i]][1]
                        else:
                            y_min, y_max = float(np.nanmin(y)), float(np.nanmax(y))
                        xx, yy = np.meshgrid(np.linspace(x_min, x_max, 60), np.linspace(y_min, y_max, 60))
                        zz = kde(np.vstack([xx.ravel(), yy.ravel()])).reshape(xx.shape)
                        ax.contour(xx, yy, zz, levels=4, colors='k', linewidths=0.6, alpha=0.6)
                    except Exception:
                        pass
            else:
                ax.axis("off")
            if i == k - 1:
                ax.set_xlabel(label_overrides.get(cols[j], cols[j]))
            else:
                ax.set_xticklabels([])
            if j == 0:
                ax.set_ylabel(label_overrides.get(cols[i], cols[i]))
            else:
                ax.set_yticklabels([])

    if mvals is not None:
        # Dedicated colorbar axis to keep it outside of subplot grid
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        # Leave space at the right for the colorbar
        fig.subplots_adjust(left=0.06, right=0.90, top=0.94, bottom=0.06)
        cax = fig.add_axes([0.92, 0.12, 0.02, 0.76])
        cbar = fig.colorbar(sm, cax=cax)
        cbar.set_label(metric_col)
    else:
        fig.subplots_adjust(left=0.06, right=0.98, top=0.94, bottom=0.06)

    fig.suptitle("Corner plot (lower triangle) with metric color", y=0.995)
    fig.savefig(outpath, dpi=200)
    plt.close(fig)


def clustering_analysis(df: pd.DataFrame, param_cols: List[str], metric_col: str | None, outdir: str,
                        lower_is_better: bool = True, k_min: int = 2, k_max: int = 6,
                        subsample: int | None = None, stability_iters: int = 20) -> Dict[str, float]:
    X = _safe_numeric(df[param_cols], param_cols)
    y = None
    if metric_col and metric_col in df.columns:
        y = pd.to_numeric(df[metric_col], errors="coerce")

    # Drop rows with any NaN in features or target
    if y is not None:
        valid = (~X.isna().any(axis=1)) & (~y.isna())
        X = X.loc[valid]
        y = y.loc[valid]
    else:
        X = X.dropna()

    if len(X) < k_max:
        k_max = max(k_min + 1, len(X))
    if k_max <= k_min:
        k_max = k_min + 1

    if subsample and len(X) > subsample:
        X = X.sample(n=subsample, random_state=0)
        if y is not None:
            y = y.loc[X.index]

    scaler = StandardScaler()
    Xs = scaler.fit_transform(X.values)

    # Choose k by silhouette
    best_k = None
    best_sil = -1.0
    for k in range(k_min, min(k_max, len(Xs)) + 1):
        try:
            km = KMeans(n_clusters=k, n_init=10, random_state=0)
            labels = km.fit_predict(Xs)
            if len(np.unique(labels)) < 2:
                continue
            sil = silhouette_score(Xs, labels)
            if sil > best_sil:
                best_sil = sil
                best_k = k
        except Exception:
            continue

    if best_k is None:
        best_k = min(3, len(Xs))
        km = KMeans(n_clusters=best_k, n_init=10, random_state=0)
        labels = km.fit_predict(Xs)
    else:
        km = KMeans(n_clusters=best_k, n_init=10, random_state=0)
        labels = km.fit_predict(Xs)

    # Summarize clusters
    summary_rows = []
    for c in range(best_k):
        idx = (labels == c)
        count = int(idx.sum())
        row = {"cluster": c, "count": count}
        # Centroid in original scale
        centroid = scaler.inverse_transform(km.cluster_centers_[c].reshape(1, -1)).ravel()
        for j, name in enumerate(param_cols):
            row[f"centroid_{name}"] = float(centroid[j])
        if y is not None:
            yvals = y.loc[X.index[idx]]
            row["metric_mean"] = float(yvals.mean())
            row["metric_median"] = float(yvals.median())
            row["metric_std"] = float(yvals.std(ddof=1)) if len(yvals) > 1 else 0.0
        summary_rows.append(row)
    summary_df = pd.DataFrame(summary_rows)
    # Save unsorted to avoid accidental key error
    summary_df.to_csv(os.path.join(outdir, "clustering_summary.csv"), index=False)

    # PCA projection plot colored by cluster and shaded by metric
    pca = PCA(n_components=2, random_state=0)
    Z = pca.fit_transform(Xs)

    fig, ax = plt.subplots(figsize=(6.0, 4.8))
    scatter = ax.scatter(Z[:, 0], Z[:, 1], c=labels, cmap="tab10", s=16, alpha=0.9, edgecolors="none")
    ax.set_title(f"KMeans clustering (k={best_k}), PCA(2) projection")
    ax.set_xlabel("PC1")
    ax.set_ylabel("PC2")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "clusters_pca.png"), dpi=200)
    plt.close(fig)

    if y is not None:
        fig, ax = plt.subplots(figsize=(6.0, 4.8))
        norm = Normalize(vmin=float(y.min()), vmax=float(y.max()))
        sc = ax.scatter(Z[:, 0], Z[:, 1], c=y.values, cmap="viridis", norm=norm, s=16, alpha=0.9, edgecolors="none")
        ax.set_title("PCA(2) projection colored by metric")
        ax.set_xlabel("PC1")
        ax.set_ylabel("PC2")
        cbar = fig.colorbar(sc, ax=ax)
        cbar.set_label(metric_col)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "pca_by_metric.png"), dpi=200)
        plt.close(fig)

    # Cluster stability via bootstrapped KMeans and ARI vs baseline labels
    stability_scores = []
    for b in range(max(1, stability_iters)):
        try:
            # Fit on a bootstrap sample, then assign labels to all points via nearest centroid
            idx_boot = np.random.choice(len(Xs), size=len(Xs), replace=True)
            km_b = KMeans(n_clusters=best_k, n_init=10, random_state=1337 + b).fit(Xs[idx_boot])
            # Assign labels to all points by nearest centroid
            dists = np.linalg.norm(Xs[:, None, :] - km_b.cluster_centers_[None, :, :], axis=2)
            labels_b = np.argmin(dists, axis=1)
            ari = adjusted_rand_score(labels, labels_b)
            stability_scores.append(ari)
        except Exception:
            continue
    if stability_scores:
        with open(os.path.join(outdir, "clustering_stability.txt"), "w") as f:
            f.write(f"k={best_k}\n")
            f.write(f"ARI_mean={np.mean(stability_scores):.4f}\n")
            f.write(f"ARI_std={np.std(stability_scores):.4f}\n")
            f.write(f"iters={len(stability_scores)}\n")

    return {"best_k": float(best_k), "best_silhouette": float(best_sil), "stability_ari_mean": float(np.mean(stability_scores)) if stability_scores else np.nan}


def main():
    parser = argparse.ArgumentParser(description="Extended BO analysis: coverage, clustering, corner plots from trials.csv")
    parser.add_argument("--csv", required=True, help="Path to trials.csv")
    parser.add_argument("--outdir", default=None, help="Output directory (default: <csvdir>/analysis/extended)")
    parser.add_argument("--lower-is-better", action="store_true", help="Metric direction: lower is better (default: False -> higher is better)")
    parser.add_argument("--bins-1d", type=int, default=20, help="Bins for 1D histograms")
    parser.add_argument("--bins-2d", type=int, default=20, help="Bins for 2D occupancy")
    parser.add_argument("--corner-max-params", type=int, default=8, help="Max params to include in corner plot")
    parser.add_argument("--corner-topN", type=int, default=0, help="Highlight top-N designs in corner plot")
    parser.add_argument("--corner-kde", action="store_true", help="Overlay KDE contours on corner plot")
    parser.add_argument("--cluster-k-min", type=int, default=2, help="Min clusters to consider")
    parser.add_argument("--cluster-k-max", type=int, default=6, help="Max clusters to consider")
    parser.add_argument("--cluster-subsample", type=int, default=2000, help="Optional subsample for clustering computations")
    parser.add_argument("--cluster-top-quantile", type=float, default=1.0, help="Quantile of best points to cluster (0<q<=1). Uses metric direction.")
    parser.add_argument("--cluster-stability-iters", type=int, default=20, help="Bootstrap iterations for cluster stability (ARI)")

    args = parser.parse_args()

    df = load_trials(args.csv)
    outdir = ensure_outdir(args.csv, args.outdir)

    param_cols, metric_col = detect_param_columns(df)
    if not param_cols:
        raise SystemExit("No varying parameter columns detected.")

    # Save a record of detected columns
    with open(os.path.join(outdir, "detected_columns.txt"), "w") as f:
        f.write(f"metric_col: {metric_col}\n")
        f.write("param_cols:\n")
        for c in param_cols:
            f.write(f"- {c}\n")

    # Bounds (if recognizable as PullTest params)
    bounds = _pull_param_bounds(df)

    # Coverage
    cov_df = summarize_coverage(df, param_cols, outdir, bins_1d=args.bins_1d, bins_2d=args.bins_2d, bounds=bounds)

    # Metrics box plot (separate figure)
    plot_metrics_boxplot(df, outdir)

    # Corner plot
    corner_plot(
        df,
        param_cols,
        metric_col,
        os.path.join(outdir, "corner_plot.png"),
        max_params=args.corner_max_params,
        bounds=bounds,
        topN=(args.corner_topN if args.corner_topN and args.corner_topN > 0 else None),
        lower_is_better=args.lower_is_better,
        add_kde=args.corner_kde,
    )

    # Optional filter to top-quantile by metric before clustering
    df_for_cluster = df.copy()
    if metric_col and 0 < args.cluster_top_quantile < 1.0:
        if args.lower_is_better:
            thr = df_for_cluster[metric_col].quantile(args.cluster_top_quantile)
            df_for_cluster = df_for_cluster[df_for_cluster[metric_col] <= thr]
        else:
            thr = df_for_cluster[metric_col].quantile(1.0 - args.cluster_top_quantile)
            df_for_cluster = df_for_cluster[df_for_cluster[metric_col] >= thr]

    # Clustering
    clust_info = clustering_analysis(
        df_for_cluster,
        param_cols,
        metric_col,
        outdir,
        lower_is_better=args.lower_is_better,
        k_min=args.cluster_k_min,
        k_max=args.cluster_k_max,
        subsample=args.cluster_subsample,
        stability_iters=args.cluster_stability_iters,
    )

    # Write a short summary
    with open(os.path.join(outdir, "summary.txt"), "w") as f:
        f.write("Extended analysis summary\n")
        f.write("=========================\n\n")
        f.write(f"CSV: {args.csv}\n")
        f.write(f"Detected metric: {metric_col}\n")
        f.write(f"Varying parameters ({len(param_cols)}): {', '.join(param_cols)}\n\n")
        f.write("Coverage (per-parameter)\n")
        f.write(cov_df.to_string(index=False))
        f.write("\n\n")
        f.write("Clustering\n")
        f.write(f"Best k: {clust_info.get('best_k')}\n")
        f.write(f"Best silhouette: {clust_info.get('best_silhouette')}\n")
        if 'stability_ari_mean' in clust_info and not np.isnan(clust_info['stability_ari_mean']):
            f.write(f"Stability ARI mean: {clust_info['stability_ari_mean']:.4f}\n")


if __name__ == "__main__":
    main()
