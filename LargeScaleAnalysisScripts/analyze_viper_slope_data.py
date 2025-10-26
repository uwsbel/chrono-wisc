#!/usr/bin/env python3

"""
Analyze Viper slope simulation CSVs to quantify how density, friction,
cohesion, gravity, and slope affect distance traveled along +x.

Input files follow pattern like:
  viper_slope_tests/viper_data_d1000_f3_c50_g37_s10.csv

Naming convention:
  - d: density (e.g., 1000)
  - f: friction coefficient scaled by 10 (e.g., f3 -> 0.3)
  - c: cohesion scaled by 100 Pa (e.g., c50 -> 5000 Pa)
  - g: gravity scaled by 10 (e.g., g37 -> 3.7 m/s^2)
  - s: slope in degrees (e.g., s10 -> 10 deg)

For each CSV, the script computes two x-distance metrics:
  - net_dx: final pos_x - initial pos_x
  - max_forward_dx: max(pos_x) - initial pos_x

It then applies several analysis methods:
  1) Correlations (Pearson and Spearman)
  2) Regularized linear model with interactions (PolynomialFeatures + RidgeCV)
     - cross-validated R^2
     - permutation importance (with RepeatedKFold averaging)
     - drop-column (group) importance per original factor via CV
  3) Gradient Boosting regressor (nonlinear)
     - permutation importance (with RepeatedKFold averaging)
     - partial dependence plots per factor
  4) Optional SHAP (if `shap` is available) for Gradient Boosting

Outputs are written under an output directory (default: viper_slope_analysis):
  - summary.csv: one row per run with parsed factors and distance metrics
  - metrics.txt: summary of correlations, CV scores, and importances
  - permutation_importance/*.csv: per-model permutation importances
  - pdp/*.png: partial dependence plots for each factor (GBM)
  - shap/*.png: SHAP summary plots, if shap is installed

Usage:
  python analyze_viper_slope_data.py --data-dir viper_slope_tests --out-dir viper_slope_analysis
  python analyze_viper_slope_data.py --per-slope --data-dir viper_slope_tests --out-dir viper_slope_analysis

The --per-slope flag performs separate analysis for each unique slope value,
creating subdirectories like slope_10/, slope_15/, etc. with individual
correlations, models, and plots for each slope.

Requires: Python 3.8+, pandas, numpy, scikit-learn, matplotlib. If shap is
available, SHAP plots will also be generated.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd

from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.linear_model import RidgeCV
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.inspection import permutation_importance
from sklearn.model_selection import RepeatedKFold, cross_val_score
from sklearn.metrics import r2_score
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


FILENAME_RE = re.compile(
    r"viper_data_d(?P<d>\d+)_f(?P<f>\d+)_c(?P<c>\d+)_g(?P<g>\d+)_s(?P<s>\-?\d+)\.csv$"
)


@dataclass
class Factors:
    density: float
    friction: float
    cohesion_pa: float
    gravity: float
    slope_deg: float


def parse_factors_from_filename(fname: str) -> Factors:
    m = FILENAME_RE.search(os.path.basename(fname))
    if not m:
        raise ValueError(f"Unexpected filename format: {fname}")
    d = float(m.group("d"))
    f = float(m.group("f")) / 10.0  # f3 -> 0.3
    c = float(m.group("c")) * 100.0  # c50 -> 5000 Pa
    g = float(m.group("g")) / 10.0  # g37 -> 3.7 m/s^2
    s = float(m.group("s"))  # slope degrees
    return Factors(density=d, friction=f, cohesion_pa=c, gravity=g, slope_deg=s)


def compute_distances(df: pd.DataFrame, min_time: float = 4.9975, max_z_vel: float = 20.0) -> Tuple[float, float, bool]:
    # Robustly compute x-travel. Some files include a trailing corrupted row with
    # astronomical values; filter to finite and reasonable magnitudes.
    cols = ["pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"]
    df = df.sort_values("time", kind="mergesort").reset_index(drop=True)
    # Keep rows with all finite values in key columns
    m_finite = np.isfinite(df[cols]).all(axis=1)
    # Also drop rows with absurd magnitudes
    m_mag = (df[cols].abs() < 1e6).all(axis=1)
    dfc = df.loc[m_finite & m_mag]
    if dfc.empty:
        return np.nan, np.nan, True  # True means abandoned
    
    # Check if simulation ended before minimum time
    max_time = dfc["time"].max()
    if max_time < min_time:
        return np.nan, np.nan, True  # True means abandoned
    
    # Check for unreasonable z velocity readings
    max_z_velocity = dfc["vel_z"].abs().max()
    if max_z_velocity > max_z_vel:
        return np.nan, np.nan, True  # True means abandoned

    x = dfc["pos_x"].to_numpy()
    initial = x[0]
    final = x[-1]
    net_dx = float(final - initial)
    max_forward_dx = float(np.nanmax(x) - initial)
    return net_dx, max_forward_dx, False  # False means not abandoned


def load_runs(data_dir: Path, min_time: float = 4.9975, max_z_vel: float = 20.0) -> Tuple[pd.DataFrame, int]:
    rows: List[Dict] = []
    abandoned_count = 0
    files = sorted([p for p in data_dir.glob("viper_data_d*_f*_c*_g*_s*.csv") if p.is_file()])
    if not files:
        raise SystemExit(f"No files found under {data_dir}")

    for fpath in files:
        try:
            factors = parse_factors_from_filename(fpath.name)
        except Exception as e:
            print(f"Skipping {fpath.name}: {e}", file=sys.stderr)
            continue

        try:
            df = pd.read_csv(fpath)
        except Exception as e:
            print(f"Failed to read {fpath.name}: {e}", file=sys.stderr)
            continue

        # Basic column validation
        for col in ["pos_x", "time"]:
            if col not in df.columns:
                print(f"Skipping {fpath.name}: missing column {col}", file=sys.stderr)
                df = None
                break
        if df is None:
            continue

        net_dx, max_forward_dx, is_abandoned = compute_distances(df, min_time, max_z_vel)
        
        if is_abandoned:
            abandoned_count += 1
            print(f"Abandoned simulation (ended before {min_time}s or |vel_z| > {max_z_vel} m/s): {fpath.name}", file=sys.stderr)
            continue  # Skip abandoned simulations from analysis
        
        rows.append({
            "file": fpath.name,
            "density": factors.density,
            "friction": factors.friction,
            "cohesion_pa": factors.cohesion_pa,
            "gravity": factors.gravity,
            "slope_deg": factors.slope_deg,
            "net_dx": net_dx,
            "max_forward_dx": max_forward_dx,
        })

    return pd.DataFrame(rows), abandoned_count


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def correlations(df: pd.DataFrame, y_col: str) -> pd.DataFrame:
    feats = ["density", "friction", "cohesion_pa", "gravity", "slope_deg"]
    out = []
    for col in feats:
        x = df[col].to_numpy()
        y = df[y_col].to_numpy()
        # Pearson
        pearson = np.corrcoef(x, y)[0, 1]
        # Spearman (via ranks)
        xr = pd.Series(x).rank().to_numpy()
        yr = pd.Series(y).rank().to_numpy()
        spearman = np.corrcoef(xr, yr)[0, 1]
        out.append({"feature": col, "pearson": pearson, "spearman": spearman})
    return pd.DataFrame(out)


def build_poly_ridge_pipeline(degree: int = 2) -> Pipeline:
    return Pipeline([
        ("poly", PolynomialFeatures(degree=degree, include_bias=False)),
        ("scaler", StandardScaler()),
        ("model", RidgeCV(alphas=np.logspace(-4, 4, 21), cv=5)),
    ])


def cv_score(model: Pipeline, X: np.ndarray, y: np.ndarray) -> float:
    cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=42)
    scores = cross_val_score(model, X, y, scoring="r2", cv=cv, n_jobs=None)
    return float(np.mean(scores))


def averaged_permutation_importance(model: Pipeline, X: np.ndarray, y: np.ndarray, feature_names: List[str], n_repeats: int = 20) -> pd.DataFrame:
    cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=42)
    importances = []
    for seed, (train_idx, test_idx) in enumerate(cv.split(X)):
        X_train, X_test = X[train_idx], X[test_idx]
        y_train, y_test = y[train_idx], y[test_idx]
        model.fit(X_train, y_train)
        r = permutation_importance(model, X_test, y_test, scoring="r2", n_repeats=n_repeats, random_state=seed)
        for i, name in enumerate(feature_names):
            importances.append({
                "feature": name,
                "import_mean": r.importances_mean[i],
                "import_std": r.importances_std[i],
            })
    imp_df = pd.DataFrame(importances)
    agg = imp_df.groupby("feature").agg(
        import_mean=("import_mean", "mean"),
        import_std=("import_mean", "std"),
        import_mean_avg_std=("import_std", "mean"),
        n=("import_mean", "count"),
    ).reset_index().sort_values("import_mean", ascending=False)
    return agg


def drop_column_importance(base_features: List[str], df: pd.DataFrame, y_col: str, degree: int = 2) -> pd.DataFrame:
    # Compute per-factor contribution via drop-column CV R^2 delta.
    full_X = df[base_features].to_numpy()
    y = df[y_col].to_numpy()
    full_model = build_poly_ridge_pipeline(degree)
    full_cv_r2 = cv_score(full_model, full_X, y)

    rows = []
    for feat in base_features:
        keep = [f for f in base_features if f != feat]
        X_red = df[keep].to_numpy()
        red_model = build_poly_ridge_pipeline(degree)
        red_cv_r2 = cv_score(red_model, X_red, y)
        rows.append({
            "feature": feat,
            "full_cv_r2": full_cv_r2,
            "reduced_cv_r2": red_cv_r2,
            "delta_r2": full_cv_r2 - red_cv_r2,
        })
    out = pd.DataFrame(rows).sort_values("delta_r2", ascending=False).reset_index(drop=True)
    return out


def gradient_boosting_model() -> GradientBoostingRegressor:
    return GradientBoostingRegressor(random_state=42)


def plot_pdp_gbm(model: GradientBoostingRegressor, X: pd.DataFrame, feature_names: List[str], out_dir: Path, target_name: str) -> None:
    from sklearn.inspection import PartialDependenceDisplay

    ensure_dir(out_dir)
    for feat in feature_names:
        fig, ax = plt.subplots(figsize=(5, 4))
        try:
            PartialDependenceDisplay.from_estimator(model, X, [feat], ax=ax, grid_resolution=20)
            ax.set_title(f"PDP: {feat} -> {target_name}")
            fig.tight_layout()
            fig.savefig(out_dir / f"pdp_{feat}.png", dpi=150)
        except Exception as e:
            plt.close(fig)
            print(f"Failed PDP for {feat}: {e}", file=sys.stderr)
            continue
        plt.close(fig)


def try_shap_plots(model, X: pd.DataFrame, out_dir: Path) -> None:
    try:
        import shap  # type: ignore
    except Exception:
        print("shap not installed; skipping SHAP analysis.")
        return

    ensure_dir(out_dir)
    try:
        explainer = shap.TreeExplainer(model)
        shap_values = explainer.shap_values(X)
        plt.figure(figsize=(7, 5))
        shap.summary_plot(shap_values, X, show=False)
        plt.tight_layout()
        plt.savefig(out_dir / "shap_summary.png", dpi=150)
        plt.close()
    except Exception as e:
        print(f"SHAP analysis failed: {e}", file=sys.stderr)


def _scale_unit_to_bounds(U: np.ndarray, bounds: List[Tuple[float, float]]) -> np.ndarray:
    U = np.asarray(U)
    X = np.empty_like(U, dtype=float)
    for i, (lo, hi) in enumerate(bounds):
        X[:, i] = lo + (hi - lo) * U[:, i]
    return X


def analyze_single_slope(df_slope: pd.DataFrame, slope_deg: float, target: str, out_dir: Path, args) -> Dict:
    """Perform analysis for a single slope value."""
    print(f"Analyzing slope {slope_deg}° with {len(df_slope)} samples...")
    
    # Create slope-specific output directory
    slope_out_dir = out_dir / f"slope_{slope_deg}"
    ensure_dir(slope_out_dir)
    ensure_dir(slope_out_dir / "permutation_importance")
    ensure_dir(slope_out_dir / "pdp")
    ensure_dir(slope_out_dir / "shap")
    
    # Filter out NaN targets if any
    df_slope = df_slope[np.isfinite(df_slope[target])].copy()
    
    base_features = ["density", "friction", "cohesion_pa", "gravity"]
    X_df = df_slope[base_features].copy()
    y = df_slope[target].to_numpy()
    X = X_df.to_numpy()
    
    if len(df_slope) == 0:
        print(f"Warning: No valid samples for slope {slope_deg}°")
        return {"slope_deg": slope_deg, "n_samples": 0, "error": "No valid samples"}
    
    # Correlations
    corr_df = correlations(df_slope, target)
    corr_path = slope_out_dir / "correlations.csv"
    corr_df.to_csv(corr_path, index=False)
    
    # Linear model with interactions
    ridge_pipe = build_poly_ridge_pipeline(degree=args.degree)
    ridge_cv_r2 = cv_score(ridge_pipe, X, y)
    ridge_perm = averaged_permutation_importance(ridge_pipe, X, y, base_features, n_repeats=20)
    ridge_perm_path = slope_out_dir / "permutation_importance" / "ridge_poly.csv"
    ridge_perm.to_csv(ridge_perm_path, index=False)
    
    drop_imp = drop_column_importance(base_features, df_slope, target, degree=args.degree)
    drop_imp_path = slope_out_dir / "permutation_importance" / "ridge_drop_column.csv"
    drop_imp.to_csv(drop_imp_path, index=False)
    
    # Gradient Boosting (nonlinear)
    gbm = gradient_boosting_model()
    gbm_cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=42)
    gbm_cv_scores = cross_val_score(gbm, X, y, scoring="r2", cv=gbm_cv)
    gbm_cv_r2 = float(np.mean(gbm_cv_scores))
    
    # Fit once on all data for PDP/SHAP
    gbm.fit(X, y)
    
    gbm_perm = averaged_permutation_importance(gbm, X, y, base_features, n_repeats=20)
    gbm_perm_path = slope_out_dir / "permutation_importance" / "gbm.csv"
    gbm_perm.to_csv(gbm_perm_path, index=False)
    
    # PDP plots
    plot_pdp_gbm(gbm, X_df, base_features, slope_out_dir / "pdp", target)
    
    # Optional SHAP
    try_shap_plots(gbm, X_df, slope_out_dir / "shap")
    
    # Sobol indices using surrogate
    sobol_results = None
    if args.sobol:
        # Choose surrogate
        if args.sobol_model == "ridge":
            ridge_pipe.fit(X, y)
            surrogate = lambda Z: ridge_pipe.predict(Z)
            model_name = "ridge_poly"
        else:
            # gbm already fit on all data
            surrogate = lambda Z: gbm.predict(Z)
            model_name = "gbm"
        
        # Define bounds from observed ranges (assume independent uniforms)
        bounds = []
        for col in base_features:
            lo = float(X_df[col].min())
            hi = float(X_df[col].max())
            # guard against degenerate bounds
            if hi == lo:
                hi = lo + 1e-9
            bounds.append((lo, hi))
        
        try:
            sob = sobol_indices_saltelli(
                surrogate,
                bounds,
                n_samples=args.sobol_n,
                seed=args.sobol_seed,
            )
            sob.insert(0, "feature", base_features)
            sob_path = slope_out_dir / f"sobol_{model_name}.csv"
            sob.to_csv(sob_path, index=False)
            
            # Simple bar plot
            fig, ax = plt.subplots(figsize=(6, 4))
            idx = np.arange(len(base_features))
            ax.bar(idx - 0.2, sob["S1"], width=0.4, label="S1")
            ax.bar(idx + 0.2, sob["ST"], width=0.4, label="ST")
            ax.set_xticks(idx)
            ax.set_xticklabels(base_features, rotation=20)
            ax.set_ylabel("Sobol index")
            ax.set_title(f"Sobol indices ({model_name}, slope={slope_deg}°, target={target})")
            ax.legend()
            fig.tight_layout()
            fig.savefig(slope_out_dir / f"sobol_{model_name}.png", dpi=150)
            plt.close(fig)
            sobol_results = {model_name: sob.set_index("feature").round(4).to_dict(orient="index")}
        except Exception as e:
            print(f"Sobol computation failed for slope {slope_deg}°: {e}", file=sys.stderr)
    
    # Write slope-specific metrics
    metrics = {
        "slope_deg": slope_deg,
        "n_runs": int(len(df_slope)),
        "target": target,
        "correlations": corr_df.set_index("feature").round(4).to_dict(orient="index"),
        "ridge_poly_cv_r2": round(ridge_cv_r2, 4),
        "ridge_perm_importance": ridge_perm.set_index("feature").round(6).to_dict(orient="index"),
        "ridge_drop_column": drop_imp.set_index("feature").round(6).to_dict(orient="index"),
        "gbm_cv_r2": round(gbm_cv_r2, 4),
        "gbm_perm_importance": gbm_perm.set_index("feature").round(6).to_dict(orient="index"),
        "sobol": sobol_results,
    }
    with open(slope_out_dir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)
    
    with open(slope_out_dir / "metrics.txt", "w") as f:
        f.write(f"Slope: {slope_deg}°\n")
        f.write(f"Total runs: {metrics['n_runs']}\n")
        f.write(f"Target: {target}\n")
        f.write("\nPearson/Spearman correlations (feature -> pearson, spearman):\n")
        for _, row in corr_df.iterrows():
            f.write(f"  {row['feature']}: {row['pearson']:.4f}, {row['spearman']:.4f}\n")
        f.write("\nRidge (poly degree {deg}) CV R^2: {r2:.4f}\n".format(deg=args.degree, r2=ridge_cv_r2))
        f.write("Ridge permutation importance (mean R^2 drop):\n")
        for _, row in ridge_perm.iterrows():
            f.write(f"  {row['feature']}: mean={row['import_mean']:.6f}, std={row['import_std']:.6f}\n")
        f.write("\nRidge drop-column importance (delta R^2):\n")
        for _, row in drop_imp.iterrows():
            f.write(f"  {row['feature']}: delta_r2={row['delta_r2']:.6f}\n")
        f.write("\nGBM CV R^2: {r2:.4f}\n".format(r2=gbm_cv_r2))
        f.write("GBM permutation importance (mean R^2 drop):\n")
        for _, row in gbm_perm.iterrows():
            f.write(f"  {row['feature']}: mean={row['import_mean']:.6f}, std={row['import_std']:.6f}\n")
        if sobol_results is not None:
            f.write("\nSobol indices (first-order S1, total-effect ST):\n")
            for model_name, table in sobol_results.items():
                f.write(f"  Model: {model_name}\n")
                for feat, vals in table.items():
                    f.write(f"    {feat}: S1={vals['S1']:.4f}, ST={vals['ST']:.4f}\n")
    
    return metrics


def sobol_indices_saltelli(model_predict, bounds: List[Tuple[float, float]], n_samples: int = 10000, seed: int = 42, batch: int = 20000) -> pd.DataFrame:
    """Compute first-order (S1) and total-effect (ST) Sobol indices using
    Saltelli estimators on a surrogate model.

    - model_predict: callable mapping X[n, d] -> y[n]
    - bounds: list of (low, high) for each of d features (assumed independent, uniform)
    - n_samples: base sample size (A and B each of size n_samples)
    - Returns DataFrame with columns feature, S1, ST
    """
    rng = np.random.default_rng(seed)
    d = len(bounds)
    # Sample A and B in unit hypercube then scale
    A_u = rng.random((n_samples, d))
    B_u = rng.random((n_samples, d))
    A = _scale_unit_to_bounds(A_u, bounds)
    B = _scale_unit_to_bounds(B_u, bounds)

    # Predict in manageable batches
    def batched_predict(X):
        ys = []
        for i in range(0, X.shape[0], batch):
            ys.append(model_predict(X[i : i + batch]))
        return np.concatenate(ys, axis=0)

    fA = batched_predict(A)
    fB = batched_predict(B)
    VY = np.var(fA, ddof=1)
    if VY <= 0:
        VY = np.var(np.concatenate([fA, fB]), ddof=1)
    if VY <= 0:
        raise ValueError("Zero variance in surrogate outputs; cannot compute Sobol indices.")

    S1 = np.zeros(d)
    ST = np.zeros(d)
    for i in range(d):
        A_Bi = A.copy()
        A_Bi[:, i] = B[:, i]
        fABi = batched_predict(A_Bi)
        # Saltelli 2010 estimators
        S1[i] = np.mean(fB * (fABi - fA)) / VY
        ST[i] = np.mean((fA - fABi) ** 2) / (2.0 * VY)

    out = pd.DataFrame({"S1": S1, "ST": ST})
    return out


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser(description="Analyze Viper slope CSVs")
    ap.add_argument("--data-dir", type=str, default="viper_slope_tests", help="Directory with viper_data_*.csv files")
    ap.add_argument("--out-dir", type=str, default="viper_slope_analysis", help="Output directory for results")
    ap.add_argument("--target", type=str, default="max_forward_dx", choices=["net_dx", "max_forward_dx"], help="Target distance metric to analyze")
    ap.add_argument("--degree", type=int, default=2, help="Polynomial degree for Ridge model")
    ap.add_argument("--sobol", action="store_true", help="Compute Sobol indices using a surrogate model")
    ap.add_argument("--sobol-n", type=int, default=20000, help="Sobol base sample size (A and B each of N)")
    ap.add_argument("--sobol-seed", type=int, default=42, help="Random seed for Sobol sampling")
    ap.add_argument("--sobol-model", type=str, default="ridge", choices=["ridge", "gbm"], help="Surrogate model for Sobol computation")
    ap.add_argument("--min-time", type=float, default=4.9975, help="Minimum simulation time to not abandon (seconds)")
    ap.add_argument("--max-z-vel", type=float, default=20.0, help="Maximum absolute z velocity to not abandon (m/s)")
    ap.add_argument("--per-slope", action="store_true", help="Perform separate analysis for each unique slope value")
    args = ap.parse_args(argv)

    data_dir = Path(args.data_dir)
    out_dir = Path(args.out_dir)
    ensure_dir(out_dir)
    ensure_dir(out_dir / "permutation_importance")
    ensure_dir(out_dir / "pdp")
    ensure_dir(out_dir / "shap")

    print(f"Loading runs from {data_dir}...")
    df, abandoned_count = load_runs(data_dir, args.min_time, args.max_z_vel)
    print(f"Abandoned simulations (ended before {args.min_time}s or |vel_z| > {args.max_z_vel} m/s): {abandoned_count}")
    if df.empty:
        print("No valid runs found.")
        return 1

    # Save raw summary
    summary_path = out_dir / "summary.csv"
    df.to_csv(summary_path, index=False)
    print(f"Wrote summary: {summary_path}")

    target = args.target
    
    if args.per_slope:
        # Per-slope analysis
        unique_slopes = sorted(df['slope_deg'].unique())
        print(f"Found {len(unique_slopes)} unique slopes: {unique_slopes}")
        
        slope_results = []
        for slope_deg in unique_slopes:
            df_slope = df[df['slope_deg'] == slope_deg].copy()
            if len(df_slope) == 0:
                print(f"Warning: No data for slope {slope_deg}°")
                continue
                
            slope_metrics = analyze_single_slope(df_slope, slope_deg, target, out_dir, args)
            slope_results.append(slope_metrics)
        
        # Create cross-slope comparison summary
        if slope_results:
            comparison_data = []
            for result in slope_results:
                if 'error' not in result:
                    comparison_data.append({
                        'slope_deg': result['slope_deg'],
                        'n_samples': result['n_runs'],
                        'ridge_cv_r2': result['ridge_poly_cv_r2'],
                        'gbm_cv_r2': result['gbm_cv_r2'],
                    })
            
            if comparison_data:
                comparison_df = pd.DataFrame(comparison_data)
                comparison_path = out_dir / "slope_comparison.csv"
                comparison_df.to_csv(comparison_path, index=False)
                print(f"Wrote slope comparison: {comparison_path}")
                
                # Create comparison plot
                fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
                
                # Sample sizes
                ax1.bar(comparison_df['slope_deg'], comparison_df['n_samples'])
                ax1.set_xlabel('Slope (degrees)')
                ax1.set_ylabel('Number of samples')
                ax1.set_title('Sample sizes by slope')
                
                # CV R² scores
                ax2.plot(comparison_df['slope_deg'], comparison_df['ridge_cv_r2'], 'o-', label='Ridge')
                ax2.plot(comparison_df['slope_deg'], comparison_df['gbm_cv_r2'], 's-', label='GBM')
                ax2.set_xlabel('Slope (degrees)')
                ax2.set_ylabel('CV R²')
                ax2.set_title('Model performance by slope')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
                
                fig.tight_layout()
                fig.savefig(out_dir / "slope_comparison.png", dpi=150)
                plt.close(fig)
                print(f"Wrote slope comparison plot: {out_dir / 'slope_comparison.png'}")
        
        print(f"Per-slope analysis complete. Results in subdirectories:")
        for slope_deg in unique_slopes:
            slope_dir = out_dir / f"slope_{slope_deg}"
            if slope_dir.exists():
                print(f"  - {slope_dir}")
        
        return 0
    
    else:
        # Original analysis across all slopes
        # Filter out NaN targets if any
        df = df[np.isfinite(df[target])].copy()

        base_features = ["density", "friction", "cohesion_pa", "gravity", "slope_deg"]
        X_df = df[base_features].copy()
        y = df[target].to_numpy()
        X = X_df.to_numpy()

        # Correlations
        corr_df = correlations(df, target)
        corr_path = out_dir / "correlations.csv"
        corr_df.to_csv(corr_path, index=False)

        # Linear model with interactions
        ridge_pipe = build_poly_ridge_pipeline(degree=args.degree)
        ridge_cv_r2 = cv_score(ridge_pipe, X, y)
        ridge_perm = averaged_permutation_importance(ridge_pipe, X, y, base_features, n_repeats=20)
        ridge_perm_path = out_dir / "permutation_importance" / "ridge_poly.csv"
        ridge_perm.to_csv(ridge_perm_path, index=False)

        drop_imp = drop_column_importance(base_features, df, target, degree=args.degree)
        drop_imp_path = out_dir / "permutation_importance" / "ridge_drop_column.csv"
        drop_imp.to_csv(drop_imp_path, index=False)

        # Gradient Boosting (nonlinear)
        gbm = gradient_boosting_model()
        gbm_cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=42)
        gbm_cv_scores = cross_val_score(gbm, X, y, scoring="r2", cv=gbm_cv)
        gbm_cv_r2 = float(np.mean(gbm_cv_scores))

        # Fit once on all data for PDP/SHAP
        gbm.fit(X, y)

        gbm_perm = averaged_permutation_importance(gbm, X, y, base_features, n_repeats=20)
        gbm_perm_path = out_dir / "permutation_importance" / "gbm.csv"
        gbm_perm.to_csv(gbm_perm_path, index=False)

        # PDP plots
        plot_pdp_gbm(gbm, X_df, base_features, out_dir / "pdp", target)

        # Optional SHAP
        try_shap_plots(gbm, X_df, out_dir / "shap")

        # Sobol indices using surrogate
        sobol_results = None
        if args.sobol:
            # Choose surrogate
            if args.sobol_model == "ridge":
                ridge_pipe.fit(X, y)
                surrogate = lambda Z: ridge_pipe.predict(Z)
                model_name = "ridge_poly"
            else:
                # gbm already fit on all data
                surrogate = lambda Z: gbm.predict(Z)
                model_name = "gbm"

            # Define bounds from observed ranges (assume independent uniforms)
            bounds = []
            for col in base_features:
                lo = float(X_df[col].min())
                hi = float(X_df[col].max())
                # guard against degenerate bounds
                if hi == lo:
                    hi = lo + 1e-9
                bounds.append((lo, hi))

            try:
                sob = sobol_indices_saltelli(
                    surrogate,
                    bounds,
                    n_samples=args.sobol_n,
                    seed=args.sobol_seed,
                )
                sob.insert(0, "feature", base_features)
                sob_path = out_dir / f"sobol_{model_name}.csv"
                sob.to_csv(sob_path, index=False)

                # Simple bar plot
                fig, ax = plt.subplots(figsize=(6, 4))
                idx = np.arange(len(base_features))
                ax.bar(idx - 0.2, sob["S1"], width=0.4, label="S1")
                ax.bar(idx + 0.2, sob["ST"], width=0.4, label="ST")
                ax.set_xticks(idx)
                ax.set_xticklabels(base_features, rotation=20)
                ax.set_ylabel("Sobol index")
                ax.set_title(f"Sobol indices ({model_name}, target={target})")
                ax.legend()
                fig.tight_layout()
                fig.savefig(out_dir / f"sobol_{model_name}.png", dpi=150)
                plt.close(fig)
                sobol_results = {model_name: sob.set_index("feature").round(4).to_dict(orient="index")}
            except Exception as e:
                print(f"Sobol computation failed: {e}", file=sys.stderr)

        # Write a concise metrics report
        metrics = {
            "n_runs": int(len(df)),
            "abandoned_count": abandoned_count,
            "target": target,
            "correlations": corr_df.set_index("feature").round(4).to_dict(orient="index"),
            "ridge_poly_cv_r2": round(ridge_cv_r2, 4),
            "ridge_perm_importance": ridge_perm.set_index("feature").round(6).to_dict(orient="index"),
            "ridge_drop_column": drop_imp.set_index("feature").round(6).to_dict(orient="index"),
            "gbm_cv_r2": round(gbm_cv_r2, 4),
            "gbm_perm_importance": gbm_perm.set_index("feature").round(6).to_dict(orient="index"),
            "sobol": sobol_results,
        }
        with open(out_dir / "metrics.json", "w") as f:
            json.dump(metrics, f, indent=2)

        with open(out_dir / "metrics.txt", "w") as f:
            f.write(f"Total runs: {metrics['n_runs']}\n")
            f.write(f"Abandoned simulations (ended before {args.min_time}s or |vel_z| > {args.max_z_vel} m/s): {abandoned_count}\n")
            f.write(f"Target: {target}\n")
            f.write("\nPearson/Spearman correlations (feature -> pearson, spearman):\n")
            for _, row in corr_df.iterrows():
                f.write(f"  {row['feature']}: {row['pearson']:.4f}, {row['spearman']:.4f}\n")
            f.write("\nRidge (poly degree {deg}) CV R^2: {r2:.4f}\n".format(deg=args.degree, r2=ridge_cv_r2))
            f.write("Ridge permutation importance (mean R^2 drop):\n")
            for _, row in ridge_perm.iterrows():
                f.write(f"  {row['feature']}: mean={row['import_mean']:.6f}, std={row['import_std']:.6f}\n")
            f.write("\nRidge drop-column importance (delta R^2):\n")
            for _, row in drop_imp.iterrows():
                f.write(f"  {row['feature']}: delta_r2={row['delta_r2']:.6f}\n")
            f.write("\nGBM CV R^2: {r2:.4f}\n".format(r2=gbm_cv_r2))
            f.write("GBM permutation importance (mean R^2 drop):\n")
            for _, row in gbm_perm.iterrows():
                f.write(f"  {row['feature']}: mean={row['import_mean']:.6f}, std={row['import_std']:.6f}\n")
            if sobol_results is not None:
                f.write("\nSobol indices (first-order S1, total-effect ST):\n")
                for model_name, table in sobol_results.items():
                    f.write(f"  Model: {model_name}\n")
                    for feat, vals in table.items():
                        f.write(f"    {feat}: S1={vals['S1']:.4f}, ST={vals['ST']:.4f}\n")

        print(f"Wrote metrics and plots under: {out_dir}")
        print("Key files:")
        print(f"  - {summary_path}")
        print(f"  - {corr_path}")
        print(f"  - {ridge_perm_path}")
        print(f"  - {drop_imp_path}")
        print(f"  - {gbm_perm_path}")
        print(f"  - {out_dir / 'pdp'} (plots)")
        print(f"  - {out_dir / 'metrics.txt'}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
