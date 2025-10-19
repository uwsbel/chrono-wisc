#!/usr/bin/env python3
import argparse
import json
import os
import sys
from datetime import datetime
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_trials(run_dir: str) -> pd.DataFrame:
    trials_csv = os.path.join(run_dir, 'trials.csv')
    if not os.path.isfile(trials_csv):
        raise FileNotFoundError(f"Missing trials.csv in {run_dir}")
    df = pd.read_csv(trials_csv)
    # Ensure expected columns exist
    if 'trial_index' not in df.columns:
        raise ValueError('trials.csv missing trial_index column')
    # Sort by trial index for time-order analyses
    df = df.sort_values('trial_index').reset_index(drop=True)
    return df


def load_failed_trials(run_dir: str) -> pd.DataFrame:
    path = os.path.join(run_dir, 'failed_trials.jsonl')
    if not os.path.isfile(path):
        return pd.DataFrame(columns=['trial_index', 'error'])
    rows = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                rows.append({'trial_index': obj.get('trial_index'), 'error': obj.get('error', 'unknown')})
            except json.JSONDecodeError:
                continue
    return pd.DataFrame(rows)


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def moving_min(series: pd.Series) -> pd.Series:
    best = []
    current = np.inf
    for v in series.values:
        current = min(current, v)
        best.append(current)
    return pd.Series(best, index=series.index)


def compute_convergence(df: pd.DataFrame, metric: str, out_dir: str):
    if metric not in df.columns:
        raise ValueError(f"Metric '{metric}' not found in trials.csv")
    # Exclude NaNs
    s = df[metric].astype(float)
    x = df['trial_index'].astype(int)
    best_so_far = moving_min(s)

    plt.figure(figsize=(8, 4.5))
    plt.plot(x, s, label=f'{metric}', alpha=0.35, marker='o')
    plt.plot(x, best_so_far, label='best_so_far', color='tab:orange', linewidth=2)
    plt.xlabel('trial_index')
    plt.ylabel(metric)
    plt.title('Convergence')
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'convergence.png'), dpi=150)
    plt.close()

    # Rolling window slope (simple) to indicate improvement trend on best-so-far
    window = max(3, min(15, len(best_so_far)//5))
    if window >= 3:
        # Slope via linear fit over sliding window on best-so-far sequence
        slopes = []
        idxs = []
        for i in range(len(best_so_far) - window + 1):
            y = best_so_far.iloc[i:i+window].values
            t = np.arange(window)
            # slope from least squares
            A = np.vstack([t, np.ones_like(t)]).T
            slope, _ = np.linalg.lstsq(A, y, rcond=None)[0]
            slopes.append(slope)
            idxs.append(i + window//2)
        if slopes:
            plt.figure(figsize=(8, 3.0))
            plt.plot(df['trial_index'].iloc[idxs], slopes, color='tab:green')
            plt.axhline(0, color='k', linewidth=1, alpha=0.5)
            plt.xlabel('trial_index (~center of window)')
            plt.ylabel('best_so_far slope')
            plt.title(f'Convergence slope (window={window})')
            plt.grid(True, alpha=0.25)
            plt.tight_layout()
            plt.savefig(os.path.join(out_dir, 'convergence_slope.png'), dpi=150)
            plt.close()


def spearman_corr(a: pd.Series, b: pd.Series) -> float:
    # Simple Spearman correlation without scipy
    ra = a.rank(method='average')
    rb = b.rank(method='average')
    ca = ra - ra.mean()
    cb = rb - rb.mean()
    denom = np.sqrt((ca**2).sum() * (cb**2).sum())
    return float((ca * cb).sum() / denom) if denom != 0 else 0.0


def analyze_dependencies(df: pd.DataFrame, metric: str, param_cols: List[str], out_dir: str):
    # Compute Spearman correlations param -> metric
    corr_rows = []
    y = df[metric].astype(float)
    for p in param_cols:
        try:
            corr = spearman_corr(df[p].astype(float), y)
        except Exception:
            corr = np.nan
        corr_rows.append({'parameter': p, 'spearman_to_metric': corr})
    corr_df = pd.DataFrame(corr_rows).sort_values('spearman_to_metric', key=lambda s: s.abs(), ascending=False)
    corr_df.to_csv(os.path.join(out_dir, 'param_metric_spearman.csv'), index=False)

    # Bar plot of correlations
    plt.figure(figsize=(8, max(3.0, 0.3 * len(param_cols))))
    plt.barh(corr_df['parameter'], corr_df['spearman_to_metric'], color='tab:blue')
    plt.xlabel('Spearman correlation with metric (lower is better)')
    plt.title('Parameter dependencies (rank correlation)')
    plt.grid(True, axis='x', alpha=0.25)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'param_metric_spearman.png'), dpi=150)
    plt.close()

    # Pairwise parameter correlation heatmap (Spearman)
    corr_mat = pd.DataFrame(index=param_cols, columns=param_cols, dtype=float)
    for i, a in enumerate(param_cols):
        for b in param_cols:
            try:
                corr_mat.loc[a, b] = spearman_corr(df[a].astype(float), df[b].astype(float))
            except Exception:
                corr_mat.loc[a, b] = np.nan
    corr_mat.to_csv(os.path.join(out_dir, 'param_param_spearman.csv'))

    fig, ax = plt.subplots(figsize=(0.6*len(param_cols)+2, 0.6*len(param_cols)+2))
    im = ax.imshow(corr_mat.values.astype(float), vmin=-1, vmax=1, cmap='coolwarm')
    ax.set_xticks(range(len(param_cols)))
    ax.set_yticks(range(len(param_cols)))
    ax.set_xticklabels(param_cols, rotation=90)
    ax.set_yticklabels(param_cols)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label='Spearman')
    ax.set_title('Parameter-Parameter Spearman Correlation')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'param_param_spearman.png'), dpi=150)
    plt.close()


def binned_trend(x: pd.Series, y: pd.Series, bins: int = 12) -> Tuple[np.ndarray, np.ndarray]:
    xv = x.values.astype(float)
    yv = y.values.astype(float)
    if len(np.unique(xv)) < 2:
        return np.array([]), np.array([])
    edges = np.linspace(xv.min(), xv.max(), bins + 1)
    centers = 0.5 * (edges[1:] + edges[:-1])
    idx = np.digitize(xv, edges) - 1
    means = []
    for b in range(bins):
        mask = (idx == b)
        if mask.any():
            means.append(np.nanmean(yv[mask]))
        else:
            means.append(np.nan)
    return centers, np.array(means)


def analyze_trends(df: pd.DataFrame, metric: str, param_cols: List[str], out_dir: str):
    y = df[metric].astype(float)
    ncols = 3
    nrows = int(np.ceil(len(param_cols) / ncols))
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(4*ncols, 3*nrows))
    axes = np.array(axes).reshape(-1)
    for i, p in enumerate(param_cols):
        ax = axes[i]
        x = df[p].astype(float)
        ax.scatter(x, y, s=14, alpha=0.45)
        c, m = binned_trend(x, y)
        if len(c) > 0:
            ax.plot(c, m, color='tab:orange', linewidth=2)
        ax.set_title(p)
        ax.set_xlabel(p)
        ax.set_ylabel(metric)
        ax.grid(True, alpha=0.2)
    # Hide extra axes
    for j in range(i+1, len(axes)):
        axes[j].axis('off')
    fig.suptitle('Parameter vs Metric (with binned trend)')
    fig.tight_layout(rect=[0, 0.03, 1, 0.98])
    fig.savefig(os.path.join(out_dir, 'param_trends.png'), dpi=150)
    plt.close(fig)


def linear_cv_r2(X: np.ndarray, y: np.ndarray, k: int = 5, seed: int = 0) -> float:
    n = len(y)
    if n < k or k < 2:
        # fallback: simple train-test split 80/20
        k = 2
    rng = np.random.default_rng(seed)
    idx = np.arange(n)
    rng.shuffle(idx)
    fold_sizes = [(n // k) + (1 if x < n % k else 0) for x in range(k)]
    current = 0
    r2s = []
    for fs in fold_sizes:
        test_idx = idx[current:current+fs]
        train_idx = np.setdiff1d(idx, test_idx, assume_unique=False)
        current += fs
        Xtr, ytr = X[train_idx], y[train_idx]
        Xte, yte = X[test_idx], y[test_idx]
        # Add intercept
        Xtr_i = np.c_[Xtr, np.ones(len(Xtr))]
        Xte_i = np.c_[Xte, np.ones(len(Xte))]
        try:
            beta, *_ = np.linalg.lstsq(Xtr_i, ytr, rcond=None)
            yhat = Xte_i @ beta
            ss_res = float(np.sum((yte - yhat) ** 2))
            ss_tot = float(np.sum((yte - np.mean(yte)) ** 2))
            r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
            if np.isfinite(r2):
                r2s.append(r2)
        except Exception:
            continue
    return float(np.mean(r2s)) if r2s else float('nan')


def analyze_identifiability(df: pd.DataFrame, metric: str, param_cols: List[str], out_dir: str):
    # Identify top 10% best trials (lower is better)
    y = df[metric].astype(float)
    n = len(df)
    k = max(3, int(np.ceil(0.10 * n)))
    top_idx = y.nsmallest(k).index
    overall_stats = df[param_cols].astype(float).describe().loc[['mean', 'std']]
    top_stats = df.loc[top_idx, param_cols].astype(float).describe().loc[['mean', 'std']]
    # Fold reduction in std
    fold_reduction = (overall_stats.loc['std'] / top_stats.loc['std']).replace([np.inf, -np.inf], np.nan)
    ident_df = pd.DataFrame({
        'overall_std': overall_stats.loc['std'],
        'top10pct_std': top_stats.loc['std'],
        'std_fold_reduction': fold_reduction
    })
    ident_df.to_csv(os.path.join(out_dir, 'identifiability_std_reduction.csv'))

    # Plot fold reduction
    srt = ident_df['std_fold_reduction'].sort_values(ascending=False)
    plt.figure(figsize=(8, max(3.0, 0.3 * len(param_cols))))
    plt.barh(srt.index, srt.values, color='tab:purple')
    plt.xlabel('Std reduction (overall / top 10%)')
    plt.title('Parameter Identifiability (lower spread among best trials)')
    plt.grid(True, axis='x', alpha=0.25)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'identifiability_std_reduction.png'), dpi=150)
    plt.close()

    # Simple model predictiveness (data usefulness): cross-validated R^2
    X = df[param_cols].astype(float).values
    r2 = linear_cv_r2(X, y.values, k=5, seed=0)
    with open(os.path.join(out_dir, 'model_predictiveness.txt'), 'w') as f:
        f.write(f'Linear model 5-fold CV R^2: {r2:.4f}\n')


def analyze_failures(df: pd.DataFrame, failed_df: pd.DataFrame, metric: str, out_dir: str):
    total_trials = len(df)
    # Heuristics for failure: explicitly logged OR metric sentinel 50 OR NaN
    failed_indices_logged = set(map(int, failed_df['trial_index'].dropna().astype(int).tolist())) if not failed_df.empty else set()
    failed_metric_sentinel = set(df.loc[(df[metric] >= 50) | (~np.isfinite(df[metric])) | (pd.isna(df[metric])), 'trial_index'].astype(int).tolist())
    failed_indices = sorted(failed_indices_logged.union(failed_metric_sentinel))
    n_failed = len(failed_indices)
    n_success = total_trials - n_failed

    # Save summary
    with open(os.path.join(out_dir, 'failures_summary.txt'), 'w') as f:
        f.write(f'Total trials: {total_trials}\n')
        f.write(f'Failed trials (from logs or sentinel): {n_failed}\n')
        f.write(f'Successful trials: {n_success}\n')
        f.write(f'Failure rate: {n_failed/total_trials:.3%}\n')
        if n_failed:
            f.write('Failed trial indices: ' + ', '.join(map(str, failed_indices)) + '\n')

    # Plot failure positions over trial index
    plt.figure(figsize=(8, 2.5))
    plt.scatter(df['trial_index'], np.zeros_like(df['trial_index']), s=20, c='tab:green', label='success', alpha=0.5)
    if n_failed:
        mask_fail = df['trial_index'].isin(failed_indices)
        plt.scatter(df.loc[mask_fail, 'trial_index'], np.zeros(mask_fail.sum()), s=28, c='tab:red', label='failed')
    plt.yticks([])
    plt.xlabel('trial_index')
    plt.title('Failed vs Successful Trials')
    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'failures_over_time.png'), dpi=150)
    plt.close()


def _fit_surrogate_predictor(df: pd.DataFrame, metric: str, param_cols: List[str], seed: int = 0):
    """Fit a simple surrogate model and return a predictor function.

    Preference order:
    - Gaussian Process Regressor (if scikit-learn available)
    - Linear regression (least squares) fallback

    Returns a tuple (predict_fn, model_name), where predict_fn accepts an
    ndarray of shape (n_samples, n_features) and returns predictions of shape (n_samples,).
    """
    # Use only successful trials (metric < 50 and finite)
    metric_vals = pd.to_numeric(df[metric], errors='coerce')
    mask_ok = metric_vals.apply(np.isfinite) & (metric_vals < 50)
    X = df.loc[mask_ok, param_cols].astype(float).values
    y = metric_vals.loc[mask_ok].astype(float).values
    if len(y) < 5:
        return None, 'insufficient_data'

    sk = try_import_sklearn()
    if sk is not None:
        try:
            GPR = sk['GaussianProcessRegressor']
            RBF = sk['RBF']
            WhiteKernel = sk['WhiteKernel']
            # Simple GP with ARD RBF + white noise
            kernel = 1.0 * RBF(length_scale=np.ones(X.shape[1]), length_scale_bounds=(1e-2, 1e3)) \
                     + WhiteKernel(noise_level=1.0, noise_level_bounds=(1e-6, 1e1))
            gp = GPR(kernel=kernel, alpha=0.0, normalize_y=True, n_restarts_optimizer=2, random_state=seed)
            gp.fit(X, y)

            def predict_fn(Xnew: np.ndarray) -> np.ndarray:
                mu = gp.predict(Xnew, return_std=False)
                return mu.astype(float)

            return predict_fn, 'GaussianProcessRegressor'
        except Exception:
            # Fall back to linear below
            pass

    # Linear regression fallback (with intercept)
    try:
        X_i = np.c_[X, np.ones(len(X))]
        beta, *_ = np.linalg.lstsq(X_i, y, rcond=None)

        def predict_fn(Xnew: np.ndarray) -> np.ndarray:
            Xn_i = np.c_[Xnew, np.ones(len(Xnew))]
            return (Xn_i @ beta).astype(float)

        return predict_fn, 'LinearLeastSquares'
    except Exception:
        return None, 'fit_failed'


def generate_ice_plots(df: pd.DataFrame, metric: str, param_cols: List[str], out_dir: str,
                       n_curves: int = 30, grid_points: int = 30, seed: int = 0):
    """Generate ICE (Individual Conditional Expectation) plots for each parameter.

    For each parameter p, we draw up to n_curves ICE lines by sampling rows in df,
    varying p over a grid within its observed range while holding other parameters
    fixed at that row's values. We overlay the PDP (mean of ICE curves).
    """
    ice_dir = os.path.join(out_dir, 'ice')
    ensure_dir(ice_dir)

    predict_fn, model_name = _fit_surrogate_predictor(df, metric, param_cols, seed=seed)
    if predict_fn is None:
        with open(os.path.join(ice_dir, 'README.txt'), 'w') as f:
            if model_name == 'insufficient_data':
                f.write('Not enough successful trials to fit a surrogate for ICE.\n')
            elif model_name == 'fit_failed':
                f.write('Could not fit surrogate model for ICE.\n')
            else:
                f.write('ICE skipped: surrogate unavailable.\n')
        print('ICE: surrogate not available; skipping ICE generation.', file=sys.stderr)
        return

    # Warn if GP is not used
    if model_name != 'GaussianProcessRegressor':
        warn_msg = f"WARNING: ICE surrogate uses {model_name}, not GaussianProcessRegressor. Results may be less faithful to BO surrogate.\n"
        with open(os.path.join(ice_dir, 'warnings.txt'), 'a') as wf:
            wf.write(warn_msg)
        print(warn_msg.strip(), file=sys.stderr)

    rng = np.random.default_rng(seed)
    # Use only successful trials for sampling
    df_ok = filter_successful(df, metric)
    if df_ok.empty:
        with open(os.path.join(ice_dir, 'README.txt'), 'w') as f:
            f.write('No successful trials; cannot compute ICE.\n')
        return

    # Sample rows for ICE curves
    n = len(df_ok)
    idx_all = np.arange(n)
    if n > n_curves:
        idx_sel = rng.choice(idx_all, size=n_curves, replace=False)
    else:
        idx_sel = idx_all

    X_base = df_ok[param_cols].astype(float).values

    for p in param_cols:
        # Grid over observed range (use finite values only)
        xv = pd.to_numeric(df_ok[p], errors='coerce').dropna().values.astype(float)
        if xv.size == 0:
            continue
        x_min, x_max = float(np.min(xv)), float(np.max(xv))
        if not np.isfinite(x_min) or not np.isfinite(x_max) or x_min == x_max:
            # Degenerate range; skip
            continue
        grid = np.linspace(x_min, x_max, grid_points)

        # Build ICE curves
        curves = []
        for i in idx_sel:
            base = X_base[i].copy()
            # For every grid value, create a point with p replaced
            Xg = np.tile(base, (grid_points, 1))
            p_idx = param_cols.index(p)
            Xg[:, p_idx] = grid
            yhat = predict_fn(Xg)
            curves.append(yhat)
        if not curves:
            continue
        C = np.vstack(curves)  # shape (n_curves, grid_points)
        pdp = np.nanmean(C, axis=0)

        # Plot ICE curves with PDP overlay
        plt.figure(figsize=(6.5, 4))
        for k in range(C.shape[0]):
            plt.plot(grid, C[k, :], color='tab:blue', alpha=0.18, linewidth=1)
        plt.plot(grid, pdp, color='tab:orange', linewidth=2.5, label='PDP (mean)')
        plt.xlabel(p)
        plt.ylabel(metric)
        plt.title(f'ICE for {p}  —  surrogate: {model_name}')
        plt.grid(True, alpha=0.25)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(ice_dir, f'ice_{p}.png'), dpi=150)
        plt.close()


def write_summary_md(df: pd.DataFrame, metric: str, out_dir: str):
    best_idx = int(df[metric].idxmin())
    best_row = df.loc[best_idx]
    best_val = float(best_row[metric])
    best_trial = int(best_row['trial_index'])
    last10 = df.tail(10)[metric].values
    if len(last10) > 1:
        improvement = (np.min(last10) - np.min(df[metric].values)).item()
    else:
        improvement = 0.0
    content = []
    content.append(f'# BO Run Analysis Summary ({datetime.utcnow().isoformat(timespec="seconds")}Z)')
    content.append('')
    content.append(f'- Metric: `{metric}` (lower is better)')
    content.append(f'- Total trials: {len(df)}')
    content.append(f'- Best metric: {best_val:.6g} at trial_index {best_trial}')
    content.append(f'- Best parameters:')
    for c in df.columns:
        if c in ['timestamp', 'trial_index', metric, 't_elapsed']:  # skip known non-param columns
            continue
        content.append(f'  - {c}: {best_row[c]}')
    content.append(f'- Last-10 best-vs-global improvement delta: {improvement:.6g}')
    content.append('')
    content.append('See generated plots and CSVs in this folder for details.')
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write('\n'.join(content) + '\n')


def infer_param_columns(df: pd.DataFrame, metric: str) -> List[str]:
    exclude = {'timestamp', 'trial_index', metric, 't_elapsed', 'rms_error', 'average_power'}
    # Parameters are the remaining numeric columns
    param_cols = []
    for c in df.columns:
        if c in exclude:
            continue
        if pd.api.types.is_numeric_dtype(df[c]):
            param_cols.append(c)
    return param_cols


def filter_successful(df: pd.DataFrame, metric: str) -> pd.DataFrame:
    """Return only successful trials: finite metric and metric < 50.

    This removes failed/sentinel trials so analyses aren't dominated by them.
    """
    metric_vals = pd.to_numeric(df[metric], errors='coerce')
    mask_ok = metric_vals.apply(np.isfinite) & (metric_vals < 50)
    return df.loc[mask_ok].copy()


def try_import_sklearn():
    try:
        from sklearn.gaussian_process import GaussianProcessRegressor
        from sklearn.gaussian_process.kernels import RBF, WhiteKernel
        from sklearn.preprocessing import StandardScaler
        from sklearn.pipeline import Pipeline
        from sklearn.metrics import r2_score
        return {
            'GaussianProcessRegressor': GaussianProcessRegressor,
            'RBF': RBF,
            'WhiteKernel': WhiteKernel,
            'StandardScaler': StandardScaler,
            'Pipeline': Pipeline,
            'r2_score': r2_score,
        }
    except Exception:
        return None


def evaluate_gp_like_quality(df: pd.DataFrame, metric: str, param_cols: List[str], out_dir: str, k: int = 5, seed: int = 0):
    sk = try_import_sklearn()
    X_full = df[param_cols].astype(float).values
    y_full = df[metric].astype(float).values
    # Filter out failed/sentinel trials and NaNs
    mask_ok = np.isfinite(y_full) & (y_full < 50)
    X = X_full[mask_ok]
    y = y_full[mask_ok]
    # Downsample for speed if very large
    max_n = 800
    if len(y) > max_n:
        rng = np.random.default_rng(seed)
        sel = rng.choice(len(y), size=max_n, replace=False)
        X = X[sel]
        y = y[sel]
    n = len(y)
    if n < 10:
        with open(os.path.join(out_dir, 'gp_quality_summary.txt'), 'w') as f:
            f.write('Not enough trials to evaluate GP quality.\n')
        return

    if sk is None:
        # Fallback to linear CV R^2 and note limitation
        r2 = linear_cv_r2(X, y, k=min(5, max(2, n//5)), seed=seed)
        with open(os.path.join(out_dir, 'gp_quality_summary.txt'), 'w') as f:
            f.write('scikit-learn not available; falling back to linear model as proxy.\n')
            f.write(f'Linear model CV R^2: {r2:.4f}\n')
        return

    # Build a simple GP pipeline: standardize X and y implicitly via normalize_y in GPR
    GPR = sk['GaussianProcessRegressor']
    RBF = sk['RBF']
    WhiteKernel = sk['WhiteKernel']
    Pipeline = sk['Pipeline']
    r2_score = sk['r2_score']

    kernel = 1.0 * RBF(length_scale=np.ones(X.shape[1]), length_scale_bounds=(1e-2, 1e3)) + WhiteKernel(noise_level=1.0, noise_level_bounds=(1e-6, 1e1))
    # Standardize inputs
    model = Pipeline([
        ('scale', None),  # placeholder; scikit-learn Pipeline allows None
        ('gp', GPR(kernel=kernel, alpha=0.0, normalize_y=True, n_restarts_optimizer=2, random_state=seed))
    ])

    # K-fold CV
    rng = np.random.default_rng(seed)
    idx = np.arange(n)
    rng.shuffle(idx)
    k = min(k, n)
    fold_sizes = [(n // k) + (1 if x < n % k else 0) for x in range(k)]
    current = 0
    r2s, rmses, nlpds = [], [], []
    y_true_all, y_pred_all, y_std_all = [], [], []
    for fs in fold_sizes:
        test_idx = idx[current:current+fs]
        train_idx = np.setdiff1d(idx, test_idx, assume_unique=False)
        current += fs
        Xtr, ytr = X[train_idx], y[train_idx]
        Xte, yte = X[test_idx], y[test_idx]
        mdl = model
        try:
            mdl.fit(Xtr, ytr)
            mu, std = mdl.named_steps['gp'].predict(Xte, return_std=True)
        except Exception:
            # In case of optimization failures, use simple defaults
            mu = np.full_like(yte, np.nan, dtype=float)
            std = np.full_like(yte, np.nan, dtype=float)

        # Metrics
        mask = np.isfinite(mu)
        if not np.any(mask):
            continue
        yte_m = yte[mask]
        mu_m = mu[mask]
        std_m = std[mask]
        r2s.append(r2_score(yte_m, mu_m))
        rmses.append(float(np.sqrt(np.mean((yte_m - mu_m)**2))))
        # Negative log predictive density under Normal with predicted std
        eps = 1e-9
        nll = 0.5*np.log(2*np.pi*(std_m**2 + eps)) + 0.5*((yte_m - mu_m)**2)/((std_m**2)+eps)
        nlpds.append(float(np.mean(nll)))
        y_true_all.append(yte_m)
        y_pred_all.append(mu_m)
        y_std_all.append(std_m)

    if not r2s:
        with open(os.path.join(out_dir, 'gp_quality_summary.txt'), 'w') as f:
            f.write('GP evaluation failed during CV.\n')
        return

    y_true = np.concatenate(y_true_all)
    y_pred = np.concatenate(y_pred_all)
    y_std = np.concatenate(y_std_all)

    with open(os.path.join(out_dir, 'gp_quality_summary.txt'), 'w') as f:
        f.write(f'GP-like (RBF) {k}-fold CV metrics:\n')
        f.write(f'- R^2: {np.mean(r2s):.4f} ± {np.std(r2s):.4f}\n')
        f.write(f'- RMSE: {np.mean(rmses):.4f} ± {np.std(rmses):.4f}\n')
        f.write(f'- NLPD: {np.mean(nlpds):.4f} ± {np.std(nlpds):.4f}\n')

    # Predicted vs true
    plt.figure(figsize=(5.5, 5))
    lims = [np.nanmin([y_true, y_pred]), np.nanmax([y_true, y_pred])]
    plt.plot(lims, lims, 'k--', alpha=0.5, label='ideal')
    plt.errorbar(y_true, y_pred, yerr=y_std, fmt='o', alpha=0.35, markersize=3, ecolor='tab:orange', label='CV preds')
    plt.xlabel('True metric')
    plt.ylabel('Predicted metric')
    plt.title('GP-like CV: True vs Predicted')
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'gp_pred_vs_true.png'), dpi=150)
    plt.close()

    # Calibration: standardized residuals should be ~N(0,1)
    z = (y_true - y_pred) / (y_std + 1e-9)
    plt.figure(figsize=(6, 3.5))
    plt.hist(z, bins=30, density=True, alpha=0.7, color='tab:blue')
    plt.title('GP-like CV: Standardized residuals')
    plt.xlabel('(y - mu) / sigma_pred')
    plt.grid(True, alpha=0.25)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'gp_residuals_calibration.png'), dpi=150)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Analyze an Ax Bayesian Optimization run directory.')
    parser.add_argument('run_dir', help='Path to run directory containing trials.csv and failed_trials.jsonl')
    parser.add_argument('--metric', default='composite_metric', help='Metric column to analyze (lower is better)')
    parser.add_argument('--evaluate-gp', action='store_true', help='Evaluate surrogate predictiveness via GP-like CV if available')
    parser.add_argument('--ice', action='store_true', help='Generate ICE plots using a surrogate (GP if available, else linear)')
    args = parser.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    if not os.path.isdir(run_dir):
        print(f"Run directory not found: {run_dir}", file=sys.stderr)
        sys.exit(1)

    out_dir = os.path.join(run_dir, 'analysis')
    ensure_dir(out_dir)

    df = load_trials(run_dir)
    failed_df = load_failed_trials(run_dir)
    if args.metric not in df.columns:
        print(f"Metric '{args.metric}' not found in trials.csv. Available: {list(df.columns)}", file=sys.stderr)
        sys.exit(2)

    param_cols = infer_param_columns(df, args.metric)
    if not param_cols:
        print('No parameter columns inferred; aborting.', file=sys.stderr)
        sys.exit(3)

    # Ensure numeric types for safety
    for c in [args.metric] + param_cols:
        df[c] = pd.to_numeric(df[c], errors='coerce')

    # Filter out failed/sentinel trials for analyses (keep failures analysis on full df)
    df_ok = filter_successful(df, args.metric)
    with open(os.path.join(out_dir, 'filtering_summary.txt'), 'w') as f:
        f.write(f"Total trials (raw): {len(df)}\n")
        f.write(f"Successful trials used (metric < 50 & finite): {len(df_ok)}\n")
        f.write(f"Filtered out: {len(df) - len(df_ok)}\n")

    # Analyses (successful trials only)
    compute_convergence(df_ok, args.metric, out_dir)
    analyze_dependencies(df_ok, args.metric, param_cols, out_dir)
    analyze_trends(df_ok, args.metric, param_cols, out_dir)
    analyze_identifiability(df_ok, args.metric, param_cols, out_dir)
    analyze_failures(df, failed_df, args.metric, out_dir)
    write_summary_md(df_ok, args.metric, out_dir)
    # ICE plots (uses surrogate fitted on successful trials) if requested
    if args.ice:
        generate_ice_plots(df, args.metric, param_cols, out_dir)

    if args.evaluate_gp:
        evaluate_gp_like_quality(df, args.metric, param_cols, out_dir, k=5, seed=0)

    print(f"Analysis artifacts written to: {out_dir}")


if __name__ == '__main__':
    main()
