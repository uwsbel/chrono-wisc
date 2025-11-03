import argparse
import os
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import KFold, cross_val_score
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler


# ----- Problem definition (PullTest single-wheel new params) -----


def compute_int_bounds(lower_m: float, upper_m: float, spacing: float) -> Tuple[int, int]:
    lb = int(np.ceil(lower_m / spacing))
    ub = int(np.floor(upper_m / spacing))
    if lb > ub:
        raise ValueError(f"Invalid bounds after scaling: [{lb}, {ub}] for spacing={spacing}")
    return lb, ub


@dataclass
class VarSpec:
    name: str
    kind: str  # 'int', 'float', or 'cat'
    bounds: Tuple[float, float] = None  # inclusive for int, [low, high] for float
    values: Tuple[int, ...] = None      # for categorical


def build_varspecs_pull_single(particle_spacing: float) -> List[VarSpec]:
    # Integer ranges derived from meters and spacing (match PullTest_global_single.py)
    # rad_outer is discrete multiples of spacing in [0.05, 0.14] m
    rad_lb, rad_ub = compute_int_bounds(0.05, 0.14, particle_spacing)
    th_lb, th_ub = 45, 135

    return [
        VarSpec("rad_outer", "int", (rad_lb, rad_ub)),
        VarSpec("w_by_r", "float", (0.7, 1.3)),
        VarSpec("what_percent_is_grouser", "float", (0.0, 0.3)),
        VarSpec("g_density", "int", (2, 16)),
        VarSpec("fan_theta_deg", "int", (th_lb, th_ub)),
    ]


"""
Only support the PullTest single-wheel NEW parameterization with these exact
column names emitted by PullTest_global_single.py.
"""

# ----- Data loading and cleaning -----

FEATURES_PULL_SINGLE = [
    "rad_outer",
    "w_by_r",
    "what_percent_is_grouser",
    "g_density",
    "fan_theta_deg",
]


def load_filtered_pull(csv_path: str, filter_penalties: bool = True):
    df = pd.read_csv(csv_path)
    # Require exact schema from PullTest_global_single.py
    required = ["metric", "total_time_to_reach", "particle_spacing"] + FEATURES_PULL_SINGLE
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise SystemExit(f"CSV missing required columns: {missing}")

    # Cast numerics for target and time
    target_col = "metric"
    time_col = "total_time_to_reach"
    df[target_col] = pd.to_numeric(df[target_col], errors="coerce")
    df[time_col] = pd.to_numeric(df[time_col], errors="coerce")

    df = df.replace([np.inf, -np.inf], np.nan)
    # Drop rows without valid target/time
    df = df.dropna(subset=[target_col, time_col])

    # Remove failed simulations if filtering is enabled
    # Failures are encoded as metric >= 500 or total_time_to_reach == 12.0 (2*tend)
    if filter_penalties:
        df = df[(df[target_col] < 500) & (df[time_col] != 12.0)]

    # Particle spacing (constant per run)
    if not df["particle_spacing"].isna().all():
        spacing = pd.to_numeric(df["particle_spacing"], errors="coerce").dropna().mode().iloc[0]
    else:
        spacing = 0.01

    # Build a normalized view with canonical feature names
    # Keep rows without NaNs in the features
    df = df.dropna(subset=FEATURES_PULL_SINGLE)

    # Cast integer-like features
    for c in ["rad_outer", "g_density", "fan_theta_deg"]:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce").astype(int)

    return df, float(spacing), target_col


def count_failed_from_jsonl(path: str) -> int:
    """Count unique failed trial indices from a failed_trials.jsonl file.
    If the file is missing or unreadable, return 0.
    """
    try:
        import json
        if not os.path.isfile(path):
            return 0
        uniq = set()
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                    if isinstance(rec, dict) and "trial_index" in rec:
                        uniq.add(int(rec["trial_index"]))
                except Exception:
                    # Skip malformed lines
                    pass
        return len(uniq)
    except Exception:
        return 0


# ----- Sobol sampling and estimators (Saltelli S1, Jansen ST) -----


def sample_unit_saltelli(n: int, d: int, rng: np.random.Generator) -> Tuple[np.ndarray, np.ndarray]:
    A = rng.random((n, d), dtype=np.float64)
    B = rng.random((n, d), dtype=np.float64)
    return A, B


def unit_to_domain(A: np.ndarray, specs: List[VarSpec]) -> np.ndarray:
    X = np.empty_like(A)
    for j, spec in enumerate(specs):
        if spec.kind == "float":
            lo, hi = spec.bounds
            X[:, j] = lo + (hi - lo) * A[:, j]
        elif spec.kind == "int":
            lo, hi = spec.bounds
            X[:, j] = np.floor(lo + (hi - lo + 1) * A[:, j]).clip(lo, hi)
        elif spec.kind == "cat":
            vals = np.array(spec.values)
            idx = np.floor(len(vals) * A[:, j]).astype(int).clip(0, len(vals) - 1)
            X[:, j] = vals[idx]
        else:
            raise ValueError(f"Unknown var kind: {spec.kind}")
    return X


def saltelli_indices(predict_fn, specs: List[VarSpec], n: int, rng: np.random.Generator):
    d = len(specs)
    A_u, B_u = sample_unit_saltelli(n, d, rng)
    A = unit_to_domain(A_u, specs)
    B = unit_to_domain(B_u, specs)

    fA = predict_fn(A)
    fB = predict_fn(B)

    V = np.var(fA, ddof=1)
    if V <= 0:
        raise ValueError("Non-positive variance encountered in surrogate outputs; cannot compute Sobol indices.")

    S1 = np.zeros(d)
    ST = np.zeros(d)
    for i in range(d):
        A_Bi = A.copy()
        A_Bi[:, i] = B[:, i]
        fABi = predict_fn(A_Bi)
        S1[i] = np.mean(fB * (fABi - fA)) / V
        ST[i] = 0.5 * np.mean((fABi - fA) ** 2) / V

    names = [s.name for s in specs]
    return S1, ST, names


# ----- Main CLI -----


def main():
    parser = argparse.ArgumentParser(description="Sobol (Saltelli/Jansen) global sensitivity for PullTest new params via surrogate.")
    parser.add_argument(
        "--csv",
        default="",  # recommend specifying explicitly
        help="Path to trials.csv from PullTest_global_single.py runs",
    )
    parser.add_argument("--n", type=int, default=2000, help="Base sample size for Saltelli (A and B)")
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    parser.add_argument("--no-filter-penalties", action="store_true", help="Keep penalty trials (metric>=500 or time==12)")
    parser.add_argument("--plot", action="store_true", help="Save a bar plot of S1 and ST")
    parser.add_argument("--outdir", default=None, help="Output directory (default: <run_dir>/analysis)")
    parser.add_argument("--cv-folds", type=int, default=5, help="K-fold CV folds to evaluate surrogate quality")
    parser.add_argument("--model", choices=["rf", "xgb", "gp"], default="rf", help="Surrogate model to use")
    parser.add_argument("--failed-log", default=None, help="Path to failed_trials.jsonl (default: alongside CSV)")
    # XGBoost options
    parser.add_argument("--xgb-estimators", type=int, default=800, help="XGBoost number of trees")
    parser.add_argument("--xgb-depth", type=int, default=8, help="XGBoost max depth")
    parser.add_argument("--xgb-learning-rate", type=float, default=0.05, help="XGBoost learning rate")
    parser.add_argument("--xgb-subsample", type=float, default=0.8, help="XGBoost subsample ratio")
    parser.add_argument("--xgb-colsample-bytree", type=float, default=0.8, help="XGBoost colsample by tree")
    # GP options
    parser.add_argument("--gp-max-samples", type=int, default=1000, help="Max training samples for GP (subsamples if exceeded)")
    parser.add_argument("--gp-n-restarts", type=int, default=2, help="GP optimizer restarts")

    args = parser.parse_args()
    filter_penalties = not args.no_filter_penalties

    if not args.csv:
        raise SystemExit("Please provide --csv pointing to your PullTest trials.csv")

    # Load data
    df, spacing, target_col = load_filtered_pull(args.csv, filter_penalties=filter_penalties)
    if df.empty:
        raise SystemExit("No usable rows after filtering.")

    # Features/target
    X = df[FEATURES_PULL_SINGLE].to_numpy(dtype=float)
    y = df[target_col].to_numpy(dtype=float)

    # Build domain spec from spacing
    specs = build_varspecs_pull_single(spacing)

    # Construct surrogate based on selection
    model_name = args.model

    if model_name == "rf":
        estimator = RandomForestRegressor(
            n_estimators=600,
            max_depth=None,
            min_samples_split=2,
            min_samples_leaf=1,
            n_jobs=-1,
            random_state=args.seed,
        )
    elif model_name == "xgb":
        try:
            from xgboost import XGBRegressor  # type: ignore
        except Exception as e:
            raise SystemExit(
                "XGBoost is not installed. Please install with: pip install xgboost"
            ) from e
        estimator = XGBRegressor(
            n_estimators=args.xgb_estimators,
            max_depth=args.xgb_depth,
            learning_rate=args.xgb_learning_rate,
            subsample=args.xgb_subsample,
            colsample_bytree=args.xgb_colsample_bytree,
            reg_lambda=1.0,
            reg_alpha=0.0,
            n_jobs=-1,
            random_state=args.seed,
            tree_method="hist",
        )
    elif model_name == "gp":
        from sklearn.gaussian_process import GaussianProcessRegressor
        from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C, WhiteKernel

        kernel = C(1.0, (1e-3, 1e3)) * RBF(length_scale=np.ones(X.shape[1]), length_scale_bounds=(1e-2, 1e2)) + WhiteKernel(noise_level=1.0, noise_level_bounds=(1e-6, 1e1))
        gpr = GaussianProcessRegressor(
            kernel=kernel,
            normalize_y=True,
            n_restarts_optimizer=args.gp_n_restarts,
            random_state=args.seed,
        )
        estimator = Pipeline([("scaler", StandardScaler(with_mean=True, with_std=True)), ("gpr", gpr)])
    else:
        raise SystemExit(f"Unknown model: {model_name}")

    # K-fold CV to assess surrogate quality (R^2)
    kf = KFold(n_splits=args.cv_folds, shuffle=True, random_state=args.seed)

    # For GP, subsample for CV if dataset is large
    X_cv = X
    y_cv = y
    gp_note = None
    if model_name == "gp" and len(X) > args.gp_max_samples:
        rng = np.random.default_rng(args.seed)
        idx = rng.choice(len(X), size=args.gp_max_samples, replace=False)
        X_cv = X[idx]
        y_cv = y[idx]
        gp_note = f"GP used subsample {len(X_cv)}/{len(X)} for CV and fitting"

    cv_scores = cross_val_score(estimator, X_cv, y_cv, cv=kf, scoring="r2")
    r2_cv_mean = float(np.mean(cv_scores))
    r2_cv_std = float(np.std(cv_scores, ddof=1)) if len(cv_scores) > 1 else 0.0

    # Refit on final training set (subsample for GP if needed)
    if model_name == "gp" and len(X) > args.gp_max_samples:
        estimator.fit(X_cv, y_cv)
    else:
        estimator.fit(X, y)

    # Prediction wrapper expects 2D array of mixed types
    def predict_fn(arr_2d: np.ndarray) -> np.ndarray:
        return estimator.predict(arr_2d)

    # Saltelli sampling / Sobol indices
    rng = np.random.default_rng(args.seed)
    saltelli_n = args.n
    if model_name == "gp" and args.n > 1000:
        print("Warning: reducing Saltelli n to 1000 for GP to keep runtime reasonable.")
        saltelli_n = 1000
    S1, ST, names = saltelli_indices(predict_fn, specs, n=saltelli_n, rng=rng)

    # Prepare output directory
    if args.outdir is None:
        run_dir = os.path.dirname(os.path.abspath(args.csv))
        outdir = os.path.join(run_dir, "analysis")
    else:
        outdir = args.outdir
    os.makedirs(outdir, exist_ok=True)

    # Save CSV
    res = pd.DataFrame({"param": names, "S1": S1, "ST": ST}).sort_values("S1", ascending=False)
    out_csv = os.path.join(outdir, "global_sobol_indices_pull_single_newParams.csv")
    res.to_csv(out_csv, index=False)

    # Optional plot
    if args.plot:
        try:
            import matplotlib.pyplot as plt

            order = np.argsort(ST)[::-1]
            names_ord = [names[i] for i in order]
            S1_ord = S1[order]
            ST_ord = ST[order]

            x = np.arange(len(names_ord))
            w = 0.35
            plt.figure(figsize=(12, 6))
            plt.bar(x - w / 2, S1_ord, width=w, label="S1 (first-order)")
            plt.bar(x + w / 2, ST_ord, width=w, label="ST (total-order)")
            plt.xticks(x, names_ord, rotation=45, ha="right")
            plt.ylabel("Sobol index")
            plt.title("Global Sobol Indices (PullTest new params)")
            plt.legend()
            plt.tight_layout()
            out_png = os.path.join(outdir, "global_sobol_indices_pull_single_newParams.png")
            plt.savefig(out_png, dpi=200)
        except Exception as e:
            print(f"Plotting failed: {e}")

    # Console summary
    print(f"Data points used: {len(df)} | spacing={spacing}")
    # Count failures from JSONL
    run_dir = os.path.dirname(os.path.abspath(args.csv))
    failed_log_path = args.failed_log if args.failed_log else os.path.join(run_dir, "failed_trials.jsonl")
    json_failures = count_failed_from_jsonl(failed_log_path)
    print(f"Failed simulations (from failed_trials.jsonl): {json_failures}")
    print(f"Model: {model_name}")
    print(f"Surrogate R^2 CV (k={args.cv_folds}) mean={r2_cv_mean:.3f} std={r2_cv_std:.3f}")
    if gp_note:
        print(gp_note)
    print("Top factors by S1:")
    for _, row in res.head(8).iterrows():
        print(f"  {row['param']}: S1={row['S1']:.3f}, ST={row['ST']:.3f}")
    print(f"Saved indices to: {out_csv}")


if __name__ == "__main__":
    main()
