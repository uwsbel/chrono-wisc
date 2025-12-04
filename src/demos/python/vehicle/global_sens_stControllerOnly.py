import argparse
import os
from dataclasses import dataclass
from typing import List, Tuple

# Use a non-interactive matplotlib backend to avoid Tkinter crashes in
# multi-threaded/process environments and headless runs.
try:
    import matplotlib
    matplotlib.use("Agg")
except Exception:
    pass

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import KFold, cross_val_score
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler


# ----- Problem definition (steering controller only: kp, ki, kd) -----


@dataclass
class VarSpec:
    name: str
    kind: str  # 'int', 'float', or 'cat'
    bounds: Tuple[float, float] = None  # inclusive for int, [low, high] for float
    values: Tuple[int, ...] = None      # for categorical


def build_varspecs_controller(feature_names: List[str]) -> List[VarSpec]:
    """Bounds match SineSideSlip_BO_stControllerOnly.py."""
    specs: List[VarSpec] = []
    if "steering_kp" in feature_names:
        specs.append(VarSpec("steering_kp", "float", (1.0, 15.0)))
    if "steering_ki" in feature_names:
        specs.append(VarSpec("steering_ki", "float", (0.0, 4.0)))
    if "steering_kd" in feature_names:
        specs.append(VarSpec("steering_kd", "float", (0.0, 4.0)))
    return specs


BASE_FEATURES = [
    "steering_kp",
    "steering_ki",
    "steering_kd",
]


def detect_features(df: pd.DataFrame) -> List[str]:
    """Only consider steering gains for the controller-only study."""
    return [c for c in BASE_FEATURES if c in df.columns]


def load_filtered_controller(csv_path: str, filter_penalties: bool = True, target_col: str = "metric"):
    df = pd.read_csv(csv_path)

    core_required = ["total_time_to_reach"]
    core_missing = [c for c in core_required if c not in df.columns]
    if core_missing:
        raise SystemExit(f"CSV missing required core columns: {core_missing}")

    if target_col not in df.columns:
        raise SystemExit(
            f"Target column '{target_col}' not found in CSV. Available columns include: "
            f"{[c for c in ['metric','beta_rms','rms_error','average_power'] if c in df.columns]}"
        )

    time_col = "total_time_to_reach"
    df[target_col] = pd.to_numeric(df[target_col], errors="coerce")
    df[time_col] = pd.to_numeric(df[time_col], errors="coerce")

    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.dropna(subset=[target_col, time_col])

    if filter_penalties and target_col == "metric":
        df = df[(df[target_col] < 500) & (df[time_col] != 12.0)]

    features = detect_features(df)
    if not features:
        raise SystemExit("No steering gain columns found in CSV (expected steering_kp, steering_ki, steering_kd).")
    df = df.dropna(subset=features)

    for c in features:
        df[c] = pd.to_numeric(df[c], errors="coerce").astype(float)

    return df, target_col, features


def count_failed_from_jsonl(path: str) -> int:
    """Count unique failed trial indices from a failed_trials jsonl file."""
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
    parser = argparse.ArgumentParser(
        description="Sobol global sensitivity for steering-only BO runs (kp, ki, kd) via surrogate."
    )
    parser.add_argument("--csv", default="", help="Path to trials_stControllerOnly.csv")
    parser.add_argument("--n", type=int, default=2000, help="Base sample size for Saltelli (A and B)")
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    parser.add_argument("--no-filter-penalties", action="store_true", help="Keep penalty trials (metric>=500 or time==12)")
    parser.add_argument("--plot", action="store_true", help="Save a bar plot of S1 and ST")
    parser.add_argument("--ice", action="store_true", help="Generate ICE plots for each parameter using the fitted surrogate")
    parser.add_argument("--ice-subsample", type=int, default=120, help="Number of data rows to use for ICE conditioning (lines)")
    parser.add_argument("--ice-grid", type=int, default=100, help="Number of grid points per feature for ICE")
    parser.add_argument("--outdir", default=None, help="Output directory (default: <run_dir>/analysis_controller)")
    parser.add_argument("--cv-folds", type=int, default=5, help="K-fold CV folds to evaluate surrogate quality")
    parser.add_argument("--model", choices=["rf", "xgb", "gp"], default="rf", help="Surrogate model to use")
    parser.add_argument("--target", type=str, default="metric", help="Target column to model (metric, beta_rms, rms_error, average_power)")
    parser.add_argument("--failed-log", default=None, help="Path to failed_trials_stControllerOnly.jsonl (default: alongside CSV)")
    parser.add_argument("--xgb-estimators", type=int, default=800, help="XGBoost number of trees")
    parser.add_argument("--xgb-depth", type=int, default=8, help="XGBoost max depth")
    parser.add_argument("--xgb-learning-rate", type=float, default=0.05, help="XGBoost learning rate")
    parser.add_argument("--xgb-subsample", type=float, default=0.8, help="XGBoost subsample ratio")
    parser.add_argument("--xgb-colsample-bytree", type=float, default=0.8, help="XGBoost colsample by tree")
    parser.add_argument("--gp-max-samples", type=int, default=1000, help="Max training samples for GP (subsamples if exceeded)")
    parser.add_argument("--gp-n-restarts", type=int, default=2, help="GP optimizer restarts")
    parser.add_argument("--restrict", type=int, default=None, help="Restrict to first N data points from the CSV")

    args = parser.parse_args()
    filter_penalties = not args.no_filter_penalties

    if not args.csv:
        raise SystemExit("Please provide --csv pointing to your trials_stControllerOnly.csv")

    df, target_col, features = load_filtered_controller(
        args.csv, filter_penalties=filter_penalties, target_col=args.target
    )
    if df.empty:
        raise SystemExit("No usable rows after filtering.")

    if args.restrict is not None:
        if args.restrict <= 0:
            raise SystemExit(f"--restrict must be positive, got {args.restrict}")
        if args.restrict < len(df):
            df = df.iloc[:args.restrict].copy()
            print(f"Restricted to first {len(df)} data points (requested: {args.restrict})")

    X = df[features].to_numpy(dtype=float)
    y = df[target_col].to_numpy(dtype=float)

    specs = build_varspecs_controller(features)

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

    kf = KFold(n_splits=args.cv_folds, shuffle=True, random_state=args.seed)

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

    if model_name == "gp" and len(X) > args.gp_max_samples:
        estimator.fit(X_cv, y_cv)
    else:
        estimator.fit(X, y)

    def predict_fn(arr_2d: np.ndarray) -> np.ndarray:
        return estimator.predict(arr_2d)

    rng = np.random.default_rng(args.seed)
    saltelli_n = args.n
    S1, ST, names = saltelli_indices(predict_fn, specs, n=saltelli_n, rng=rng)

    if args.outdir is None:
        run_dir = os.path.dirname(os.path.abspath(args.csv))
        outdir = os.path.join(run_dir, "analysis_controller")
    else:
        outdir = args.outdir
    os.makedirs(outdir, exist_ok=True)

    res = pd.DataFrame({"param": names, "S1": S1, "ST": ST}).sort_values("S1", ascending=False)
    out_csv = os.path.join(outdir, "global_sobol_indices_stControllerOnly.csv")
    res.to_csv(out_csv, index=False)

    if args.plot:
        try:
            import matplotlib.pyplot as plt

            order = np.argsort(ST)[::-1]
            names_ord = [names[i] for i in order]
            S1_ord = S1[order]
            ST_ord = ST[order]

            x = np.arange(len(names_ord))
            w = 0.35
            plt.figure(figsize=(10, 5))
            plt.bar(x - w / 2, S1_ord, width=w, label="S1 (first-order)")
            plt.bar(x + w / 2, ST_ord, width=w, label="ST (total-order)")
            plt.xticks(x, names_ord, rotation=45, ha="right")
            plt.ylabel("Sobol index")
            plt.title("Global Sobol Indices (steering controller only)")
            plt.legend()
            plt.tight_layout()
            out_png = os.path.join(outdir, "global_sobol_indices_stControllerOnly.png")
            plt.savefig(out_png, dpi=200)
            plt.close()
        except Exception as e:
            print(f"Plotting failed: {e}")

    if args.ice:
        try:
            import matplotlib.pyplot as plt

            ice_dir = os.path.join(outdir, "ice")
            os.makedirs(ice_dir, exist_ok=True)

            rng = np.random.default_rng(args.seed)
            n_rows = len(df)
            m = min(n_rows, max(10, int(args.ice_subsample)))
            idx = rng.choice(n_rows, size=m, replace=False) if m < n_rows else np.arange(n_rows)
            X_ref = df[features].to_numpy(dtype=float)[idx]

            pdp_swing_lines = []

            for j, spec in enumerate(specs):
                name = spec.name
                if spec.kind == "int":
                    lo, hi = int(spec.bounds[0]), int(spec.bounds[1])
                    grid = np.arange(lo, hi + 1, max(1, (hi - lo) // max(5, args.ice_grid - 1)))
                    grid = np.unique(np.clip(grid, lo, hi))
                elif spec.kind == "float":
                    lo, hi = float(spec.bounds[0]), float(spec.bounds[1])
                    grid = np.linspace(lo, hi, num=max(5, args.ice_grid))
                else:
                    continue

                C = np.zeros((len(X_ref), len(grid)), dtype=float)
                for i_row in range(len(X_ref)):
                    X_tmp = np.tile(X_ref[i_row], (len(grid), 1))
                    X_tmp[:, j] = grid
                    try:
                        preds = estimator.predict(X_tmp)
                    except Exception:
                        preds = np.full(len(grid), np.nan)
                    C[i_row, :] = preds

                pdp = np.nanmean(C, axis=0)

                delta = float(np.nanmax(pdp) - np.nanmin(pdp))
                baseline = float(np.nanmean(np.abs(pdp)))
                if np.isnan(baseline) or baseline <= 0.0:
                    percent_swing = float("nan")
                else:
                    percent_swing = 100.0 * delta / baseline

                line = (
                    f"{name}: PDP percent swing = "
                    f"{percent_swing:.2f}% (delta={delta:.6g}, baseline={baseline:.6g})"
                )
                pdp_swing_lines.append(line)
                print(line)

                plt.figure(figsize=(6, 4))
                for k in range(C.shape[0]):
                    plt.plot(grid, C[k, :], color="tab:blue", alpha=0.18, linewidth=1)
                plt.plot(grid, pdp, color="tab:orange", linewidth=2.5, label="PDP (mean)")
                plt.xlabel(name)
                plt.ylabel(target_col)
                plt.title(f"ICE for {name}")
                plt.grid(True, alpha=0.25)
                plt.legend()
                plt.tight_layout()
                plt.savefig(os.path.join(ice_dir, f"ice_{name}.png"), dpi=150)
                plt.close()

            try:
                swing_path = os.path.join(outdir, "ice_pdp_percent_swing.txt")
                with open(swing_path, "w") as f:
                    f.write(
                        "PDP-based percent swing for ICE features "
                        f"(target={target_col}):\n"
                    )
                    for ln in pdp_swing_lines:
                        f.write(ln + "\n")
                print(f"Saved ICE PDP percent swing statistics to: {swing_path}")
            except Exception as e:
                print(f"Failed to write ICE PDP percent swing file: {e}")
        except Exception as e:
            print(f"ICE plotting failed: {e}")

    print(f"Data points used: {len(df)}")
    run_dir = os.path.dirname(os.path.abspath(args.csv))
    failed_log_path = args.failed_log if args.failed_log else os.path.join(run_dir, "failed_trials_stControllerOnly.jsonl")
    json_failures = count_failed_from_jsonl(failed_log_path)
    print(f"Failed simulations (from failed_trials_stControllerOnly.jsonl): {json_failures}")
    print(f"Model: {model_name}")
    print(f"Target: {target_col}")
    print(f"Surrogate R^2 CV (k={args.cv_folds}) mean={r2_cv_mean:.3f} std={r2_cv_std:.3f}")
    if r2_cv_mean < 0.0:
        print(
            "Warning: negative CV R^2 indicates the surrogate is performing worse than a constant-baseline model. "
            "Consider trying a different target (e.g., --target beta_rms for side-slip studies) or a different model (e.g., --model rf)."
        )
    if gp_note:
        print(gp_note)
    print("Top factors by S1:")
    for _, row in res.head(8).iterrows():
        print(f"  {row['param']}: S1={row['S1']:.3f}, ST={row['ST']:.3f}")
    print(f"Saved indices to: {out_csv}")


if __name__ == "__main__":
    main()
