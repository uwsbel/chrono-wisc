import argparse
import glob
import os
from dataclasses import dataclass
from typing import List, Tuple

# Use a non-interactive matplotlib backend to avoid Tkinter crashes in
# multi-threaded/process environments and headless runs.
# This must be set before importing pyplot anywhere.
try:
    import matplotlib
    matplotlib.use("Agg")
except Exception:
    # If matplotlib is not installed or the import fails, plotting blocks
    # below already handle exceptions gracefully.
    pass

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


def build_varspecs_pull_single(particle_spacing: float, feature_names: List[str]) -> List[VarSpec]:
    """Build VarSpecs for Pull/Sine studies, optionally including control gains.

    Always supports geometry parameters. If steering/speed control columns are
    present in the CSV, include them with reasonable bounds (aligned with
    Sine/Slalom scripts in this repo):
      - steering_kp in [1.0, 15.0] (if steering_ki present), else [0.1, 20.0]
      - steering_ki in [0.0, 4.0]
      - steering_kd in [0.0, 4.0]  (if steering_ki present), else [0.0, 5.0]
      - speed_kp    in [0.1, 5.0]
      - speed_ki    in [0.0, 2.0]
      - speed_kd    in [0.0, 1.0]
    """
    # Integer ranges derived from meters and spacing (match PullTest_global_single.py)
    rad_lb, rad_ub = compute_int_bounds(0.05, 0.14, particle_spacing)
    th_lb, th_ub = 45, 135

    specs: List[VarSpec] = []
    if "rad_outer" in feature_names:
        specs.append(VarSpec("rad_outer", "int", (rad_lb, rad_ub)))
    if "w_by_r" in feature_names:
        # Reflect widened bound seen in SineSideSlip data generation
        specs.append(VarSpec("w_by_r", "float", (0.7, 1.4)))
    if "what_percent_is_grouser" in feature_names:
        specs.append(VarSpec("what_percent_is_grouser", "float", (0.0, 0.3)))
    if "g_density" in feature_names:
        specs.append(VarSpec("g_density", "int", (2, 16)))
    if "fan_theta_deg" in feature_names:
        specs.append(VarSpec("fan_theta_deg", "int", (th_lb, th_ub)))

    # Optional controls
    # If steering_ki is present, match the BO_wControl study bounds (SineSideSlip_BO_wControl.py).
    # Otherwise keep broader historical defaults for older datasets.
    has_steering_ki = "steering_ki" in feature_names
    steering_kp_bounds = (1.0, 15.0) if has_steering_ki else (0.1, 20.0)
    steering_kd_bounds = (0.0, 4.0) if has_steering_ki else (0.0, 5.0)
    if "steering_kp" in feature_names:
        specs.append(VarSpec("steering_kp", "float", steering_kp_bounds))
    if "steering_ki" in feature_names:
        specs.append(VarSpec("steering_ki", "float", (0.0, 4.0)))
    if "steering_kd" in feature_names:
        specs.append(VarSpec("steering_kd", "float", steering_kd_bounds))
    if "speed_kp" in feature_names:
        specs.append(VarSpec("speed_kp", "float", (0.1, 5.0)))
    if "speed_ki" in feature_names:
        specs.append(VarSpec("speed_ki", "float", (0.0, 2.0)))
    if "speed_kd" in feature_names:
        specs.append(VarSpec("speed_kd", "float", (0.0, 1.0)))

    return specs


"""
Only support the PullTest single-wheel NEW parameterization with these exact
column names emitted by PullTest_global_single.py.
"""

# ----- Data loading and cleaning -----

BASE_FEATURES = [
    "rad_outer",
    "w_by_r",
    "what_percent_is_grouser",
    "g_density",
    "fan_theta_deg",
]


def detect_features(df: pd.DataFrame) -> List[str]:
    """Return list of feature columns present, including optional controls.

    Always prioritizes the canonical geometry features; adds any of the
    optional controller gains if present in the CSV columns.
    """
    feats = [c for c in BASE_FEATURES if c in df.columns]
    for c in ["steering_kp", "steering_ki", "steering_kd", "speed_kp", "speed_ki", "speed_kd"]:
        if c in df.columns:
            feats.append(c)
    return feats


def load_filtered_pull(csv_path: str, filter_penalties: bool = True, target_col: str = "metric"):
    df = pd.read_csv(csv_path)

    # Core columns we expect regardless of target
    core_required = ["total_time_to_reach", "particle_spacing"]
    core_missing = [c for c in core_required if c not in df.columns]
    if core_missing:
        raise SystemExit(f"CSV missing required core columns: {core_missing}")

    # Validate target availability
    if target_col not in df.columns:
        raise SystemExit(
            f"Target column '{target_col}' not found in CSV. Available columns include: "
            f"{[c for c in ['metric','beta_rms','rms_error','average_power'] if c in df.columns]}"
        )

    # Cast numerics for target and time
    time_col = "total_time_to_reach"
    df[target_col] = pd.to_numeric(df[target_col], errors="coerce")
    df[time_col] = pd.to_numeric(df[time_col], errors="coerce")

    df = df.replace([np.inf, -np.inf], np.nan)
    # Drop rows without valid target/time
    df = df.dropna(subset=[target_col, time_col])

    # Remove failed simulations if filtering is enabled.
    # Failures are encoded as metric >= 500 and/or total_time_to_reach == 12.0 (2*tend).
    if filter_penalties:
        if "metric" in df.columns:
            df["metric"] = pd.to_numeric(df["metric"], errors="coerce")
            df = df[df["metric"] < 500]
        df = df[df[time_col] != 12.0]

    # Particle spacing (constant per run)
    if not df["particle_spacing"].isna().all():
        spacing = pd.to_numeric(df["particle_spacing"], errors="coerce").dropna().mode().iloc[0]
    else:
        spacing = 0.01

    # Determine features present and drop rows with NaNs in those
    features = detect_features(df)
    if not features:
        raise SystemExit("No supported feature columns found in CSV.")
    df = df.dropna(subset=features)

    # Cast integer-like features
    for c in ["rad_outer", "g_density", "fan_theta_deg"]:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce").astype(int)

    # Cast controllers to float if present
    for c in ["steering_kp", "steering_ki", "steering_kd", "speed_kp", "speed_ki", "speed_kd"]:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce").astype(float)

    return df, float(spacing), target_col, features


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
    parser.add_argument("--ice", action="store_true", help="Generate ICE plots for each parameter using the fitted surrogate")
    parser.add_argument("--ice-subsample", type=int, default=120, help="Number of data rows to use for ICE conditioning (lines)")
    parser.add_argument("--ice-grid", type=int, default=100, help="Number of grid points per feature for ICE")
    parser.add_argument("--ice-only-controls", action="store_true", help="Only plot ICE for controller gains (still computes sensitivities for all)")
    parser.add_argument("--outdir", default=None, help="Output directory (default: <run_dir>/analysis)")
    parser.add_argument("--cv-folds", type=int, default=5, help="K-fold CV folds to evaluate surrogate quality")
    parser.add_argument("--model", choices=["rf", "xgb", "gp"], default="rf", help="Surrogate model to use")
    parser.add_argument("--target", type=str, default="metric", help="Target column to model (e.g., metric, beta_rms, rms_error, average_power)")
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
    parser.add_argument("--restrict", type=int, default=None, help="Restrict to first N data points from the CSV")

    args = parser.parse_args()
    filter_penalties = not args.no_filter_penalties

    if not args.csv:
        raise SystemExit("Please provide --csv pointing to your PullTest trials.csv")

    # Load data
    df, spacing, target_col, features = load_filtered_pull(
        args.csv, filter_penalties=filter_penalties, target_col=args.target
    )
    if df.empty:
        raise SystemExit("No usable rows after filtering.")

    # Apply restriction if specified
    if args.restrict is not None:
        if args.restrict <= 0:
            raise SystemExit(f"--restrict must be positive, got {args.restrict}")
        if args.restrict < len(df):
            df = df.iloc[:args.restrict].copy()
            print(f"Restricted to first {len(df)} data points (requested: {args.restrict})")

    # Features/target
    X = df[features].to_numpy(dtype=float)
    y = df[target_col].to_numpy(dtype=float)

    # Build domain spec from spacing (include controls if present)
    specs = build_varspecs_pull_single(spacing, features)

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
    # if model_name == "gp" and args.n > 1000:
    #     print("Warning: reducing Saltelli n to 1000 for GP to keep runtime reasonable.")
    #     saltelli_n = 1000
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
            plt.close()
        except Exception as e:
            print(f"Plotting failed: {e}")

    # ICE plots
    if args.ice:
        try:
            import seaborn as sns
            import matplotlib.pyplot as plt

            ice_dir = os.path.join(outdir, "ice")
            os.makedirs(ice_dir, exist_ok=True)

            sns.set_theme(style="whitegrid")
            plt.rcParams.update(
                {
                    # "font.family": "serif",
                    # "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
                    "axes.labelsize": 10,
                    "axes.titlesize": 10,
                    "xtick.labelsize": 9,
                    "ytick.labelsize": 9,
                    "legend.fontsize": 9,
                }
            )

            label_map = {
                "rad_outer": r"$r_o$",
                "w_by_r": r"$w_r$",
                "what_percent_is_grouser": r"$g_r$",
                "g_density": r"$n_g$",
                "fan_theta_deg": r"$\alpha_g$",
                "steering_kp": r"$K_{p,s}$",
                "steering_ki": r"$K_{i,s}$",
                "steering_kd": r"$K_{d,s}$",
                "speed_kp": r"$K_{p,t}$",
                "speed_ki": r"$K_{i,t}$",
                "speed_kd": r"$K_{d,t}$",
            }

            # Subsample rows for ICE lines
            rng = np.random.default_rng(args.seed)
            n_rows = len(df)
            m = min(n_rows, max(10, int(args.ice_subsample)))
            idx = rng.choice(n_rows, size=m, replace=False) if m < n_rows else np.arange(n_rows)
            X_ref = df[features].to_numpy(dtype=float)[idx]

            pdp_swing_lines = []
            ice_results = []
            control_names = {"steering_kp", "steering_ki", "steering_kd", "speed_kp", "speed_ki", "speed_kd"}

            # Grid and prediction for each feature
            for j, spec in enumerate(specs):
                name = spec.name
                if args.ice_only_controls and name not in control_names:
                    continue
                # Build grid across the domain bounds
                if spec.kind == "int":
                    lo, hi = int(spec.bounds[0]), int(spec.bounds[1])
                    grid = np.arange(lo, hi + 1, max(1, (hi - lo) // max(5, args.ice_grid - 1)))
                    grid = np.unique(np.clip(grid, lo, hi))
                elif spec.kind == "float":
                    lo, hi = float(spec.bounds[0]), float(spec.bounds[1])
                    grid = np.linspace(lo, hi, num=max(5, args.ice_grid))
                else:
                    # categorical not used in this problem; skip if present
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

                # PDP mean
                pdp = np.nanmean(C, axis=0)

                # PDP-based percent swing statistic
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

                label = label_map.get(name, name)
                if name == "rad_outer":
                    label = r"$r_o$ [m]"
                    grid_plot = grid * spacing
                else:
                    grid_plot = grid

                ice_results.append(
                    {
                        "name": name,
                        "label": label,
                        "grid": grid_plot,
                        "C": C,
                        "pdp": pdp,
                    }
                )

            if ice_results:
                n_feats = len(ice_results)
                y_min = min(np.nanmin(item["C"]) for item in ice_results)
                y_max = max(np.nanmax(item["C"]) for item in ice_results)
                y_min = min(y_min, min(np.nanmin(item["pdp"]) for item in ice_results))
                y_max = max(y_max, max(np.nanmax(item["pdp"]) for item in ice_results))
                if n_feats == 5:
                    fig = plt.figure(figsize=(9.6, 5.2))
                    gs = fig.add_gridspec(2, 6, wspace=0.35, hspace=0.4)
                    axes = [
                        fig.add_subplot(gs[0, 0:2]),
                        fig.add_subplot(gs[0, 2:4]),
                        fig.add_subplot(gs[0, 4:6]),
                        fig.add_subplot(gs[1, 1:3]),
                        fig.add_subplot(gs[1, 3:5]),
                    ]
                    ncols = 3
                elif n_feats == 8:
                    nrows, ncols = 2, 4
                    fig, axes = plt.subplots(
                        nrows=nrows,
                        ncols=ncols,
                        sharey=True,
                        figsize=(3.2 * ncols, 2.6 * nrows),
                    )
                    axes = np.atleast_1d(axes).ravel()
                elif n_feats <= 4:
                    nrows, ncols = 1, n_feats
                    fig, axes = plt.subplots(
                        nrows=nrows,
                        ncols=ncols,
                        sharey=True,
                        figsize=(3.2 * ncols, 2.6 * nrows),
                    )
                    axes = np.atleast_1d(axes).ravel()
                else:
                    ncols = 4
                    nrows = int(np.ceil(n_feats / ncols))
                    fig, axes = plt.subplots(
                        nrows=nrows,
                        ncols=ncols,
                        sharey=True,
                        figsize=(3.2 * ncols, 2.6 * nrows),
                    )
                    axes = np.atleast_1d(axes).ravel()

                for ax_i, ax in enumerate(axes):
                    if ax_i >= n_feats:
                        ax.axis("off")
                        continue
                    item = ice_results[ax_i]
                    grid = item["grid"]
                    C = item["C"]
                    pdp = item["pdp"]
                    for k in range(C.shape[0]):
                        ax.plot(grid, C[k, :], color="#4C78A8", alpha=0.18, linewidth=0.8)
                    ax.plot(grid, pdp, color="#F58518", linewidth=2.0, label="PDP (mean)")
                    ax.set_xlabel(item["label"])
                    show_y = (n_feats == 5 and ax_i in [0, 3]) or (n_feats != 5 and ax_i % ncols == 0)
                    if show_y:
                        ax.set_ylabel("objective")
                    else:
                        ax.set_ylabel("")
                        ax.tick_params(left=False, labelleft=False)
                    ax.grid(True, alpha=0.25)
                    ax.set_ylim(y_min, y_max)

                axes[0].legend(loc="best", frameon=False)
                fig.tight_layout()
                fig.subplots_adjust(wspace=0.25, hspace=0.3)
                out_png = os.path.join(ice_dir, "ice_combined.png")
                fig.savefig(out_png, dpi=200)
                plt.close(fig)

            # Save PDP-based percent swing statistics to a text file
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

    # Console summary
    print(f"Data points used: {len(df)} | spacing={spacing}")
    # Count failures from JSONL
    run_dir = os.path.dirname(os.path.abspath(args.csv))
    if args.failed_log:
        failed_log_path = args.failed_log
    else:
        candidates = [
            os.path.join(run_dir, "failed_trials.jsonl"),
            os.path.join(run_dir, "failed_trials_wControl.jsonl"),
        ]
        failed_log_path = ""
        for cand in candidates:
            if os.path.isfile(cand):
                failed_log_path = cand
                break
        if not failed_log_path:
            globbed = sorted(glob.glob(os.path.join(run_dir, "failed_trials*.jsonl")))
            failed_log_path = globbed[0] if globbed else os.path.join(run_dir, "failed_trials.jsonl")
    json_failures = count_failed_from_jsonl(failed_log_path)
    print(f"Failed simulations (from {os.path.basename(failed_log_path)}): {json_failures}")
    print(f"Model: {model_name}")
    print(f"Target: {target_col}")
    print(f"Surrogate R^2 CV (k={args.cv_folds}) mean={r2_cv_mean:.3f} std={r2_cv_std:.3f}")
    if r2_cv_mean < 0.0:
        print("Warning: negative CV R^2 indicates the surrogate is performing worse than a constant-baseline model."
              " Consider trying a different target (e.g., --target beta_rms for side-slip studies) or a different model (e.g., --model rf).")
    if gp_note:
        print(gp_note)
    print("Top factors by S1:")
    for _, row in res.head(8).iterrows():
        print(f"  {row['param']}: S1={row['S1']:.3f}, ST={row['ST']:.3f}")
    print(f"Saved indices to: {out_csv}")


if __name__ == "__main__":
    main()
