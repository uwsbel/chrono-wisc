import pandas as pd
import numpy as np
import os
import sys
from simple_wheel_gen import GenSimpleWheelPointCloud

# === Step 1: Parse args ===
# Usage: python top_10_wheels.py <folder>
if len(sys.argv) < 2:
    raise SystemExit("Usage: python top_10_wheels.py <folder>")

folder = sys.argv[1]

# === Step 2: Load the CSV file ===
df = pd.read_csv(folder + "/trials.csv")

# === Step 3: Inspect column names ===
print("Columns in dataset:", df.columns.tolist())

# === Step 4: Use composite metric column ===
metric_col = 'composite_metric'
if metric_col not in df.columns:
    raise ValueError(
        f"Expected column '{metric_col}' not found. Available columns: {df.columns.tolist()}"
    )

# === Step 5: Clean and filter metrics ===
# Coerce to numeric, drop NaN/Inf, and remove failed runs (use 50.0 cutoff)
df[metric_col] = pd.to_numeric(df[metric_col], errors='coerce')
df = df[df[metric_col].notna()]
df = df[np.isfinite(df[metric_col])]
df = df[df[metric_col] < 50.0]  # Remove failed simulations

# === Step 6: Sort by best (lowest) metrics ===
df_sorted = df.sort_values(by=metric_col, ascending=True)

# === Step 7: Select the top 10 runs ===
top_10 = df_sorted.head(10)

# === Step 8: Display the parameters for these runs ===
print("\nTop 10 Best (Lowest) Composite Metrics and Corresponding Parameters:")
for rank, (orig_idx, row) in enumerate(top_10.iterrows(), start=1):
    print(f"\nRank {rank}:")
    for col in top_10.columns:
        print(f"  {col}: {row[col]}")

# === Step 9: Save ranked table next to input ===
output_path = folder + "/top_10_best_metrics.csv"
top_10.to_csv(output_path, index=False)
print(f"\nResults saved to '{output_path}'")


# === Step 10: Generate wheel CSVs for top 10 into {folder}_top10 ===
out_wheels_dir = f"{folder}_top10"
os.makedirs(out_wheels_dir, exist_ok=True)

def _get_value(row, key, default=None):
    return row[key] if key in row and pd.notna(row[key]) else default

for rank, (orig_idx, row) in enumerate(top_10.iterrows(), start=1):
    # Required fields with safe casting
    spacing = float(_get_value(row, 'particle_spacing', 0.01))
    rad_units = float(_get_value(row, 'rad', 0.0))
    width_units = float(_get_value(row, 'width', 0.0))
    g_height_units = float(_get_value(row, 'g_height', 0.0))
    g_width_units = float(_get_value(row, 'g_width', 0.0))
    g_density = int(float(_get_value(row, 'g_density', 8)))

    # Optional fields
    cp_deviation = float(_get_value(row, 'cp_deviation', 0.0))
    fan_theta_deg = float(_get_value(row, 'fan_theta_deg', 90.0))

    # Grouser type handling: allow 0/1 or strings
    gt_raw = _get_value(row, 'grouser_type', 0)
    if isinstance(gt_raw, str):
        gt_val = gt_raw.strip().lower()
        grouser_type = 'semi_circle' if 'semi' in gt_val else 'straight'
    else:
        try:
            gt_num = int(float(gt_raw))
        except Exception:
            gt_num = 0
        grouser_type = 'straight' if gt_num == 0 else 'semi_circle'

    # Convert from multiples of spacing to meters
    rad_m = rad_units * spacing
    width_m = width_units * spacing
    g_height_m = g_height_units * spacing
    g_width_m = g_width_units * spacing

    # Generate point cloud
    pts = GenSimpleWheelPointCloud(
        rad=rad_m,
        width=width_m,
        cp_deviation=cp_deviation,
        g_height=g_height_m,
        g_width=g_width_m,
        g_density=g_density,
        particle_spacing=spacing,
        grouser_type=grouser_type,
        fan_theta_deg=fan_theta_deg,
        filename=""
    )

    # Save to CSV
    pts_df = pd.DataFrame(pts, columns=['x', 'y', 'z'])
    wheel_csv_path = os.path.join(out_wheels_dir, f"rank_{rank}_idx_{orig_idx}.csv")
    pts_df.to_csv(wheel_csv_path, index=False)
    print(f"Saved wheel CSV: {wheel_csv_path}")
