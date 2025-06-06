#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import argparse
import os

# Parse command line arguments
parser = argparse.ArgumentParser(description='Plot terrain benchmark results')
parser.add_argument('--folders', nargs='+', required=True,
                    help='List of folders containing benchmark results')
parser.add_argument('--names', nargs='+', required=True,
                    help='List of legend names corresponding to each folder')
parser.add_argument('--output', default='benchmark_results',
                    help='Output directory for the generated plots (default: benchmark_results)')
args = parser.parse_args()

# Validate arguments
if len(args.folders) != len(args.names):
    print("Error: Number of folders must match number of legend names")
    exit(1)

# Create output directory if it doesn't exist
os.makedirs(args.output, exist_ok=True)

# Set plot style to match SphereDrop_PS_Comparison_zeta.py
plt.rcParams["figure.dpi"] = 300
plt.rcParams["font.size"] = 10
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams['font.weight'] = 'bold'
sns.set(style="ticks", context="talk")

# Create figure
plt.figure(figsize=(8, 8 - 3.2), dpi=600)

# Read and combine benchmark results from all folders
dfs = []
for folder, name in zip(args.folders, args.names):
    try:
        df = pd.read_csv(f"{folder}/terrain_benchmark_results.csv")
        df['gpu'] = name
        print(f"Loaded {len(df)} data points from {folder}")
        print(df)
        dfs.append(df)
    except Exception as e:
        print(f"Error reading CSV file from {folder}: {e}")
        continue

if not dfs:
    print("Error: No valid data found in any of the provided folders")
    exit(1)

# Combine all dataframes
df = pd.concat(dfs, ignore_index=True)

# Create the plot using seaborn
ax = sns.lineplot(
    x='terrain_length',
    y='rtf',
    hue='gpu',
    style='gpu',
    data=df,
    marker='o',
    markersize=8,
    linewidth=2
)

# Remove old reference line and implement new tangent lines per GPU
unique_gpus = df['gpu'].unique()
plot_x_coords_for_tangents = np.sort(df['terrain_length'].unique())

print("\nCalculating and plotting 45-degree tangent lines:")
for i, gpu_name in enumerate(unique_gpus):
    gpu_df = df[df['gpu'] == gpu_name].sort_values(by='terrain_length')
    x_coords = gpu_df['terrain_length'].values
    y_coords = gpu_df['rtf'].values

    if len(x_coords) < 2:
        print(f"  Skipping GPU {gpu_name}: Not enough data points ({len(x_coords)}) to calculate slope.")
        continue

    # Ensure all coordinates are positive for log transformation
    if np.any(x_coords <= 0) or np.any(y_coords <= 0):
        print(f"  Skipping GPU {gpu_name}: Non-positive values found in coordinates, cannot compute log-log slope.")
        continue
        
    log_x = np.log10(x_coords)
    log_y = np.log10(y_coords)

    diff_log_x = np.diff(log_x)
    diff_log_y = np.diff(log_y)
    
    slopes = np.full_like(diff_log_x, np.nan)
    # Calculate slopes only where diff_log_x is not zero (avoid division by zero)
    # and where diff_log_x is not extremely small (avoid numerical instability if desired, though not strictly done here)
    valid_diffs = diff_log_x != 0
    
    if not np.any(valid_diffs):
        print(f"  Skipping GPU {gpu_name}: No valid segments with varying x-values to calculate slopes.")
        continue

    slopes[valid_diffs] = diff_log_y[valid_diffs] / diff_log_x[valid_diffs]

    print(slopes)
    
    if np.all(np.isnan(slopes)):
        print(f"  Skipping GPU {gpu_name}: All segment slopes are undefined.")
        continue

    tangent_slope_idx = np.nanargmin(np.abs(slopes - 1.0))

    x_tangent_point = x_coords[tangent_slope_idx + 1]
    y_tangent_point = y_coords[tangent_slope_idx + 1]
    
    print(f"  For GPU {gpu_name}:")
    print(f"    Tangent point (terrain length): {x_tangent_point:.2f} m (RTF: {y_tangent_point:.2f})")

    K = y_tangent_point / x_tangent_point
    tangent_line_y_values = K * plot_x_coords_for_tangents

    line_color = ax.lines[i].get_color()

    plt.plot(plot_x_coords_for_tangents, tangent_line_y_values,
             linestyle='--', color=line_color, alpha=0.7)

# Use logarithmic scale for x-axis since terrain lengths grow logarithmically
plt.xscale('log')
plt.yscale('log')

# Set y-axis limits and ticks
plt.ylim(bottom=1) # Start y-axis at 1 (10^0)
plt.ylim(top=10**3)
plt.xlim(left=1)  # Set x-axis to start at 10^0
plt.xlim(right=10**5)

# Calculate ticks based on the new y-axis range
current_ylim = plt.gca().get_ylim()
min_y_for_ticks = current_ylim[0] 
max_y_for_ticks = current_ylim[1]

# Ensure max_y_for_ticks is positive and at least 1 for log calculation
# (min_y_for_ticks is already >= 1 due to plt.ylim(bottom=1))
safe_max_y_for_ticks = max(1.0, max_y_for_ticks)

start_power = np.floor(np.log10(min_y_for_ticks)) 
end_power = np.ceil(np.log10(safe_max_y_for_ticks))

# Ensure start_power is at least 0 (for 10^0) and consistent
start_power = max(0.0, start_power)
if end_power < start_power: # Handles cases where max_y_for_ticks might be 1
    end_power = start_power

num_ticks = int(end_power - start_power + 1)
# Ensure at least one tick if start and end power are the same (e.g. max data is 1)
if num_ticks < 1:
    num_ticks = 1

y_ticks_values = np.logspace(start_power, end_power, num=num_ticks, base=10.0)

# Filter out ticks that might be slightly less than 1 due to floating point issues,
# though ylim and start_power should generally prevent this.
y_ticks_values = y_ticks_values[y_ticks_values >= 0.9999] # Check against a value very close to 1
if not y_ticks_values.size: # If all data was < 1 (unlikely with ylim(bottom=1)) or y_ticks_values became empty
    y_ticks_values = np.array([1.0])


plt.yticks(y_ticks_values, [f'$10^{{{int(round(np.log10(tick)))}}}$' for tick in y_ticks_values])

# Add labels and title with bold fonts
plt.xlabel("Terrain length (m)", fontsize=14, fontweight='bold')
plt.ylabel("Real-Time Factor (RTF)", fontsize=14, fontweight='bold')
# plt.title("Impact of Terrain Length on Simulation Performance", fontsize=14, fontweight='bold')

# Set aspect ratio for visually correct 45-degree lines
ax.set_aspect('equal', adjustable='box')

# Set tick parameters
plt.tick_params(axis='both', which='major', labelsize=12)

# Add legend
plt.legend(loc='upper left', fontsize=12)

# Add text box annotation
# plt.text(0.95, 0.05, "Polaris RZR driving on\nCRM deformable Terrain",
#          transform=plt.gca().transAxes,
#          bbox=dict(facecolor='white', edgecolor='black', alpha=0.8,
#                   linestyle=':', linewidth=1),
#          verticalalignment='bottom',
#          horizontalalignment='right',
#          fontsize=10)

# Enhance the plot appearance
sns.despine(left=False, bottom=False)
plt.grid(False)
plt.tight_layout()

# Save the figure
output_png = os.path.join(args.output, "rtf_vs_terrain_length_trials.png")
output_pdf = os.path.join(args.output, "rtf_vs_terrain_length_trials.pdf")

plt.savefig(output_png, dpi=600, bbox_inches='tight')
plt.savefig(output_pdf, bbox_inches='tight')

print(f"Plot created: {output_png}")
print(f"Plot created: {output_pdf}")

# Show the plot
plt.show()
