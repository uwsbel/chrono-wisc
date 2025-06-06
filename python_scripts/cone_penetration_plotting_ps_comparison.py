import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import json
import seaborn as sns
# Create custom legend using matplotlib's built-in legend handler
from matplotlib.lines import Line2D
"""
--- Description ---
- Plots penetration depth vs time for a single Relative Density and a single cone type across all three Hdrops
- Compares different ps values (1, 2, 5, 10) on each subplot
- The experimental data is plotted as a horizontal line at the experimental depth read from the json file
--- Output ---
- Plots are saved as granMaterial_relDen_coneType_ps_comparison.png in paper_plots/cone_penetration
"""
EXPERIMENTAL_DATA_PATH = "./conePenetration_experimental.json"


# Map spacing to time step
time_step_dict = {
    0.001: 2e-5,
    0.0005: 1e-5,
    0.002: 5e-5
}


def read_experimental_json(file_path):
    with open(file_path, 'r') as file:
        # Filter out comment lines
        json_lines = [
            line for line in file if not line.strip().startswith('#')]
        # Join the lines and parse the JSON
        data = json.loads(''.join(json_lines))
    # Convert string keys in the inner dictionaries to tuple keys
    for outer_key, inner_dict in data.items():
        data[outer_key] = {
            tuple(eval(inner_key)): value for inner_key, value in inner_dict.items()}
    return data


def calculate_error_metrics(sim_value, exp_value):
    """
    Calculate error metrics between simulation and experimental values.
    
    Parameters:
    - sim_value: Simulation value (final penetration depth)
    - exp_value: Experimental value
    
    Returns:
    - Dictionary containing error metrics
    """
    absolute_error = abs(sim_value - exp_value)
    relative_error = absolute_error / exp_value * 100  # percentage
    
    return {
        'Absolute Error (m)': absolute_error,
        'Relative Error (%)': relative_error,
        'Simulation Value (m)': sim_value,
        'Experimental Value (m)': exp_value
    }


def print_error_metrics(metrics_dict):
    """
    Print error metrics in a formatted table.
    
    Parameters:
    - metrics_dict: Dictionary with ps values as keys and lists of metric dictionaries as values
                   Each list contains metrics for Hdrop 0.0, 0.5, and 1.0
    """
    hdrop_values = ["0.0", "0.5", "1.0"]
    ps_values = sorted(metrics_dict.keys())
    
    for ps in ps_values:
        print("\n" + "="*100)
        print(f"{'PENETRATION DEPTH ERROR METRICS FOR PS = ' + str(ps):^100}")
        print("="*100)
        print(f"{'Hdrop':<10} {'Sim Depth (m)':<15} {'Exp Depth (m)':<15} {'Abs Error (m)':<15} {'Rel Error (%)':<15}")
        print("-"*100)
        
        for i, metrics in enumerate(metrics_dict[ps]):
            print(f"{hdrop_values[i]:<10} {metrics['Simulation Value (m)']:.6e} {metrics['Experimental Value (m)']:.6e} "
                  f"{metrics['Absolute Error (m)']:.6e} {metrics['Relative Error (%)']:.2f}")
        
        print("="*100)


def plot_penetration_depth_ps_comparison(data_dict, folder, defaults: list[str], vertical_line_stats: tuple[float, float, str] = None):
    """
    Plots a line graph of penetration depth over time comparing different ps values and saves it as an image.

    Parameters:
    - data_dict: Dictionary with ps values as keys and lists of DataFrame (one per Hdrop) as values
    - folder: The directory where the plot should be saved
    - defaults: A list containing [granMaterial, relDensity, coneType] values
    - vertical_line_stats: Tuple containing parameters for experimental data lookup
    """
    # Set the style and context for the plot
    sns.set(style="ticks", context="talk")

    # Read experimental data
    experimental_data = read_experimental_json(EXPERIMENTAL_DATA_PATH)

    h0_exp = experimental_data["0.0"]
    h05_exp = experimental_data["0.5"]
    h1_exp = experimental_data["1.0"]

    h0_depth = h0_exp[vertical_line_stats]
    h05_depth = h05_exp[vertical_line_stats]
    h1_depth = h1_exp[vertical_line_stats]

    # Create a figure with a specific size and 3 subplots
    fig, axes = plt.subplots(1, 3, figsize=(15, 6), sharey=True)

    # Add titles to each subplot
    axes[0].set_title("Hdrop 0.0", fontsize=14, fontweight='bold')
    axes[1].set_title("Hdrop 0.5", fontsize=14, fontweight='bold')
    axes[2].set_title("Hdrop 1.0", fontsize=14, fontweight='bold')

    # Define color palette for different ps values
    ps_values = list(data_dict.keys())
    colors = sns.color_palette("viridis", len(ps_values))
    ps_color_dict = dict(zip(ps_values, colors))

    # Gray for experimental data
    exp_color = (0.5, 0.5, 0.5)

    # Keep track of the line objects for the legend
    freq_lines = []
    experimental_line = None

    # Plot data for each Hdrop (column) and each ps value (line)
    for idx, hdrop_idx in enumerate([0, 1, 2]):  # 0.0, 0.5, 1.0
        # Plot horizontal line for experimental data
        if hdrop_idx == 0:
            experimental_line = axes[idx].axhline(
                y=h0_depth, color=exp_color, linestyle='--')
        elif hdrop_idx == 1:
            axes[idx].axhline(y=h05_depth, color=exp_color, linestyle='--')
        else:
            axes[idx].axhline(y=h1_depth, color=exp_color, linestyle='--')

        # Plot lines for each ps value
        for ps in ps_values:
            data = data_dict[ps][hdrop_idx]
            line = sns.lineplot(
                ax=axes[idx],
                x="Time",
                y="PenetrationDepth",
                data=data,
                color=ps_color_dict[ps],
                label=None  # No label here to avoid automatic legend creation
            )
            # Only save the lines from the first subplot for the legend
            if idx == 0:
                freq_lines.append((line.lines[0], f"{ps}"))

        # Set axis labels
        axes[idx].set_xlabel("Time (s)", fontsize=14, fontweight='bold')
        if idx == 0:
            axes[idx].set_ylabel("Penetration Depth (m)",
                                 fontsize=14, fontweight='bold')

    # Use scientific notation for the y-axis and remove grid lines
    for ax in axes:
        ax.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.grid(False)

    # First create custom legend elements
    legend_elements = [
        Line2D([0], [0], color=exp_color, linestyle='--', label='Experimental')]

    # Then add the ps value lines with "Freq." prefix
    for ps, color in zip(ps_values, colors):
        legend_elements.append(
            Line2D([0], [0], color=color, linestyle='-', label=f'Freq. {ps}'))

    # Create the legend on the rightmost plot
    axes[2].legend(handles=legend_elements, loc='lower right', prop={'weight': 'bold'})

    # Apply tight layout
    plt.tight_layout()

    # Calculate and print error metrics
    exp_depths = [h0_depth, h05_depth, h1_depth]
    error_metrics_dict = {}
    
    for ps in ps_values:
        error_metrics_ps = []
        for i, df in enumerate(data_dict[ps]):
            # Skip empty dataframes
            if df.empty:
                metrics = {
                    'Absolute Error (m)': float('nan'),
                    'Relative Error (%)': float('nan'),
                    'Simulation Value (m)': float('nan'),
                    'Experimental Value (m)': exp_depths[i]
                }
            else:
                # Get final penetration depth from simulation
                final_sim_depth = df['PenetrationDepth'].iloc[-1]
                
                # Get corresponding experimental depth
                exp_depth = exp_depths[i]
                
                # Calculate metrics
                metrics = calculate_error_metrics(final_sim_depth, exp_depth)
            
            error_metrics_ps.append(metrics)
        
        error_metrics_dict[ps] = error_metrics_ps
    
    # Print error metrics
    print_error_metrics(error_metrics_dict)

    # Ensure the paper_plots/cone_penetration directory exists
    output_dir = os.path.join("paper_plots", "cone_penetration")
    os.makedirs(output_dir, exist_ok=True)

    # Extract parameters from defaults
    if len(defaults) >= 3:
        granMaterial = defaults[0]
        rel_den = defaults[1]
        cone_type = defaults[2]

        # Save the plot with a name that indicates it's comparing ps values
        output_filename = f"{granMaterial}_{rel_den}_{cone_type}_ps_comparison.png"
        output_path = os.path.join(output_dir, output_filename)
        plt.savefig(output_path, dpi=600)

        print(f"Plot saved to: {output_path}")
    else:
        print(
            "Error: defaults list should contain at least [granMaterial, relDensity, coneType] values")

    plt.show()


if __name__ == "__main__":
    mu_s = "0.80"
    mu_2 = "1.00"
    mu_i0 = "0.08"
    base_dir = f"./DEMO_OUTPUT/FSI_ConePenetration_mu_s_{mu_s}_mu_2_{mu_2}_mu_i0_{mu_i0}/"

    Hdrop = [0.0, 0.5, 1.0]
    granMaterial = "sand"
    rel_density = 0
    cone_type = 1
    boundary_type = "adami"
    viscosity_type = "artificial_bilateral"
    kernel_type = "wendland"

    ps_values = [1, 2, 5, 10]  # The ps values to compare
    s = 0.001
    d0 = 1.3
    t = time_step_dict[s]
    av = 0.2

    # Bead parameters
    # mu_s = "0.70"
    # mu_2 = "0.80"
    # mu_i0 = "0.08"
    # base_dir = f"./DEMO_OUTPUT/FSI_ConePenetration_mu_s_{mu_s}_mu_2_{mu_2}_mu_i0_{mu_i0}/"

    # Hdrop = [0.0, 0.5, 1.0]
    # granMaterial = "bead"
    # rel_density = 1
    # cone_type = 2
    # boundary_type = "adami"
    # viscosity_type = "artificial_bilateral"
    # kernel_type = "wendland"

    # ps_values = [1, 2, 5, 10]  # The ps values to compare
    # s = 0.001
    # d0 = 1.3
    # t = time_step_dict[s]
    # av = 0.2

    # Dictionary to store data for each ps value
    data_dict = {}

    # For each ps value, collect data for all three Hdrops
    for ps in ps_values:
        ps_data = []
        for Hd in Hdrop:
            folder = os.path.join(
                base_dir,
                f"Hdrop_{Hd}",
                f"granMaterial_{granMaterial}",
                f"relDensity_{rel_density}",
                f"coneType_{cone_type}",
                f"boundaryType_{boundary_type}",
                f"viscosityType_{viscosity_type}",
                f"kernelType_{kernel_type}",
                f"ps_{ps}_s_{s}_d0_{d0}_t_{t}_av_{av}"
            )

            # Load data
            file_path = os.path.join(folder, "cone_penetration_depth.txt")
            try:
                data = pd.read_csv(file_path, header='infer')

                # Filter data to include only Time <= 0.5 seconds
                data = data[data["Time"] <= 0.5]

                # Zero first row penetration depth and subtract from all rows
                data["PenetrationDepth"] = data["PenetrationDepth"] - \
                    data.loc[0, "PenetrationDepth"]
                data.loc[0, "PenetrationDepth"] = 0

                ps_data.append(data)
            except FileNotFoundError:
                print(
                    f"Warning: File not found for ps={ps}, Hdrop={Hd}. Using empty dataframe.")
                # Create an empty dataframe with the same columns
                ps_data.append(pd.DataFrame(
                    columns=["Time", "PenetrationDepth"]))

        # Store data for this ps value
        data_dict[ps] = ps_data

    # Generate plot with all ps values
    defaults = [granMaterial, rel_density, cone_type]
    plot_penetration_depth_ps_comparison(
        data_dict,
        folder,
        defaults,
        vertical_line_stats=(cone_type, rel_density, granMaterial)
    )
