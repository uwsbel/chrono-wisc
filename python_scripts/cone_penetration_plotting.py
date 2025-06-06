import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import json
import seaborn as sns
"""
--- Description ---
- Plots penetration depth vs time for a single Relative Density and a single cone type across all three Hdrops
- The experimental data is plotted as a horizontal line at the experimental depth read from the json file which can be found at - https://uwmadison.box.com/s/bhxl6myenyzcr17i5ic10mlwg1fhw129
--- Output ---
- Plots are saved by relDen_ConeType_Hdrop.png in paper_plots/cone_penetration
"""
EXPERIMENTAL_DATA_PATH = "./conePenetration_experimental.json"


# Map spacing to time step
time_step_dict = {
    0.001: 2e-5,
    0.0005: 1e-5,
    0.002: 5e-5
}

# Read correct experimental results


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


def print_error_metrics(metrics_list):
    """
    Print error metrics in a formatted table.
    
    Parameters:
    - metrics_list: List of dictionaries containing error metrics for each Hdrop
    """
    hdrop_values = ["0.0", "0.5", "1.0"]
    
    print("\n" + "="*80)
    print(f"{'PENETRATION DEPTH ERROR METRICS':^80}")
    print("="*80)
    print(f"{'Hdrop':<10} {'Sim Depth (m)':<15} {'Exp Depth (m)':<15} {'Abs Error (m)':<15} {'Rel Error (%)':<15}")
    print("-"*80)
    
    for i, metrics in enumerate(metrics_list):
        print(f"{hdrop_values[i]:<10} {metrics['Simulation Value (m)']:.6e} {metrics['Experimental Value (m)']:.6e} "
              f"{metrics['Absolute Error (m)']:.6e} {metrics['Relative Error (%)']:.2f}")
    
    print("="*80)


def plot_penetration_depth(data, folder, defaults: list[str], vertical_line_stats: tuple[float, float, str] = None):
    """
    Plots a line graph of penetration depth over time and saves it as an image.

    Parameters:
    - data: A pandas DataFrame containing 'Time' and 'PenetrationDepth' columns.
    - folder: The directory where the plot should be saved.
    - defaults: A list containing [relDen, ConeType, Hdrop] values.
    - vertical_line_stats: Tuple containing parameters for experimental data lookup.
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
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), sharey=True)

    # Define matte, aesthetically pleasing colors
    # Using pastel palette for a more matte finish
    sim_color = (0.35, 0.6, 0.6)  # Matte teal
    exp_color = (0.5, 0.5, 0.5)   # Gray for experimental data

    # Plot data for Hdrop 0.0
    sns.lineplot(ax=axes[0], x="Time",
                 y="PenetrationDepth", data=data[0], color=sim_color)
    axes[0].axhline(y=h0_depth, color=exp_color,
                    linestyle='--', label='Experimental')
    # axes[0].set_title("Hdrop 0.0", fontsize=14, fontweight='bold')
    # Remove x-axis label
    axes[0].set_xlabel("Time (s)", fontsize=14, fontweight='bold')
    axes[0].set_ylabel("Penetration Depth (m)", fontsize=14, fontweight='bold')
    # Set legend to bottom right
    axes[0].legend(loc='lower right')

    # Plot data for Hdrop 0.5
    sns.lineplot(ax=axes[1], x="Time",
                 y="PenetrationDepth", data=data[1], color=sim_color)
    axes[1].axhline(y=h05_depth, color=exp_color,
                    linestyle='--')
    # axes[1].set_title("Hdrop 0.5", fontsize=14, fontweight='bold')
    # Remove x-axis label
    axes[1].set_xlabel("Time (s)", fontsize=14, fontweight='bold')

    # Plot data for Hdrop 1.0
    sns.lineplot(ax=axes[2], x="Time",
                 y="PenetrationDepth", data=data[2], color=sim_color)
    axes[2].axhline(y=h1_depth, color=exp_color,
                    linestyle='--')
    # axes[2].set_title("Hdrop 1.0", fontsize=14, fontweight='bold')
    # Remove x-axis label
    axes[2].set_xlabel("Time (s)", fontsize=14, fontweight='bold')

    # Use scientific notation for the y-axis and remove grid lines
    for ax in axes:
        ax.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.grid(False)

    # Apply tight layout
    plt.tight_layout()

    # Ensure the paper_plots/cone_penetration directory exists
    output_dir = os.path.join("paper_plots", "cone_penetration")
    os.makedirs(output_dir, exist_ok=True)

    # Calculate and print error metrics
    error_metrics = []
    for i, df in enumerate(data):
        # Get final penetration depth from simulation
        final_sim_depth = df['PenetrationDepth'].iloc[-1]
        
        # Get corresponding experimental depth
        exp_depths = [h0_depth, h05_depth, h1_depth]
        exp_depth = exp_depths[i]
        
        # Calculate metrics
        metrics = calculate_error_metrics(final_sim_depth, exp_depth)
        error_metrics.append(metrics)
    
    # Print error metrics
    print_error_metrics(error_metrics)

    # Extract relDen and ConeType from defaults
    if len(defaults) >= 2:
        granMaterial = defaults[0]
        rel_den = defaults[1]
        cone_type = defaults[2]

        # Save the plot as a PNG file with the correct naming convention: relDen_ConeType_Hdrop.png
        # Note: This function creates a single plot with all three Hdrop values (0.0, 0.5, 1.0)
        # So we're using a single filename for the combined plot
        output_filename = f"{granMaterial}_{rel_den}_{cone_type}.png"
        output_path = os.path.join(output_dir, output_filename)
        plt.savefig(output_path, dpi=600)

        print(f"Plot saved to: {output_path}")
    else:
        print(
            "Error: defaults list should contain at least [relDen, ConeType] values")

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

    ps = 1  # The ps values to compare
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

    # ps = 1  # The ps values to compare
    # s = 0.001
    # d0 = 1.3
    # t = time_step_dict[s]
    # av = 0.2
    datas = []
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

        # Print parameters to verify if everything is correct
        json_file_path = os.path.join(folder, 'parameters.json')

        # load data
        file_path = os.path.join(folder, "cone_penetration_depth.txt")
        data = pd.read_csv(file_path, header='infer')

        # Filter data to include only Time <= 0.5 seconds
        data = data[data["Time"] <= 0.5]
        # zero first row penetration depth and subtract from all rows

        data["PenetrationDepth"] = data["PenetrationDepth"] - \
            data.loc[0, "PenetrationDepth"]
        data.loc[0, "PenetrationDepth"] = 0
        defaults = [granMaterial, rel_density, cone_type]
        datas.append(data)

    plot_penetration_depth(datas, folder, defaults, vertical_line_stats=(
        cone_type, rel_density, granMaterial))
