import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import json
import seaborn as sns


"""
--- Description ---
- This script plots the penetration depth of the cone for a single experiment.
- Set the parameters of the test you want plotted 
- All three Hdrops will be plotted on the same graph as subplots along the row
- The experimental data is plotted as a horizontal line at the experimental depth read from the json file which can be found at - https://uwmadison.box.com/s/bhxl6myenyzcr17i5ic10mlwg1fhw129

--- Output ---
- The plot will be saved in the folder where the data was read from.
"""
EXPERIMENTAL_DATA_PATH = "./conePenetration_experimental.json"


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


def plot_penetration_depth(data, folder, defaults: list[str], vertical_line_stats: tuple[float, float, str] = None):
    """
    Plots a line graph of penetration depth over time and saves it as an image.

    Parameters:
    - data: A pandas DataFrame containing 'Time' and 'PenetrationDepth' columns.
    - folder: The directory where the plot should be saved.
    """
    # Set the style and context for the plot
    sns.set(style="whitegrid", context="talk")
    # Read experimental data
    experimental_data = read_experimental_json(EXPERIMENTAL_DATA_PATH)

    h0_exp = experimental_data["0.0"]
    h05_exp = experimental_data["0.5"]
    h1_exp = experimental_data["1.0"]

    h0_depth = h0_exp[vertical_line_stats]
    h05_depth = h05_exp[vertical_line_stats]
    h1_depth = h1_exp[vertical_line_stats]

    # Create a figure with a specific size and 3 subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), sharey=True)
    # Plot data for Hdrop 0.0
    sns.lineplot(ax=axes[0], x="Time",
                 y="PenetrationDepth", data=data[0])
    axes[0].axhline(y=h0_depth, color=(0.8, 0.2, 0.2),
                    linestyle='--', label='Experimental')
    axes[0].set_title("Hdrop 0.0", fontsize=14, fontweight='bold')
    axes[0].set_xlabel("Time (s)", fontsize=14, fontweight='bold')
    axes[0].set_ylabel("Penetration Depth (m)", fontsize=14, fontweight='bold')
    # Set legend to bottom right
    axes[0].legend(loc='lower right')

    # Plot data for Hdrop 0.5
    sns.lineplot(ax=axes[1], x="Time",
                 y="PenetrationDepth", data=data[1])
    axes[1].axhline(y=h05_depth, color=(0.8, 0.2, 0.2),
                    linestyle='--')
    axes[1].set_title("Hdrop 0.5", fontsize=14, fontweight='bold')
    axes[1].set_xlabel("Time (s)", fontsize=14, fontweight='bold')

    # Plot data for Hdrop 1.0
    sns.lineplot(ax=axes[2], x="Time",
                 y="PenetrationDepth", data=data[2])
    axes[2].axhline(y=h1_depth, color=(0.8, 0.2, 0.2),
                    linestyle='--')
    axes[2].set_title("Hdrop 1.0", fontsize=14, fontweight='bold')
    axes[2].set_xlabel("Time (s)", fontsize=14, fontweight='bold')

    # Use scientific notation for the y-axis
    for ax in axes:
        ax.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)

    # Apply tight layout
    plt.tight_layout()

    # Print path where the plot is saved
    defaults_str = "_".join(map(str, defaults))
    # Save the plot as a PNG file in the specified folder
    plt.savefig(os.path.join(
        folder, f"conePenetrationPlot_specs_{defaults_str}.png"), dpi=300)

    print(
        f"Plot saved to: {os.path.join(folder, f'conePenetrationPlot_specs_{defaults_str}.png')}")
    # Show the plot
    plt.show()


if __name__ == "__main__":
    base_dir = "./DEMO_OUTPUT/FSI_ConePenetration/"
    # base_dir = "./DEMO_OUTPUT/FSI_ConePenetrationWithCyl/"

    Hdrop = [0.0, 0.5, 1.0]
    granMaterial = "bead"
    rel_density = 1
    cone_type = 1
    boundary_type = "holmes"
    viscosity_type = "artificial_bilateral"
    kernel_type = "cubic"

    # Single folder
    ps = 1
    s = 0.001  # Initial spacing
    d0 = 1.3
    t = 2e-5  # Time step
    av = 0.5

    use_av = True
    datas = []
    for Hd in Hdrop:
        if use_av:
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
        else:
            folder = os.path.join(
                base_dir,
                f"Hdrop_{Hd}",
                f"granMaterial_{granMaterial}",
                f"relDensity_{rel_density}",
                f"coneType_{cone_type}",
                f"boundaryType_{boundary_type}",
                f"viscosityType_{viscosity_type}",
                f"kernelType_{kernel_type}",
                f"ps_{ps}_s_{s}_d0_{d0}_t_{t}"
            )

        # Print parameters to verify if everything is correct
        json_file_path = os.path.join(folder, 'parameters.json')

        # Read and print the JSON file
        with open(json_file_path, 'r') as file:
            parameters = json.load(file)
            print(json.dumps(parameters, indent=4))

        # load data
        file_path = os.path.join(folder, "cone_penetration_depth.txt")
        data = pd.read_csv(file_path, header='infer')

        # Filter data to include only Time <= 0.5 seconds
        data = data[data["Time"] <= 0.5]
        # zero first row penetration depth and subtract from all rows

        data["PenetrationDepth"] = data["PenetrationDepth"] - \
            data.loc[0, "PenetrationDepth"]
        data.loc[0, "PenetrationDepth"] = 0
        defaults = [granMaterial, rel_density, cone_type, boundary_type, viscosity_type,
                    kernel_type, ps, s, d0, t]
        datas.append(data)
    if use_av:
        defaults.append(av)
    plot_penetration_depth(datas, folder, defaults, vertical_line_stats=(
        cone_type, rel_density, granMaterial))
