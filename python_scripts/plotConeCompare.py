import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import json
import seaborn as sns
import json

"""
--- Description ---
- This script plots the penetration depth of the cone and compares across different parameters - this can be set by the user in the main function
- Set the parameters of the test you want plotted 
- All three Hdrops will be plotted on the same graph as subplots along the row
- The experimental data is plotted as a horizontal line at the experimental depth read from the json file which can be found at - https://uwmadison.box.com/s/bhxl6myenyzcr17i5ic10mlwg1fhw129

--- Output ---
- The plot will be saved in folder conePenetrationCompare with sub folders for granMaterial, coneType, relDensity
- File name will indicate what was being compared and what were the defaults
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


def plot_Hdrop_compare(datas: dict[str, list[pd.DataFrame]], folder, compare: list[str], defaults: list[str], plot_text: str, vertical_line_stats: tuple[float, float, str] = None):
    """
    Plots a comparison of penetration depth over time for different Hdrop values.

    Parameters:
    - dataH0, dataH05, dataH1: DataFrames containing 'Time' and 'PenetrationDepth' columns for Hdrop 0.0, 0.5, and 1.0.
    - folder: The directory where the plot should be saved.
    - defaults: List of default that were not compared in file name.
    - plot_text: Text to be displayed in the plot - can be anything.
    """
    # Ensure the folder exists
    os.makedirs(folder, exist_ok=True)
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
    for comp in compare:
        sns.lineplot(ax=axes[0], x="Time",
                     y="PenetrationDepth", data=datas[comp][0], label=comp)
    # Add horizontal line at experimental depth
    axes[0].axhline(y=h0_depth, color=(0.8, 0.2, 0.2),
                    linestyle='--', label='Experimental')
    axes[0].set_title("Hdrop 0.0", fontsize=14, fontweight='bold')
    axes[0].set_xlabel("Time (s)", fontsize=14, fontweight='bold')
    axes[0].set_ylabel("Penetration Depth (m)", fontsize=14, fontweight='bold')
    if plot_text:
        axes[0].text(0.05, 0.95, plot_text, transform=axes[0].transAxes,
                     fontsize=14, fontweight='bold')

    # Set legend to bottom right
    axes[0].legend(loc='lower right')

    # Plot data for Hdrop 0.5
    for comp in compare:
        sns.lineplot(ax=axes[1], x="Time",
                     y="PenetrationDepth", data=datas[comp][1])
    axes[1].axhline(y=h05_depth, color=(0.8, 0.2, 0.2),
                    linestyle='--')
    axes[1].set_title("Hdrop 0.5", fontsize=14, fontweight='bold')
    axes[1].set_xlabel("Time (s)", fontsize=14, fontweight='bold')

    # Plot data for Hdrop 1.0
    for comp in compare:
        sns.lineplot(ax=axes[2], x="Time",
                     y="PenetrationDepth", data=datas[comp][2])
    axes[2].axhline(y=h1_depth, color=(0.8, 0.2, 0.2), linestyle='--')
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
        folder, f"{compare[0]}vs{compare[1]}ComparisonPlot_defaults_{defaults_str}.png"), dpi=300)

    print(
        f"Plot saved to: {os.path.join(folder, f'{compare[0]}vs{compare[1]}ComparisonPlot_defaults_{defaults_str}.png')}")
    # Show the plot
    plt.show()


def create_output_folder(base_folder, granMaterial, cone_type, rel_density, ps, s, d0, t):
    """
    Creates a nested directory structure for saving plots.

    Parameters:
    - base_folder: The base directory for saving plots.
    - granMaterial, cone_type, rel_density, ps, s, d0, t: Parameters for folder naming.
    """
    folder_components = [
        f"granMaterial_{granMaterial}",
        f"coneType_{cone_type}",
        f"relDensity_{rel_density}"
    ]

    folder = base_folder
    os.makedirs(folder, exist_ok=True)
    for component in folder_components:
        folder = os.path.join(folder, component)
        os.makedirs(folder, exist_ok=True)

    return folder


if __name__ == "__main__":
    base_dir = "./DEMO_OUTPUT/FSI_ConePenetration/"
    # base_dir = "./DEMO_OUTPUT/FSI_ConePenetrationWithCyl/"
    # All options
    rel_densities = [0, 1]
    granMaterials = ["bead", "sand"]
    boundary_types = ["adami", "holmes"]
    viscosity_types = ["artificial_bilateral", "artificial_unilateral"]
    kernel_types = ["cubic", "wendland"]
    s_types = [0.002, 0.001]
    ps_types = [1, 2, 4, 8]
    d0_types = [1.1, 1.2, 1.3, 1.4, 1.5]
    t_types = [5e-5, 2e-5]

    # Subfolders and defaults for ones not getting compared
    Hdrop = [0.0, 0.5, 1.0]
    granMaterial = "bead"
    rel_density = 0
    cone_type = 2
    boundary_type = "holmes"
    viscosity_type = "artificial_bilateral"
    kernel_type = "cubic"
    av = 0.5

    # Single folder
    ps = 1
    s = 0.0005  # Initial spacing
    d0 = 1.3
    t = 1e-5  # Time step

    compare = "s"
    include_av = True

    if compare == "boundaryType":
        infos = {}
        for boundary_type in boundary_types:
            datas = []
            for Hd in Hdrop:
                if include_av:
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
                data["PenetrationDepth"] = data["PenetrationDepth"] - \
                    data.loc[0, "PenetrationDepth"]
                data.loc[0, "PenetrationDepth"] = 0
                datas.append(data)
            infos[boundary_type] = datas

        if (base_dir == "./DEMO_OUTPUT/FSI_ConePenetration/"):
            folder = create_output_folder(
                "conePenetrationCompare", granMaterial, cone_type, rel_density, ps, s, d0, t)
        else:
            folder = create_output_folder(
                "conePenetrationCompareWithCyl", granMaterial, cone_type, rel_density, ps, s, d0, t)
        if include_av:
            plot_Hdrop_compare(infos, folder, boundary_types,
                               defaults=[kernel_type, viscosity_type, ps, s, d0, t, av], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}_av_{av}", vertical_line_stats=(cone_type, rel_density, granMaterial))
        else:
            plot_Hdrop_compare(infos, folder, boundary_types,
                               defaults=[kernel_type, viscosity_type, ps, s, d0, t], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}", vertical_line_stats=(cone_type, rel_density, granMaterial))

    if compare == "viscosityType":
        infos = {}
        for viscosity_type in viscosity_types:
            datas = []
            for Hd in Hdrop:
                if include_av:
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
                data["PenetrationDepth"] = data["PenetrationDepth"] - \
                    data.loc[0, "PenetrationDepth"]
                data.loc[0, "PenetrationDepth"] = 0
                datas.append(data)

            infos[viscosity_type] = datas

        if (base_dir == "./DEMO_OUTPUT/FSI_ConePenetration/"):
            folder = create_output_folder(
                "conePenetrationCompare", granMaterial, cone_type, rel_density, ps, s, d0, t)
        else:
            folder = create_output_folder(
                "conePenetrationCompareWithCyl", granMaterial, cone_type, rel_density, ps, s, d0, t)
        if include_av:
            plot_Hdrop_compare(infos, folder, viscosity_types,
                               defaults=[kernel_type, boundary_type, ps, s, d0, t, av], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}_av_{av}", vertical_line_stats=(cone_type, rel_density, granMaterial))
        else:
            plot_Hdrop_compare(infos, folder, viscosity_types,
                               defaults=[kernel_type, boundary_type, ps, s, d0, t], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}", vertical_line_stats=(cone_type, rel_density, granMaterial))

    if compare == "kernelType":
        infos = {}
        for kernel_type in kernel_types:
            datas = []
            for Hd in Hdrop:
                if include_av:
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
                data["PenetrationDepth"] = data["PenetrationDepth"] - \
                    data.loc[0, "PenetrationDepth"]
                data.loc[0, "PenetrationDepth"] = 0
                datas.append(data)

            infos[kernel_type] = datas

        if (base_dir == "./DEMO_OUTPUT/FSI_ConePenetration/"):
            folder = create_output_folder(
                "conePenetrationCompare", granMaterial, cone_type, rel_density, ps, s, d0, t)
        else:
            folder = create_output_folder(
                "conePenetrationCompareWithCyl", granMaterial, cone_type, rel_density, ps, s, d0, t)
        if include_av:
            plot_Hdrop_compare(infos, folder, kernel_types,
                               defaults=[viscosity_type, boundary_type, ps, s, d0, t, av], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}_av_{av}", vertical_line_stats=(cone_type, rel_density, granMaterial))
        else:
            plot_Hdrop_compare(infos, folder, kernel_types,
                               defaults=[viscosity_type, boundary_type, ps, s, d0, t], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}", vertical_line_stats=(cone_type, rel_density, granMaterial))

    if compare == "s":
        infos = {}
        for i, s in enumerate(s_types):
            datas = []
            for Hd in Hdrop:
                if include_av:
                    folder = os.path.join(
                        base_dir,
                        f"Hdrop_{Hd}",
                        f"granMaterial_{granMaterial}",
                        f"relDensity_{rel_density}",
                        f"coneType_{cone_type}",
                        f"boundaryType_{boundary_type}",
                        f"viscosityType_{viscosity_type}",
                        f"kernelType_{kernel_type}",
                        f"ps_{ps}_s_{s}_d0_{d0}_t_{t_types[i]}_av_{av}"
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
                        f"ps_{ps}_s_{s}_d0_{d0}_t_{t_types[i]}"
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
                data["PenetrationDepth"] = data["PenetrationDepth"] - \
                    data.loc[0, "PenetrationDepth"]
                data.loc[0, "PenetrationDepth"] = 0
                datas.append(data)

            infos[s] = datas

        if (base_dir == "./DEMO_OUTPUT/FSI_ConePenetration/"):
            folder = create_output_folder(
                "conePenetrationCompare", granMaterial, cone_type, rel_density, ps, s, d0, t)
        else:
            folder = create_output_folder(
                "conePenetrationCompareWithCyl", granMaterial, cone_type, rel_density, ps, s, d0, t)
        if include_av:
            plot_Hdrop_compare(infos, folder, s_types,
                               defaults=[kernel_type, viscosity_type, boundary_type, ps, d0, t, av], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}_av_{av}", vertical_line_stats=(cone_type, rel_density, granMaterial))
        else:
            plot_Hdrop_compare(infos, folder, s_types,
                               defaults=[kernel_type, viscosity_type, boundary_type, ps, d0, t], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}", vertical_line_stats=(cone_type, rel_density, granMaterial))

    if compare == "d0":
        infos = {}
        for i, d0 in enumerate(d0_types):
            datas = []
            for Hd in Hdrop:
                if include_av:
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
                data["PenetrationDepth"] = data["PenetrationDepth"] - \
                    data.loc[0, "PenetrationDepth"]
                data.loc[0, "PenetrationDepth"] = 0
                datas.append(data)

            infos[d0] = datas
        if (base_dir == "./DEMO_OUTPUT/FSI_ConePenetration/"):
            folder = create_output_folder(
                "conePenetrationCompare", granMaterial, cone_type, rel_density, ps, s, d0, t)
        else:
            folder = create_output_folder(
                "conePenetrationCompareWithCyl", granMaterial, cone_type, rel_density, ps, s, d0, t)
        if include_av:
            plot_Hdrop_compare(infos, folder, d0_types,
                               defaults=[kernel_type, viscosity_type, boundary_type, ps, s, t, av], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}_av_{av}", vertical_line_stats=(cone_type, rel_density, granMaterial))
        else:
            plot_Hdrop_compare(infos, folder, d0_types,
                               defaults=[kernel_type, viscosity_type, boundary_type, ps, s, t], plot_text=f"ps_{ps}_s_{s}_d0_{d0}_t_{t}", vertical_line_stats=(cone_type, rel_density, granMaterial))
