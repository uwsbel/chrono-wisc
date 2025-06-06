import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import argparse
import seaborn as sns
from sklearn.metrics import r2_score
from matplotlib.lines import Line2D


dirToPlotName_dict = {
    "DEMO_OUTPUT": "New Code",
    "home": "Old Code"
}

# Function to calculate empirical solution


def empirical_solution(constant, mu_s):
    return (0.14 / mu_s) * constant


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Analyze and plot sphere drop data with PS frequency comparison.")
    parser.add_argument("--no-plot", action="store_true",
                        help="Disable plotting (default: False)")
    parser.add_argument("--ps-values", type=int, nargs='+', default=[1, 2, 5, 10],
                        help="PS frequency values to compare (default: 1 2 5 10)")
    return parser.parse_args()


def main():
    # Set constants
    dens = [700, 2200]  # Sphere densities
    hdrops = [0.2, 0.1, 0.05]  # Drop heights
    rho_granular = 1510  # Granular material density
    r_sphere = 0.0125  # Sphere radius
    viscosity_type = "artificial_bilateral"  # Viscosity type
    kernel_type = "cubic"  # Kernel type
    boundary_type = "adami"  # Boundary type
    s = 0.0025  # Smoothing length
    d0 = 1.3  # Initial particle spacing
    t = 5e-05  # Time step

    # Parse command line arguments
    args = parse_arguments()
    ps_values = args.ps_values  # PS frequency values to compare

    # Set the common rcParams for all plots (matching other scripts)
    plt.rcParams["figure.dpi"] = 300
    plt.rcParams["font.size"] = 10
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams['font.weight'] = 'bold'

    # Define color scheme for PS values using viridis palette (matching other scripts)
    ps_colors = dict(zip(ps_values, sns.color_palette("viridis", len(ps_values))))

    # Markers for densities
    markers = {700: 's', 2200: 'o'}  # square for 700, circle for 2200

    # Dictionary to store results for each PS frequency
    all_results = {}
    all_constants_dict = {}
    all_second_values_dict = {}
    regression_results = {}

    # Collect data for each PS frequency
    for ps in ps_values:
        results = {}
        constants_dict = {}
        second_values_dict = {}

        for d in dens:
            for h in hdrops:
                # Calculate the constant term using the empirical formula
                constant = (d/rho_granular)**0.5 * \
                    (2 * r_sphere) ** (2/3) * h ** (1/3)

                # Construct the file path
                file_path = f"../DEMO_OUTPUT/FSI_Cratering/{viscosity_type}_{boundary_type}_ps{ps}_d{d}_h{h}_s{s}/sphere_penetration_depth.txt"
                file_label = f"Freq. {ps}"

                try:
                    # Read the data file
                    with open(file_path, "r") as f:
                        data = f.readlines()
                        if data:
                            # Extract the last 20 lines for averaging
                            last_20_lines = data[-20:]
                            # Parse the second column values (penetration depth)
                            second_values = [float(line.split()[1])
                                             for line in last_20_lines]
                            # Calculate the average penetration depth
                            average_second_value = sum(
                                second_values) / len(second_values)

                            # Store the results
                            results[file_path] = [
                                constant, average_second_value, file_label, d]
                            if file_path not in constants_dict:
                                constants_dict[file_path] = []
                                second_values_dict[file_path] = []
                            constants_dict[file_path].append(constant)
                            second_values_dict[file_path].append(
                                average_second_value)
                except FileNotFoundError:
                    print(f"Warning: File {file_path} not found. Skipping.")
                except Exception as e:
                    print(f"Error processing {file_path}: {e}")

        # Store the results for this PS frequency
        all_results[ps] = results
        all_constants_dict[ps] = constants_dict
        all_second_values_dict[ps] = second_values_dict

    # Plot combined results for different PS frequencies
    plot_combined_ps(all_results, all_constants_dict, all_second_values_dict,
                     ps_values, ps_colors, markers, regression_results)

    # Print slopes and other regression metrics in table format
    print_regression_results(regression_results, ps_values)


def plot_combined_ps(all_results, all_constants_dict, all_second_values_dict,
                     ps_values, ps_colors, markers, regression_results):
    """Plot combined results for different PS frequencies"""
    # Set the style and context for the plot (matching other scripts)
    sns.set(style="ticks", context="talk")
    
    plots_dir = 'SphereDropPlots'
    os.makedirs(plots_dir, exist_ok=True)

    plt.figure(figsize=(6, 6), dpi=600)

    # Create separate legend for density and PS frequency
    density_legend_elements = [
        Line2D([0], [0], marker='s', color='gray', markerfacecolor='gray', markersize=8, linestyle='None',
               label=r'$\rho_{\mathrm{Sphere}} = 700 \ \mathrm{kg/m^3}$'),
        Line2D([0], [0], marker='o', color='gray', markerfacecolor='gray', markersize=8, linestyle='None',
               label=r'$\rho_{\mathrm{Sphere}} = 2200 \ \mathrm{kg/m^3}$')
    ]

    # Track which elements have been added to the legend
    ps_legend_elements = []

    # Create regression lines by PS frequency
    for ps in ps_values:
        # Skip if no data for this PS value
        if not all_results[ps]:
            continue

        results = all_results[ps]
        constants_dict = all_constants_dict[ps]
        second_values_dict = all_second_values_dict[ps]

        # Get the color for this PS value
        color = ps_colors.get(ps, 'black')

        # For the legend
        ps_legend_elements.append(
            Line2D([0], [0], color=color, linestyle='--',
                   alpha=0.7, label=f'Freq. {ps}')
        )

        # Plot scatter points for this PS value
        for file_path, (constant, second_value, label, density) in results.items():
            plt.scatter(constant, second_value, color=color, marker=markers[density],
                        s=50, alpha=0.7)

        # Perform linear regression for all data points with this PS value
        # First, collect all x and y values
        x_values = []
        y_values = []
        for file_path in results:
            file_constants = constants_dict[file_path]
            file_second_values = second_values_dict[file_path]
            x_values.extend(file_constants)
            y_values.extend(file_second_values)

        if x_values and y_values:
            x = np.array(x_values)
            y = np.array(y_values)

            # Linear regression
            m, c = np.polyfit(x, y, 1)
            y_pred = m * x + c
            r2 = r2_score(y, y_pred)  # Calculate R² score

            # Calculate empirical values and MSE
            empirical_y = empirical_solution(x, 0.3)
            mse = np.mean((y - empirical_y) ** 2)

            # Store regression results
            if ps not in regression_results:
                regression_results[ps] = {}
            regression_results[ps]['slope'] = m
            regression_results[ps]['intercept'] = c
            regression_results[ps]['r2'] = r2
            regression_results[ps]['mse'] = mse

            # Plot regression line
            x_discretized = np.linspace(0, 0.08, 100)
            y_discretized = m * x_discretized + c
            plt.plot(x_discretized, y_discretized,
                     color=color, linestyle='--', alpha=0.7)

    # Add empirical solution line
    constants = np.linspace(0, 0.08, 100)
    empirical_values = empirical_solution(constants, 0.3)
    plt.plot(constants, empirical_values, 'k--',
             label=r'Empirical solution $\mu_s = 0.3$')

    # Add legend with two parts - density markers and PS frequency colors
    first_legend = plt.legend(
        handles=density_legend_elements, loc='upper left')
    plt.gca().add_artist(first_legend)

    # Add second legend for PS frequencies and empirical solution
    all_elements = ps_legend_elements + [Line2D([0], [0], color='k', linestyle='--',
                                                label="Experimental")]
    plt.legend(handles=all_elements, loc='lower right', fontsize=6, framealpha=0.7,
           prop={'weight': 'bold'}, markerscale=0.7,
           borderpad=0.3, labelspacing=0.2, handlelength=1.5)

    # Add plot labels and formatting
    plt.xlabel(r'$(\rho_{\mathrm{sphere}} / \rho_{\mathrm{granular}})^{1/2} (2R_{\mathrm{sphere}})^{2/3} H_{\mathrm{drop}}^{1/3}$',
               fontsize=14, fontweight='bold')
    plt.ylabel(r'$D_{\mathrm{sphere}} \ (\mathrm{m})$',
               fontsize=14, fontweight='bold')

    # Set axis limits and grid (matching other scripts' style)
    plt.grid(False)  # Remove grid to match other scripts
    plt.tick_params(axis='both', which='major', labelsize=12)
    plt.xlim(0, 0.08)  # Set x-axis limit
    plt.ylim(0, 0.04)  # Set y-axis limit

    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'ps_frequency_comparison.png'))
    plt.close()

    print(
        f"Plot saved to {os.path.join(plots_dir, 'ps_frequency_comparison.png')}")


def print_regression_results(regression_results, ps_values):
    """Print regression results in a formatted table"""
    print("\nRegression Results by PS Frequency:")
    print(f"{'PS Freq.':<10} {'Slope':<10} {'Intercept':<12} {'R²':<10} {'MSE':<15}")
    print("-" * 60)

    for ps in sorted(ps_values):
        if ps in regression_results:
            values = regression_results[ps]
            print(f"{ps:<10} {values['slope']:<10.4f} {values['intercept']:<12.4f} "
                  f"{values['r2']:<10.4f} {values['mse']:<15.8f}")

    # Compute and print percentage differences between PS frequencies
    if len(regression_results) > 1:
        print("\nPercentage Differences in Slope (relative to PS=1):")
        if 1 in regression_results:
            base_slope = regression_results[1]['slope']
            print(f"{'PS Freq.':<10} {'% Diff':<10}")
            print("-" * 25)
            for ps in sorted(ps_values):
                if ps in regression_results and ps != 1:
                    current_slope = regression_results[ps]['slope']
                    pct_diff = ((current_slope - base_slope) /
                                base_slope) * 100
                    print(f"{ps:<10} {pct_diff:<10.2f}")


if __name__ == "__main__":
    main()
