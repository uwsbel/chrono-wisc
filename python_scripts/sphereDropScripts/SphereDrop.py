import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import argparse
from sklearn.metrics import r2_score
from matplotlib.lines import Line2D

def empirical_solution(constant, mu_s):
    return (0.14 / mu_s) * constant

def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Analyze and plot sphere drop data with PS=1 only.")
    parser.add_argument("--no-plot", action="store_true",
                        help="Disable plotting (default: False)")
    return parser.parse_args()

def main():
    # Set constants
    dens = [700, 2200]  # Sphere densities
    hdrops = [0.2, 0.1, 0.05]  # Drop heights
    rho_granular = 1510  # Granular material density
    r_sphere = 0.0125  # Sphere radius
    viscosity_type = "artificial_bilateral"  # Viscosity type
    boundary_type = "adami"  # Boundary type
    s = 0.0025  # Smoothing length
    ps = 1  # Fixed PS value

    # Parse command line arguments
    args = parse_arguments()

    # Markers for densities
    markers = {700: 's', 2200: 'o'}  # square for 700, circle for 2200

    # Dictionary to store results
    results = {}
    constants_dict = {}
    second_values_dict = {}
    regression_results = {}

    for d in dens:
        for h in hdrops:
            # Calculate the constant term using the empirical formula
            constant = (d/rho_granular)**0.5 * \
                (2 * r_sphere) ** (2/3) * h ** (1/3)

            # Construct the file path
            file_path = f"../DEMO_OUTPUT/FSI_Cratering/{viscosity_type}_{boundary_type}_ps{ps}_d{d}_h{h}_s{s}/sphere_penetration_depth.txt"

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
                            constant, average_second_value, d]
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

    # Plot results if not disabled
    if not args.no_plot:
        plot_ps1_only(results, constants_dict, second_values_dict, markers, regression_results)

    # Print regression results
    if regression_results:
        print_regression_results(regression_results)

def plot_ps1_only(results, constants_dict, second_values_dict, markers, regression_results):
    """Plot results for PS=1 only"""
    plots_dir = 'SphereDropPlots'
    os.makedirs(plots_dir, exist_ok=True)

    plt.figure(figsize=(6, 5), dpi=600)

    # Create legend for density
    density_legend_elements = [
        Line2D([0], [0], marker='s', color='blue', markerfacecolor='blue', markersize=8, linestyle='None',
               label=r'$\rho_{\mathrm{Sphere}} = 700 \ \mathrm{kg/m^3}$'),
        Line2D([0], [0], marker='o', color='blue', markerfacecolor='blue', markersize=8, linestyle='None',
               label=r'$\rho_{\mathrm{Sphere}} = 2200 \ \mathrm{kg/m^3}$')
    ]

    # Plot scatter points
    for file_path, (constant, second_value, density) in results.items():
        plt.scatter(constant, second_value, color='blue', marker=markers[density],
                    s=50, alpha=0.7)

    # Perform linear regression for all data points
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
        regression_results['slope'] = m
        regression_results['intercept'] = c
        regression_results['r2'] = r2
        regression_results['mse'] = mse

        # Plot regression line
        x_discretized = np.linspace(0, 0.08, 100)
        y_discretized = m * x_discretized + c
        plt.plot(x_discretized, y_discretized,
                 color='blue', linestyle='--', alpha=0.7)

    # Add empirical solution line
    constants = np.linspace(0, 0.08, 100)
    empirical_values = empirical_solution(constants, 0.3)
    plt.plot(constants, empirical_values, 'k--',
             label=r'Empirical solution $\mu_s = 0.3$')

    # Add empirical solution formula as text
    # plt.text(0.03, 0.035,
    #          r'$D_{\mathrm{sphere}} = \frac{0.14}{\mu_s} \cdot (\frac{\rho_{\mathrm{sphere}}}{\rho_{\mathrm{granular}}})^{1/2} (2R_{\mathrm{sphere}})^{2/3} H_{\mathrm{drop}}^{1/3}$',
    #          fontsize=12, bbox=dict(facecolor='white', alpha=0.5))

    # Add legend with density markers and empirical solution
    first_legend = plt.legend(
        handles=density_legend_elements, loc='upper left')
    plt.gca().add_artist(first_legend)

    # Add second legend for empirical solution only
    plt.legend(handles=[Line2D([0], [0], color='k', linestyle='--',
                             label=r'Empirical solution $\mu_s = 0.3$')], 
             loc='lower right')

    # Add plot labels and formatting
    plt.xlabel(r'$(\rho_{\mathrm{sphere}} / \rho_{\mathrm{granular}})^{1/2} (2R_{\mathrm{sphere}})^{2/3} H_{\mathrm{drop}}^{1/3}$',
               fontsize=15, fontweight='bold')
    plt.ylabel(r'$D_{\mathrm{sphere}} \ (\mathrm{m})$',
               fontsize=15, fontweight='bold')


    # Set axis limits and grid
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim(0, 0.08)  # Set x-axis limit
    plt.ylim(0, 0.04)  # Set y-axis limit

    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'ps1_only_plot.png'), dpi=600)
    plt.close()

    print(f"Plot saved to {os.path.join(plots_dir, 'ps1_only_plot.png')}")

def print_regression_results(regression_results):
    """Print regression results in a formatted table"""
    print("\nRegression Results:")
    print(f"{'Slope':<10} {'Intercept':<12} {'R²':<10} {'MSE':<15}")
    print("-" * 50)
    print(f"{regression_results['slope']:<10.4f} {regression_results['intercept']:<12.4f} "
          f"{regression_results['r2']:<10.4f} {regression_results['mse']:<15.8f}")

if __name__ == "__main__":
    main()