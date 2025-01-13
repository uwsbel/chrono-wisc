import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import argparse
from sklearn.metrics import r2_score


dirToPlotName_dict = {
    "DEMO_OUTPUT": "New Code",
    "home": "Old Code"
}

# Function to calculate empirical solution


def empirical_solution(constant, mu_s):
    return (0.14 / mu_s) * constant


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Analyze and plot sphere drop data.")
    parser.add_argument("--no-plot", action="store_true",
                        help="Disable plotting (default: False)")
    return parser.parse_args()


dens = [700, 2200]
hdrops = [0.2, 0.1, 0.05]
rho_granular = 1510
r_sphere = 0.0125
viscosity_type = "artificial_bilateral"
kernel_type = "cubic"
boundary_type = "adami"
s = 0.0025
d0 = 1.3
t = 5e-05
ps = 1

results = {}
constants_dict = {}
second_values_dict = {}
args = parse_arguments()

for d in dens:
    for h in hdrops:

        constant = (d/rho_granular)**0.5 * \
            (2 * r_sphere) ** (2/3) * h ** (1/3)

        file1 = f"../DEMO_OUTPUT/FSI_Cratering/{viscosity_type}_{boundary_type}_ps{ps}_d{d}_h{h}_s{s}_d0{d0}_t{format(t, '.0e')}/sphere_penetration_depth.txt"
        # file1 = f"../DEMO_OUTPUT/FSI_Cratering/{viscosity_type}_{boundary_type}_{kernel_type}_ps{ps}_d{d}_h{h}_s{s}_d0{d0}_t{format(t, '.0e')}/sphere_penetration_depth.txt"
        new_file_label = f"Latest"

        file_list = [file1]
        file_labels = [new_file_label]

        for file, label in zip(file_list, file_labels):
            with open(file, "r") as f:
                data = f.readlines()
                if data:
                    last_20_lines = data[-20:]
                    second_values = [float(line.split()[1])
                                     for line in last_20_lines]
                    average_second_value = sum(
                        second_values) / len(second_values)

            results[file] = [constant, average_second_value, label]
            if file not in constants_dict:
                constants_dict[file] = []
                second_values_dict[file] = []
            constants_dict[file].append(constant)
            second_values_dict[file].append(average_second_value)

# Plotting
colors = ['b']  # Only 1 file
markers = {700: 's', 2200: 'o'}  # square for 700, circle for 2200
plots_dir = 'SphereDropPlots'
os.makedirs(plots_dir, exist_ok=True)

# Dictionary to store slopes
slopes_dict = {density: {} for density in dens}

# Function to plot and fit linear regression


def plot_and_fit(density, filename):
    plt.figure(figsize=(8, 6), dpi=600)
    added_labels = set()
    for idx, (file, (constant, second_value, label)) in enumerate(results.items()):
        file_density = int(file.split('_d')[1].split('_')[0])
        if file_density == density:
            label = results[file][2]
            if label not in added_labels:
                plt.scatter(constant, second_value, color=colors[idx % len(
                    colors)], marker=markers[density], label=label, s=50, alpha=0.5)
                added_labels.add(label)
            else:
                plt.scatter(constant, second_value, color=colors[idx % len(
                    colors)], marker=markers[density], s=50, alpha=0.5)

            # Linear regression
            folder_name = file.split('/')[1]
            x = np.concatenate([np.array(constants_dict[f])
                               for f in constants_dict if f.split('/')[1] == folder_name and int(f.split('_d')[1].split('_')[0]) == density])
            y = np.concatenate([np.array(second_values_dict[f])
                               for f in second_values_dict if f.split('/')[1] == folder_name and int(f.split('_d')[1].split('_')[0]) == density])

            m, c = np.polyfit(x, y, 1)
            y_pred = m * x + c
            r2 = r2_score(y, y_pred)  # Calculate R² score

            # Light line
            x_discretized = np.linspace(0, max(x) + 0.05, 100)
            y_discretized = m * x_discretized + c

            plt.plot(x_discretized, y_discretized, color=colors[idx %
                     len(colors)], linestyle='--', alpha=0.3)

            # Store slope and R² in dictionary
            if folder_name not in slopes_dict[density]:
                slopes_dict[density][folder_name] = {}
            slopes_dict[density][folder_name][h] = (m, r2)

    # Add empirical solution line
    constants = np.linspace(0, 0.08, 100)
    empirical_values = empirical_solution(constants, 0.3)
    plt.plot(constants, empirical_values, 'k--',
             label=r'Empirical solution $\mu_s = 0.3$')

    # Add empirical solution formula as text
    plt.text(0.04, 0.01, r'$D_{\mathrm{sphere}} = \frac{0.14}{\mu_s} \cdot (\frac{\rho_{\mathrm{sphere}}}{\rho_{\mathrm{granular}}})^{1/2} (2R_{\mathrm{sphere}})^{2/3} H_{\mathrm{drop}}^{1/3}$',
             fontsize=12, bbox=dict(facecolor='white', alpha=0.5))

    plt.xlabel(r'$(\rho_{\mathrm{sphere}} / \rho_{\mathrm{granular}})^{1/2} (2R_{\mathrm{sphere}})^{2/3} H_{\mathrm{drop}}^{1/3}$',
               fontsize=14, fontweight='bold')
    plt.ylabel(r'$D_{\mathrm{sphere}} \ (\mathrm{m})$',
               fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.title('Sphere Drop Penetration Depth Analysis',
              fontsize=14, fontweight='bold')
    plt.text(0.06, 0.002, rf'$\rho_{{\mathrm{{sphere}}}} = {density} \ \mathrm{{kg/m^3}}$' '\n' r'$R_{{\mathrm{{sphere}}}} = 0.0125 \ \mathrm{m}$' '\n' r'$\rho_{{\mathrm{{granular}}}} = 1510 \ \mathrm{{kg/m^3}}$',
             fontsize=12, bbox=dict(facecolor='white', alpha=0.5))
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.ylim(0, 0.025 if density == 700 else 0.04)  # Set y-axis limits
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, filename))
    # plt.show()
    # plt.show()


# Plot for density 700
plot_and_fit(700, 'density_700_plot_reduced.png')

# Plot for density 2200
plot_and_fit(2200, 'density_2200_plot_reduced.png')

# Function to plot and fit linear regression for combined densities

regression_results = {}


def plot_combined():
    plt.figure(figsize=(8, 6), dpi=600)
    added_labels = set()
    for idx, (file, (constant, second_value, label)) in enumerate(results.items()):
        file_density = int(file.split('_d')[1].split('_')[0])
        if label not in added_labels:
            plt.scatter(constant, second_value, color=colors[idx % len(colors)],
                        marker=markers[file_density], label=label, s=50, alpha=0.5)
            added_labels.add(label)
        else:
            plt.scatter(constant, second_value, color=colors[idx % len(colors)],
                        marker=markers[file_density], s=50, alpha=0.5)

        # Linear regression
        folder_name = file.split('/')[1]
        x = np.concatenate([np.array(constants_dict[f])
                           for f in constants_dict if f.split('/')[1] == folder_name])
        y = np.concatenate([np.array(second_values_dict[f])
                           for f in second_values_dict if f.split('/')[1] == folder_name])

        m, c = np.polyfit(x, y, 1)
        y_pred = m * x + c
        r2 = r2_score(y, y_pred)  # Calculate R² score

        x_discretized = np.linspace(0, max(x) + 0.05, 100)
        y_discretized = m * x_discretized + c

        # Calculate empirical values for the given constants
        empirical_y = empirical_solution(x, 0.3)
        print(empirical_y)
        print(y)

        # Calculate mean squared error (MSE)
        mse = np.mean((y - empirical_y) ** 2)

        # Print MSE
        print(f"MSE for {folder_name} with density {file_density}: {mse:.8f}")

        if folder_name not in regression_results:
            regression_results[folder_name] = {}
        regression_results[folder_name]['slope'] = m
        regression_results[folder_name]['r2'] = r2  # Store R² score
        # Store MSE in regression results
        regression_results[folder_name]['mse'] = mse

        # Plot the linear regression line
        plt.plot(x_discretized, y_discretized, color=colors[idx % len(colors)],
                 linestyle='--', alpha=0.3)

    # Add empirical solution line
    constants = np.linspace(0, 0.08, 100)
    empirical_values = empirical_solution(constants, 0.3)
    plt.plot(constants, empirical_values, 'k--',
             label=r'Empirical solution $\mu_s = 0.3$')

    # Add empirical solution formula as text
    plt.text(0.04, 0.01, r'$D_{\mathrm{sphere}} = \frac{0.14}{\mu_s} \cdot (\frac{\rho_{\mathrm{sphere}}}{\rho_{\mathrm{granular}}})^{1/2} (2R_{\mathrm{sphere}})^{2/3} H_{\mathrm{drop}}^{1/3}$',
             fontsize=12, bbox=dict(facecolor='white', alpha=0.5))

    plt.xlabel(r'$(\rho_{\mathrm{sphere}} / \rho_{\mathrm{granular}})^{1/2} (2R_{\mathrm{sphere}})^{2/3} H_{\mathrm{drop}}^{1/3}$',
               fontsize=14, fontweight='bold')
    plt.ylabel(r'$D_{\mathrm{sphere}} \ (\mathrm{m})$',
               fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.title('Combined Sphere Drop Penetration Depth Analysis',
              fontsize=14, fontweight='bold')
    plt.text(0.04, 0.002, r'$R_{\mathrm{sphere}} = 0.0125 \ \mathrm{m}$' '\n' r'$\rho_{\mathrm{granular}} = 1510 \ \mathrm{kg/m^3}$',
             fontsize=12, bbox=dict(facecolor='white', alpha=0.5))
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim(0, 0.08)  # Set x-axis limit
    plt.ylim(0, 0.04)  # Set y-axis limit
    plt.tight_layout()
    plt.savefig(os.path.join(
        plots_dir, 'combined_density_plot_consistent.png'))
    # plt.show()


# Plot for combined densities
plot_combined()

# Print slopes in a table format
print("\nSlopes Table:")
print(f"{'Code Version':<30} {'Sphere Density':<30} {'Slope':<10}")
for density, folders in slopes_dict.items():
    for folder, heights in folders.items():
        for height, (slope, r2) in heights.items():
            print(
                f"{dirToPlotName_dict[folder]:<30} {density:<30} {slope:<10.4f}")
# Print slopes and r² values in a table format
print("\nRegression Results Table:")
print(f"{'Code Version':<30} {'Slope':<10} {'R²':<10} {'MSE':<20}")
for folder, values in regression_results.items():
    print(
        f"{dirToPlotName_dict[folder]:<30} {values['slope']:<10.4f} {values['r2']:<10.4f} {values['mse']:<20.8f}")
