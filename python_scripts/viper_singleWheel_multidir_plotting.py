import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import argparse
from matplotlib.ticker import ScalarFormatter, FuncFormatter
from matplotlib.lines import Line2D

# Example command:
# python viper_singleWheel_multidir_plotting.py --vary_ps --data-dirs "./DEMO_OUTPUT" "/path/to/other/output" --legend-names "DEMO 1" "DEMO 2"

slope_dict = {
    "1": r"$0^\circ$",
    "2": r"$2.5^\circ$",
    "3": r"$5^\circ$",
    "4": r"$10^\circ$",
    "5": r"$15^\circ$",
    "6": r"$20^\circ$",
    "7": r"$25^\circ$",
}

# Set the common rcParams for all plots
plt.rcParams["figure.dpi"] = 300
plt.rcParams["font.size"] = 10
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams['font.weight'] = 'bold'


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Plot Viper wheel data with support for multiple directories and frequency comparison')
    parser.add_argument('--vary_ps', action='store_true',
                        help='Plot multiple lines for different frequency values')
    parser.add_argument('--ps-values', type=int, nargs='+', default=[1, 2, 5, 10],
                        help='PS frequency values to compare (default: 1 2 5 10)')
    parser.add_argument('--data-dirs', type=str, nargs='+', default=['./DEMO_OUTPUT'],
                        help='Base directories containing FSI_Viper_Single_Wheel2 data (default: ./DEMO_OUTPUT)')
    parser.add_argument('--legend-names', type=str, nargs='+', default=None,
                        help='Names to use in the legend for each data directory (default: directory names)')
    parser.add_argument('--spacing', type=str, default='0.010',
                        help='Spacing value (default: 0.010)')
    parser.add_argument('--d0', type=float, default=1.2,
                        help='d0 value (default: 1.2)')
    parser.add_argument('--av', type=float, default=0.5,
                        help='av value (default: 0.5)')
    return parser.parse_args()


def read_position_data(file_path):
    # Read data with header row
    data = pd.read_csv(file_path, sep='\t')

    # Drop the initial row (row index 0)
    data = data.iloc[1:].reset_index(drop=True)

    # Convert columns to numeric (they may be read as strings due to scientific notation)
    for col in data.columns:
        data[col] = pd.to_numeric(data[col], errors='coerce')

    # Rename columns to match our expected format
    column_mapping = {
        'time': 'time',
        'x': 'w_pos_x',
        'y': 'w_pos_y',
        'z': 'w_pos_z',
        'vx': 'w_vel_x',
        'vy': 'w_vel_y',
        'vz': 'w_vel_z',
        'ax': 'angvel_x',
        'ay': 'angvel_y',
        'az': 'angvel_z',
        'fx': 'force_x',
        'fy': 'force_y',
        'fz': 'force_z',
        'tx': 'torque_x',
        'ty': 'torque_y',
        'tz': 'torque_z'
    }

    data = data.rename(columns=column_mapping)
    return data


def scientific_notation_formatter(x, pos):
    return f'{x:.0e}'


def main():
    # Parse command line arguments
    args = parse_arguments()
    
    # Get parameters from arguments
    vary_ps = args.vary_ps
    ps_values = args.ps_values if vary_ps else [1]  # Use just the default PS if not varying
    data_dirs = args.data_dirs
    legend_names = args.legend_names
    spacing = args.spacing
    d0 = args.d0
    av = args.av
    
    # Set legend names (use directory names if not provided)
    if legend_names is None:
        legend_names = [os.path.basename(dir_path) for dir_path in data_dirs]
    
    # Ensure we have the same number of legend names as data directories
    if len(legend_names) != len(data_dirs):
        print(f"Error: Number of legend names ({len(legend_names)}) does not match the number of data directories ({len(data_dirs)})")
        return

    # Set the style and context for the plot
    sns.set(style="ticks", context="talk")

    slopes = ["1", "2", "3", "4", "5", "6", "7"]
    rad_plus_grouser = 0.225 + 0.025
    calc_start_time = 3
    
    # Create a figure with proper dimensions
    plt.figure(figsize=(6, 6))

    # Define color palette for PS values
    ps_colors = dict(zip(ps_values, sns.color_palette("viridis", len(ps_values))))
    
    # Define linestyles for different directories
    linestyles = ['-', '--', '-.', ':']
    if len(data_dirs) > len(linestyles):
        # If we have more directories than linestyles, cycle through them
        linestyles = linestyles * (len(data_dirs) // len(linestyles) + 1)
    
    # Define color for experimental data
    exp_color = (0.5, 0.5, 0.5)  # Gray for experimental data

    # Create a nested dictionary to store data for each directory, PS, and slope
    data_dict = {}  # (dir_idx, ps) -> {slope -> data}
    slip_dict = {}  # (dir_idx, ps) -> {slope -> slip}

    # Initialize dictionaries
    for dir_idx in range(len(data_dirs)):
        for ps in ps_values:
            data_dict[(dir_idx, ps)] = {}
            slip_dict[(dir_idx, ps)] = {}

    # Load data for each directory, PS, and slope
    for dir_idx, base_dir in enumerate(data_dirs):
        for ps in ps_values:
            for slope in slopes:
                file_path = f'{base_dir}/FSI_Viper_Single_Wheel2/ps_{ps}_s_{spacing}_d0_{d0}_av_{av}/{slope}/results.txt'
                try:
                    data = read_position_data(file_path)
                    data_dict[(dir_idx, ps)][slope] = data
                except FileNotFoundError:
                    print(f"Warning: File not found for dir={legend_names[dir_idx]}, ps={ps}, slope={slope}. Skipping.")
                    continue

    # Calculate slip for each directory, PS, and slope
    for (dir_idx, ps), slopes_data in data_dict.items():
        for slope, data in slopes_data.items():
            # Find the closest time value to calc_start_time
            closest_start_idx = (data['time'] - calc_start_time).abs().idxmin()
            pos_at_start = data.loc[closest_start_idx, 'w_pos_x']

            # Now get the pos at the last time step
            pos_at_last_step = data.iloc[-1]['w_pos_x']

            # Then the averaged velocity is
            avg_vel = (pos_at_last_step - pos_at_start) / \
                (data.iloc[-1]['time'] - data.loc[closest_start_idx, 'time'])

            # For this duration, compute the average angular velocity in z of the wheel
            ang_vel_z = data['angvel_z']
            avg_ang_vel = ang_vel_z.mean()
            print(f"Dir={legend_names[dir_idx]}, PS={ps}, Slope={slope}: Average Angular Velocity = {avg_ang_vel}")

            # Compute the slip
            slip = 1 - avg_vel / (avg_ang_vel * rad_plus_grouser)
            print(f"Dir={legend_names[dir_idx]}, PS={ps}, Slope={slope}: Slip = {slip}")
            slip_dict[(dir_idx, ps)][slope] = slip

    # Keep track of the legend elements
    legend_elements = []

    # Plot slip vs slope for each directory and PS value
    for dir_idx, dir_name in enumerate(legend_names):
        linestyle = linestyles[dir_idx % len(linestyles)]
        
        for ps in ps_values:
            key = (dir_idx, ps)
            if not slip_dict[key]:  # Skip if no data for this PS and directory
                continue

            # Get slopes and slips for this PS and directory
            valid_slopes = [s for s in slopes if s in slip_dict[key]]
            slopes_numeric = [float(slope_dict[slope].strip('$^\\circ$'))
                              for slope in valid_slopes]
            slips = [slip_dict[key][slope] for slope in valid_slopes]

            # Get color for this PS value
            color = ps_colors[ps]

            # Plot with appropriate styling
            plt.plot(slips, slopes_numeric, 'o-', 
                     color=color, 
                     linestyle=linestyle,
                     markersize=8,
                     linewidth=2)
            
            # Add to legend elements
            legend_elements.append(
                Line2D([0], [0], color=color, linestyle=linestyle, marker='o',
                       label=f"{dir_name} Freq. {ps}", markersize=6, linewidth=1.5))

    # Use the original experimental data
    experimental2 = np.array([
        [3.1504614500604013, 0.27322413552669644],
        [5.154396334609679, 5.237445584511807],
        [13.465437029246175, 10.026680600615933],
        [43.41343674026234, 14.961499624489019],
        [75.59749928174078, 20.23066622535026],
        [88.45956494491666, 25.09928778925685]
    ])

    dem_data = np.array([
        [0.0, 0.04],
        [2.5, 0.06],
        [5.0, 0.105],
        [10.0, 0.21],
        [15.0, 0.415],
        [20.0, 0.62],
        [25.0, 0.72],
    ])

    # Convert slip percentages to decimal (0-1 range)
    experimental2[:, 0] /= 100

    # Plot the experimental data
    plt.plot(experimental2[:, 0], experimental2[:, 1], linestyle='--',
             marker='^', color=exp_color, linewidth=2, markersize=10)
    
    # Add experimental data to legend elements
    legend_elements.append(
        Line2D([0], [0], color=exp_color, linestyle='--', marker='^',
               label='Experimental', markersize=8, linewidth=1.5))
             
    # Plot the DEM data
    plt.plot(dem_data[:, 1], dem_data[:, 0], linestyle='-.', 
             marker='s', color='#D95F02', linewidth=2, markersize=8)
    
    # Add DEM data to legend elements
    legend_elements.append(
        Line2D([0], [0], color='#D95F02', linestyle='-.', marker='s',
               label='DEM', markersize=6, linewidth=1.5))

    # Set labels and title with consistent font styling
    plt.xlabel('Slip Ratio', fontsize=14, fontweight='bold')
    plt.ylabel('Slope (deg)', fontsize=14, fontweight='bold')
    
    # Set y-ticks using slope values
    plt.yticks(ticks=[0, 2.5, 5, 10, 15, 20, 25],
               labels=[r'$0^\circ$', r'$2.5^\circ$', r'$5^\circ$', r'$10^\circ$',
                       r'$15^\circ$', r'$20^\circ$', r'$25^\circ$'])

    # Remove grid to match cone_penetration_plotting style
    plt.grid(False)
    plt.xlim(0, 1.0)
    plt.ylim(0, 30)

    # Increase tick label size
    plt.tick_params(axis='both', which='major', labelsize=12)

    # Add legend with better positioning and formatting
    plt.legend(handles=legend_elements, loc='lower right', fontsize=6, framealpha=0.7,
               prop={'weight': 'bold'}, markerscale=0.7,
               borderpad=0.3, labelspacing=0.2, handlelength=1.5)

    # Apply tight layout for better formatting
    plt.tight_layout()

    # Save the plot with a descriptive filename
    os.makedirs("./paper_plots", exist_ok=True)
    
    output_filename = f"./paper_plots/Viper_slip_vs_slope_multi_dir_comparison.png"
    
    plt.savefig(output_filename, dpi=600)
    print(f"Plot saved as: {output_filename}")

    # Show the plot
    plt.show()


if __name__ == "__main__":
    main() 