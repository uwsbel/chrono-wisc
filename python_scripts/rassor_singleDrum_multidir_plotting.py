import numpy as np
import seaborn as sns
import pandas as pd
import os
import matplotlib.pyplot as plt
import argparse
from matplotlib.lines import Line2D

# Example command:
# python rassor_singleDrum_multidir_plotting.py --vary_ps --data-dirs "./DEMO_OUTPUT" "/path/to/other/output" --legend-names "DEMO 1" "DEMO 2"

# Set the common rcParams for all plots (matching Viper style)
plt.rcParams["figure.dpi"] = 300
plt.rcParams["font.size"] = 10
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams['font.weight'] = 'bold'


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Plot RASSOR Drum data with support for multiple directories and frequency comparison')
    parser.add_argument('--vary_ps', action='store_true',
                        help='Plot multiple lines for different frequency values')
    parser.add_argument('--include_scm', action='store_true',
                        help='Include SCM data in the plot')
    parser.add_argument('--include_dem', action='store_true',
                        help='Include DEM data in the plot (default: True)', default=True)
    parser.add_argument('--ps-values', type=int, nargs='+', default=[1, 2, 5, 10],
                        help='PS frequency values to compare (default: 1 2 5 10)')
    parser.add_argument('--data-dirs', type=str, nargs='+', default=['./DEMO_OUTPUT'],
                        help='Base directories containing FSI_Rassor_SingleDrum data (default: ./DEMO_OUTPUT)')
    parser.add_argument('--legend-names', type=str, nargs='+', default=None,
                        help='Names to use in the legend for each data directory (default: directory names)')
    parser.add_argument('--wheel-vel', type=float, default=0.15,
                        help='Wheel velocity (default: 0.15)')
    parser.add_argument('--wheel-ang-vel', type=float, default=2.09,
                        help='Wheel angular velocity (default: 2.09)')
    parser.add_argument('--total-mass', type=str, default="7.50",
                        help='Total mass (default: 7.50)')
    parser.add_argument('--s', type=float, default=0.005,
                        help='Spacing value (default: 0.005)')
    parser.add_argument('--d0', type=float, default=1.2,
                        help='d0 value (default: 1.2)')
    parser.add_argument('--av', type=float, default=0.05,
                        help='av value (default: 0.05)')
    parser.add_argument('--time-start', type=float, default=2.0,
                        help='Start time for analysis (default: 2.0)')
    parser.add_argument('--time-end', type=float, default=6.5,
                        help='End time for analysis (default: 6.5)')
    parser.add_argument('--boundary-type', type=str, default="adami",
                        help='Boundary type (default: adami)')
    parser.add_argument('--viscosity-type', type=str, default="artificial_bilateral",
                        help='Viscosity type (default: artificial_bilateral)')
    return parser.parse_args()


def process_data(file_name, time_start, time_end, dem=False, scm=False):
    try:
        if dem:
            print("Processing DEM file: ", file_name)
            data = pd.read_csv(file_name)

            data['pos_z'] = pd.to_numeric(data['pos_z'], errors='coerce')

            print(f"DEM data shape: {data.shape}")
            window_size = 4
            time = np.convolve(data['time'], np.ones(
                window_size)/window_size, mode='valid')
            pos_x = np.convolve(data['pos_x'], np.ones(
                window_size)/window_size, mode='valid')
            pos_z = np.convolve(data['pos_z'], np.ones(
                window_size)/window_size, mode='valid')
            driving_torque = np.convolve(data['driving_torque_local'], np.ones(
                window_size)/window_size, mode='valid')
        
        elif scm:
            print("Processing SCM file: ", file_name)
            # Read tab-separated data
            data = pd.read_csv(file_name, delimiter='\t')
            
            print(f"SCM data shape: {data.shape}")
            window_size = 4
            time = np.convolve(data['time'], np.ones(
                window_size)/window_size, mode='valid')
            pos_x = np.convolve(data['x'], np.ones(
                window_size)/window_size, mode='valid')
            pos_z = np.convolve(data['z'], np.ones(
                window_size)/window_size, mode='valid')
            # For SCM, we use tz column for torque
            driving_torque = np.convolve(data['tz'], np.ones(
                window_size)/window_size, mode='valid')

        else:
            print("Processing CRM file: ", file_name)
            data = np.genfromtxt(file_name, delimiter=',', skip_header=1)

            print(f"CRM data shape: {data.shape}")
            window_size = 4
            time = np.convolve(data[:, 0], np.ones(
                window_size)/window_size, mode='valid')
            pos_x = np.convolve(data[:, 1], np.ones(
                window_size)/window_size, mode='valid')
            pos_z = np.convolve(data[:, 3], np.ones(
                window_size)/window_size, mode='valid')
            driving_torque = - np.convolve(data[:, 11], np.ones(
                window_size)/window_size, mode='valid')

        # Select data within time range
        mask = (time >= time_start) & (time <= time_end)
        pos_x = pos_x[mask]
        pos_z = pos_z[mask]
        if not scm:  # Don't negate torque for SCM data
            driving_torque = -driving_torque
        driving_torque = driving_torque[mask]
        time = time[mask]
        
        return time, driving_torque, pos_x, pos_z
    except Exception as e:
        print(f"Error processing file {file_name}: {e}")
        return None, None, None, None


def main():
    # Parse command line arguments
    args = parse_arguments()
    
    # Get parameters from arguments
    vary_ps = args.vary_ps
    include_scm = args.include_scm
    include_dem = args.include_dem
    ps_values = args.ps_values if vary_ps else [1]  # Use just the default PS if not varying
    data_dirs = args.data_dirs
    legend_names = args.legend_names
    
    # Simulation parameters
    wheel_vel = args.wheel_vel
    wheel_ang_vel = args.wheel_ang_vel
    total_mass = args.total_mass
    s = args.s
    d0 = args.d0
    av = args.av
    boundary_type = args.boundary_type
    viscosity_type = args.viscosity_type
    time_start = args.time_start
    time_end = args.time_end
    
    # Set legend names (use directory names if not provided)
    if legend_names is None:
        legend_names = [os.path.basename(dir_path) for dir_path in data_dirs]
    
    # Ensure we have the same number of legend names as data directories
    if len(legend_names) != len(data_dirs):
        print(f"Error: Number of legend names ({len(legend_names)}) does not match the number of data directories ({len(data_dirs)})")
        return

    # Set the style and context for the plot
    sns.set(style="ticks", context="talk")
    
    # Create a figure with proper dimensions
    plt.figure(figsize=(6, 6))

    # Define color palette for PS values
    ps_colors = dict(zip(ps_values, sns.color_palette("viridis", len(ps_values))))
    
    # Define linestyles for different directories
    linestyles = ['-', '--', '-.', ':']
    if len(data_dirs) > len(linestyles):
        # If we have more directories than linestyles, cycle through them
        linestyles = linestyles * (len(data_dirs) // len(linestyles) + 1)
    
    # Define colors for external data
    dem_color = (0.8, 0.4, 0.2)  # Warm orange for DEM data
    scm_color = (0.2, 0.6, 0.8)  # Cool blue for SCM data
    
    # Keep track of the legend elements
    legend_elements = []
    
    # Process DEM data if required
    dem_data = None
    if include_dem:
        dem_folder = "./DEMO_OUTPUT/dem_rassor_data/"
        dem_file = f"wheel_vel_{wheel_vel}_wheel_AngVel_{wheel_ang_vel}_total_mass_{total_mass}/driving_torque.csv"
        dem_file_path = os.path.join(dem_folder, dem_file)
        
        dem_time, dem_torque, dem_pos_x, dem_pos_z = process_data(
            dem_file_path, time_start, time_end, dem=True)
        
        if dem_time is not None:
            # Plot DEM data
            plt.plot(dem_time, dem_torque, color=dem_color, linewidth=2,
                    linestyle='--', label='DEM')
            
            # Add DEM to legend elements
            legend_elements.append(
                Line2D([0], [0], color=dem_color, linestyle='--',
                       label='DEM', linewidth=1.5))
        else:
            print("Warning: DEM data could not be loaded. Skipping DEM plot.")
    
    # Process SCM data if required
    if include_scm:
        for dir_idx, base_dir in enumerate(data_dirs):
            scm_folder = os.path.join(base_dir, "SCM_Rassor_SingleDrum")
            
            # SCM has a different directory structure
            scm_grid_size = "0.010"  # This is from the directory name - keep as string to preserve format
            scm_dir = f"grid_{scm_grid_size}_AngVel_{wheel_ang_vel}/0/"
            scm_file_path = os.path.join(scm_folder, scm_dir, "results.txt")
            
            print(f"Attempting to read SCM data from: {scm_file_path}")
            
            scm_time, scm_torque, scm_pos_x, scm_pos_z = process_data(
                scm_file_path, time_start, time_end, scm=True)
            
            if scm_time is not None:
                # Use different line styles if we have multiple directories
                scm_linestyle = linestyles[dir_idx % len(linestyles)] if len(data_dirs) > 1 else ':'
                
                # Plot SCM data
                plt.plot(scm_time, scm_torque, color=scm_color, linewidth=2,
                        linestyle=scm_linestyle, label=f'{legend_names[dir_idx]} SCM')
                
                # Add SCM to legend elements with appropriate style
                legend_elements.append(
                    Line2D([0], [0], color=scm_color, linestyle=scm_linestyle,
                           label=f'{legend_names[dir_idx]} SCM', linewidth=1.5))
            else:
                print(f"Warning: SCM data for directory {legend_names[dir_idx]} could not be loaded. Skipping.")
    
    # Process and plot CRM data for each directory and PS value
    for dir_idx, base_dir in enumerate(data_dirs):
        base_folder = os.path.join(base_dir, "FSI_Rassor_SingleDrum")
        test_name = f"wheel_vel_{wheel_vel}_wheel_AngVel_{wheel_ang_vel}_total_mass_{total_mass}/"
        
        # Use a specific linestyle for this directory
        linestyle = linestyles[dir_idx % len(linestyles)]
        
        for ps_idx, ps in enumerate(ps_values):
            param_name = f"boundary_{boundary_type}_viscosity_{viscosity_type}_ps_{ps}_s_{s}_d0_{d0}_av_{av}/"
            base_file = "results.txt"
            file_path = os.path.join(base_folder, test_name, param_name, base_file)
            
            # Get color for this PS value
            color = ps_colors[ps]
            
            try:
                time, torque, pos_x, pos_z = process_data(
                    file_path, time_start, time_end)
                
                if time is not None:
                    # Plot CRM data with appropriate styling
                    plt.plot(time, torque, color=color, linestyle=linestyle, linewidth=2)
                    
                    # Add to legend elements
                    legend_elements.append(
                        Line2D([0], [0], color=color, linestyle=linestyle,
                               label=f"{legend_names[dir_idx]}{' Freq. ' + str(ps) if vary_ps else ''}", linewidth=1.5))
            except Exception as e:
                print(f"Error processing CRM data for dir={legend_names[dir_idx]}, ps={ps}: {e}")
    
    # Set labels with consistent font styling
    plt.xlabel('Time (s)', fontsize=14, fontweight='bold')  
    plt.ylabel('Driving torque (Nm)', fontsize=14, fontweight='bold')
    
    # Set x-limits
    plt.xlim(time_start, time_end)
    
    # Remove grid to match other plotting styles
    plt.grid(False)
    
    # Increase tick label size
    plt.tick_params(axis='both', which='major', labelsize=12)
    
    # Add legend with optimal formatting for many entries
    plt.legend(handles=legend_elements, loc='upper right', fontsize=6, framealpha=0.7,
               prop={'weight': 'bold'}, markerscale=0.7,
               borderpad=0.3, labelspacing=0.2, handlelength=1.5)
    
    # Apply tight layout for better formatting
    plt.tight_layout()
    
    # Ensure paper_plots directory exists
    os.makedirs("./paper_plots", exist_ok=True)
    
    # Save the plot with a descriptive filename
    output_filename = f"./paper_plots/RASSOR_torque_multi_dir_comparison.png"
    
    plt.savefig(output_filename, dpi=600)
    print(f"Plot saved as: {output_filename}")
    
    # Show the plot
    plt.show()


if __name__ == "__main__":
    main() 