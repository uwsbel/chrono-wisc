import numpy as np
import seaborn as sns
import pandas as pd
import os
import matplotlib.pyplot as plt
import argparse

# Set the common rcParams for all plots (matching Viper style)
plt.rcParams["figure.dpi"] = 300
plt.rcParams["font.size"] = 10
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams['font.weight'] = 'bold'

# Parse command line arguments
parser = argparse.ArgumentParser(
    description='Plot RASSOR Drum data with option to vary ps parameter')
parser.add_argument('--vary_ps', action='store_true',
                    help='Plot multiple CRM lines for different ps values')
parser.add_argument('--include_scm', action='store_true',
                    help='Include SCM data in the plot')
args = parser.parse_args()

# write function that process the DEM data


def process_data(file_name, time_start, time_end, dem=False, scm=False):

    if (dem):
        print("Processing file: ", file_name)
        data = pd.read_csv(file_name)

        data['pos_z'] = pd.to_numeric(data['pos_z'], errors='coerce')

        print(data)
        print(data.shape)
        window_size = 4
        time = np.convolve(data['time'], np.ones(
            window_size)/window_size, mode='valid')
        pos_x = np.convolve(data['pos_x'], np.ones(
            window_size)/window_size, mode='valid')
        pos_z = np.convolve(data['pos_z'], np.ones(
            window_size)/window_size, mode='valid')
        driving_torque = np.convolve(data['driving_torque_local'], np.ones(
            window_size)/window_size, mode='valid')
    
    elif (scm):
        print("Processing SCM file: ", file_name)
        # Read tab-separated data
        data = pd.read_csv(file_name, delimiter='\t')
        
        print(data.shape)
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
        print("Processing file: ", file_name)

        data = np.genfromtxt(file_name, delimiter=',', skip_header=1)

        print(data.shape)
        window_size = 4
        time = np.convolve(data[:, 0], np.ones(
            window_size)/window_size, mode='valid')
        pos_x = np.convolve(data[:, 1], np.ones(
            window_size)/window_size, mode='valid')
        pos_z = np.convolve(data[:, 3], np.ones(
            window_size)/window_size, mode='valid')
        driving_torque = - \
            np.convolve(data[:, 11], np.ones(
                window_size)/window_size, mode='valid')

    # select data within time_end
    pos_x = pos_x[(time >= time_start) & (time <= time_end)]
    pos_z = pos_z[(time >= time_start) & (time <= time_end)]
    if not scm:  # Don't negate torque for SCM data
        driving_torque = -driving_torque
    driving_torque = driving_torque[(time >= time_start) & (time <= time_end)]
    time = time[(time >= time_start) & (time <= time_end)]

    return time, driving_torque, pos_x, pos_z


# end time is 6.5
time_end = 6.5
time_start = 2

# Process base data
base_folder = "./DEMO_OUTPUT/FSI_Rassor_SingleDrum_noActive/"
scm_folder = "./DEMO_OUTPUT/SCM_Rassor_SingleDrum/"

wheel_vel = 0.15
wheel_ang_vel = 2.09
total_mass = "7.50"

test_name = f"wheel_vel_{wheel_vel}_wheel_AngVel_{wheel_ang_vel}_total_mass_{total_mass}/"

boundary_type = "adami"
viscosity_type = "artificial_bilateral"
default_ps = 1  # Default ps value
s = 0.005
d = 1.2
av = 0.05

# Define ps values to use when varying ps
ps_values = [1, 2, 5, 10] if args.vary_ps else [default_ps]

# Process DEM data
dem_folder = "./DEMO_OUTPUT/dem_rassor_data/"
dem_file = f"wheel_vel_{wheel_vel}_wheel_AngVel_{wheel_ang_vel}_total_mass_{total_mass}/driving_torque.csv"
dem_file_path = dem_folder + dem_file
dem_time, dem_torque, dem_pos_x, dem_pos_z = process_data(
    dem_file_path, time_start, time_end, dem=True)

# Set the style and context for the plot (matching Viper style)
sns.set(style="ticks", context="talk")

# Create a figure with proper dimensions
plt.figure(figsize=(6, 6))

# Define color palette for multiple CRM lines
if args.vary_ps:
    # Generate a color palette for multiple ps values
    crm_colors = sns.color_palette("viridis", len(ps_values))
else:
    # Use the original matte teal color
    sim_color = (0.35, 0.6, 0.6)  # Matte teal

# Define DEM color
dem_color = (0.8, 0.4, 0.2)  # Warm orange for DEM data
# Define SCM color
scm_color = (0.2, 0.6, 0.8)  # Cool blue for SCM data

# Process and plot CRM data for each ps value
crm_data = []
for i, ps in enumerate(ps_values):
    param_name = f"boundary_{boundary_type}_viscosity_{viscosity_type}_ps_{ps}_s_{s}_d0_{d}_av_{av}/"
    base_file = "results.txt"
    file_path = base_folder + test_name + param_name + base_file

    try:
        time, torque, pos_x, pos_z = process_data(
            file_path, time_start, time_end)
        crm_data.append((ps, time, torque))

        # Plot CRM data with appropriate label and color
        if args.vary_ps:
            plt.plot(time, torque, color=crm_colors[i], linewidth=2,
                     label=f'Freq. {ps}')
        else:
            plt.plot(time, torque, color=sim_color, linewidth=2, label='CRM')
    except Exception as e:
        print(f"Error processing CRM data for ps={ps}: {e}")

# Process and plot SCM data if flag is set
if args.include_scm:
    try:
        # SCM has a different directory structure
        scm_grid_size = "0.010"  # This is from the directory name - keep as string to preserve format
        scm_dir = f"grid_{scm_grid_size}_AngVel_{wheel_ang_vel}/0/"
        scm_file_path = scm_folder + scm_dir + "results.txt"
        print(f"Attempting to read SCM data from: {scm_file_path}")
        scm_time, scm_torque, scm_pos_x, scm_pos_z = process_data(
            scm_file_path, time_start, time_end, scm=True)
        plt.plot(scm_time, scm_torque, color=scm_color, linewidth=2,
                linestyle=':', label='SCM')
    except Exception as e:
        print(f"Error processing SCM data: {e}")

# Plot DEM data
plt.plot(dem_time, dem_torque, color=dem_color, linewidth=2,
         linestyle='--', label='DEM')

# Set labels and title with consistent font styling
plt.xlabel('Time (s)', fontsize=14, fontweight='bold')  
plt.ylabel('Driving torque (Nm)', fontsize=14, fontweight='bold')
# if args.vary_ps:
#     # plt.title('RASSOR Drum Driving Torque - Frequency Comparison',
#     #           fontsize=14, fontweight='bold')
# else:
#     title = 'RASSOR Drum Driving Torque'
#     if args.include_scm:
#         title += ' (with SCM comparison)'
#     plt.title(title, fontsize=14, fontweight='bold')

# Set x-limits
plt.xlim(time_start, time_end)

# Remove grid to match Viper plotting style
plt.grid(False)

# Increase tick label size
plt.tick_params(axis='both', which='major', labelsize=12)

# Add legend with much smaller text and normal font weight (not bold)
plt.legend(fontsize=6, loc='upper right', framealpha=0.7,
           prop={'weight': 'bold'}, markerscale=0.7,
           borderpad=0.3, labelspacing=0.2, handlelength=1.5)

# Apply tight layout for better formatting
plt.tight_layout()

# Ensure paper_plots directory exists
os.makedirs("./paper_plots", exist_ok=True)

# Save the plot with a more descriptive filename
if args.vary_ps:
    output_filename = f"./paper_plots/RASSOR_torque_comparison_varying_ps.png"
else:
    scm_suffix = "_with_scm" if args.include_scm else ""
    output_filename = f"./paper_plots/RASSOR_torque_comparison_ps{default_ps}_s{s}_d0{d}_av{av}{scm_suffix}.png"

plt.savefig(output_filename, dpi=600)
print(f"Plot saved as: {output_filename}")

# Show the plot
plt.show()
