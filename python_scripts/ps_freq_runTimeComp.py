import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import seaborn as sns

def parse_runtime_file(file_path):
    """
    Parse a runtime.txt file and extract key metrics.
    
    Args:
        file_path (str): Path to the runtime.txt file
        
    Returns:
        dict: Dictionary containing the parsed metrics
    """
    metrics = {}
    
    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()
            
            for line in lines:
                line = line.strip()
                
                if line.startswith("Runtime:"):
                    metrics["runtime"] = float(line.split("Runtime:")[1].strip().split()[0])
                    
                elif line.startswith("Simulation time:"):
                    metrics["simulation_time"] = float(line.split("Simulation time:")[1].strip())
                    
                elif line.startswith("Average RTF:"):
                    metrics["average_rtf"] = float(line.split("Average RTF:")[1].strip())
                    
                elif line.startswith("ps_freq:"):
                    metrics["ps_freq"] = int(line.split("ps_freq:")[1].strip())
                    
                elif "Simulation finished" in line:
                    metrics["simulation_finished"] = True
                    
        return metrics
    
    except Exception as e:
        print(f"Error parsing file {file_path}: {e}")
        return None

# Example usage
if __name__ == "__main__":
    # Set common plot parameters to match other scripts
    plt.rcParams["figure.dpi"] = 300
    plt.rcParams["font.size"] = 10
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams['font.weight'] = 'bold'
    
    # Set the style and context for the plot (same as in viper_singleWheel_plotting.py)
    sns.set(style="ticks", context="talk")

    # Demos to consider
    demos = ["MGRU3 Single Wheel", "Rassor Single Drum", "Sphere Cratering", "Cone Penetration"]

    ps_freqs = [1, 2, 5, 10]
    
    mgru3_spacing = "0.010"
    mgru3_d0 = 1.2
    mgru3_av = "0.02"

    rassor_wheel_vel = 0.15
    rassor_wheel_ang_vel = 2.09
    rassor_total_mass = "7.50"

    sphere_d = 2200  # Sphere densities
    sphere_h = 0.2  # Drop heights
    sphere_viscosity_type = "artificial_bilateral"  # Viscosity type
    sphere_boundary_type = "adami"  # Boundary type
    sphere_s = 0.0025  # Smoothing length

    # Create data structure to store results
    results = {demo: {ps: None for ps in ps_freqs} for demo in demos}

    for ps in ps_freqs:
        mu_s = "0.80"
        mu_2 = "1.00"
        mu_i0 = "0.08"
        base_dir = f"./DEMO_OUTPUT/FSI_ConePenetration_mu_s_{mu_s}_mu_2_{mu_2}_mu_i0_{mu_i0}/"
        cone_folder = os.path.join(
                    base_dir,
                    f"Hdrop_1.0",
                    f"granMaterial_sand",
                    f"relDensity_0",
                    f"coneType_1",
                    f"boundaryType_adami",
                    f"viscosityType_artificial_bilateral",
                    f"kernelType_wendland",
                    f"ps_{ps}_s_{0.001}_d0_{1.3}_t_{2e-5}_av_{0.2}"
                )
        # Folder map for each demo
        folder_map = {
            "MGRU3 Single Wheel": f"./DEMO_OUTPUT/FSI_Viper_Single_Wheel3/ps_{ps}_s_{mgru3_spacing}_d0_{mgru3_d0}_av_{mgru3_av}/1",
            "Rassor Single Drum": f"./DEMO_OUTPUT/FSI_Rassor_SingleDrum_noActive/wheel_vel_{rassor_wheel_vel}_wheel_AngVel_{rassor_wheel_ang_vel}_total_mass_{rassor_total_mass}/boundary_adami_viscosity_artificial_bilateral_ps_{ps}_s_0.005_d0_1.2_av_0.05",
            "Sphere Cratering": f"./DEMO_OUTPUT/FSI_Cratering/{sphere_viscosity_type}_{sphere_boundary_type}_ps{ps}_d{sphere_d}_h{sphere_h}_s{sphere_s}",
            "Cone Penetration": cone_folder
        }
        for demo in demos:
            file_path = folder_map[demo] + "/runtime.txt"
            result = parse_runtime_file(file_path)
            
            if result:
                results[demo][ps] = result
                print("Parsed metrics:")
                for key, value in result.items():
                    print(f"  {key}: {value}")

    # Generate LaTeX table
    print("\n\n")
    print("% LaTeX Table - RTF and Speedup Values")
    print("\\begin{table}[h]")
    print("\\centering")
    print("\\caption{Real-Time Factor (RTF) and Speedup for Different Simulations at Varying PS Frequencies}")
    print("\\label{tab:rtf_speedup}")
    print("\\begin{tabular}{l|cccc}")
    print("\\hline")
    print("\\textbf{Simulation} & \\textbf{ps$_{\\text{freq}}$=1} & \\textbf{ps$_{\\text{freq}}$=2} & \\textbf{ps$_{\\text{freq}}$=5} & \\textbf{ps$_{\\text{freq}}$=10} \\\\")
    print("\\hline")

    demo_display_names = {
        "MGRU3 Single Wheel": "MGRU3 Wheel",
        "Rassor Single Drum": "RASSOR Drum",
        "Sphere Cratering": "Sphere Cratering",
        "Cone Penetration": "Cone Penetration"
    }

    # Store the speedup data for plotting
    speedup_data = {demo: [] for demo in demos}
    
    for demo in demos:
        display_name = demo_display_names.get(demo, demo)
        rtf_values = []
        
        # Get baseline RTF (ps_freq=1)
        baseline_rtf = None
        if results[demo][1] and "average_rtf" in results[demo][1]:
            baseline_rtf = results[demo][1]["average_rtf"]
        
        row = f"{display_name} & "
        
        for ps in ps_freqs:
            if results[demo][ps] and "average_rtf" in results[demo][ps]:
                rtf = results[demo][ps]["average_rtf"]
                
                if ps == 1:
                    row += f"{rtf:.2f} & "
                    # Baseline speedup is 1.0
                    speedup_data[demo].append(1.0)
                else:
                    if baseline_rtf:
                        speedup = rtf / baseline_rtf
                        row += f"{rtf:.2f} ({speedup:.2f}×) & "
                        speedup_data[demo].append(speedup)
                    else:
                        row += f"{rtf:.2f} (--) & "
                        speedup_data[demo].append(0)
            else:
                row += "-- & "
                if ps == 1:
                    speedup_data[demo].append(1.0)
                else:
                    speedup_data[demo].append(0)
        
        # Remove the last '& ' and add line break
        row = row[:-2] + " \\\\"
        print(row)
    
    print("\\hline")
    print("\\end{tabular}")
    print("\\end{table}")
    print("")
    print("% Note: Values in parentheses show the speedup ratio relative to ps$_{\\text{freq}}$=1")

    # Create bar chart with matching style from viper_singleWheel_plotting.py
    # Increase figure size to make more room
    plt.figure(figsize=(12, 7))
    
    # Setup the width of the bars and positions
    bar_width = 0.2
    ps_to_plot = [2, 5, 10]     # Only plot these ps_freq values (excluding baseline)
    index = np.arange(len(demos))
    
    # Colors - use the same viridis palette from cone_penetration_plotting
    colors = sns.color_palette("viridis", len(ps_to_plot))
    
    # Plot the bars
    bars = []
    for i, ps in enumerate(ps_to_plot):
        ps_index = ps_freqs.index(ps)
        speedups = [speedup_data[demo][ps_index] for demo in demos]
        bar = plt.bar(index + (i-1)*bar_width, speedups, bar_width, 
                      label=f'Freq. {ps}', color=colors[i])
        bars.append(bar)
    
    # Set labels and title with consistent font styling
    plt.xlabel('Simulation Case', fontsize=14, fontweight='bold')
    plt.ylabel('Speedup Factor (RTF$_{ps}$/RTF$_{1}$)', fontsize=14, fontweight='bold')
    
    # Set x-ticks at the center of each group
    plt.xticks(index, [demo_display_names[demo] for demo in demos], fontsize=12)
    
    # Draw a horizontal line at y=1 to represent the baseline
    plt.axhline(y=1.0, color='gray', linestyle='--', alpha=0.7)
    
    # Move legend to the top-right corner outside the plot
    plt.legend(fontsize=12, loc='upper right', bbox_to_anchor=(1, 1), 
               frameon=True, framealpha=0.9)
    
    # Remove grid to match style in other scripts
    plt.grid(False)
    
    # Increase tick label size
    plt.tick_params(axis='both', which='major', labelsize=12)
    
    # Add value labels on top of each bar
    for bar_group in bars:
        for bar in bar_group:
            height = bar.get_height()
            plt.annotate(f'{height:.2f}×',
                        xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom',
                        fontsize=10, fontweight='bold')
    
    # Create paper_plots directory if it doesn't exist
    os.makedirs("paper_plots", exist_ok=True)
    
    # Save figures in the paper_plots directory
    plt.tight_layout()
    plt.savefig('paper_plots/ps_freq_speedup.png', dpi=600)
    plt.savefig('paper_plots/ps_freq_speedup.pdf')
    print("\nGenerated bar chart: paper_plots/ps_freq_speedup.png and paper_plots/ps_freq_speedup.pdf")
