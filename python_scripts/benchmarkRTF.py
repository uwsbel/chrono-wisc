import json
import os
import re
import glob
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from matplotlib.ticker import ScalarFormatter

demo_name_map = {
    "FSI_Flexible_Cable": "Flexible Cable",
    "FSI_Baffle_Flow": "Baffle Flow",
    "FSI_RASSOR_SingleDrum": "RASSOR Single Drum",
    "FSI_RASSOR": "RASSOR",
    "FSI_Viper": "VIPER",
    "FSI_TRACKED_VEHICLE": "Tracked Vehicle",
    "SCM_TRACKED_VEHICLE": "Tracked Vehicle",
    "SCM_Viper": "SCM Viper"
}

# Map FSI demos to their SCM counterparts
scm_demo_map = {
    "FSI_TRACKED_VEHICLE": "SCM_TRACKED_VEHICLE",
    "FSI_Viper": "SCM_Viper"
}


def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)

    simulation_parameters = data.get('simulationParameters', {})
    simulation_settings = data.get('simulationSettings', {})
    timingInfo = data.get('timingInfo', {})

    t_end = simulation_settings.get('tEnd', 0.0)
    cfd_time = timingInfo.get('CFDTime', 0.0)
    fsi_exchange_time = timingInfo.get('FSIExchangeTime', 0.0)
    stepTime = timingInfo.get('stepTime', 0.0)
    # rtf = (cfd_time + fsi_exchange_time) / t_end
    rtf = stepTime / t_end

    # Storing data in a dictionary with type enforcement
    result = {
        'Total Number of Particles': int(simulation_parameters.get('numAllMarkers', 0)),
        'Total Number of Active Particles': int(simulation_parameters.get('numActiveParticles', 0)),
        'Step Size': float(simulation_settings.get('stepSize', 0.0)),
        'RTF': round(float(rtf), 1)
    }

    return result


def read_txt_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
    
    # Extract step size (not directly available in text file)
    step_size = 0.01  # Default step size if not found
    
    # Extract total simulated time
    sim_time_match = re.search(r'Total simulated time: (\d+(\.\d+)?) s', content)
    sim_time = float(sim_time_match.group(1)) if sim_time_match else 0.0
    
    # Extract total real time
    real_time_match = re.search(r'Total real time: (\d+(\.\d+)?) s', content)
    real_time = float(real_time_match.group(1)) if real_time_match else 0.0
    
    # Calculate RTF correctly as Total real time / Total simulated time
    if sim_time > 0 and real_time > 0:
        rtf = real_time / sim_time
    else:
        rtf = 0.0
    
    # For SCM demos, we don't have particle counts, but we can extract mesh resolution
    mesh_res_match = re.search(r'Mesh resolution: (\d+\.\d+) m', content)
    mesh_resolution = float(mesh_res_match.group(1)) if mesh_res_match else 0.0
    
    # Storing data in a dictionary
    result = {
        'Total Number of Particles': 0,  # Not applicable for SCM
        'Total Number of Active Particles': 0,  # Not applicable for SCM
        'Step Size': step_size,
        'RTF': round(float(rtf), 1),
        'Mesh Resolution': mesh_resolution
    }
    
    return result


def read_benchmark_file(file_path):
    if not os.path.exists(file_path):
        return None
        
    _, ext = os.path.splitext(file_path)
    if ext.lower() == '.json':
        return read_json_file(file_path)
    elif ext.lower() == '.txt':
        return read_txt_file(file_path)
    else:
        raise ValueError(f"Unsupported file format: {ext}")


def check_demo_exists(benchmark_dir, demo):
    """Check if a demo exists in a benchmark directory"""
    demo_path = os.path.join(os.path.dirname(__file__), benchmark_dir, demo)
    return os.path.exists(demo_path)


def find_json_file(base_path, pattern):
    """Find a JSON file matching a specific pattern in the given path"""
    search_pattern = os.path.join(base_path, pattern)
    files = glob.glob(search_pattern)
    
    if files:
        return files[0]  # Return the first matching file
    
    # Try alternate patterns if original one doesn't work
    if "ps1_" in pattern:
        # Try with underscore variations
        alt_pattern = pattern.replace("ps1_", "ps_1_")
        search_pattern = os.path.join(base_path, alt_pattern)
        files = glob.glob(search_pattern)
        if files:
            return files[0]
            
    # For Baffle Flow which may have scale variants
    if "scale" not in pattern and "Baffle" in base_path:
        for scale in ["scale1", "scale2", "scale3"]:
            mod_pattern = pattern.replace(".json", f"_{scale}.json")
            search_pattern = os.path.join(base_path, mod_pattern)
            files = glob.glob(search_pattern)
            if files:
                return files[0]
                
    # Last resort: find any json file if specific pattern fails
    search_pattern = os.path.join(base_path, "*.json")
    files = glob.glob(search_pattern)
    if files:
        print(f"Warning: Using alternative file {files[0]} instead of {pattern}")
        return files[0]
        
    return None


def create_bar_chart(results_data):
    """Create a grouped bar chart to visualize RTF performance across demos and configurations"""
    # Set up plot styling
    # Set common rcParams
    plt.rcParams["figure.dpi"] = 300
    plt.rcParams["font.size"] = 8  # Reduced base font size by 2
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams['font.weight'] = 'bold'
    
    # Set the style and context using seaborn
    sns.set(style="ticks", context="talk")
    
    # Convert results to DataFrame for easier handling
    df = pd.DataFrame(results_data)
    
    # Replace 'N/A' strings with NaN for proper plotting
    for col in df.columns:
        if col != 'Demo Name':
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # Select columns to plot with their actual data column names
    plot_columns = ['Baseline', 'Freq. 1 - No Active', 'Freq. 10 - No Active', 
                   'Freq. 10 - Active', 'SCM - No Active', 'SCM - Active']
    
    # Define simplified legend labels for cleaner display
    legend_labels = {
        'Baseline': 'Baseline',
        'Freq. 1 - No Active': 'Freq. 1',
        'Freq. 10 - No Active': 'Freq. 10',
        'Freq. 10 - Active': 'Freq. 10 Active',
        'SCM - No Active': 'SCM',
        'SCM - Active': 'SCM Active'
    }
    
    # Get unique demo names
    demo_names = df['Demo Name'].tolist()
    
    # Set up the figure and axis
    plt.figure(figsize=(10, 6))
    ax = plt.subplot(111)
    
    # Set width of bars and positions
    bar_width = 0.15  # Increased bar width
    index = np.arange(len(demo_names))
    
    # Define colors for each configuration using a seaborn color palette
    colors = sns.color_palette("viridis", len(plot_columns))
    
    # Plot each configuration as a group of bars
    for i, column in enumerate(plot_columns):
        # Skip columns with all NaN values
        if df[column].isna().all():
            continue
            
        # Plot bars for this configuration using simplified legend labels
        bars = ax.bar(index + i*bar_width, df[column], bar_width,
                    label=legend_labels[column], color=colors[i], alpha=0.8)
    
    # Set y-axis to logarithmic scale
    ax.set_yscale('log')
    
    # Format y-axis to show actual values instead of powers
    formatter = ScalarFormatter()
    formatter.set_scientific(False)
    ax.yaxis.set_major_formatter(formatter)
    
    # Set labels with consistent font styling
    ax.set_ylabel('Real-Time Factor (RTF)', fontsize=12, fontweight='bold')
    
    # Set x-axis ticks and labels
    ax.set_xticks(index + bar_width * (len([col for col in plot_columns if not df[col].isna().all()]) - 1) / 2)
    ax.set_xticklabels(demo_names, rotation=45, ha='right', fontsize=10)
    
    # Increase tick label size
    plt.tick_params(axis='both', which='major', labelsize=10)
    
    # Remove grid entirely
    ax.grid(False)
    
    # Add legend with better positioning and formatting
    legend = ax.legend(fontsize=12, ncol=2, loc='upper right',
                      frameon=True, framealpha=0.9, bbox_to_anchor=(1.0, 1.0))
    
    # Adjust layout to prevent clipping of labels
    plt.tight_layout()
    
    # Make sure the plots directory exists
    os.makedirs("paper_plots", exist_ok=True)
    
    # Save figure with high resolution
    plt.savefig('paper_plots/benchmark_rtf_comparison.png', dpi=600, bbox_inches='tight')
    print("\nBar chart saved as 'paper_plots/benchmark_rtf_comparison.png'")
    
    # Show plot if in interactive mode
    plt.show()


if __name__ == "__main__":
    # List of FSI demos
    fsi_demos = ["FSI_Flexible_Cable", "FSI_Baffle_Flow", 
                "FSI_RASSOR", "FSI_Viper", "FSI_TRACKED_VEHICLE"]
    
    benchmark_baseline = "BENCHMARK_BASELINE_RTF"
    benchmark_no_active = "BENCHMARK3_RTF_noActive"
    benchmark_active = "BENCHMARK3_RTF_Active"
    
    results = []

    for demo in fsi_demos:
        row = {
            'Demo Name': demo_name_map[demo],
            'Total Particles': 0,
            'Active Particles': 0,
            'Time Step': 0.0,
            'Baseline': 'N/A',
            'Freq. 1 - No Active': 'N/A',
            'Freq. 10 - No Active': 'N/A',
            'Freq. 10 - Active': 'N/A',
            'SCM - No Active': 'N/A',
            'SCM - Active': 'N/A',
            'Speedup Freq. 1': 'N/A',
            'Speedup Freq. 10': 'N/A',
            'Speedup Active': 'N/A',
            'Speedup SCM No Active': 'N/A',
            'Speedup SCM Active': 'N/A'
        }
        
        # First, check if this demo has an active variant
        has_active = check_demo_exists(benchmark_active, demo)
        
        # Check if baseline exists and get preliminary data
        if check_demo_exists(benchmark_baseline, demo):
            base_path = os.path.join(os.path.dirname(__file__), benchmark_baseline, demo, 'CRM_WCSPH')
            
            # Special case for FSI_Baffle_Flow, which uses a different naming convention
            if demo == "FSI_Baffle_Flow":
                file_pattern = "rtf_default_default_ps1_d01.2_scale3.json"
            else:
                file_pattern = "rtf_default_default_ps1_d01.2.json"
                
            file_path = find_json_file(base_path, file_pattern)
            
            if file_path:
                result_dict = read_benchmark_file(file_path)
                if result_dict:
                    row['Total Particles'] = result_dict['Total Number of Particles']
                    # If it has an active variant, don't set active particles yet
                    if not has_active:
                        row['Active Particles'] = result_dict['Total Number of Active Particles']
                    row['Time Step'] = result_dict['Step Size']
                    row['Baseline'] = result_dict['RTF']
        
        # Check if benchmark_no_active exists and gather particle data
        if check_demo_exists(benchmark_no_active, demo):
            base_path = os.path.join(os.path.dirname(__file__), benchmark_no_active, demo, 'CRM_WCSPH')
            
            # Freq 1 - No Active
            if demo == "FSI_Baffle_Flow":
                file_pattern = "rtf_artificial_bilateral_adami_ps1_d01.2_scale3.json"
            else:
                file_pattern = "rtf_artificial_bilateral_adami_ps1_d01.2.json"
                
            file_path = find_json_file(base_path, file_pattern)
            
            if file_path:
                result_dict = read_benchmark_file(file_path)
                if result_dict:
                    row['Total Particles'] = max(row['Total Particles'], result_dict['Total Number of Particles'])
                    # Only set active particles here if demo doesn't have an active variant
                    if not has_active:
                        row['Active Particles'] = max(row['Active Particles'], result_dict['Total Number of Active Particles'])
                    row['Freq. 1 - No Active'] = result_dict['RTF']
                    
                    # Calculate speedup compared to baseline
                    if row['Baseline'] != 'N/A' and row['Freq. 1 - No Active'] != 'N/A':
                        baseline = float(row['Baseline'])
                        freq1 = float(row['Freq. 1 - No Active'])
                        speedup = baseline / freq1
                        row['Speedup Freq. 1'] = f"{speedup:.1f}x" if speedup >= 1 else f"{1/speedup:.1f}x↓"
            
            # Freq 10 - No Active
            if demo == "FSI_Baffle_Flow":
                file_pattern = "rtf_artificial_bilateral_adami_ps10_d01.2_scale3.json"
            else:
                file_pattern = "rtf_artificial_bilateral_adami_ps10_d01.2.json"
                
            file_path = find_json_file(base_path, file_pattern)
                
            if file_path:
                result_dict = read_benchmark_file(file_path)
                if result_dict:
                    row['Total Particles'] = max(row['Total Particles'], result_dict['Total Number of Particles'])
                    # Only set active particles here if demo doesn't have an active variant
                    if not has_active:
                        row['Active Particles'] = max(row['Active Particles'], result_dict['Total Number of Active Particles'])
                    row['Freq. 10 - No Active'] = result_dict['RTF']
                    
                    # Calculate speedup compared to baseline
                    if row['Baseline'] != 'N/A' and row['Freq. 10 - No Active'] != 'N/A':
                        baseline = float(row['Baseline'])
                        freq10 = float(row['Freq. 10 - No Active'])
                        speedup = baseline / freq10
                        row['Speedup Freq. 10'] = f"{speedup:.1f}x" if speedup >= 1 else f"{1/speedup:.1f}x↓"
                
        # Check if benchmark_active exists
        if has_active:
            base_path = os.path.join(os.path.dirname(__file__), benchmark_active, demo, 'CRM_WCSPH')
            
            # Freq 10 - Active
            if demo == "FSI_Baffle_Flow":
                file_pattern = "rtf_artificial_bilateral_adami_ps10_d01.2_scale3.json"
            else:
                file_pattern = "rtf_artificial_bilateral_adami_ps10_d01.2.json"
                
            file_path = find_json_file(base_path, file_pattern)
                
            if file_path:
                result_dict = read_benchmark_file(file_path)
                if result_dict:
                    row['Total Particles'] = max(row['Total Particles'], result_dict['Total Number of Particles'])
                    # For demos with active variants, get active particles count from this file
                    row['Active Particles'] = result_dict['Total Number of Active Particles']
                    row['Freq. 10 - Active'] = result_dict['RTF']
                    
                    # Calculate speedup compared to baseline
                    if row['Baseline'] != 'N/A' and row['Freq. 10 - Active'] != 'N/A':
                        baseline = float(row['Baseline'])
                        active = float(row['Freq. 10 - Active'])
                        speedup = baseline / active
                        row['Speedup Active'] = f"{speedup:.1f}x" if speedup >= 1 else f"{1/speedup:.1f}x↓"
        else:
            row['Freq. 10 - Active'] = 'N/A'
        
        # Check if SCM variant exists
        if demo in scm_demo_map:
            scm_demo = scm_demo_map[demo]
            
            # SCM - No Active
            scm_no_active_path = os.path.join(os.path.dirname(__file__), benchmark_no_active, 
                                             scm_demo, 'benchmark_results.txt')
            result_dict = read_benchmark_file(scm_no_active_path)
            if result_dict:
                row['SCM - No Active'] = result_dict['RTF']
                
                # Calculate speedup compared to baseline
                if row['Baseline'] != 'N/A' and row['SCM - No Active'] != 'N/A':
                    baseline = float(row['Baseline'])
                    scm_no_active = float(row['SCM - No Active'])
                    speedup = baseline / scm_no_active
                    row['Speedup SCM No Active'] = f"{speedup:.1f}x" if speedup >= 1 else f"{1/speedup:.1f}x↓"
                
            # SCM - Active
            scm_active_path = os.path.join(os.path.dirname(__file__), benchmark_active, 
                                          scm_demo, 'benchmark_results.txt')
            result_dict = read_benchmark_file(scm_active_path)
            if result_dict:
                row['SCM - Active'] = result_dict['RTF']
                
                # Calculate speedup compared to baseline
                if row['Baseline'] != 'N/A' and row['SCM - Active'] != 'N/A':
                    baseline = float(row['Baseline'])
                    scm_active = float(row['SCM - Active'])
                    speedup = baseline / scm_active
                    row['Speedup SCM Active'] = f"{speedup:.1f}x" if speedup >= 1 else f"{1/speedup:.1f}x↓"
        
        results.append(row)

    # Convert results to a list format for table generation
    table_rows = []
    for row in results:
        table_row = [
            row['Demo Name'],
            row['Total Particles'],
            row['Active Particles'],
            row['Time Step'],
            row['Baseline'],
            f"{row['Freq. 1 - No Active']} ({row['Speedup Freq. 1']})" if row['Speedup Freq. 1'] != 'N/A' else row['Freq. 1 - No Active'],
            f"{row['Freq. 10 - No Active']} ({row['Speedup Freq. 10']})" if row['Speedup Freq. 10'] != 'N/A' else row['Freq. 10 - No Active'],
            f"{row['Freq. 10 - Active']} ({row['Speedup Active']})" if row['Speedup Active'] != 'N/A' else row['Freq. 10 - Active'],
            f"{row['SCM - No Active']} ({row['Speedup SCM No Active']})" if row['Speedup SCM No Active'] != 'N/A' else row['SCM - No Active'],
            f"{row['SCM - Active']} ({row['Speedup SCM Active']})" if row['Speedup SCM Active'] != 'N/A' else row['SCM - Active']
        ]
        table_rows.append(table_row)

    # Generate LaTeX table
    print("\\begin{table}[h!]")
    print("\\centering")
    print("\\begin{tabular}{|l|r|r|r|r|r|r|r|r|r|}")
    print("\\hline")
    print("Demo Name & Total Particles & Active Particles & Time Step & Baseline & Freq. 1 - No Active & Freq. 10 - No Active & Freq. 10 - Active & SCM - No Active & SCM - Active \\\\ \\hline")
    for row in table_rows:
        # Convert time step to scientific notation if it's a number
        if isinstance(row[3], (int, float)):
            row[3] = "{:.1e}".format(float(row[3]))
        print(" & ".join(map(str, row)) + " \\\\ \\hline")
    print("\\end{tabular}")
    print("\\caption{Comparison of RTF values across benchmarks with speedup relative to baseline in parentheses}")
    print("\\end{table}")

    # Generate plain text table
    print("\nPlain Text Table:")
    header = ["Demo Name", "Total Particles", "Active Particles",
              "Time Step", "Baseline", "Freq. 1 - No Active", "Freq. 10 - No Active", 
              "Freq. 10 - Active", "SCM - No Active", "SCM - Active"]
    print("{:<20} {:<15} {:<15} {:<15} {:<12} {:<25} {:<25} {:<25} {:<25} {:<25}".format(*header))
    print("-" * 200)
    for row in table_rows:
        if isinstance(row[3], (int, float)):
            row[3] = "{:.1e}".format(float(row[3]))
        print("{:<20} {:<15} {:<15} {:<15} {:<12} {:<25} {:<25} {:<25} {:<25} {:<25}".format(*row))
    
    # Create and save the bar chart visualization
    create_bar_chart(results)
