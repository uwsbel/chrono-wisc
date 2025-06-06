import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl
import argparse
import os
import sys
from pathlib import Path
import datetime

def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot and compare vehicle simulation results.')
    
    # Add arguments for up to 4 paths and corresponding labels
    parser.add_argument('--path1', type=str, required=True,
                        help='Path to first results file or directory containing results.txt')
    parser.add_argument('--name1', type=str, default='Simulation 1',
                        help='Legend name for first results')
    
    parser.add_argument('--path2', type=str, required=False,
                        help='Path to second results file or directory containing results.txt')
    parser.add_argument('--name2', type=str, default='Simulation 2',
                        help='Legend name for second results')
    
    parser.add_argument('--path3', type=str, required=False,
                        help='Path to third results file or directory containing results.txt')
    parser.add_argument('--name3', type=str, default='Simulation 3',
                        help='Legend name for third results')
    
    parser.add_argument('--path4', type=str, required=False,
                        help='Path to fourth results file or directory containing results.txt')
    parser.add_argument('--name4', type=str, default='Simulation 4',
                        help='Legend name for fourth results')
    
    # Add argument for output file
    parser.add_argument('--output', type=str, required=False,
                        help='Output filename (without extension, will be saved as .png)')
    parser.add_argument('--no-display', action='store_true',
                        help='Do not display the plot, just save it')
    
    # Add argument for plot type
    parser.add_argument('--plot-type', type=str, choices=['all', 'position', 'velocity'],
                        default='all',
                        help='Type of plot to generate: "all", "position" only, or "velocity" only. Default is "all".')
    
    return parser.parse_args()

def load_data(path):
    """Load data from a path, which can be either a file or directory containing results.txt"""
    # Handle if path is a directory
    if os.path.isdir(path):
        path = os.path.join(path, 'results.txt')
    
    # Check if file exists
    if not os.path.isfile(path):
        print(f"Error: Could not find results file at {path}")
        return None
    
    try:
        return pd.read_csv(path)
    except Exception as e:
        print(f"Error loading data from {path}: {e}")
        return None

def ensure_dir_exists(directory):
    """Create directory if it doesn't exist"""
    os.makedirs(directory, exist_ok=True)

def main():
    # Parse command line arguments
    args = parse_arguments()
    
    # Load data for each provided path
    datasets = []
    names = []
    
    # Always load the first dataset (required)
    data1 = load_data(args.path1)
    if data1 is not None:
        datasets.append(data1)
        names.append(args.name1)
    else:
        print("Error loading primary dataset. Exiting.")
        sys.exit(1)
    
    # Load second dataset if provided
    if args.path2:
        data2 = load_data(args.path2)
        if data2 is not None:
            datasets.append(data2)
            names.append(args.name2)
    
    # Load third dataset if provided
    if args.path3:
        data3 = load_data(args.path3)
        if data3 is not None:
            datasets.append(data3)
            names.append(args.name3)
    
    # Load fourth dataset if provided
    if args.path4:
        data4 = load_data(args.path4)
        if data4 is not None:
            datasets.append(data4)
            names.append(args.name4)
    
    # Set up the style for a professional, clean presentation
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.size': 12,
        'axes.linewidth': 1.5,
        'axes.edgecolor': 'black',
        'xtick.major.width': 1.5,
        'ytick.major.width': 1.5,
        'xtick.direction': 'out',
        'ytick.direction': 'out',
        'lines.linewidth': 2.5,
        'lines.markersize': 8,
        'legend.frameon': True,
        'legend.framealpha': 0.9,
        'legend.edgecolor': 'lightgray',
        'figure.facecolor': 'white'
    })
    
    # Set color palette based on number of datasets
    colors = ['#1f77b4', '#e41a1c', '#2ca02c', '#ff7f0e']  # Blue, Red, Green, Orange
    markers = ['o', 's', '^', 'D']  # Circle, Square, Triangle, Diamond
    
    # Create a figure with subplots based on plot_type
    num_subplots = 0
    if args.plot_type == 'all':
        num_subplots = 2
    elif args.plot_type in ['position', 'velocity']:
        num_subplots = 1

    if num_subplots == 0: # Should not happen due to choices in argparse
        print("Invalid plot_type specified. Exiting.")
        sys.exit(1)

    if num_subplots == 2:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        fig.subplots_adjust(hspace=0.15)  # Reduce space between subplots
    elif num_subplots == 1:
        fig, ax1 = plt.subplots(1, 1, figsize=(10, 5)) # Adjusted figsize for single plot
        ax2 = None # No second axis
    
    # Plot PosZ over time for each dataset if requested
    if args.plot_type == 'all' or args.plot_type == 'position':
        current_ax = ax1
        for i, (data, name) in enumerate(zip(datasets, names)):
            markevery = max(1, len(data) // 20)
            sns.lineplot(x='Time', y='PosZ', data=data, ax=current_ax, 
                        color=colors[i % len(colors)], 
                        marker=markers[i % len(markers)],
                        markevery=markevery,
                        label=name)
        
        current_ax.set_ylabel('Position Z (m)', fontweight='bold')
        current_ax.set_title('Vehicle Z Position', fontweight='bold')
        current_ax.grid(True, linestyle='--', alpha=0.7)
        current_ax.spines['top'].set_visible(False)
        current_ax.spines['right'].set_visible(False)
        current_ax.legend()
        if args.plot_type == 'position': # If only position is plotted, set xlabel
             current_ax.set_xlabel('Time (s)', fontweight='bold')

    # Plot VelX for each dataset if requested
    if args.plot_type == 'all' or args.plot_type == 'velocity':
        # If 'all', ax2 is the second subplot. If 'velocity' only, ax1 is the target.
        current_ax = ax2 if args.plot_type == 'all' else ax1
        for i, (data, name) in enumerate(zip(datasets, names)):
            markevery = max(1, len(data) // 20)
            sns.lineplot(x='Time', y='VelX', data=data, ax=current_ax, 
                        color=colors[i % len(colors)], 
                        marker=markers[i % len(markers)],
                        markevery=markevery,
                        label=name)
        
        current_ax.set_ylabel('Velocity X (m/s)', fontweight='bold')
        current_ax.set_xlabel('Time (s)', fontweight='bold')
        current_ax.set_title('Vehicle X Velocity', fontweight='bold')
        current_ax.grid(True, linestyle='--', alpha=0.7)
        current_ax.spines['top'].set_visible(False)
        current_ax.spines['right'].set_visible(False)
        current_ax.legend()

    # Tight layout
    plt.tight_layout()
    
    # Create plots directory if it doesn't exist
    plots_dir = os.path.join(os.getcwd(), 'plots')
    ensure_dir_exists(plots_dir)
    
    # Generate filename if not provided
    if not args.output:
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        sim_names = "_".join([name.replace(" ", "-") for name in names])
        if len(sim_names) > 30:  # Truncate if too long
            sim_names = sim_names[:30]
        filename = f"vehicle_plot_{timestamp}_{sim_names}.png"
    else:
        filename = f"{args.output}.png"
    
    # Full path for saving
    save_path = os.path.join(plots_dir, filename)
    
    # Save the figure
    fig.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {save_path}")
    
    # Show plot unless --no-display is specified
    if not args.no_display:
        plt.show()

if __name__ == "__main__":
    main()