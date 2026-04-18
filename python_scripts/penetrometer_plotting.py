import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import json
import os

# Set IEEE journal-style font sizes
IEEE_FONT_SIZES = {
    'label': 16,
    'title': 18,
    'tick': 14,
    'legend': 12,
    'linewidth': 2.5
}

def plot_depth_vs_cone_pressure(force_data, figsize=(8, 5), density=1800, save_path=None):
    cone_length = 0.01756
    """
    Plots Penetration Depth (in cm) vs Cone-Pressure (kPa).

    Parameters:
        force_data: dict or DataFrame
            - If a dictionary is provided, keys are labels and values are force_df DataFrames.
            - If a single DataFrame is provided, it is plotted as a single line.
        figsize: tuple, optional
            - Figure size (width, height).
        density: int, optional
            - The density to use for experimental data.
        save_path: str, optional
            - If provided, saves the plot to this path (e.g., 'plot.png' or 'plot.pdf').
            - If None, displays the plot interactively.
    """

    sns.set_style("whitegrid")
    sns.set_context("talk")
    plt.rcParams.update({
        'font.size': IEEE_FONT_SIZES['tick'],
        'axes.labelsize': IEEE_FONT_SIZES['label'],
        'axes.titlesize': IEEE_FONT_SIZES['title'],
        'xtick.labelsize': IEEE_FONT_SIZES['tick'],
        'ytick.labelsize': IEEE_FONT_SIZES['tick'],
        'legend.fontsize': IEEE_FONT_SIZES['legend'],
        'figure.titlesize': IEEE_FONT_SIZES['title']
    })
    fig, ax = plt.subplots(figsize=figsize)

    # Determine min values for axis limits
    min_x, min_y = float('inf'), float('inf')

    if isinstance(force_data, dict):
        for label, force_df in force_data.items():
            if force_df is not None and 'penetration-depth' in force_df.columns:
                ax.plot((force_df['penetration-depth'] + cone_length) * 100,
                        force_df['cone-pressure (kPa)'], label=label, linewidth=IEEE_FONT_SIZES['linewidth'])
                min_x = min(
                    min_x, ((force_df['penetration-depth'] + cone_length) * 100).min())
                min_y = min(min_y, force_df['cone-pressure (kPa)'].min())
            else:
                print(
                    f"Skipping {label}: 'penetration-depth' column not found.")

    elif isinstance(force_data, pd.DataFrame):
        if 'penetration-depth' in force_data.columns:
            ax.plot((force_data['penetration-depth'] + cone_length) * 100,
                    force_data['cone-pressure (kPa)'], label="Sim", linewidth=IEEE_FONT_SIZES['linewidth'])
            min_x = ((force_data['penetration-depth'] +
                     cone_length) * 100).min()
            min_y = force_data['cone-pressure (kPa)'].min()
        else:
            print("Skipping plot: 'penetration-depth' column not found.")
            return

    else:
        print("Invalid input type for force_data. Must be a dict or DataFrame.")
        return

    # Formatting
    ax.set_xlabel("Penetration Depth (cm)", fontsize=IEEE_FONT_SIZES['label'])
    ax.set_ylabel("Cone Pressure (kPa)", fontsize=IEEE_FONT_SIZES['label'])

    # Plot the experimental data on the same axes
    plot_experimental_data(ax, density)

    # ax.set_title(f"Density: {density}", fontsize=IEEE_FONT_SIZES['title'])
    ax.legend(fontsize=IEEE_FONT_SIZES['legend'], frameon=True, fancybox=True, shadow=True)
    ax.grid(True, linestyle='--', alpha=0.7, linewidth=1.2)
    # Set the x-axis limit to stop at 17.5 cm
    ax.set_xlim(left=min_x, right=17.5)
    ax.set_ylim(bottom=min_y)
    ax.tick_params(axis='both', which='major', labelsize=IEEE_FONT_SIZES['tick'], width=1.5, length=5)
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=600, bbox_inches='tight')
        print(f"Plot saved to: {save_path}")
    plt.show()


def plot_all_rows(df, varying_column, base_path, figsize=(8, 5), density=1800):
    cone_length = 0.01756
    """
    Plots Penetration Depth vs Cone-Pressure for all rows in the DataFrame,
    varying by a specified column.

    Parameters:
        df: DataFrame
            - The DataFrame containing the data.
        varying_column: str
            - The column that varies and will be used for labeling.
        base_path: str
            - The base path to the directories containing 'force_vs_time.txt'.
        figsize: tuple, optional
            - Figure size (width, height).
        density: int, optional
            - The density to use for experimental data.
    """

    sns.set_style("whitegrid")
    sns.set_context("talk")
    plt.rcParams.update({
        'font.size': IEEE_FONT_SIZES['tick'],
        'axes.labelsize': IEEE_FONT_SIZES['label'],
        'axes.titlesize': IEEE_FONT_SIZES['title'],
        'xtick.labelsize': IEEE_FONT_SIZES['tick'],
        'ytick.labelsize': IEEE_FONT_SIZES['tick'],
        'legend.fontsize': IEEE_FONT_SIZES['legend'],
        'figure.titlesize': IEEE_FONT_SIZES['title']
    })
    fig, ax = plt.subplots(figsize=figsize)

    # Iterate over each unique value in the varying column
    for value in df[varying_column].unique():
        # Filter the DataFrame for the current value
        df_filtered = df[df[varying_column] == value]

        # Iterate over each row in the filtered DataFrame
        for _, row in df_filtered.iterrows():
            # Construct the path to the 'force_vs_time.txt' file
            file_path = os.path.join(
                base_path, row['relative_path'], 'force_vs_time.txt')

            # Read the data
            force_df = pd.read_csv(file_path)
            force_df['cone-pressure (kPa)'] = force_df['cone-pressure'] / 1000.0

            # Plot the data
            ax.plot((force_df['penetration-depth'] + cone_length) * 100,
                    force_df['cone-pressure (kPa)'],
                    label=f'{varying_column} = {value}', linewidth=IEEE_FONT_SIZES['linewidth'])

    # Formatting
    ax.set_xlabel("Penetration Depth (cm)", fontsize=IEEE_FONT_SIZES['label'])
    ax.set_ylabel("Cone Pressure (kPa)", fontsize=IEEE_FONT_SIZES['label'])

    # Plot the experimental data on the same axes
    plot_experimental_data(ax, density)

    ax.set_title(f"Density: {density}", fontsize=IEEE_FONT_SIZES['title'])
    ax.legend(fontsize=IEEE_FONT_SIZES['legend'], frameon=True, fancybox=True, shadow=True)
    ax.grid(True, linestyle='--', alpha=0.7, linewidth=1.2)
    ax.set_xlim(right=17.5)
    ax.tick_params(axis='both', which='major', labelsize=IEEE_FONT_SIZES['tick'], width=1.5, length=5)
    plt.tight_layout()
    plt.show()


def plot_with_subplots(df, first_column, second_column, base_path, settings, figsize=(12, 8), density=1800):
    cone_length = 0.01756
    """
    Plots Penetration Depth vs Cone-Pressure with subplots for each unique value in the second column,
    and multiple lines for each unique value in the first column.

    Parameters:
        df: DataFrame
            - The DataFrame containing the data.
        first_column: str
            - The column that varies and will be used for line labels.
        second_column: str
            - The column that varies and will be used for creating subplots.
        base_path: str
            - The base path to the directories containing 'force_vs_time.txt'.
        settings: dict
            - A dictionary of constant settings to include in the plot title.
        figsize: tuple, optional
            - Figure size (width, height).
        density: int, optional
            - The density to use for experimental data.
    """

    sns.set_style("whitegrid")
    sns.set_context("talk")
    plt.rcParams.update({
        'font.size': IEEE_FONT_SIZES['tick'],
        'axes.labelsize': IEEE_FONT_SIZES['label'],
        'axes.titlesize': IEEE_FONT_SIZES['title'],
        'xtick.labelsize': IEEE_FONT_SIZES['tick'],
        'ytick.labelsize': IEEE_FONT_SIZES['tick'],
        'legend.fontsize': IEEE_FONT_SIZES['legend'],
        'figure.titlesize': IEEE_FONT_SIZES['title']
    })
    unique_second_values = df[second_column].unique()
    unique_first_values = df[first_column].unique()
    num_subplots = len(unique_second_values)

    fig, axes = plt.subplots(
        nrows=num_subplots, figsize=figsize, sharex=True, sharey=True)

    # Ensure axes is always iterable
    if num_subplots == 1:
        axes = [axes]

    # Create a color map for the first column
    color_map = {value: color for value, color in zip(
        unique_first_values, sns.color_palette("tab10", len(unique_first_values)))}

    for i, (ax, second_value) in enumerate(zip(axes, unique_second_values)):
        df_second_filtered = df[df[second_column] == second_value]

        for first_value in unique_first_values:
            df_first_filtered = df_second_filtered[df_second_filtered[first_column] == first_value]

            for _, row in df_first_filtered.iterrows():
                file_path = os.path.join(
                    base_path, row['relative_path'], 'force_vs_time.txt')
                force_df = pd.read_csv(file_path)
                force_df['cone-pressure (kPa)'] = force_df['cone-pressure'] / 1000.0

                # Use the color map and add label only to the first subplot
                label = f'{first_column} = {first_value}' if i == 0 else None
                ax.plot((force_df['penetration-depth'] + cone_length) * 100,
                        force_df['cone-pressure (kPa)'],
                        label=label, linewidth=IEEE_FONT_SIZES['linewidth'], color=color_map[first_value])

        # Plot the experimental data on the same axes
        plot_experimental_data(ax, density)

        ax.set_title(f"{second_column} = {second_value}", fontsize=IEEE_FONT_SIZES['title'])
        ax.set_xlabel("Penetration Depth (cm)", fontsize=IEEE_FONT_SIZES['label'])
        ax.set_ylabel("Cone Pressure (kPa)", fontsize=IEEE_FONT_SIZES['label'])
        if i == 0:
            ax.legend(fontsize=IEEE_FONT_SIZES['legend'], frameon=True, fancybox=True, shadow=True)
        ax.grid(True, linestyle='--', alpha=0.7, linewidth=1.2)
        ax.set_xlim(right=17.5)
        ax.tick_params(axis='both', which='major', labelsize=IEEE_FONT_SIZES['tick'], width=1.5, length=5)

    # Format youngsModulus in scientific notation and exclude penetrationDepth and density
    settings_str = ', '.join(
        [f"{key}: {value:.1e}" if key == 'youngsModulus' else f"{key}: {value}"
         for key, value in settings.items() if key not in ['penetrationDepth', 'density']])

    plt.suptitle(
        f"Density: {density} | Constants: {settings_str}", fontsize=IEEE_FONT_SIZES['title'])
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

# Example usage
# plot_with_subplots(filtered_df, 'artificialViscosity', 'viscosityType', '/path/to/base/directory', settings_av)


def plot_experimental_data(ax, density):
    cone_length = 0.01756
    with open('penetrometer_experimentalData.json', 'r') as file:
        experimental_data = json.load(file)

    # Print the experimental data
    print(experimental_data)

    # Check if the specified density exists in the data
    density_key = f"{density}"
    if density_key in experimental_data:
        data = experimental_data[density_key]
        depths = data['depth']
        pressures = data['pressure']

        # Plotting on the provided axes
        ax.plot(depths, pressures, marker='o', linestyle='-',
                color='red', label='Experimental', linewidth=IEEE_FONT_SIZES['linewidth'], 
                markersize=8, markeredgewidth=1.5, markeredgecolor='darkred')
    else:
        print(f"Density {density} not found in the data.")
