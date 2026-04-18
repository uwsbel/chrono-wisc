import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os


def parse_special_folder_name(folder_name):
    """
    Parse folder name of form:
      ps_<val>_s_<val>_d0_<val>_t_<val>_av_<val>
    Returns dict with keys: proximitySteps, initialSpacing, d0, timeStep, artificialViscosity
    """
    parts = folder_name.split('_')
    # Expecting exactly 10 parts: ["ps", val, "s", val, "d0", val, "t", val, "av", val]
    return {
        'proximitySteps': parts[1],
        'initialSpacing': parts[3],
        'd0': parts[5],
        'timeStep': parts[7],
        'artificialViscosity': parts[9]
    }


def parse_generic_folder_name(folder_name):
    """
    Parse a folder name into {column_name: value}.

    - Special handling if folder starts with "mu_s_" or "mu_2_":
      e.g. "mu_s_0.25" => {"mu_s": "0.25"}
           "mu_2_0.9131" => {"mu_2": "0.9131"}
    - Otherwise, for "column_value" styles (e.g. "penetrationDepth_0.18"),
      we split on the first underscore.
    """
    # Special handling for mu_s_... or mu_2_...
    if folder_name.startswith("mu_s_") or folder_name.startswith("mu_2_"):
        parts = folder_name.split('_')
        col = "_".join(parts[:2])    # "mu_s" or "mu_2"
        val = "_".join(parts[2:])    # e.g. "0.25" or "0.9131"
        return {col: val}
    else:
        # Split only on the *first* underscore
        split_once = folder_name.split('_', 1)
        if len(split_once) == 2:
            key, val = split_once
            return {key: val}
        else:
            # No underscore found -> ignore or handle differently
            return {}


def collect_experiments(root_dir):
    """
    Walk the directory tree under root_dir, collecting one row for every folder
    that contains 'force_vs_time.txt'. Each row is a combination of parsed folder names
    from the root down to that final folder, plus 'completed=True'.

    We also store the relative path so we can find the file again later.
    """
    records = []

    for dirpath, _, filenames in os.walk(root_dir):
        # If force_vs_time.txt is in this directory, we treat it as a completed experiment
        if 'force_vs_time.txt' in filenames:
            relative_path = os.path.relpath(dirpath, start=root_dir)
            subfolders = relative_path.split(os.sep)

            row_data = {}
            for folder in subfolders:
                # Check if folder is the special ps_..._s_..._d0_..._t_..._av_...
                if (folder.startswith('ps_') and '_s_' in folder and '_d0_' in folder
                        and '_t_' in folder and '_av_' in folder):
                    row_data.update(parse_special_folder_name(folder))
                else:
                    row_data.update(parse_generic_folder_name(folder))

            # Mark the experiment as complete
            row_data['completed'] = True
            # Store relative path so we can retrieve the file easily
            row_data['relative_path'] = relative_path

            records.append(row_data)

    df = pd.DataFrame(records)

    # Now cast columns to the desired data types if they exist.
    desired_dtypes = {
        'penetrationDepth': float,
        'youngsModulus': float,
        'density': float,
        'mu_s': float,
        'mu_2': float,
        'cohesion': float,
        'boundaryType': 'object',  # or str
        'viscosityType': 'object',
        'kernelType': 'object',
        'proximitySteps': int,
        'initialSpacing': float,
        'd0': float,
        'timeStep': float,
        'artificialViscosity': float,
        'completed': bool
    }

    # Only attempt to cast columns that actually exist in df:
    for col, col_type in desired_dtypes.items():
        if col in df.columns:
            df[col] = df[col].astype(col_type)

    return df


def extract_force_data(df, root_dir, query_dict):
    """
    Given a DataFrame of experiments and a dictionary specifying column->value pairs,
    locate the *first* matching experiment, read 'force_vs_time.txt' (comma separated),
    then plot Time vs cone-pressure (converted from Pa to kPa) using Seaborn.
    """

    df_filtered = df.copy()

    for col, val in query_dict.items():
        # If the column is not in df, skip or handle appropriately:
        if col not in df_filtered.columns:
            # You could raise an error or continue.
            print(f"Column '{col}' not found in DataFrame. Skipping.")
            return

        col_dtype = df_filtered[col].dtype

        # If both the column and the query value are floats, do an approximate match:
        if pd.api.types.is_float_dtype(col_dtype) and isinstance(val, float):
            # Adjust tolerances as needed
            rtol = 1e-08
            atol = 1e-08
            # Filter rows where df[col] is "close" to val
            df_filtered = df_filtered[np.isclose(df_filtered[col], val)]
        # If it's an integer column and the query value is int, compare directly
        elif pd.api.types.is_integer_dtype(col_dtype) and isinstance(val, int):
            df_filtered = df_filtered[df_filtered[col] == val]
        else:
            # Otherwise do a direct equality check (works for strings, bool, etc.)
            df_filtered = df_filtered[df_filtered[col] == val]
    if df_filtered.empty:
        print("No matching experiment found for", query_dict)
        return

    # If multiple matches, take the first
    match = df_filtered.iloc[0]
    rel_path = match['relative_path']
    force_vs_time_path = os.path.join(root_dir, rel_path, 'force_vs_time.txt')

    if not os.path.isfile(force_vs_time_path):
        print("force_vs_time.txt not found at", force_vs_time_path)
        return

    # Read the force_vs_time file (comma-separated)
    force_df = pd.read_csv(force_vs_time_path)

    if 'Time' not in force_df.columns or 'cone-pressure' not in force_df.columns:
        print("Expected columns 'Time' and 'cone-pressure' not found in file.")
        print("Columns are:", force_df.columns)
        return

    # Convert cone-pressure from Pa to kPa
    force_df['cone-pressure (kPa)'] = force_df['cone-pressure'] / 1000.0

    return force_df
