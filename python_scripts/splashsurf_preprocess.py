#!/usr/bin/env python3

"""
Convert a CSV file (or all fluid*.csv files in a folder) containing particle data into one of:
- JSON file with Vec<[f32; 3]>-style triplets
- VTK file (.vtk legacy format)
- VTU file (.vtu XML format)

Each CSV file should contain at least x, y, z columns. Additional columns are stored as attributes
in the VTK/VTU output. The user chooses the desired output format via command-line argument.

============================================
Usage:
    python splashsurf_preprocess.py --input <file_or_folder> --format json|vtk|vtu [--workers <num_workers>]

Examples:
    python splashsurf_preprocess.py --input fluid995.csv --format json
    python splashsurf_preprocess.py --input ./csv_folder --format vtu
    python splashsurf_preprocess.py --input ./csv_folder --format json --workers 8

============================================
Requirements:
    - pandas
    - numpy
    - meshio

Install the required dependencies using:
    pip install pandas numpy meshio
"""

import os
import argparse
import pandas as pd
import numpy as np
import json
import meshio
import glob
from pathlib import Path
import multiprocessing
from functools import partial
import time

def convert_to_json(csv_path: Path):
    try:
        # Reading CSV file
        df = pd.read_csv(csv_path)
        
        # Ensure file has x, y, z columns
        if not all(col in df.columns for col in ['x', 'y', 'z']):
            raise ValueError(f"{csv_path}: Missing x, y, or z columns.")
        
        data = df[['x', 'y', 'z']].astype(float).values.tolist()

        output_path = csv_path.with_suffix('.json')
        with output_path.open('w') as f:
            json.dump(data, f, indent=2)
        
        return f"Written JSON: {output_path}"
    except Exception as e:
        return f"Error processing {csv_path.name}: {str(e)}"

def convert_to_vtk_or_vtu(csv_path: Path, fmt: str):
    # Reading CSV file
    df = pd.read_csv(csv_path)
    
    # Ensure file has x, y, z columns
    if not all(col in df.columns for col in ['x', 'y', 'z']):
        raise ValueError(f"{csv_path}: Missing x, y, or z columns.")

    points = df[['x', 'y', 'z']].to_numpy(dtype=np.float32)
    point_data = {}

    # Optionally group vector fields like v_x, v_y, v_z into 'velocity'
    if all(k in df.columns for k in ['v_x', 'v_y', 'v_z']):
        point_data['velocity'] = df[['v_x', 'v_y', 'v_z']].to_numpy()
    
    # Add any additional columns as point data
    for col in df.columns:
        if col not in ['x', 'y', 'z', 'v_x', 'v_y', 'v_z']:
            point_data[col] = df[col].to_numpy()

    cells = [("vertex", np.arange(len(points)).reshape(-1, 1))]
    mesh = meshio.Mesh(points, cells, point_data=point_data)

    output_path = csv_path.with_suffix(f'.{fmt}')
    meshio.write(output_path, mesh, file_format=fmt)
    print(f"Written {fmt.upper()}: {output_path}")

def process_file(csv_path: Path, fmt: str):
    if fmt == "json":
        return convert_to_json(csv_path)
    elif fmt in {"vtk", "vtu"}:
        try:
            convert_to_vtk_or_vtu(csv_path, fmt)
            return f"Written {fmt.upper()}: {csv_path.with_suffix(f'.{fmt}')}"
        except Exception as e:
            return f"Error processing {csv_path.name}: {str(e)}"
    else:
        return f"Unsupported format: {fmt}"

def process_files_in_parallel(csv_files, fmt, num_workers):
    """Process multiple files in parallel using a process pool."""
    start_time = time.time()
    print(f"Starting parallel processing with {num_workers} workers...")
    
    # Create a pool of worker processes
    with multiprocessing.Pool(processes=num_workers) as pool:
        # Create a partial function with the format argument
        process_func = partial(process_file, fmt=fmt)
        
        # Process files in parallel and collect results
        results = list(pool.imap_unordered(process_func, csv_files))
        
        # Print results
        for result in results:
            print(result)
    
    end_time = time.time()
    print(f"Finished processing {len(csv_files)} files in {end_time - start_time:.2f} seconds")

def main():
    parser = argparse.ArgumentParser(description="Convert fluid*.csv particle files to JSON, VTK, or VTU format.")
    parser.add_argument("--input", "-i", required=True, help="Path to CSV file or folder containing fluid*.csv files.")
    parser.add_argument("--format", "-f", required=True, choices=["json", "vtk", "vtu"], help="Output format.")
    parser.add_argument("--workers", "-w", type=int, default=multiprocessing.cpu_count(), 
                        help=f"Number of parallel workers (default: {multiprocessing.cpu_count()}).")
    args = parser.parse_args()

    input_path = Path(args.input)
    num_workers = max(1, min(args.workers, multiprocessing.cpu_count()))

    if input_path.is_file() and input_path.name.startswith("fluid") and input_path.suffix.lower() == ".csv":
        # For a single file, no need for parallelization
        result = process_file(input_path, args.format)
        print(result)
    elif input_path.is_dir():
        # Find all fluid*.csv files
        csv_files = list(input_path.glob("fluid*.csv"))
        
        if not csv_files:
            print(f"No fluid*.csv files found in {input_path}")
            return
        
        print(f"Found {len(csv_files)} fluid*.csv files to process")
        
        # Use parallel processing for multiple files
        if len(csv_files) > 1:
            process_files_in_parallel(csv_files, args.format, num_workers)
        else:
            # For a single file, no need for parallelization
            result = process_file(csv_files[0], args.format)
            print(result)
    else:
        print("Error: Input must be a fluid*.csv file or a folder containing fluid*.csv files.")

if __name__ == "__main__":
    main()
