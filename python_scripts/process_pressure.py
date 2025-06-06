import pandas as pd
import glob
import argparse
from multiprocessing import Pool
import os

def process_file(file_path):
    """
    Reads a CSV file, filters particles, calculates the pressure column, and overwrites the file.
    """
    try:
        df = pd.read_csv(file_path)

        # Clean column names by removing content within parentheses and stripping spaces
        original_columns = df.columns
        cleaned_columns = [col.split('(')[0].strip() for col in original_columns]
        df.columns = cleaned_columns

        # Filter particles based on y-coordinate
        df = df[df['y'] < 0.616]

        # If all rows are filtered out, don't attempt to process further
        if df.empty:
            print(f"All particles filtered out for: {file_path}. File not overwritten.")
            # Optionally, you might want to delete the file or save an empty file
            # For now, it just skips overwriting
            return

        # Calculate pressure
        df['pressure'] = -0.33 * (df['p11'] + df['p22'] + df['p33'])

        # Restore original column names for writing, except for the new pressure column
        # Ensure the number of original columns matches the DataFrame columns before adding 'pressure'
        df.columns = list(original_columns[:len(df.columns)-1]) + ['pressure']

        df.to_csv(file_path, index=False)
        print(f"Processed, filtered, and overwritten: {file_path}")
    except Exception as e:
        print(f"Error processing file {file_path}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Process fluid CSV files to add a pressure column and filter particles.")
    parser.add_argument("folder_path", type=str, help="Path to the folder containing CSV files.")
    parser.add_argument("num_workers", type=int, help="Number of worker processes to use.")

    args = parser.parse_args()

    folder_path = args.folder_path
    num_workers = args.num_workers

    if not os.path.isdir(folder_path):
        print(f"Error: Folder not found at {folder_path}")
        return

    # Find all fluid*.csv files
    search_pattern = os.path.join(folder_path, "fluid*.csv")
    csv_files = glob.glob(search_pattern)

    if not csv_files:
        print(f"No 'fluid*.csv' files found in {folder_path}")
        return

    print(f"Found {len(csv_files)} files to process.")

    # Process files in parallel
    with Pool(processes=num_workers) as pool:
        pool.map(process_file, csv_files)

    print("All files processed.")

if __name__ == "__main__":
    main()
