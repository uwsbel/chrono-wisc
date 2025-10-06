import pandas as pd
import sys

# === Step 1: Parse args ===
# Usage: python top_10_wheels.py <folder>
if len(sys.argv) < 2:
    raise SystemExit("Usage: python top_10_wheels.py <folder>")

folder = sys.argv[1]

# === Step 2: Load the CSV file ===
df = pd.read_csv(folder + "/trials.csv")

# === Step 3: Inspect column names ===
print("Columns in dataset:", df.columns.tolist())

# === Step 4: Use hardcoded time column ===
time_col = 'total_time_to_reach'
if time_col not in df.columns:
    raise ValueError(
        f"Expected column '{time_col}' not found. Available columns: {df.columns.tolist()}"
    )

# === Step 5: Sort by best (lowest) times ===
df_sorted = df.sort_values(by=time_col, ascending=True)

# === Step 6: Select the top 10 runs ===
top_10 = df_sorted.head(10)

# === Step 7: Display the parameters for these runs ===
print("\nTop 10 Best (Lowest) Times and Corresponding Parameters:")
for rank, (_, row) in enumerate(top_10.iterrows(), start=1):
    print(f"\nRank {rank}:")
    for col in top_10.columns:
        print(f"  {col}: {row[col]}")

# === Step 8: Save results next to input ===
output_path = folder + "/top_10_best_times.csv"
top_10.to_csv(output_path, index=False)
print(f"\nResults saved to '{output_path}'")

