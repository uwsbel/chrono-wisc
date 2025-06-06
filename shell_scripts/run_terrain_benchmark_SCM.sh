#!/bin/bash
#SBATCH --job-name=penetrometer_paperRuns
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=8
#SBATCH --output=penetrometer_paperRuns_%A_%a.out
#SBATCH --error=penetrometer_paperRuns_%A_%a.err
#SBATCH --time=0-48:00:00
#SBATCH --mem=80G
#SBATCH --partition=gpu
#SBATCH --gres=gpu:h100:1
#SBATCH --account=155366303681              #Set billing account
#SBATCH --mail-type=ALL              #Send email on all job events
#SBATCH --mail-user=unjhawala@wisc.edu    #Send all emails to email_address

# Configuration
BENCHMARK_DIR="benchmark_results_SCM"

# Script to run terrain simulations with increasing terrain length
# Values increase logarithmically from 20 to 2000

# Delete old benchmark results directory if it exists
echo "Removing old benchmark results..."
rm -rf $BENCHMARK_DIR

# Create output directory
mkdir -p $BENCHMARK_DIR

# Output file for results
OUTPUT_FILE="$BENCHMARK_DIR/terrain_benchmark_results.csv"

# Write header to output file
echo "terrain_length,time_simulated,real_time_taken,rtf" > $OUTPUT_FILE

# Set executable path (adjust as needed)
EXECUTABLE="./benchmark_RTF_VEH_SCMTerrain_WheeledVehicleScaling"

# Generate logarithmically spaced values between 20 and 2000
# We'll use 10 points in total
for i in {0..35}; do
    # Calculate terrain length using logarithmic spacing formula
    # First convert i to a value between 0 and 1
    normalized=$(echo "scale=6; $i / 25" | bc)
    
    # Then calculate log-spaced value between log(20) and log(2000)
    exponent=$(echo "scale=6; (1.0 - $normalized) * l(20) + $normalized * l(8000)" | bc -l)
    
    # Convert back from logarithm
    terrain_length=$(echo "scale=0; e($exponent) / 1" | bc -l)
    
    echo "Running simulation with terrain length: $terrain_length"
    
    # Run the simulation with current terrain length
    $EXECUTABLE --length $terrain_length > $BENCHMARK_DIR/terrain_length_${terrain_length}.log
    
    # Extract results from the output log
    time_sim=$(grep "Time simulated:" $BENCHMARK_DIR/terrain_length_${terrain_length}.log | awk '{print $3}')
    time_real=$(grep "Real time taken:" $BENCHMARK_DIR/terrain_length_${terrain_length}.log | awk '{print $4}')
    rtf=$(grep "Real-time factor:" $BENCHMARK_DIR/terrain_length_${terrain_length}.log | awk '{print $3}')
    
    # Append results to the CSV file
    echo "$terrain_length,$time_sim,$time_real,$rtf" >> $OUTPUT_FILE
    
    echo "Completed terrain length: $terrain_length (RTF: $rtf)"
    echo "------------------------------------------------------"
done

echo "All simulations completed. Results saved to $OUTPUT_FILE"
echo "Run the plotting script to visualize the results."
