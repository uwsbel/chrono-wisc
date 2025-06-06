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
BENCHMARK_DIR="benchmark_results_CRM"
MIN_TERRAIN_LENGTH=6000   # Minimum terrain length to simulate
MAX_TERRAIN_LENGTH=10000   # Maximum terrain length to simulate
NUM_POINTS=10             # Number of points to simulate between min and max

# Script to run terrain simulations with increasing terrain length
# Values increase logarithmically from $MIN_TERRAIN_LENGTH to $MAX_TERRAIN_LENGTH

# Delete old benchmark results directory if it exists
echo "Removing old benchmark results..."
# rm -rf $BENCHMARK_DIR

# Create output directory
mkdir -p $BENCHMARK_DIR

# Output file for results
OUTPUT_FILE="$BENCHMARK_DIR/terrain_benchmark_results.csv"

# Write header to output file
# echo "terrain_length,time_simulated,real_time_taken,rtf" > $OUTPUT_FILE

# Set executable path (adjust as needed)
EXECUTABLE="./benchmark_RTF_VEH_CRMTerrain_WheeledVehicleScaling"

# Generate logarithmically spaced values between $MIN_TERRAIN_LENGTH and $MAX_TERRAIN_LENGTH
# We'll use $NUM_POINTS points in total
for i in $(seq 0 $(($NUM_POINTS-1))); do
    # Calculate terrain length using logarithmic spacing formula
    # First convert i to a value between 0 and 1
    normalized=$(echo "scale=6; $i / ($NUM_POINTS-1)" | bc)
    
    # Then calculate log-spaced value between log($MIN_TERRAIN_LENGTH) and log($MAX_TERRAIN_LENGTH)
    exponent=$(echo "scale=6; (1.0 - $normalized) * l($MIN_TERRAIN_LENGTH) + $normalized * l($MAX_TERRAIN_LENGTH)" | bc -l)
    
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
