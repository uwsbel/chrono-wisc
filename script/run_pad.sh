#!/bin/bash
#SBATCH --job-name=sph_validation
#SBATCH --gres=gpu:1
#SBATCH --time=40:00:00
#SBATCH -o drop_output_%A_%a.out
#SBATCH -e drop_output_%A_%a.err
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --account=bdgx-delta-gpu
#SBATCH --array=0-314

# Navigate to the program directory
cd /projects/bdgx/lbakke/pad/chrono-wisc/bin

# Define parameter sets
mu_s_vals=($(seq 0.65 0.01 0.85))              # 21 values
density_vals=($(seq 1600 20 1880))             # 15 values
cohesion_vals=(0 10 50 100 250 500 750 1000 1500 2000)  # 10 values
programs=(demo_FSI_pad_translational:0 demo_FSI_pad_translational:1 demo_FSI_pad)

# Resolve SLURM array task index
task_id=${SLURM_ARRAY_TASK_ID}
let mu_index=task_id/15
let density_index=task_id%15

mu_s=${mu_s_vals[$mu_index]}
density=${density_vals[$density_index]}

echo ">>> Running job ID: $task_id"
echo "    mu_s: $mu_s"
echo "    density: $density"

# Loop through cohesion and run all 3 program variations
for cohesion in "${cohesion_vals[@]}"; do
    for entry in "${programs[@]}"; do
        IFS=":" read program tilt_grav <<< "$entry"
        cmd="./$program --mu_s $mu_s --cohesion $cohesion --density $density"
        if [[ -n "$tilt_grav" ]]; then
            cmd="$cmd --tilt_grav $tilt_grav"
        fi
        echo "Running: $cmd"
        $cmd
    done
done
