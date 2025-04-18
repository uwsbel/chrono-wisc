#!/usr/bin/env zsh
#SBATCH --gres=gpu:a100:1
#SBATCH --time=10-0:0:0
#SBATCH --partition=sbel
#SBATCH -o pad_%A_%a.out
#SBATCH -e pad_%A_%a.err
#SBATCH --account=sbel
#SBATCH --qos=priority
##SBATCH --mem-per-cpu=20000
##SBATCH --cpus-per-task=8
##SBATCH --mem=40G

#SBATCH --array=1-6
module load nvidia/cuda/12.0.0
module load gcc/11.3.0

nvidia-smi

# Define arrays for parameters
h_array=(0.008 0.008 0.006 0.006 0.005 0.005)
dt_array=(5e-5 1e-4 5e-5 1e-4 5e-5 1e-4)

# Get array index from SLURM
id=$SLURM_ARRAY_TASK_ID

# Get parameters for this run
step_size=${dt_array[${id}]}
h=${h_array[${id}]}
#density=${density_array[${id}]}

#step_size=2e-5
#density=998.5

# Fixed parameters

cd /srv/home/fang/lander/build/bin/


# Run the simulation with all parameters
./demo_FSI_pad --initial_spacing ${h} --time_step ${step_size}
