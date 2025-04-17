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

#SBATCH --array=1
module load nvidia/cuda/12.0.0
module load gcc/11.3.0

nvidia-smi

# Define arrays for parameters
#d0_array=(1.1 1.3 1.5 1.7 1.9 2.0 )
#density_array=(998.5 998.5 1000 1000 998.5 998.5 1000 1000)

# Get array index from SLURM
#id=$SLURM_ARRAY_TASK_ID

# Get parameters for this run
#step_size=${step_size_array[${id}]}
#d0=${d0_array[${id}]}
#density=${density_array[${id}]}

#step_size=2e-5
#density=998.5

# Fixed parameters

cd /srv/home/fang/lander/build/bin/


# Run the simulation with all parameters
./demo_FSI_pad
