#!/usr/bin/env zsh
#SBATCH --gres=gpu:a100:1
#SBATCH --time=10-0:0:0
#SBATCH --partition=sbel
#SBATCH -o object_drop_%A_%a.out
#SBATCH -e object_drop_%A_%a.err
#SBATCH --account=sbel
#SBATCH --qos=priority
##SBATCH --mem-per-cpu=20000
##SBATCH --cpus-per-task=8
##SBATCH --mem=40G

#SBATCH --array=1-4
module load nvidia/cuda/12.0.0
module load gcc/11.3.0

nvidia-smi

# Define arrays for parameters
d0_array=(1.7 1.7 1.7 1.7 1.75 1.75 1.75 1.75 )
density_array=(998 998 1000 1000 998 998 1000 1000)
step_size_array=(2e-4 2.5e-4 2e-4 2.5e-4 2e-4 2.5e-4 2e-4 2.5e-4)

# Get array index from SLURM
id=$SLURM_ARRAY_TASK_ID

# Get parameters for this run
step_size=${step_size_array[${id}]}
d0=${d0_array[${id}]}
density=${density_array[${id}]}

# Fixed parameters
v_max=8.0
viscosity_type="laminar_dual"
kernel_type="wendland"
initial_spacing=0.015

# Create run tag using SLURM job name and ID
run_tag="${SLURM_JOB_NAME}_${SLURM_JOB_ID}"

cd ../build/bin/

# Print parameters for this run
echo "Running with parameters:"
echo "step_size=${step_size}"
echo "initial_spacing=${initial_spacing}"
echo "d0=${d0}"
echo "density=${density}"
echo "v_max=${v_max}"
echo "viscosity_type=${viscosity_type}"
echo "kernel_type=${kernel_type}"
echo "run_tag=${run_tag}"

# Run the simulation with all parameters
./demo_FSI_ObjectDrop \
    --step_size ${step_size} \
    --initial_spacing ${initial_spacing} \
    --d0 ${d0} \
    --density ${density} \
    --v_max ${v_max} \
    --viscosity_type ${viscosity_type} \
    --kernel_type ${kernel_type} \
    --run_tag ${run_tag} 