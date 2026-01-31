#!/bin/bash
#SBATCH --job-name=f8_25_3
#SBATCH --time=48:00:00
#SBATCH --mem=32G
#SBATCH -o f8_25_3.out
#SBATCH -e f8_25_3.err
#SBATCH --partition=gpuA100x4
#SBATCH --cpus-per-task=8
#SBATCH --gpus=1
#SBATCH --account=bfqh-delta-gpu
##SBATCH --account=bdgx-delta-gpu
module load gcc/13.2.0
export LD_LIBRARY_PATH=/sw/spack/deltas11-2023-03/apps/linux-rhel8-x86_64/gcc-8.5.0/gcc-13.2.0-blv4b5f/lib64:$LD_LIBRARY_PATH
source /u/hunjhawala/venv/bin/activate

unset PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/u/hunjhawala/chrono-wisc-wheel/build_python/bin/

python3 PullTest_global_single.py --particle_spacing 0.005 --sobol 2048 --cohesion 0 --friction 0.8 --max_force 25 --target_speed 3 --out_dir Pull_global_0.005_0_0.8_25_3
