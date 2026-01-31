#!/bin/bash
#SBATCH --job-name=WheelOnlyBo25_3_4402
#SBATCH --time=48:00:00
#SBATCH --mem=32G
#SBATCH -o WheelOnlyBo25_3_4402.out
#SBATCH -e WheelOnlyBo25_3_4402.err
#SBATCH --partition=gpuA100x4
#SBATCH --cpus-per-task=8
#SBATCH --gpus=1
#SBATCH --account=bfuv-delta-gpu
##SBATCH --account=bfqh-delta-gpu
##SBATCH --account=bdgx-delta-gpu
module load cray-python/3.11.7
source ~/venvs/bo_venvs/bin/activate
export PYTHONPATH=$PYTHONPATH:/u/hunjhawala/chrono-wisc-wheel/build_python/bin/
python SineSideSlip_BO.py --particle_spacing 0.005 --density 1700 --cohesion 0 --friction 0.6 --max_force 25 --target_speed 3 --terrain_length 5 --weight_speed 0.4 --weight_power 0.4 --weight_beta 0 --q 1 --out_dir SineSideSlip_global_1700_0_0.6_25_3_5_17Dec --resume
