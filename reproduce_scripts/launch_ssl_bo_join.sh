#!/bin/bash
#SBATCH --job-name=JointBo25_3_4402
#SBATCH --time=48:00:00
#SBATCH --mem=32G
#SBATCH -o JointBo25_3_4402.out
#SBATCH -e JointBo25_3_4402.err
#SBATCH --partition=gpuA100x4
#SBATCH --cpus-per-task=8
#SBATCH --gpus=1
#SBATCH --account=bfuv-delta-gpu
##SBATCH --account=bfqh-delta-gpu
##SBATCH --account=bdgx-delta-gpu
module load cray-python/3.11.7
source ~/venvs/bo_venvs/bin/activate
export PYTHONPATH=$PYTHONPATH:/u/hunjhawala/chrono-wisc-wheel/build_python/bin/

python3 SineSideSlip_BO_wControl.py --weight_speed 0.4 --weight_power 0.4 --weight_beta 0 --resume
#python3 SineSideSlip_BO_stControllerOnly.py SineSideSlip_global_1700_0_0.6_25_3_5/trials.csv --weight_speed 0.0 --weight_power 0.0 --weight_beta 0 --trials_path SineSideSlip_global_1700_0_0.6_25_3_5/trials_stControllerOnly_0.0_0.0_0.0_1.0.csv --state_path SineSideSlip_global_1700_0_0.6_25_3_5/ax_state_stControllerOnly_0.0_0.0_0.0_1.0.json
