#!/bin/bash
#SBATCH --job-name=PIDBo25_3_0001
#SBATCH --time=48:00:00
#SBATCH --mem=32G
#SBATCH -o PIDBo25_3_0001.out
#SBATCH -e PIDBo25_3_0001.err
#SBATCH --partition=gpuA100x4
#SBATCH --cpus-per-task=8
#SBATCH --gpus=1
#SBATCH --account=bfuv-delta-gpu
##SBATCH --account=bfqh-delta-gpu
##SBATCH --account=bdgx-delta-gpu
module load cray-python/3.11.7
source ~/venvs/bo_venvs/bin/activate
export PYTHONPATH=$PYTHONPATH:/u/hunjhawala/chrono-wisc-wheel/build_python/bin/

#python SineSideSlip_BO.py --particle_spacing 0.005 --seed_csv SineSideSlip_noBeta_global_1700_0_0.6_25_3_5/trials_mod.csv --density 1700 --cohesion 0 --friction 0.6 --max_force 25 --target_speed 3 --terrain_length 5 --weight_speed 0.4 --weight_power 0.4 --weight_beta 0
#python SineSideSlip_BO.py --particle_spacing 0.005 --resume --density 1700 --cohesion 0 --friction 0.6 --max_force 25 --target_speed 3 --terrain_length 5 --weight_speed 0.4 --weight_power 0.4 --weight_beta 0 --q 1
python3 SineSideSlip_BO_stControllerOnly.py SineSideSlip_global_1700_0_0.6_25_3_5_17Dec/trials.csv --weight_speed 0.0 --weight_power 0.0 --weight_beta 0 --trials_path SineSideSlip_global_1700_0_0.6_25_3_5_17Dec/trials_stPIDControllerOnly_0.0_0.0_0.0_1.0.csv --state_path SineSideSlip_global_1700_0_0.6_25_3_5_17Dec/ax_state_stPIDControllerOnly_0.0_0.0_0.0_1.0.json  --max_trials 1000
