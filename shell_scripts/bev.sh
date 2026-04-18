#!/bin/bash
#SBATCH --job-name=bev20_0.01_0.02
#SBATCH --cpus-per-task=8
#SBATCH --gres=gpu:1
#SBATCH --time=24:00:00
#SBATCH --mem=64G
#SBATCH --account=wecind
#SBATCH -o bev20_0.01_0.02.out
#SBATCH -e bev20_0.01_0.02.err
#SBATCH --partition=gpu-h100
module load intel-oneapi-mkl
module load cuda/12.3

# Heights in meters for 2.4 cm, 5.8 cm, 11.6 cm, 23.2 cm, 46.4 cm 
./demo_FSI_NormalBevameter --rheology_model_crm MCC --pre_pressure_scale 20 --container_height 0.24


cohesions=(0 100 1000)

for cohesion in "${cohesions[@]}"; do
    ./demo_FSI_NormalBevameter --rheology_model_crm MU_OF_I --cohesion $cohesion --container_height 0.24
done