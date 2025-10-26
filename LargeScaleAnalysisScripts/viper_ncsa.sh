#!/bin/bash
#SBATCH --job-name=viper
#SBATCH --cpus-per-task=1
#SBATCH --time=48:00:00
#SBATCH --mem=32G
#SBATCH -o viper_slope_tests.out
#SBATCH -e viper_slope_tests.err
#SBATCH --partition=gpuH200x8
#SBATCH --cpus-per-task=8
#SBATCH --gpus=1
#SBATCH --array=0-959%16


mkdir -p logs


# Parameter grids
densities=(500 1000 1500 2000 2500)          # 5
gravities=(9.81 3.73 1.62)                   # 3
cohesions=(0 100 1000 5000)                  # 4
frictions=(0.3 0.5 0.7 0.9)                  # 4
slopes=(0 10 25 40)                          # 4

D=${#densities[@]}
G=${#gravities[@]}
C=${#cohesions[@]}
F=${#frictions[@]}
S=${#slopes[@]}

TOTAL=$((D*G*C*F*S))

TASK_ID=${SLURM_ARRAY_TASK_ID:?Missing SLURM_ARRAY_TASK_ID}
if (( TASK_ID < 0 || TASK_ID >= TOTAL )); then
  echo "Error: SLURM_ARRAY_TASK_ID ${TASK_ID} out of range [0, $((TOTAL-1))]" >&2
  exit 1
fi

# Map linear index -> multi-dimensional indices (order: slope, friction, cohesion, gravity, density)
idx=${TASK_ID}
s_idx=$(( idx % S )); idx=$(( idx / S ))
f_idx=$(( idx % F )); idx=$(( idx / F ))
c_idx=$(( idx % C )); idx=$(( idx / C ))
g_idx=$(( idx % G )); idx=$(( idx / G ))
d_idx=$(( idx % D ))

density=${densities[$d_idx]}
gravity=${gravities[$g_idx]}
cohesion=${cohesions[$c_idx]}
friction=${frictions[$f_idx]}
slope=${slopes[$s_idx]}

echo "[Task ${SLURM_ARRAY_TASK_ID}/${TOTAL}] density=${density} gravity=${gravity} cohesion=${cohesion} friction=${friction} slope=${slope}"

# Run the demo with the required flags; disable visualization on the cluster
./demo_ROBOT_Viper_Slope \
  --no_vis \
  --slope_angle "${slope}" \
  --density "${density}" \
  --gravity "${gravity}" \
  --cohesion "${cohesion}" \
  --friction "${friction}"
