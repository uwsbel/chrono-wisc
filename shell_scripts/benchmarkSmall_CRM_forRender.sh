#!/bin/bash

# Create a timestamped log file
LOG_FILE="BENCHMARK3_$(date +%Y%m%d_%H%M%S).log"
echo "Starting benchmark run at $(date)" > "$LOG_FILE"
echo "===============================================" >> "$LOG_FILE"

# Bash script only the fastest case of the benchmark
# No Active CRM only demos
echo "Running benchmark_RTF_FSI_Flexible_Cable ps_freq 1..." >> "$LOG_FILE"
./benchmark_RTF_FSI_Flexible_Cable --t_end 3 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_FSI_Flexible_Cable --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_FSI_Flexible_Cable --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_FSI_Flexible_Cable ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_Flexible_Cable --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 >> "$LOG_FILE" 2>&1

# echo "Running benchmark_RTF_FSI_BaffleFlow scale 1 ps_freq 1..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 1 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 1 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 1 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_FSI_BaffleFlow scale 1 ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 1 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 >> "$LOG_FILE" 2>&1
# echo "Running benchmark_RTF_FSI_BaffleFlow scale 2 ps_freq 1..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_FSI_BaffleFlow scale 2 ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 >> "$LOG_FILE" 2>&1
echo "Running benchmark_RTF_FSI_BaffleFlow scale 3 ps_freq 1..." >> "$LOG_FILE"
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 3 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 3 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 3 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_FSI_BaffleFlow scale 3 ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 3 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 >> "$LOG_FILE" 2>&1

# Demo already discussed in paper with accuracy
# echo "Running benchmark_RTF_FSI_RASSOR_SingleDrum ps_freq 1..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_RASSOR_SingleDrum --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_FSI_RASSOR_SingleDrum --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_FSI_RASSOR_SingleDrum --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_FSI_RASSOR_SingleDrum ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_FSI_RASSOR_SingleDrum --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 >> "$LOG_FILE" 2>&1

# CRM demos without active domains
# echo "Running benchmark_RTF_ROBOT_Rassor_SPH ps_freq 1..." >> "$LOG_FILE"
# ./benchmark_RTF_ROBOT_Rassor_SPH --t_end 0.1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 --active_domains false >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_ROBOT_Rassor_SPH --t_end 0.1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_ROBOT_Rassor_SPH --t_end 0.1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_ROBOT_Rassor_SPH ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_ROBOT_Rassor_SPH --t_end 0.1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains false >> "$LOG_FILE" 2>&1

# echo "Running benchmark_RTF_ROBOT_Viper_SPH ps_freq 1..." >> "$LOG_FILE"
# ./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 --active_domains false >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_ROBOT_Viper_SPH ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains false >> "$LOG_FILE" 2>&1

# echo "Running benchmark_RTF_VEH_CRMTerrain_TrackedVehicle ps_freq 1..." >> "$LOG_FILE"
# ./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 --active_domains false >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
# echo "Running benchmark_RTF_VEH_CRMTerrain_TrackedVehicle ps_freq 10..." >> "$LOG_FILE"
# ./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains false >> "$LOG_FILE" 2>&1

# CRM demos with active domains
echo "Running benchmark_RTF_ROBOT_Rassor_SPH with active domains..." >> "$LOG_FILE"
./benchmark_RTF_ROBOT_Rassor_SPH --t_end 5 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains true >> "$LOG_FILE" 2>&1
echo "Running benchmark_RTF_ROBOT_Viper_SPH with active domains..." >> "$LOG_FILE"
./benchmark_RTF_ROBOT_Viper_SPH --t_end 5 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains true >> "$LOG_FILE" 2>&1
echo "Running benchmark_RTF_VEH_CRMTerrain_TrackedVehicle with active domains..." >> "$LOG_FILE"


# ./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 15 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains true >> "$LOG_FILE" 2>&1

echo "===============================================" >> "$LOG_FILE"
echo "Benchmark completed at $(date)" >> "$LOG_FILE"
echo "Log file saved to $LOG_FILE"
