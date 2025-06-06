#!/bin/bash

# Create a timestamped log file
LOG_FILE="BENCHMARK3_$(date +%Y%m%d_%H%M%S).log"
echo "Starting benchmark run at $(date)" > "$LOG_FILE"
echo "===============================================" >> "$LOG_FILE"

echo "Running benchmark_RTF_ROBOT_Viper_SPH ps_freq 1..." >> "$LOG_FILE"
./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1 --active_domains false >> "$LOG_FILE" 2>&1
# ./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 2
# ./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 5
echo "Running benchmark_RTF_ROBOT_Viper_SPH ps_freq 10..." >> "$LOG_FILE"
./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains false >> "$LOG_FILE" 2>&1


echo "Running benchmark_RTF_ROBOT_Viper_SPH with active domains..." >> "$LOG_FILE"
./benchmark_RTF_ROBOT_Viper_SPH --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 10 --active_domains true >> "$LOG_FILE" 2>&1


echo "===============================================" >> "$LOG_FILE"
echo "Benchmark completed at $(date)" >> "$LOG_FILE"
echo "Log file saved to $LOG_FILE"

