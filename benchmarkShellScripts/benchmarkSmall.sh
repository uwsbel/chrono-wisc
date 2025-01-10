# Bash script to run different CRM benchmarks to check for consistency in parameters and number of particles

./benchmark_RTF_FSI_Flexible_Cable --t_end 2 --d0_multiplier 1.2

./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 1
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 2
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 3


./benchmark_RTF_FSI_RASSOR_SingleDrum --t_end 2 --d0_multiplier 1.2

./benchmark_RTF_ROBOT_Rassor_SPH --t_end 2 --d0_multiplier 1.2

./benchmark_RTF_ROBOT_Viper_SPH --t_end 2 --d0_multiplier 1.2

./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 1 --d0_multiplier 1.2

# ./btest_FSI_RigidBCE_Scaling