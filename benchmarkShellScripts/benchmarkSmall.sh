# Bash script only the fastest case of the benchmark
./benchmark_RTF_FSI_Flexible_Cable --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_FSI_Flexible_Cable --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8

./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 1 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 1 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 3 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_FSI_BaffleFlow --t_end 1 --d0_multiplier 1.2 --scale 3 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8


./benchmark_RTF_FSI_RASSOR_SingleDrum --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_FSI_RASSOR_SingleDrum --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8

./benchmark_RTF_ROBOT_Rassor_SPH --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_ROBOT_Rassor_SPH --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8

./benchmark_RTF_ROBOT_Viper_SPH --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_ROBOT_Viper_SPH --t_end 2 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8

./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 1
./benchmark_RTF_VEH_CRMTerrain_TrackedVehicle --t_end 1 --d0_multiplier 1.2 --viscosity_type artificial_bilateral --boundary_type adami --ps_freq 8

./btest_FSI_RigidBCE_Scaling
