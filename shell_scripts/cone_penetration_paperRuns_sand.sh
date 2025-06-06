# Default simulation parameters
t_end=0.5
initial_spacing=0.001
d0_multiplier=1.3
time_step=2e-5
boundary_type="adami"
viscosity_type="artificial_bilateral"
kernel_type="wendland"
artificial_viscosity=0.2
mu_s=0.80
mu_2=1.00
mu_i0=0.08

# Configurable parameters with defaults
cone_type_values=(1)      # 1 for 30 degrees, 2 for 60 degrees
gran_material="sand"       # "sand" or "bead"
rel_density=0              # 0 for max density, 1 for min density
container_depth=0.15       # in meters
Hdrop_values=(0 0.5 1)     # Drop heights as multiples of cone length
#Hdrop_values=(1)     # Drop heights as multiples of cone length
neighbor_search_freq_values=(1 2 5 10)  # Neighbor search frequency values to test
#neighbor_search_freq_values=(1)  # Neighbor search frequency values to test
# Print the configuration
echo "Running cone penetration simulations with the following parameters:"
echo "t_end: $t_end"
echo "ps_freq: $ps_freq"
echo "initial_spacing: $initial_spacing"
echo "d0_multiplier: $d0_multiplier"
echo "time_step: $time_step"
echo "boundary_type: $boundary_type"
echo "viscosity_type: $viscosity_type"
echo "kernel_type: $kernel_type"
echo "gran_material: $gran_material"
echo "rel_density: $rel_density"
echo "cone types: ${cone_type_values[*]}"
echo "container_depth: $container_depth"
echo "Hdrop values: ${Hdrop_values[*]}"
echo "Neighbor search frequency values: ${neighbor_search_freq_values[*]}"
echo "artificial_viscosity: $artificial_viscosity"
echo "mu_s: $mu_s"
echo "mu_2: $mu_2"
echo "mu_i0: $mu_i0"
echo ""

# Run simulations for each cone type, neighbor search frequency, and Hdrop value
for cone_type in "${cone_type_values[@]}"; do
  echo "======= Testing cone type = $cone_type ======="
  
  for ps_freq in "${neighbor_search_freq_values[@]}"; do
    echo "===== Testing neighbor search frequency = $ps_freq ====="
    
    for Hdrop in "${Hdrop_values[@]}"; do
      echo "Running simulation with cone_type = $cone_type, ps_freq = $ps_freq, Hdrop = $Hdrop"
      
      # Construct the command
      ./benchmark_Acc_FSI_ConePenetration \
        --t_end $t_end \
        --ps_freq $ps_freq \
        --initial_spacing $initial_spacing \
        --d0_multiplier $d0_multiplier \
        --time_step $time_step \
        --boundary_type $boundary_type \
        --viscosity_type $viscosity_type \
        --kernel_type $kernel_type \
        --gran_material $gran_material \
        --rel_density $rel_density \
        --cone_type $cone_type \
        --container_depth $container_depth \
        --Hdrop $Hdrop \
        --artificial_viscosity $artificial_viscosity \
        --mu_s $mu_s \
        --mu_2 $mu_2 \
        --mu_i0 $mu_i0

    done
  done
done

echo "All simulations completed."
