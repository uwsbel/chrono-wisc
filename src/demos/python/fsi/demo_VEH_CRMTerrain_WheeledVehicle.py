import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
import pychrono.vsg as vsg
import os



# TODO(uebian): Expose CRMTerrain and related FSI coupling classes.

    
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
# Problem settings (mirroring the C++ demo)
target_speed = 7.0
tend = 30.0
verbose = True

# Visualization settings
render = True
render_fps = 200
visualization_sph = True
visualization_bndry_bce = False
visualization_rigid_bce = False

# CRM material properties
density = 1700
cohesion = 5e3
friction = 0.8
youngs_modulus = 1e6
poisson_ratio = 0.3

# CRM (moving) active box dimension
active_box_dim = 0.8
settling_time = 0

# Set SPH spacing
spacing = 0.04

# SPH integration scheme (if exposed)
# integration_scheme = fsi.IntegrationScheme_RK2  # Example, adjust as needed

# Vehicle specification files (adjust paths as needed)
vehicle_json = "Polaris/Polaris.json"
engine_json = "Polaris/Polaris_EngineSimpleMap.json"
transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json"
tire_json = "Polaris/Polaris_RigidTire.json"
# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = chrono.VisualizationType_MESH
suspension_vis_type = chrono.VisualizationType_PRIMITIVES
steering_vis_type = chrono.VisualizationType_PRIMITIVES
wheel_vis_type = chrono.VisualizationType_NONE
tire_vis_type = chrono.VisualizationType_MESH

# --------------
# Create vehicle
# --------------
print("Create vehicle...")
vehicle_init_height = 0.25
# This assumes a WheeledVehicle class and similar API as C++
vehicle = veh.WheeledVehicle(veh.GetDataFile(vehicle_json), chrono.ChContactMethod_SMC)
vehicle.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(3.5, 0, vehicle_init_height), chrono.QUNIT))
vehicle.GetChassis().SetFixed(False)
vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)

# Powertrain
engine = veh.ReadEngineJSON(veh.GetDataFile(engine_json))
transmission = veh.ReadTransmissionJSON(veh.GetDataFile(transmission_json))
powertrain = veh.ChPowertrainAssembly(engine, transmission)
vehicle.InitializePowertrain(powertrain)

# Tires
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.ReadTireJSON(veh.GetDataFile(tire_json))
        vehicle.InitializeTire(tire, wheel, chrono.VisualizationType_MESH)

sysMBS = vehicle.GetSystem()
sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ----------------------
# Create the CRM terrain
# ----------------------
# This assumes pychrono.vehicle.CRMTerrain exists and has a similar API to C++
terrain = veh.CRMTerrain(sysMBS, spacing)
sysFSI = terrain.GetSystemFSI()
terrain.SetVerbose(verbose)
terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
terrain.SetStepSizeCFD(5e-4)

# Register the vehicle with the CRM terrain
terrain.RegisterVehicle(vehicle)

# Set SPH parameters and soil material properties
mat_props = fsi.ElasticMaterialProperties()
mat_props.density = density
mat_props.Young_modulus = youngs_modulus
mat_props.Poisson_ratio = poisson_ratio
mat_props.mu_I0 = 0.04
mat_props.mu_fric_s = friction
mat_props.mu_fric_2 = friction
mat_props.average_diam = 0.005
mat_props.cohesion_coeff = cohesion
terrain.SetElasticSPH(mat_props)

# Set SPH solver parameters (if available)
sph_params = fsi.SPHParameters()
sph_params.initial_spacing = spacing
sph_params.d0_multiplier = 1
sph_params.kernel_threshold = 0.8
sph_params.artificial_viscosity = 0.5
sph_params.consistent_gradient_discretization = False
sph_params.consistent_laplacian_discretization = False
# ... add more as needed
terrain.SetSPHParameters(sph_params)

# Set output level from SPH simulation (if available)
# terrain.SetOutputLevel(fsi.OutputLevel_STATE)

# Add vehicle wheels as FSI solids (if needed)
# This step may require a helper function, as in the C++ demo
# For now, assume terrain.RegisterVehicle handles this

terrain.SetActiveDomain(chrono.ChVector3d(active_box_dim, active_box_dim, active_box_dim))
terrain.SetActiveDomainDelay(settling_time)

# Construct the terrain and associated path
print("Create terrain...")
terrain_length = 20
terrain_width = 3
terrain.Construct(chrono.ChVector3d(terrain_length, terrain_width, 0.25), chrono.ChVector3d(terrain_length / 2, 0, 0), (fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS))

# Create straight line path (if needed for driver)
path = veh.StraightLinePath(chrono.ChVector3d(0, 0, vehicle_init_height), chrono.ChVector3d(terrain_length, 0, vehicle_init_height), 1)

# Initialize the terrain system
terrain.Initialize()

aabb = terrain.GetSPHBoundingBox()
print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
print(f"  SPH AABB:          {aabb.min}   {aabb.max}")

# Set maximum vehicle X location (based on CRM patch size)
x_max = aabb.max.x - 4.5

# --------------------------------
# Create the path-following driver
# --------------------------------
print("Create path...")
driver = veh.ChPathFollowerDriver(vehicle, path, "my_path", target_speed)
driver.GetSteeringController().SetLookAheadDistance(2.0)
driver.GetSteeringController().SetGains(1.0, 0, 0)
driver.GetSpeedController().SetGains(0.6, 0.05, 0)
driver.Initialize()

# -----------------------------
# Set up output
# -----------------------------
out_dir = chrono.GetChronoOutputPath() + "CRM_Wheeled_Vehicle/"
os.makedirs(out_dir, exist_ok=True)
out_file = os.path.join(out_dir, "results.txt")
# Use a simple CSV writer
import csv
csvfile = open(out_file, 'w', newline='')
csvwriter = csv.writer(csvfile, delimiter=' ')

# -----------------------------
# Create run-time visualization
# -----------------------------
vis = None
if render:
    # FSI plugin (assume similar to DamBreak demo)
    col_callback = fsi.ParticleHeightColorCallback(aabb.min.z, aabb.max.z)
    visFSI = fsi.ChFsiVisualizationVSG(sysFSI)
    visFSI.EnableFluidMarkers(visualization_sph)
    visFSI.EnableBoundaryMarkers(visualization_bndry_bce)
    visFSI.EnableRigidBodyMarkers(visualization_rigid_bce)
    visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_BROWN)

    visVSG = veh.ChWheeledVehicleVisualSystemVSG()
    visVSG.AttachVehicle(vehicle)
    visVSG.AttachPlugin(visFSI)
    visVSG.SetWindowTitle("Wheeled vehicle on CRM deformable terrain")
    visVSG.SetWindowSize(1280, 800)
    visVSG.SetWindowPosition(100, 100)
    visVSG.EnableSkyBox()
    visVSG.SetLightIntensity(1.0)
    visVSG.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
    visVSG.SetCameraAngleDeg(40)
    visVSG.SetChaseCamera(chrono.VNULL, 6.0, 2.0)
    visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, 8, 1.5))
    visVSG.Initialize()
    vis = visVSG

# ---------------
# Simulation loop
# ---------------
time = 0
sim_frame = 0
render_frame = 0
braking = False
step_size = 5e-4

print("Start simulation...")
while time < tend:
    veh_loc = vehicle.GetPos()
    driver_inputs = driver.GetInputs()

    # Ramp up throttle to value requested by the cruise controller
    if time < 0.5:
        driver_inputs.m_throttle = 0
        driver_inputs.m_braking = 1
    else:
        driver_inputs.m_throttle = min(driver_inputs.m_throttle, (time - 0.5) / 0.5)

    # Stop vehicle before reaching end of terrain patch, then end simulation after 2 more seconds
    if veh_loc.x > x_max:
        driver_inputs.m_throttle = 0
        driver_inputs.m_braking = 1
        if not braking:
            print("Start braking...")
            tend = time + 2
            braking = True

    # Run-time visualization
    if render and time >= render_frame / render_fps:
        if not vis.Run():
            break
        vis.Render()
        render_frame += 1

    # Synchronize systems
    driver.Synchronize(time)
    if vis:
        vis.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)

    # Advance system state
    driver.Advance(step_size)
    if vis:
        vis.Advance(step_size)
    terrain.Advance(step_size)

    # Output results
    csvwriter.writerow([time, veh_loc.x, veh_loc.y, veh_loc.z, vehicle.GetSpeed()])

    time += step_size
    sim_frame += 1

csvfile.close()
print("Simulation complete. Results written to:", out_file)

