# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2023 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Author: Huzaifa Unjhawala
# =============================================================================
#
# Bayesian Optimization setup for the Pull Test with ART
#
# =============================================================================

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
# import pychrono.vsg3d as vsg
import os
from simple_wheel_gen import GenSimpleWheelPointCloud
from cuda_error_checker import check_cuda_error, clear_cuda_error, safe_advance, safe_synchronize

# Global flag for signal handling
simulation_error = None

class Params:
    rad=50, # Radius is 50 * particle_spacing
    width=40, # Width is 40 * particle_spacing
    cp_deviation=0, 
    g_height=10, # Grouser height is 10 * particle_spacing
    g_width=2, # Grouser width is 2 * particle_spacing
    g_density=8, # Number of grousers per revolution
    particle_spacing=0.005, # Particle spacing
    grouser_type=0 # 0 for Straight, 1 for Semi-Circle
    fan_theta_deg=60.0 # Only for Straight - Its the angle with horizontal in clockwise direction


# =============================================================================
# Definitions
# =============================================================================

# Terrain dimensions
terrain_length = 2.5
terrain_width = 1
terrain_height = 0.10

wheel_BCE_csvfile = "vehicle/artcar/wheel_straight.csv"
# Pull force is constant
class PullForceFunctor(chrono.ForceFunctor):
    """
    Custom force functor for TSDA to apply pulling force.
    """
    def __init__(self, force_func):
        super().__init__()
        self.m_force_func = force_func
    
    def evaluate(self, time, rest_length, length, vel, link):
        return self.m_force_func.GetVal(time)

def numpy_to_ChVector3d(numpy_array):
    return [chrono.ChVector3d(row[0], row[1], row[2]) for row in numpy_array]

def CreateFSIWheels(vehicle, terrain, Params):
    """
    Create FSI wheels for the vehicle based on the specified wheel type.
    
    Args:
        vehicle: ARTcar object
        terrain: CRMTerrain object
        wheel_type: Type of wheel representation (SIMPLE, MESH, BCE_MARKERS)
    
    Returns:
        List of FsiBody objects for BCE_MARKERS type, empty list otherwise
    """
    fsi_bodies = []
    for axle in vehicle.GetVehicle().GetAxles():
        for wheel in axle.GetWheels():
            sysFSI = terrain.GetFsiSystemSPH()

            # Convert the optimization variables into actual values
            # Everything is scaled by initial spacing
            grouser_type = "straight" if Params.grouser_type == 0 else "semi_circle"
            radius = Params.rad * Params.particle_spacing
            width = Params.width * Params.particle_spacing
            g_height = Params.g_height * Params.particle_spacing
            g_width = Params.g_width * Params.particle_spacing
            g_density = Params.g_density
            fan_theta_deg = Params.fan_theta_deg
            
            BCE_wheel = GenSimpleWheelPointCloud(rad = radius, width = width, cp_deviation = Params.cp_deviation, g_height = g_height, g_width = g_width, g_density = g_density, particle_spacing = Params.particle_spacing, grouser_type = grouser_type, fan_theta_deg = fan_theta_deg)

            # Convert BCE Wheel array into ChVector3d
            BCE_wheel = numpy_to_ChVector3d(BCE_wheel)

            sysFSI = terrain.GetFsiSystemSPH()
            fsi_body = sysFSI.AddFsiBody(wheel.GetSpindle(), BCE_wheel, 
                                        chrono.ChFramed(chrono.VNULL, chrono.Q_ROTATE_Z_TO_Y), False)
            fsi_bodies.append(fsi_body)
    
    return fsi_bodies
    


def sim(Params):
    # Set the data path for vehicle models
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Problem settings
    tend = 10
    verbose = False
    
    # Visualization settings
    render = True
    render_fps = 200
    visualization_sph = True
    visualization_bndry_bce = False
    visualization_rigid_bce = False
    
    # CRM material properties
    density = 1700
    cohesion = 1e3
    friction = 0.8
    youngs_modulus = 1e6
    poisson_ratio = 0.3
    
    # CRM active box dimension
    active_box_dim = 0.4
    settling_time = 0
    
    # SPH spacing - Needs to be same for the wheel generated
    spacing = Params.particle_spacing
    
    # SPH integration scheme
    integration_scheme = fsi.IntegrationScheme_RK2
    
    # Create vehicle
    # print("Create vehicle...")
    vehicle_init_height = 0.5
    
    artCar = veh.ARTcar()
    artCar.SetContactMethod(chrono.ChContactMethod_SMC)
    artCar.SetChassisFixed(False)
    artCar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0.5, 0, vehicle_init_height), chrono.QUNIT))
    artCar.SetTireType(veh.TireModelType_RIGID)
    artCar.SetMaxMotorVoltageRatio(0.16)
    artCar.SetStallTorque(3)
    artCar.SetTireRollingResistance(0)
    artCar.Initialize()
    
    sysMBS = artCar.GetSystem()
    sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    artCar.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    artCar.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    artCar.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    artCar.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    artCar.SetTireVisualizationType(chrono.VisualizationType_MESH)
    
    # Set solver and integrator for MBD
    step_size = 5e-4
    solver_type = chrono.ChSolver.Type_BARZILAIBORWEIN
    integrator_type = chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED
    
    num_threads_chrono = 8
    num_threads_collision = 1
    num_threads_eigen = 7
    num_threads_pardiso = 0
    
    sysMBS.SetSolverType(solver_type)
    sysMBS.SetTimestepperType(integrator_type)
    sysMBS.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen)
    
    # Create the CRM terrain
    terrain = veh.CRMTerrain(sysMBS, spacing)
    # sysFSI = terrain.GetFsiSystemSPH()
    terrain.SetVerbose(verbose)
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    terrain.SetStepSizeCFD(1e-4)
    terrain.GetFluidSystemSPH().EnableCudaErrorCheck(False)
    
    # Register the vehicle with the CRM terrain
    terrain.RegisterVehicle(artCar.GetVehicle())
    
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
    
    # Set SPH solver parameters
    sph_params = fsi.SPHParameters()
    sph_params.integration_scheme = integration_scheme
    sph_params.initial_spacing = spacing
    sph_params.d0_multiplier = 1.2
    sph_params.free_surface_threshold = 0.8
    sph_params.artificial_viscosity = 0.2
    sph_params.shifting_method = fsi.ShiftingMethod_PPST_XSPH
    sph_params.shifting_xsph_eps = 0.5
    sph_params.shifting_ppst_pull = 1.0
    sph_params.shifting_ppst_push = 3.0
    sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_BILATERAL
    sph_params.boundary_method = fsi.BoundaryMethod_ADAMI
    sph_params.kernel_type = fsi.KernelType_WENDLAND
    terrain.SetSPHParameters(sph_params)
    
    # Set output level from SPH simulation
    terrain.SetOutputLevel(fsi.OutputLevel_STATE)
    
    # Add vehicle wheels as FSI solids
    fsi_bodies = CreateFSIWheels(artCar, terrain, Params)
    terrain.SetActiveDomain(chrono.ChVector3d(active_box_dim))
    terrain.SetActiveDomainDelay(settling_time)
    
    # Construct the terrain
    # print("Create terrain...")
    terrain.Construct(chrono.ChVector3d(terrain_length, terrain_width, terrain_height),
                      chrono.ChVector3d(terrain_length / 2, 0, 0),
                      (fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS))
    
    # Initialize the terrain system
    terrain.Initialize()
    
    aabb = terrain.GetSPHBoundingBox()
    # print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
    # print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
    # print(f"  SPH AABB:          {aabb.min}   {aabb.max}")
    
    # Set maximum vehicle X location
    x_max = aabb.max.x - 0.5
    
    # Create the path-following driver
    # print("Create path...")
    path = veh.StraightLinePath(chrono.ChVector3d(0, 0, vehicle_init_height),
                               chrono.ChVector3d(terrain_length, 0, vehicle_init_height), 1)
    
    target_speed = 1.0
    driver = veh.ChPathFollowerDriver(artCar.GetVehicle(), path, "straight_path", target_speed)
    driver.GetSteeringController().SetLookAheadDistance(1.0)
    driver.GetSteeringController().SetGains(1.0, 0, 0)
    driver.GetSpeedController().SetGains(0.6, 0.05, 0)
    driver.Initialize()
    
    # Set up TSDA to apply pulling force
    # print("Create pulling force...")
    tsda = chrono.ChLinkTSDA()
    max_force = 20
    zero_force_duration = 0.6
    fast_step_to_max_force_duration = 0.01
    max_force_duration = tend - zero_force_duration - fast_step_to_max_force_duration
    
    # Create force sequence
    seq = chrono.ChFunctionSequence()
    f_const = chrono.ChFunctionConst(0)
    f_fast = chrono.ChFunctionRamp(0, -max_force / fast_step_to_max_force_duration)
    f_const_max = chrono.ChFunctionConst(-max_force)
    seq.InsertFunct(f_const, zero_force_duration)
    seq.InsertFunct(f_fast, fast_step_to_max_force_duration)
    seq.InsertFunct(f_const_max, max_force_duration)
    seq.Setup()
    
    chassis_body = artCar.GetVehicle().GetChassisBody()
    
    # Create spring box (anchor point)
    spring_box = chrono.ChBody()
    spring_box.SetFixed(True)
    spring_box_pos = chrono.ChVector3d(0, 0, terrain_height / 2 + 0.1)
    spring_box.SetPos(spring_box_pos)
    sysMBS.AddBody(spring_box)
    
    # Initialize TSDA between chassis and spring box
    tsda.Initialize(spring_box, chassis_body, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    tsda.SetSpringCoefficient(0)
    tsda.SetDampingCoefficient(0)
    
    # Register the force functor
    force_functor = PullForceFunctor(seq)
    tsda.RegisterForceFunctor(force_functor)
    
    sysMBS.AddLink(tsda)
    
    # Add visualization
    # Red sphere on vehicle attachment point
    vis_sphere = chrono.ChVisualShapeSphere(0.05)
    vis_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
    chassis_body.AddVisualShape(vis_sphere, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    
    # Spring box visualization
    vis_box = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
    vis_box.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    spring_box.AddVisualShape(vis_box, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    
    # Visual spring (cylinder representing the TSDA)
    tsda.AddVisualShape(chrono.ChVisualShapeSpring(0.1, 80, 15))
    
    # Set up output
    out_dir = chrono.GetChronoOutputPath() + "ARTcar_PullTest/"
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, "results.txt")
    
    # Prepare per-tire CSV writers
    num_axles = artCar.GetVehicle().GetNumberAxles()
    tire_writers = []
    tire_counts = []
    
    for ia in range(num_axles):
        tire_writers.append([None, None])
        tire_counts.append([0, 0])
        for is_side in range(2):
            tire_file = os.path.join(out_dir, f"tire_ax{ia}_{'L' if is_side == 0 else 'R'}.csv")
            tire_writers[ia][is_side] = open(tire_file, 'w', newline='')
    
    # Create run-time visualization
    # vis = None
    # if render:
    #     # FSI plugin
    #     col_callback = fsi.ParticleHeightColorCallback(aabb.min.z, aabb.max.z)
    #     visFSI = fsi.ChSphVisualizationVSG(sysFSI)
    #     visFSI.EnableFluidMarkers(visualization_sph)
    #     visFSI.EnableBoundaryMarkers(visualization_bndry_bce)
    #     visFSI.EnableRigidBodyMarkers(visualization_rigid_bce)
    #     visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_BROWN)
        
    #     # Wheeled vehicle VSG visual system
    #     visVSG = veh.ChWheeledVehicleVisualSystemVSG()
    #     visVSG.AttachVehicle(artCar.GetVehicle())
    #     visVSG.AttachPlugin(visFSI)
    #     visVSG.SetWindowTitle("Wheeled vehicle on CRM deformable terrain")
    #     visVSG.SetWindowSize(1280, 800)
    #     visVSG.SetWindowPosition(100, 100)
    #     visVSG.EnableSkyBox()
    #     visVSG.SetLightIntensity(1.0)
    #     visVSG.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
    #     visVSG.SetCameraAngleDeg(40)
    #     visVSG.SetChaseCamera(chrono.VNULL, 1.0, 0.0)
    #     visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, -1, 0.5))
        
    #     visVSG.Initialize()
    #     vis = visVSG
    
    # Simulation loop
    time = 0
    sim_frame = 0
    render_frame = 0
    braking = False
    sim_failed = False
    
    # print("Start simulation...")
    total_time_to_reach = tend + 1
    
        
    while time < tend:
        veh_loc = artCar.GetVehicle().GetPos()

        veh_z_vel = artCar.GetChassis().GetPointVelocity(chrono.ChVector3d(0, 0, 0)).z
        veh_roll_rate = artCar.GetChassis().GetRollRate()
        veh_pitch_rate = artCar.GetChassis().GetPitchRate()
        veh_yaw_rate = artCar.GetChassis().GetYawRate()

        print(f"Veh roll rate: {veh_roll_rate}, Veh pitch rate: {veh_pitch_rate}, Veh yaw rate: {veh_yaw_rate}")

        if(veh_z_vel > 15):
            # This means vehicle is flying
            sim_failed = True
            total_time_to_reach = tend + 1
            break
        
        # If any of the roll, pitch and yaw rate go above 10, it means the sim has crashed
        if(time > 0.3 and (veh_roll_rate > 10 or veh_pitch_rate > 10 or veh_yaw_rate > 10)):
            sim_failed = True
            total_time_to_reach = tend + 1
            break
        
        # Get driver inputs from path follower
        driver_inputs = driver.GetInputs()
        
        # Override throttle control with custom logic
        if time < zero_force_duration - 0.1:
            driver_inputs.m_throttle = 0
            driver_inputs.m_braking = 1
        elif time < zero_force_duration and time > zero_force_duration - 0.1:
            driver_inputs.m_throttle = (-(time - zero_force_duration) / 0.1) * 0.7
            driver_inputs.m_braking = 0
        else:
            driver_inputs.m_throttle = 0.7
            driver_inputs.m_braking = 0
        
        # Stop vehicle before reaching end of terrain patch
        if veh_loc.x > x_max:
            total_time_to_reach = time
            break
        
        # Run-time visualization
        # if render and time >= render_frame / render_fps:
        #     if not vis.Run():
        #         break
        #     vis.Render()
        #     render_frame += 1
        try:
            # Synchronize systems
            driver.Synchronize(time)
            # if vis:
            #     vis.Synchronize(time, driver_inputs)
            terrain.Synchronize(time)
            
            # Use safe vehicle synchronization
            success, error_msg = safe_synchronize(artCar.GetVehicle(), time, driver_inputs, terrain)
            if not success:
                print(f"Vehicle sync failed at time {time:.3f}s: {error_msg}")
                sim_failed = True
                total_time_to_reach = tend + 1
                break
            
            # Advance system state
            driver.Advance(step_size)
            # if vis:
            #     vis.Advance(step_size)
            
            # Use safe terrain advance
            success, error_msg = safe_advance(terrain, step_size)
            if not success:
                print(f"Terrain advance failed at time {time:.3f}s: {error_msg}")
                sim_failed = True
                total_time_to_reach = tend + 1
                break
            
        except BaseException as e:
            # Catch all exceptions including C++ wrapped exceptions
            print(f"Simulation error at time {time:.3f}s: {type(e).__name__}: {e}")
            sim_failed = True
            total_time_to_reach = tend + 1
            break
            
        time += step_size
        sim_frame += 1
    
    # Final check for CUDA errors
    error_occurred, error_message = check_cuda_error()
    if error_occurred:
        print(f"CUDA error detected: {error_message}")
        sim_failed = True
        total_time_to_reach = tend + 1
    
    print(f"Parameters used:")
    print(f"  rad: {Params.rad}")
    print(f"  width: {Params.width}")
    print(f"  g_height: {Params.g_height}")
    print(f"  g_width: {Params.g_width}")
    print(f"  g_density: {Params.g_density}")
    print(f"  particle_spacing: {Params.particle_spacing}")
    print(f"  grouser_type: {Params.grouser_type}")
    print(f"  fan_theta_deg: {Params.fan_theta_deg}")
    print(f"  cp_deviation: {Params.cp_deviation}")
    if(total_time_to_reach < 1):
        print(f"Total time to reach is less than 1 second")
        sim_failed = True
        total_time_to_reach = tend + 1
    
    if(sim_failed):
        print(f"Simulation failed")
    else:
        print(f"Total time to reach: {total_time_to_reach}")

    import gc; gc.collect()
    return total_time_to_reach, sim_failed


if __name__ == "__main__":
    Params = Params()
    Params.rad = 7
    Params.width = 17
    Params.g_height = 3
    Params.g_width = 5
    Params.g_density = 16
    Params.particle_spacing = 0.01
    Params.grouser_type = 0
    Params.fan_theta_deg = 61
    Params.cp_deviation = 0
    
    total_time_to_reach, sim_failed = sim(Params)
    print(f"Total time to reach: {total_time_to_reach}")
