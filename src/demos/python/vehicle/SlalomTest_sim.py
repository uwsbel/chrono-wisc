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
# Bayesian Optimization setup for the Slalom Test with ART
#
# =============================================================================

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
# import pychrono.vsg3d as vsg
import os
import math
import argparse
from simple_wheel_gen import GenSimpleWheelPointCloud
from cuda_error_checker import check_cuda_error, clear_cuda_error, safe_advance, safe_synchronize, safe_initialize

# Global flag for signal handling
simulation_error = None
class Params:
    rad=50, # Radius is 50 * particle_spacing
    width=40, # Width is 40 * particle_spacing
    cp_deviation=0, 
    g_height=10, # Grouser height is 10 * particle_spacing
    g_width=2, # Grouser width is 2 * particle_spacing
    g_density=8, # Number of grousers per revolution
    particle_spacing=0.01, # Particle spacing
    grouser_type=0 # 0 for Straight, 1 for Semi-Circle
    fan_theta_deg=60.0 # Only for Straight - Its the angle with horizontal in clockwise direction
    # Controller gains for joint optimization
    steering_kp=0.8, # Steering proportional gain
    steering_kd=0.0, # Steering derivative gain
    speed_kp=0.6, # Speed proportional gain
    speed_kd=0.05, # Speed derivative gain


# =============================================================================
# Definitions
# =============================================================================

# Terrain dimensions
terrain_length = 5.0
terrain_width = 2
terrain_height = 0.10
# Problem settings
tend = 15
# CRM material properties
density = 1700
cohesion = 1e3
friction = 0.8
youngs_modulus = 1e6
poisson_ratio = 0.3
# Driver controls both steering and throttle to target 5 m/s along slalom
target_speed = 5.0
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
    


def _build_slalom_waypoints(lateral_amplitude_y, vehicle_init_height, start_x, end_x):
    # Simple single lane change within [start_x, end_x]
    # Start at y=0, gentle turn to +Y, hold, and return to 0 by end_x
    length = max(0.1, end_x - start_x)
    f1 = 0.30
    f2 = 0.70
    x0 = start_x
    x1 = start_x + f1 * length
    x2 = start_x + f2 * length
    x3 = end_x
    pts = [
        chrono.ChVector3d(x0, 0.0, vehicle_init_height),
        chrono.ChVector3d(x1, +lateral_amplitude_y, vehicle_init_height),
        chrono.ChVector3d(x2, +lateral_amplitude_y, vehicle_init_height),
        chrono.ChVector3d(x3, 0.0, vehicle_init_height),
    ]
    return pts


def _estimate_polyline_length(points):
    length = 0.0
    for i in range(1, len(points)):
        dx = points[i].x - points[i - 1].x
        dy = points[i].y - points[i - 1].y
        dz = points[i].z - points[i - 1].z
        length += math.sqrt(dx * dx + dy * dy + dz * dz)
    return length


def _nearest_point_distance_xy(point_xy, seg_start_xy, seg_end_xy):
    # Compute distance from a 2D point to a 2D segment
    vx = seg_end_xy[0] - seg_start_xy[0]
    vy = seg_end_xy[1] - seg_start_xy[1]
    wx = point_xy[0] - seg_start_xy[0]
    wy = point_xy[1] - seg_start_xy[1]
    seg_len2 = vx * vx + vy * vy
    if seg_len2 <= 0.0:
        dx = point_xy[0] - seg_start_xy[0]
        dy = point_xy[1] - seg_start_xy[1]
        return math.sqrt(dx * dx + dy * dy)
    t = (wx * vx + wy * vy) / seg_len2
    if t < 0.0:
        closest_x = seg_start_xy[0]
        closest_y = seg_start_xy[1]
    elif t > 1.0:
        closest_x = seg_end_xy[0]
        closest_y = seg_end_xy[1]
    else:
        closest_x = seg_start_xy[0] + t * vx
        closest_y = seg_start_xy[1] + t * vy
    dx = point_xy[0] - closest_x
    dy = point_xy[1] - closest_y
    return math.sqrt(dx * dx + dy * dy)


def _compute_cross_track_error_xy(vehicle_xy, path_points_xy):
    # path_points_xy: list of (x,y) tuples
    # Return nearest lateral distance to polyline
    best = None
    for i in range(1, len(path_points_xy)):
        d = _nearest_point_distance_xy(vehicle_xy, path_points_xy[i - 1], path_points_xy[i])
        if best is None or d < best:
            best = d
    return best if best is not None else 0.0


def sim(Params, weight_speed=0.5, weight_power=0.25, slalom_y=0.2, num_samples=200):
    # Set the data path for vehicle models
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    

    verbose = False
    
    # Visualization settings
    render = True
    render_fps = 200
    visualization_sph = True
    visualization_bndry_bce = False
    visualization_rigid_bce = False
    

    
    # CRM active box dimension
    active_box_dim = 0.4
    settling_time = 0
    
    # SPH spacing - Needs to be same for the wheel generated
    spacing = Params.particle_spacing
    
    # SPH integration scheme
    integration_scheme = fsi.IntegrationScheme_RK2
    
    # Create vehicle
    # print("Create vehicle...")
    vehicle_init_height = 0.3
    
    artCar = veh.ARTcar()
    artCar.SetContactMethod(chrono.ChContactMethod_SMC)
    artCar.SetChassisFixed(False)
    vehicle_x = 0.5
    artCar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(vehicle_x, 0, vehicle_init_height), chrono.QUNIT))
    artCar.SetTireType(veh.TireModelType_RIGID)
    artCar.SetMaxMotorVoltageRatio(0.32)
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
    terrain.GetFluidSystemSPH().EnableCudaErrorCheck(True)  # CRITICAL: Enable CUDA error checking
    
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
    sph_params.num_proximity_search_steps = 10
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
    print("Create terrain...")
    terrain.Construct(chrono.ChVector3d(terrain_length, terrain_width, terrain_height),
                      chrono.ChVector3d(terrain_length / 2, 0, 0),
                      (fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS))
    print(f"Finished constructing terrain")
    
    # Initialize the terrain system
    success, error_msg = safe_initialize(terrain)
    if not success:
        print(f"Terrain initialization failed: {error_msg}")
        sim_failed = True
        total_time_to_reach = tend + 1
        # -50 because this needs to discarded
        return -50, 0, 0, True
    
    aabb = terrain.GetSPHBoundingBox()
    # print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
    # print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
    # print(f"  SPH AABB:          {aabb.min}   {aabb.max}")
    
    # Set maximum vehicle X location
    x_max = aabb.max.x - 0.3
    
    # Create a gentle slalom path (Bezier from waypoints)
    # Build waypoints and simple Bezier control vectors based on local tangents
    # Start the path slightly ahead of the vehicle so it has a short run-up
    start_offset = vehicle_x + 0.1
    start_x = start_offset
    end_x = x_max
    waypoints = _build_slalom_waypoints(slalom_y, vehicle_init_height, start_x, end_x)
    in_cvs = []
    out_cvs = []
    cv_offset = 0.1  # control vector magnitude in meters
    for i in range(len(waypoints)):
        if i == 0:
            dirx = waypoints[i + 1].x - waypoints[i].x
            diry = waypoints[i + 1].y - waypoints[i].y
        elif i == len(waypoints) - 1:
            dirx = waypoints[i].x - waypoints[i - 1].x
            diry = waypoints[i].y - waypoints[i - 1].y
        else:
            dirx = (waypoints[i + 1].x - waypoints[i - 1].x) * 0.5
            diry = (waypoints[i + 1].y - waypoints[i - 1].y) * 0.5
        norm = math.sqrt(dirx * dirx + diry * diry)
        if norm > 1e-9:
            dirx /= norm
            diry /= norm
        # In and out control points
        in_cvs.append(chrono.ChVector3d(waypoints[i].x - dirx * cv_offset,
                                        waypoints[i].y - diry * cv_offset,
                                        waypoints[i].z))
        out_cvs.append(chrono.ChVector3d(waypoints[i].x + dirx * cv_offset,
                                         waypoints[i].y + diry * cv_offset,
                                         waypoints[i].z))

    bezier_path = chrono.ChBezierCurve(waypoints, in_cvs, out_cvs)


    driver = veh.ChPathFollowerDriver(artCar.GetVehicle(), bezier_path, "slalom_path", target_speed)
    driver.GetSteeringController().SetLookAheadDistance(0.2)
    driver.GetSteeringController().SetGains(Params.steering_kp, 0.0, Params.steering_kd)
    driver.GetSpeedController().SetGains(Params.speed_kp, 0.0, Params.speed_kd)
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
    out_dir = chrono.GetChronoOutputPath() + "ARTcar_SlalomTest/"
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, "results.txt")
    snap_dir = os.path.join(os.curdir, "ARTcar_SlalomTest_snapshots")
    os.makedirs(snap_dir, exist_ok=True)
    
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
    #     visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, 0, 2.5))
        
    #     visVSG.Initialize()
    #     vis = visVSG
    
    # Simulation loop
    time = 0
    sim_frame = 0
    render_frame = 0
    braking = False
    sim_failed = False

    # Prepare error metrics
    # For cross-track error, use polyline interpolation of waypoints (XY only)
    path_points_xy = [(wp.x, wp.y) for wp in waypoints]
    path_length = _estimate_polyline_length(waypoints)
    control_delay = 0.5
    sum_squared_error = 0.0
    error_samples = 0
    
    # print("Start simulation...")
    total_time_to_reach = tend + 1
    
    initial_x_position =  artCar.GetVehicle().GetPos().x
    min_speed = 0.15
    net_power = 0
    power_count = 0
    print(f"Starting simulation")
    while time < tend:
        veh_loc = artCar.GetVehicle().GetPos()
        current_x_position = veh_loc.x

        veh_z_vel = artCar.GetChassis().GetPointVelocity(chrono.ChVector3d(0, 0, 0)).z
        veh_roll_rate = artCar.GetChassis().GetRollRate()
        veh_pitch_rate = artCar.GetChassis().GetPitchRate()
        veh_yaw_rate = artCar.GetChassis().GetYawRate()
        veh_z_pos = veh_loc.z

        engine_speed = artCar.GetVehicle().GetEngine().GetMotorSpeed()
        engine_torque = artCar.GetVehicle().GetEngine().GetOutputMotorshaftTorque()
        
        if(time > 0.5):
            net_power += engine_torque * engine_speed
            power_count += 1

        # If Z vel is crazy high, break
        if(veh_z_vel > 15):
            print(f"Veh Z vel is crazy high: {veh_z_vel}")
            # This means vehicle is flying
            sim_failed = True
            total_time_to_reach = tend*2
            break

        # If Z position is too high after 0.3 seconds, break
        if(time > 0.3 and veh_z_pos > 0.8):
            print(f"Veh Z pos is too high: {veh_z_pos}")
            sim_failed = True
            total_time_to_reach = tend*2
            break
        
        # If any of the roll, pitch and yaw rate go above 10, it means the sim has crashed
        if(time > 0.3 and (veh_roll_rate > 10 or veh_pitch_rate > 10 or veh_yaw_rate > 10)):
            print(f"Veh roll rate is too high: {veh_roll_rate}, Veh pitch rate is too high: {veh_pitch_rate}, Veh yaw rate is too high: {veh_yaw_rate}")
            sim_failed = True
            total_time_to_reach = tend*2
            break

        # After 3 seconds check if any meaningful progress has been made
        # If no, then break
        if(time > 3 and current_x_position - initial_x_position < min_speed * 2):
            print(f"No meaningful progress has been made: {current_x_position - initial_x_position}")
            sim_failed = True
            total_time_to_reach = tend*2
            break
        
        # Get driver inputs from path follower after delay
        if time < control_delay:
            driver_inputs = veh.DriverInputs()
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking = 0.0
        else:
            driver_inputs = driver.GetInputs()
        
        # Accumulate cross-track error (XY) against slalom polyline
        e = _compute_cross_track_error_xy((veh_loc.x, veh_loc.y), path_points_xy)
        sum_squared_error += e * e
        error_samples += 1

        # Stop when reaching end of patch length
        if veh_loc.x >= x_max:
            total_time_to_reach = time
            break
        
        # Run-time visualization
        # if render and time >= render_frame / render_fps:
        #     if not vis.Run():
        #         break
        #     vis.Render()
        #     if snapshots:
        #         print(f"Snapshot frame {render_frame} written to {os.path.join(snap_dir, f'img_{render_frame}.png')}")
        #         vis.WriteImageToFile(os.path.join(snap_dir, f"img_{render_frame}.png"))
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
    print(f"  steering_kp: {Params.steering_kp}")
    print(f"  steering_kd: {Params.steering_kd}")
    print(f"  speed_kp: {Params.speed_kp}")
    print(f"  speed_kd: {Params.speed_kd}")
    if(total_time_to_reach < 1):
        print(f"Total time to reach is less than 1 second")
        sim_failed = True
        total_time_to_reach = tend*2
    
    # Compute metrics
    if error_samples > 0:
        rms_error = math.sqrt(max(0.0, sum_squared_error) / error_samples)
    else:
        rms_error = float('inf')

    e_norm = rms_error
    ideal_e_norm = 0.01
    e_norm_score = (e_norm / ideal_e_norm) * 10
    # Time metrics (exclude initial control delay)
    t_elapsed = total_time_to_reach - control_delay

    ideal_time = path_length / target_speed

    r_t = (t_elapsed / ideal_time) * 10
    
    # Power metrics
    if power_count > 0:
        average_power = net_power / power_count
    else:
        # This basically means things have failed
        average_power = float('inf')
    
    # Normalize power (assuming reasonable power range 0-1000W, penalize high power)
    ideal_power = 20.0  # Target average power in watts
    power_score = (average_power / ideal_power) * 10
    
    # Composite metric with three components: speed, tracking, and power
    # Ensure weights sum to 1.0
    weight_tracking = 1.0 - weight_speed - weight_power
    weight_tracking = max(0.0, weight_tracking)  # Ensure non-negative
    
    metric = weight_speed * r_t + weight_tracking * e_norm_score + weight_power * power_score

    if(sim_failed):
        print(f"Simulation failed")
        metric = 500


    print(f"Metric components:")
    print(f"  path_length: {path_length:.4f} m, ideal_time@5m/s: {ideal_time:.4f} s")
    print(f"  elapsed_time: {t_elapsed:.4f} s, ideal_time: {ideal_time:.4f} s, time_ratio_score: {r_t:.4f}")
    print(f"  rms_cross_track_error: {rms_error:.4f} m, ideal_error_norm: {ideal_e_norm:.4f}, error_norm_score: {e_norm_score:.4f}")
    print(f"  average_power: {average_power:.2f} W, ideal_power: {ideal_power:.2f} W, power_score: {power_score:.4f}")
    print(f"  weights: speed={weight_speed:.3f}, tracking={weight_tracking:.3f}, power={weight_power:.3f}")
    print(f"  composite_metric: {metric:.4f}")

    try:
        # Clear CUDA errors before cleanup
        clear_cuda_error()
        
        # Force garbage collection
        #import gc
        #gc.collect()
        
        # Final CUDA error check
        error_occurred, error_message = check_cuda_error()
        if error_occurred:
            print(f"CUDA error during cleanup: {error_message}")
            sim_failed = True
            
    except Exception as cleanup_error:
        print(f"Cleanup error: {cleanup_error}")
        sim_failed = True
    print(f"We have returned") 
    return metric, t_elapsed, rms_error, average_power, sim_failed


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--weight_speed", type=float, default=0.7)
    parser.add_argument("--weight_power", type=float, default=0.1)
    parser.add_argument("--slalom_y", type=float, default=0.4)
    parser.add_argument("--num_samples", type=int, default=200)
    parser.add_argument("--steering_kp", type=float, default=10)
    parser.add_argument("--steering_kd", type=float, default=0.0)
    parser.add_argument("--speed_kp", type=float, default=0.6)
    parser.add_argument("--speed_kd", type=float, default=0.05)
    args = parser.parse_args()

    Params = Params()
    Params.rad = 12
    Params.width = 8
    Params.g_height = 4
    Params.g_width = 4
    Params.g_density = 6
    Params.particle_spacing = 0.01
    Params.grouser_type = 1
    Params.fan_theta_deg = 108
    Params.cp_deviation = 0
    Params.steering_kp = 11.538847
    Params.steering_kd = 1.795085
    Params.speed_kp = 1.557757
    Params.speed_kd = 0.02518
   
    metric, t_elapsed, rms_error, average_power, sim_failed = sim(Params, weight_speed=args.weight_speed, weight_power=args.weight_power, slalom_y=args.slalom_y, num_samples=args.num_samples)
    print(f"Composite metric: {metric:.4f}, elapsed: {t_elapsed:.4f} s, rms_error: {rms_error:.4f} m, avg_power: {average_power:.2f} W, failed: {sim_failed}")
