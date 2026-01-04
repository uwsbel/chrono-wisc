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
import time
import math
import argparse
import os
import shutil
from simple_wheel_gen import GenSimpleWheelPointCloud
from cuda_error_checker import check_cuda_error, clear_cuda_error, safe_advance, safe_synchronize

SAVE_OUTPUT_HZ = 10

# Global flag for signal handling
simulation_error = None

class Params:
    rad=50, # Radius is 50 * particle_spacing (inner/body radius)
    width=40, # Width is 40 * particle_spacing
    g_height=10, # Grouser height is 10 * particle_spacing
    g_density=8, # Number of grousers per revolution
    particle_spacing=0.005, # Particle spacing
    fan_theta_deg=60.0 # Only for Straight - Its the angle with horizontal in clockwise direction
    steering_kp=5.5 # Steering gain
    steering_ki=0.0 # Steering integral gain
    steering_kd=1.0 # Steering derivative gain
    speed_kp=0.5 # Speed gain
    speed_kd=0.1 # Speed derivative gain


class SimParams:
    density=1700 # Material density
    cohesion=0 # Material cohesion
    friction=0.8 # Material friction coefficient
    max_force=20 # Maximum pulling force
    target_speed=2.0 # Target vehicle speed
    terrain_length=10.0 # Terrain length


# =============================================================================
# Definitions
# =============================================================================

# Terrain dimensions
terrain_width = 1
terrain_height = 0.10
# Vehicle drop height
vehicle_init_height = 0.3
vehicle_x = 0.5
# Time for vehicle to settle
vehicle_init_time = 0.5
# Step size for the simulation
step_size = 2e-4
# Control input frequency (100 Hz = 1e-2 s period)
control_period = 1e-2


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
            grouser_type = "straight"
            radius = Params.rad * Params.particle_spacing
            width = Params.width * Params.particle_spacing
            g_height = Params.g_height * Params.particle_spacing
            g_width = 2 * Params.particle_spacing  # Fixed value, not optimized
            g_density = Params.g_density
            fan_theta_deg = Params.fan_theta_deg
            
            BCE_wheel = GenSimpleWheelPointCloud(rad = radius, width = width, cp_deviation = 0.0, g_height = g_height, g_width = g_width, g_density = g_density, particle_spacing = Params.particle_spacing, grouser_type = grouser_type, fan_theta_deg = fan_theta_deg)

            # Convert BCE Wheel array into ChVector3d
            BCE_wheel = numpy_to_ChVector3d(BCE_wheel)

            sysFSI = terrain.GetFsiSystemSPH()
            fsi_body = sysFSI.AddFsiBody(wheel.GetSpindle(), BCE_wheel, 
                                        chrono.ChFramed(chrono.VNULL, chrono.Q_ROTATE_Z_TO_Y), False)
            fsi_bodies.append(fsi_body)
    
    return fsi_bodies


def buildSineWaypoints(lateral_amplitude_y, vehicle_init_height, start_x, end_x, num_periods=2):
    # Create sine wave path within [start_x, end_x]
    # num_periods controls how many sine wave oscillations
    length = max(0.1, end_x - start_x)
    num_points = 20  # Number of waypoints for smooth sine curve
    pts = []
    for i in range(num_points + 1):
        x = start_x + (i / num_points) * length
        y = lateral_amplitude_y * math.sin(num_periods * math.pi * (x - start_x) / length)
        pts.append(chrono.ChVector3d(x, y, vehicle_init_height))
    return pts


def estimatePolylineLength(points):
    length = 0.0
    for i in range(1, len(points)):
        dx = points[i].x - points[i - 1].x
        dy = points[i].y - points[i - 1].y
        dz = points[i].z - points[i - 1].z
        length += math.sqrt(dx * dx + dy * dy + dz * dz)
    return length


def nearestPointDistanceXY(point_xy, seg_start_xy, seg_end_xy):
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


def computeCrossTrackErrorXY(vehicle_xy, path_points_xy):
    # path_points_xy: list of (x,y) tuples
    # Return nearest lateral distance to polyline
    best = None
    for i in range(1, len(path_points_xy)):
        d = nearestPointDistanceXY(vehicle_xy, path_points_xy[i - 1], path_points_xy[i])
        if best is None or d < best:
            best = d
    return best if best is not None else 0.0


def _reset_output_dir(path):
    if path is None:
        return
    if os.path.isdir(path):
        shutil.rmtree(path)
    elif os.path.exists(path):
        os.remove(path)
    os.makedirs(path, exist_ok=True)


def sim(Params, SimParams, weight_speed=0.6, weight_power=0.0, weight_beta=0.2, sine_amplitude=0.28, num_periods=2,
        snapshot_dir=None, particle_sph_dir=None, particle_fsi_dir=None, blender_dir=None, visualize=False):

    # Set the data path for vehicle models
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Problem settings
    tend = 6
    verbose = False
    

    render_fps = 200
    visualization_sph = True
    visualization_bndry_bce = False
    visualization_rigid_bce = True
    
    # CRM material properties
    density = SimParams.density
    cohesion = SimParams.cohesion
    friction = SimParams.friction
    youngs_modulus = 1e6
    poisson_ratio = 0.3
    
    # CRM active box dimension
    active_box_dim = 0.4
    settling_time = 0
    
    # SPH spacing - Needs to be same for the wheel generated
    spacing = Params.particle_spacing
    
    # SPH integration scheme
    integration_scheme = fsi.IntegrationScheme_RK2
    

    
    artCar = veh.ARTcar()
    artCar.SetContactMethod(chrono.ChContactMethod_SMC)
    artCar.SetChassisFixed(False)
    artCar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(vehicle_x, 0, vehicle_init_height), chrono.QUNIT))
    artCar.SetTireType(veh.TireModelType_RIGID)
    artCar.SetMaxMotorVoltageRatio(1.0)
    artCar.SetStallTorque(5)
    artCar.SetTireRollingResistance(0)
    artCar.Initialize()

    sysMBS = artCar.GetSystem()
    sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    artCar.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    artCar.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    artCar.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    artCar.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    artCar.SetTireVisualizationType(chrono.VisualizationType_NONE)
    

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
    terrain.SetStepSizeCFD(step_size)
    terrain.GetFluidSystemSPH().EnableCudaErrorCheck(True)
    
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
    sph_params.num_proximity_search_steps = 10
    terrain.SetSPHParameters(sph_params)
    
    # Set output level from SPH simulation
    terrain.SetOutputLevel(fsi.OutputLevel_STATE)
    
    # Add vehicle wheels as FSI solids
    fsi_bodies = CreateFSIWheels(artCar, terrain, Params)
    terrain.SetActiveDomain(chrono.ChVector3d(active_box_dim))
    terrain.SetActiveDomainDelay(settling_time)
    if(SimParams.terrain_length == 10.0):
        sine_amplitude = 0.28
        sph_file = f"sph_10_1_0.1_{spacing}_xyz.csv"
        bce_file = f"boundary_10_1_0.1_{spacing}_xyz.csv"
        ideal_e_norm = 0.05
        ideal_power = 200.0
        ideal_beta_rms = 0.05
    elif(SimParams.terrain_length == 5.0):
        sine_amplitude = 0.2
        sph_file = f"sph_5_1_0.1_{spacing}_xyz.csv"
        bce_file = f"boundary_5_1_0.1_{spacing}_xyz.csv"
        ideal_e_norm = 0.03
        ideal_power = 100.0
        ideal_beta_rms = 0.05
    else:
        raise ValueError(f"Invalid terrain length: {SimParams.terrain_length}")
    
    # Construct the terrain
    # print("Create terrain...")
    terrain_start_time = time.time()
    # terrain.Construct(chrono.ChVector3d(terrain_length, terrain_width, terrain_height),
    #                   chrono.ChVector3d(terrain_length / 2, 0, 0),
    #                   (fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS))
    # Read SPH particles from file
    terrain.Construct(sph_file, bce_file, chrono.ChVector3d(0, 0, 0), False)
    terrain_end_time = time.time()
    terrain_construction_time = terrain_end_time - terrain_start_time
    print(f"Terrain construction time: {terrain_construction_time} seconds")
    # Initialize the terrain system
    terrain.Initialize()
    
    aabb = terrain.GetSPHBoundingBox()
    # print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
    # print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
    # print(f"  SPH AABB:          {aabb.min}   {aabb.max}")
    
    # Set maximum vehicle X location
    x_max = aabb.max.x - 1.0
    
    # Create sine wave path (Bezier from waypoints)
    start_x = vehicle_x + 0.1
    end_x = x_max
    waypoints = buildSineWaypoints(sine_amplitude, vehicle_init_height, start_x, end_x, num_periods)
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

    driver = veh.ChPathFollowerDriver(artCar.GetVehicle(), bezier_path, "sine_path", SimParams.target_speed,vehicle_init_time, 2.0)
    driver.GetSteeringController().SetLookAheadDistance(0.25)
    driver.GetSteeringController().SetGains(Params.steering_kp, Params.steering_ki, Params.steering_kd)
    driver.GetSpeedController().SetGains(Params.speed_kp, 0.2, Params.speed_kd)
    driver.Initialize()
    
    # Prepare error metrics
    # For cross-track error, use polyline interpolation of waypoints (XY only)
    path_points_xy = [(wp.x, wp.y) for wp in waypoints]
    path_length = estimatePolylineLength(waypoints)
    
    # Set up TSDA to apply pulling force
    # print("Create pulling force...")
    tsda = chrono.ChLinkTSDA()
    max_force = SimParams.max_force
    zero_force_duration = vehicle_init_time + 0.5
    fast_step_to_max_force_duration = 0.1
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
    # Attachment point on chassis: rear wheel x-position, CG y and z positions
    rear_wheel_x = -0.32264 - 0.05  # Rear wheel spindle x-position relative to chassis reference
    cg_y = -0.0014  # CG y-position (essentially 0)
    cg_z = -0.048 + 0.1   # CG z-position
    tsda.Initialize(spring_box, chassis_body, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(rear_wheel_x, cg_y, cg_z))
    tsda.SetSpringCoefficient(0)
    tsda.SetDampingCoefficient(0)
    
    # Register the force functor
    force_functor = PullForceFunctor(seq)
    tsda.RegisterForceFunctor(force_functor)
    
    sysMBS.AddLink(tsda)
    
    # Add visualization
    # Red sphere on vehicle attachment point (same location as TSDA attachment)
    vis_sphere = chrono.ChVisualShapeSphere(0.05)
    vis_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
    chassis_body.AddVisualShape(vis_sphere, chrono.ChFramed(chrono.ChVector3d(rear_wheel_x, cg_y, cg_z)))
    
    # Spring box visualization
    vis_box = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
    vis_box.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    spring_box.AddVisualShape(vis_box, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    
    # Visual spring (cylinder representing the TSDA)
    tsda.AddVisualShape(chrono.ChVisualShapeSpring(0.1, 80, 1))
    
    # Set up output
    out_dir = chrono.GetChronoOutputPath() + "sine/"
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, "results.txt")
    
    # Create run-time visualization
    vis = None
    if visualize:
        try:
            import pychrono.vsg3d as vsg  # noqa: F401
        except Exception:
            vsg = None
        # FSI plugin
        col_callback = fsi.ParticleHeightColorCallback(aabb.min.z, aabb.max.z)
        visFSI = fsi.ChSphVisualizationVSG(terrain.GetFsiSystemSPH())
        visFSI.EnableFluidMarkers(visualization_sph)
        visFSI.EnableBoundaryMarkers(visualization_bndry_bce)
        visFSI.EnableRigidBodyMarkers(visualization_rigid_bce)
        visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_BROWN)
        
        # Wheeled vehicle VSG visual system
        visVSG = veh.ChWheeledVehicleVisualSystemVSG()
        visVSG.AttachVehicle(artCar.GetVehicle())
        visVSG.AttachPlugin(visFSI)
        visVSG.SetWindowTitle("Wheeled vehicle on CRM deformable terrain")
        visVSG.SetWindowSize(1280, 800)
        visVSG.SetWindowPosition(100, 100)
        visVSG.EnableSkyBox()
        visVSG.SetLightIntensity(1.0)
        visVSG.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
        visVSG.SetCameraAngleDeg(40)
        visVSG.SetChaseCamera(chrono.VNULL, 1.0, 0.0)
        # visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, -1, 0.5))
        visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, 0, 2.5))
        
        visVSG.Initialize()
        vis = visVSG
    
    blender_exporter = None
    if blender_dir is not None:
        try:
            import pychrono.postprocess as postprocess
        except Exception as exc:
            raise RuntimeError("pychrono.postprocess is required for Blender output") from exc
        blender_exporter = postprocess.ChBlender(sysMBS)
        blender_exporter.SetBasePath(blender_dir)
        blender_exporter.AddAll()
        blender_exporter.SetCamera(chrono.ChVector3d(3.0, -2.0, 1.0), chrono.ChVector3d(0, 0, 0.2), 50)
        blender_exporter.ExportScript()

    # Simulation loop
    t_sim = 0
    sim_frame = 0
    sim_failed = False
    
    # print("Start simulation...")
    total_time_to_reach = tend*2
    initial_x_position =  artCar.GetVehicle().GetPos().x
    min_speed = 0.15
    net_power = 0
    power_count = 0
    render_frame = 0
    output_frame = 0
    sim_start_time = time.time()
    
    # Error tracking for cross-track error
    sum_squared_error = 0.0
    error_samples = 0
    control_delay = vehicle_init_time
    
    # Control input timing
    last_control_time = -control_period  # Initialize to allow first control at t=0
    driver_inputs = veh.DriverInputs()
    sum_beta_sq = 0.0
    beta_samples = 0

    # To print average speed
    sum_speed = 0.0
    speed_samples = 0
    
    while t_sim < tend:
        veh_loc = artCar.GetVehicle().GetPos()
        current_x_position = veh_loc.x

        veh_z_vel = artCar.GetChassis().GetPointVelocity(chrono.ChVector3d(0, 0, 0)).z
        veh_roll_rate = artCar.GetChassis().GetRollRate()
        veh_pitch_rate = artCar.GetChassis().GetPitchRate()
        veh_yaw_rate = artCar.GetChassis().GetYawRate()
        veh_z_pos = veh_loc.z

        engine_speed = artCar.GetVehicle().GetEngine().GetMotorSpeed()
        engine_torque = artCar.GetVehicle().GetEngine().GetOutputMotorshaftTorque()
        
        if(t_sim > 0.5):
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
        if(t_sim > 0.3 and veh_z_pos > 0.8):
            print(f"Veh Z pos is too high: {veh_z_pos}")
            sim_failed = True
            total_time_to_reach = tend*2
            break
        
        # If any of the roll, pitch and yaw rate go above 10, it means the sim has crashed
        if(t_sim > 0.3 and (veh_roll_rate > 10 or veh_pitch_rate > 10 or veh_yaw_rate > 10)):
            print(f"Veh roll rate is too high: {veh_roll_rate}, Veh pitch rate is too high: {veh_pitch_rate}, Veh yaw rate is too high: {veh_yaw_rate}")
            sim_failed = True
            total_time_to_reach = tend*2
            break

        # After 3 seconds check if any meaningful progress has been made
        # If no, then break
        if(t_sim > 3 and current_x_position - initial_x_position < min_speed * 2):
            print(f"No meaningful progress has been made: {current_x_position - initial_x_position}")
            sim_failed = True
            total_time_to_reach = tend*2
            break

        # Get driver inputs from path follower at control frequency
        if t_sim >= last_control_time + control_period:
            if t_sim < vehicle_init_time:
                driver_inputs = veh.DriverInputs()
                driver_inputs.m_throttle = 0.0
                driver_inputs.m_steering = 0.0
                driver_inputs.m_braking = 0.0
            else:
                driver_inputs = driver.GetInputs()
                driver_inputs.m_braking = 0.0
            last_control_time = t_sim
        
        # Accumulate cross-track error (XY) against sine path polyline
        if t_sim >= control_delay:
            e = computeCrossTrackErrorXY((veh_loc.x, veh_loc.y), path_points_xy)
            sum_squared_error += e * e
            error_samples += 1
            # Compute body-frame sideslip angle beta = atan2(v_y_body, v_x_body)
            # Use CG velocity expressed in the chassis local frame
            chassis_body = artCar.GetVehicle().GetChassisBody()
            v_world = artCar.GetChassis().GetPointVelocity(chrono.ChVector3d(0, 0, 0))
            v_body = chassis_body.TransformDirectionParentToLocal(v_world)
            # Only compute beta when vehicle speed is above threshold to avoid unstable values at startup
            v_speed = math.sqrt(v_body.x * v_body.x + v_body.y * v_body.y)
            min_speed_for_beta = 0.1  # Minimum speed threshold (m/s) for meaningful beta calculation
            if v_speed >= min_speed_for_beta:
                beta = math.atan2(v_body.y, v_body.x)
                sum_beta_sq += beta * beta
                beta_samples += 1
            sum_speed += v_speed
            speed_samples += 1
        # Stop vehicle before reaching end of terrain patch
        if veh_loc.x >= x_max:
            total_time_to_reach = t_sim
            break
        
        # Run-time visualization
        if visualize and t_sim >= render_frame / render_fps and vis is not None:
            if not vis.Run():
                break
            vis.Render()
            if snapshot_dir is not None:
                try:
                    os.makedirs(snapshot_dir, exist_ok=True)
                    img_path = os.path.join(snapshot_dir, f"img_{render_frame:06d}.png")
                    vis.WriteImageToFile(img_path)
                except Exception:
                    pass
            render_frame += 1
        if (particle_sph_dir is not None or blender_exporter is not None) and t_sim >= output_frame / SAVE_OUTPUT_HZ:
            if particle_sph_dir is not None:
                terrain.SaveOutputData(t_sim, particle_sph_dir, particle_fsi_dir)
            if blender_exporter is not None:
                blender_exporter.ExportData()
            output_frame += 1
        try:
            # Synchronize systems
            driver.Synchronize(t_sim)
            if vis is not None:
                vis.Synchronize(t_sim, driver_inputs)
            terrain.Synchronize(t_sim)
            
            # Use safe vehicle synchronization
            success, error_msg = safe_synchronize(artCar.GetVehicle(), t_sim, driver_inputs, terrain)
            if not success:
                print(f"Vehicle sync failed at time {t_sim:.3f}s: {error_msg}")
                sim_failed = True
                total_time_to_reach = tend*2
                break
            
            # Advance system state
            driver.Advance(step_size)
            if vis is not None:
                vis.Advance(step_size)
            
            # Use safe terrain advance
            success, error_msg = safe_advance(terrain, step_size)
            if not success:
                print(f"Terrain advance failed at time {t_sim:.3f}s: {error_msg}")
                sim_failed = True
                total_time_to_reach = tend*2
                break
            
        except BaseException as e:
            # Catch all exceptions including C++ wrapped exceptions
            print(f"Simulation error at time {t_sim:.3f}s: {type(e).__name__}: {e}")
            sim_failed = True
            total_time_to_reach = tend*2
            break
            
        t_sim += step_size
        sim_frame += 1
    sim_end_time = time.time()
    total_sim_time = sim_end_time - sim_start_time
    print(f"Simulation time: {total_sim_time} seconds")
    time_simulated = t_sim
    if time_simulated > 0:
        realtime_factor = total_sim_time / time_simulated
    else:
        realtime_factor = float('inf')
    print(f"Realtime factor: {realtime_factor} (simulation_time / time_simulated)")
    print(f"Terrain construction time: {terrain_construction_time} seconds")
    # Final check for CUDA errors
    error_occurred, error_message = check_cuda_error()
    if error_occurred:
        print(f"CUDA error detected: {error_message}")
        sim_failed = True
        total_time_to_reach = tend*2
    
    print(f"Parameters used:")
    print(f"  rad: {Params.rad}")
    print(f"  width: {Params.width}")
    print(f"  g_height: {Params.g_height}")
    print(f"  g_density: {Params.g_density}")
    print(f"  particle_spacing: {Params.particle_spacing}")
    print(f"  fan_theta_deg: {Params.fan_theta_deg}")
    if(total_time_to_reach < 1):
        print(f"Total time to reach is less than 1 second")
        sim_failed = True
        total_time_to_reach = tend*2
    
    

    
    # Compute metrics
    # Compute RMS cross-track error
    if error_samples > 0:
        rms_error = math.sqrt(max(0.0, sum_squared_error) / error_samples)
    else:
        rms_error = float('inf')

    e_norm = rms_error

    e_norm_score = (e_norm / ideal_e_norm) * 10
    
    # Time metrics (exclude initial control delay)
    t_elapsed = total_time_to_reach - control_delay
    ideal_time = path_length / SimParams.target_speed
    time_cost_score = (t_elapsed / ideal_time) * 10

    # Power metrics
    average_power = 0
    if power_count > 0:
        average_power = net_power / power_count
    else:
        # This basically means things have failed
        average_power = float('inf')
    power_score = (average_power / ideal_power) * 10

    # Compute RMS body-frame sideslip (beta)
    if beta_samples > 0:
        beta_rms = math.sqrt(max(0.0, sum_beta_sq) / beta_samples)
    else:
        beta_rms = float('inf')

    beta_rms_score = (beta_rms / ideal_beta_rms) * 10


    # Composite metric with three components: speed, tracking, and power
    # Ensure weights sum to 1.0
    weight_tracking = 1.0 - weight_speed - weight_power - weight_beta
    weight_tracking = max(0.0, weight_tracking)  # Ensure non-negative
    
    if(sim_failed):
        print(f"Simulation failed")
        metric = 500
    else:
        metric = weight_speed * time_cost_score + weight_tracking * e_norm_score + weight_power * power_score + weight_beta * beta_rms_score

    print(f"Metric components:")
    print(f"  average_speed: {sum_speed / speed_samples:.4f} m/s")
    print(f"  path_length: {path_length:.4f} m, ideal_time@{SimParams.target_speed}m/s: {ideal_time:.4f} s")
    print(f"  elapsed_time: {t_elapsed:.4f} s, ideal_time: {ideal_time:.4f} s, time_ratio_score: {time_cost_score:.4f}")
    print(f"  rms_cross_track_error: {rms_error:.4f} m, ideal_error_norm: {ideal_e_norm:.4f}, error_norm_score: {e_norm_score:.4f}")
    print(f"  rms_body_sideslip: {beta_rms:.4f} rad ({math.degrees(beta_rms):.2f} deg), ideal_beta_rms: {ideal_beta_rms:.4f} rad ({math.degrees(ideal_beta_rms):.2f} deg)q, beta_rms_score: {beta_rms_score:.4f}")
    print(f"  average_power: {average_power:.2f} W, ideal_power: {ideal_power:.2f} W, power_score: {power_score:.4f}")
    print(f"  weights: speed={weight_speed:.3f}, tracking={weight_tracking:.3f}, power={weight_power:.3f}, beta={weight_beta:.3f}")
    print(f"  composite_metric: {metric:.4f}")

    print(f"Metric: {metric}")

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
    return metric, total_time_to_reach, rms_error, average_power, beta_rms, sim_failed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sine test simulation runner")
    parser.add_argument("--rad", type=int, help="Inner radius in multiples of particle_spacing", required=True)
    parser.add_argument("--width", type=int, help="Width in multiples of particle_spacing", required=True)
    parser.add_argument("--g_height", type=int, help="Grouser height in multiples of particle_spacing", required=True)
    parser.add_argument("--g_density", type=int, required=True)
    parser.add_argument("--particle_spacing", type=float, required=True)
    parser.add_argument("--fan_theta_deg", type=float, default=60.0)
    # Optional controllers (defaults preserved if omitted)
    parser.add_argument("--steering_kp", type=float, default=None)
    parser.add_argument("--steering_ki", type=float, default=None)
    parser.add_argument("--steering_kd", type=float, default=None)
    parser.add_argument("--speed_kp", type=float, default=None)
    parser.add_argument("--speed_kd", type=float, default=None)
    # Material / environment (optional)
    parser.add_argument("--density", type=float, default=1700)
    parser.add_argument("--cohesion", type=float, default=0.0)
    parser.add_argument("--friction", type=float, default=0.8)
    parser.add_argument("--max_force", type=float, default=20.0)
    parser.add_argument("--target_speed", type=float, default=2.0)
    parser.add_argument("--terrain_length", type=float, default=5.0)
    # Weights and sine shape
    parser.add_argument("--weight_speed", type=float, default=0.9)
    parser.add_argument("--weight_power", type=float, default=0.1)
    parser.add_argument("--sine_amplitude", type=float, default=0.28)
    parser.add_argument("--num_periods", type=int, default=2)
    # Visualization / outputs
    parser.add_argument("--visualize", action="store_true", help="Enable visualization")
    parser.add_argument("--snapshots", action="store_true", help="Enable snapshot saving (implies visualize)")
    parser.add_argument("--save-blender", action="store_true", help="Enable Blender output")
    parser.add_argument("--save-particles", action="store_true", help="Enable particle output")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory for snapshots and logs")
    args = parser.parse_args()

    p = Params()
    p.rad = int(args.rad)
    p.width = int(args.width)
    p.g_height = int(args.g_height)
    p.g_density = int(args.g_density)
    p.particle_spacing = float(args.particle_spacing)
    p.fan_theta_deg = float(args.fan_theta_deg)
    # Optional controller overrides
    if args.steering_kp is not None:
        p.steering_kp = float(args.steering_kp)
    if args.steering_ki is not None:
        p.steering_ki = float(args.steering_ki)
    if args.steering_kd is not None:
        p.steering_kd = float(args.steering_kd)
    if args.speed_kp is not None:
        p.speed_kp = float(args.speed_kp)
    if args.speed_kd is not None:
        p.speed_kd = float(args.speed_kd)

    sp = SimParams()
    sp.density = float(args.density)
    sp.cohesion = float(args.cohesion)
    sp.friction = float(args.friction)
    sp.max_force = float(args.max_force)
    sp.target_speed = float(args.target_speed)
    try:
        sp.terrain_length = float(args.terrain_length)
    except Exception:
        pass

    visualize = args.visualize or args.snapshots
    snapshot_dir = os.path.join(args.output_dir, "snapshots") if args.snapshots and args.output_dir else None
    blender_dir = os.path.join(args.output_dir, "blender") if args.save_blender and args.output_dir else None
    particle_dir = os.path.join(args.output_dir, "particle_files") if args.save_particles and args.output_dir else None
    if snapshot_dir is not None:
        _reset_output_dir(snapshot_dir)
    if blender_dir is not None:
        _reset_output_dir(blender_dir)
    particle_sph_dir = None
    particle_fsi_dir = None
    if particle_dir is not None:
        _reset_output_dir(particle_dir)
        particle_sph_dir = os.path.join(particle_dir, "particles")
        particle_fsi_dir = os.path.join(particle_dir, "fsi")
        os.makedirs(particle_sph_dir, exist_ok=True)
        os.makedirs(particle_fsi_dir, exist_ok=True)

    metric, total_time_to_reach, rms_error, average_power, beta_rms, sim_failed = sim(
        p, sp,
        weight_speed=float(args.weight_speed),
        weight_power=float(args.weight_power),
        sine_amplitude=float(args.sine_amplitude),
        num_periods=int(args.num_periods),
        snapshot_dir=snapshot_dir,
        particle_sph_dir=particle_sph_dir,
        particle_fsi_dir=particle_fsi_dir,
        blender_dir=blender_dir,
        visualize=visualize,
    )
    print(f"Metric: {metric:.4f}")
    print(f"Total time to reach: {total_time_to_reach:.4f}")
    print(f"RMS cross-track error: {rms_error:.4f} m")
    print(f"Average power: {average_power:.2f}W")
    print(f"RMS body sideslip (beta): {beta_rms:.4f} rad ({math.degrees(beta_rms):.2f} deg)")
    print(f"Simulation failed: {sim_failed}")
