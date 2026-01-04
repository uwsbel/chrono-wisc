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
import argparse
import os
from simple_wheel_gen import GenSimpleWheelPointCloud
from cuda_error_checker import check_cuda_error, clear_cuda_error, safe_advance, safe_synchronize

# Global flag for signal handling
simulation_error = None

class Params:
    rad=50, # Radius is 50 * particle_spacing (inner/body radius)
    width=40, # Width is 40 * particle_spacing
    g_height=10, # Grouser height is 10 * particle_spacing
    g_density=8, # Number of grousers per revolution
    particle_spacing=0.005, # Particle spacing
    fan_theta_deg=60.0 # Only for Straight - Its the angle with horizontal in clockwise direction


class SimParams:
    density=1700 # Material density
    cohesion=0 # Material cohesion
    friction=0.8 # Material friction coefficient
    max_force=20 # Maximum pulling force
    target_speed=2.0 # Target vehicle speed


# =============================================================================
# Definitions
# =============================================================================

# Terrain dimensions
terrain_length = 5.0
terrain_width = 1
terrain_height = 0.10
# Vehicle drop height
vehicle_init_height = 0.3
vehicle_x = 0.5
# Time for vehicle to settle
vehicle_init_time = 0.5
# Step size for the simulation
step_size = 2e-4


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
    


def sim(Params, SimParams, weight_speed=0.9, weight_power=0.1, snapshot_dir=None, visualize=False, log_recorder=None):
    # Set the data path for vehicle models
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Problem settings
    tend = 3
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
    wheels = [wheel for axle in artCar.GetVehicle().GetAxles() for wheel in axle.GetWheels()]

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
    
    # Construct the terrain
    # print("Create terrain...")
    start_time = time.time()
    # terrain.Construct(chrono.ChVector3d(terrain_length, terrain_width, terrain_height),
    #                   chrono.ChVector3d(terrain_length / 2, 0, 0),
    #                   (fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS))
    # Read SPH particles from file
    sph_file = f"sph_5_1_0.1_{spacing}_xyz.csv"
    bce_file = f"boundary_5_1_0.1_{spacing}_xyz.csv"
    terrain.Construct(sph_file, bce_file, chrono.ChVector3d(0, 0, 0), False)
    end_time = time.time()
    print(f"Terrain construction time: {end_time - start_time} seconds")
    # Initialize the terrain system
    terrain.Initialize()
    
    aabb = terrain.GetSPHBoundingBox()
    # print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
    # print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
    # print(f"  SPH AABB:          {aabb.min}   {aabb.max}")
    
    # Set maximum vehicle X location
    x_max = aabb.max.x - 1.0
    
    # Driver inputs: ramp throttle to 0.7 with zero steering/brake
    throttle_ramp_duration = 3.0
    target_throttle = 0.7
    driver_inputs = veh.DriverInputs()

    chassis_body = artCar.GetVehicle().GetChassisBody()
    
    # Create anchor body and constraint that blocks forward motion while allowing vertical motion
    anchor_body = chrono.ChBody()
    anchor_body.SetFixed(True)
    sysMBS.AddBody(anchor_body)
    
    constraint = chrono.ChLinkMateGeneric()
    constraint.SetConstrainedCoords(True, False, False, True, True, True)

    # Attachment point on chassis: rear wheel x-position, CG y and z positions
    rear_wheel_x = -0.32264 - 0.05  # Rear wheel spindle x-position relative to chassis reference
    cg_y = -0.0014  # CG y-position (essentially 0)
    cg_z = -0.048 + 0.05   # CG z-position
    attach_local = chrono.ChVector3d(rear_wheel_x, cg_y, cg_z)
    attach_world = chassis_body.TransformPointLocalToParent(attach_local)
    anchor_body.SetPos(attach_world)
    constraint.Initialize(
        chassis_body,
        anchor_body,
        True,
        chrono.ChFramed(attach_local, chrono.QUNIT),
        chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
    )
    sysMBS.AddLink(constraint)
    
    # Add visualization
    # Red sphere on vehicle attachment point (same location as TSDA attachment)
    vis_sphere = chrono.ChVisualShapeSphere(0.05)
    vis_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
    chassis_body.AddVisualShape(vis_sphere, chrono.ChFramed(chrono.ChVector3d(rear_wheel_x, cg_y, cg_z)))
    
    # Anchor visualization
    vis_box = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
    vis_box.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    anchor_body.AddVisualShape(vis_box, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    
    # Set up output
    out_dir = chrono.GetChronoOutputPath() + "pull/"
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
        visVSG.SetWindowSize(1920, 1080)
        visVSG.SetWindowPosition(100, 100)
        visVSG.EnableSkyBox()
        visVSG.SetLightIntensity(1.0)
        visVSG.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
        visVSG.SetCameraAngleDeg(40)
        visVSG.SetChaseCamera(chrono.VNULL, 1.0, 0.0)
        visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, -2.0, 0.5))
        
        visVSG.Initialize()
        vis = visVSG
    
    # Simulation loop
    t_sim = 0
    sim_frame = 0
    sim_failed = False
    
    # print("Start simulation...")
    total_time_to_reach = tend*2
    net_power = 0
    power_count = 0
    pull_forces = []
    render_frame = 0
    start_time = time.time()
    while t_sim < tend:
        veh_loc = artCar.GetVehicle().GetPos()

        veh_z_vel = artCar.GetChassis().GetPointVelocity(chrono.ChVector3d(0, 0, 0)).z
        veh_roll_rate = artCar.GetChassis().GetRollRate()
        veh_pitch_rate = artCar.GetChassis().GetPitchRate()
        veh_yaw_rate = artCar.GetChassis().GetYawRate()
        veh_z_pos = veh_loc.z
        veh_x_vel = artCar.GetChassis().GetPointVelocity(chrono.ChVector3d(0, 0, 0)).x

        engine_speed = artCar.GetVehicle().GetEngine().GetMotorSpeed()
        engine_torque = artCar.GetVehicle().GetEngine().GetOutputMotorshaftTorque()
        inst_power = engine_torque * engine_speed
        
        if(t_sim > 0.5):
            net_power += inst_power
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

        # Ramp throttle; keep steering/braking zero
        throttle = min(target_throttle, target_throttle * (t_sim / throttle_ramp_duration))
        driver_inputs.m_throttle = throttle
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking = 0.0
        # Record reaction force along +X (vehicle pull on anchor is opposite of reaction on anchor)
        wrench_anchor = constraint.GetReaction2()
        pull_force_x = -wrench_anchor.force.x
        pull_forces.append(pull_force_x)
        if log_recorder:
            wheel_rpms = []
            for wheel in wheels:
                state = wheel.GetState()
                rpm = state.omega * 60.0 / (2 * chrono.CH_PI)
                wheel_rpms.append(rpm)
            log_recorder(t_sim, pull_force_x, veh_x_vel, wheel_rpms, inst_power)
        
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
        try:
            # Synchronize systems
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
    end_time = time.time()
    total_sim_time = end_time - start_time
    print(f"Simulation time: {total_sim_time} seconds")
    total_time_to_reach = t_sim
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
    
    # Compute metrics for slip test
    max_pull_force = max((abs(f) for f in pull_forces), default=0.0)
    average_power = net_power / power_count if power_count > 0 else 0.0

    print(f"Simulated time: {total_time_to_reach}")
    print(f"Max pull force at constraint (X): {max_pull_force}")
    print(f"Average power: {average_power}")
    metric = max_pull_force
    if sim_failed:
        print("Simulation failed")
        metric = 0

    print(f"Metric (max pull force): {metric}")
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
    return metric, total_time_to_reach, 0, average_power, sim_failed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pull test simulation runner")
    parser.add_argument("--rad", type=int, help="Inner radius in multiples of particle_spacing", required=True)
    parser.add_argument("--width", type=int, help="Width in multiples of particle_spacing", required=True)
    parser.add_argument("--g_height", type=int, help="Grouser height in multiples of particle_spacing", required=True)
    parser.add_argument("--g_density", type=int, required=True)
    parser.add_argument("--particle_spacing", type=float, required=True)
    parser.add_argument("--fan_theta_deg", type=float, default=60.0)
    # Optional material and driver settings (defaults preserved)
    parser.add_argument("--density", type=float, default=1700)
    parser.add_argument("--cohesion", type=float, default=0.0)
    parser.add_argument("--friction", type=float, default=0.8)
    parser.add_argument("--max_force", type=float, default=20.0)
    parser.add_argument("--target_speed", type=float, default=2.0)
    # Visualization / snapshots
    parser.add_argument("--visualize", action="store_true", help="Enable visualization")
    parser.add_argument("--snapshots", action="store_true", help="Enable snapshot saving (implies visualize)")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory for snapshots and logs")
    # Weights (kept for completeness)
    parser.add_argument("--weight_speed", type=float, default=0.9)
    parser.add_argument("--weight_power", type=float, default=0.1)
    args = parser.parse_args()

    p = Params()
    p.rad = int(args.rad)
    p.width = int(args.width)
    p.g_height = int(args.g_height)
    p.g_density = int(args.g_density)
    p.particle_spacing = float(args.particle_spacing)
    p.fan_theta_deg = float(args.fan_theta_deg)

    sp = SimParams()
    sp.density = float(args.density)
    sp.cohesion = float(args.cohesion)
    sp.friction = float(args.friction)
    sp.max_force = float(args.max_force)
    sp.target_speed = float(args.target_speed)

    visualize = args.visualize or args.snapshots
    snapshot_dir = args.output_dir if args.snapshots else None
    if snapshot_dir is not None:
        os.makedirs(snapshot_dir, exist_ok=True)

    metric, total_time_to_reach, _, average_power, sim_failed = sim(
        p, sp,
        weight_speed=float(args.weight_speed),
        weight_power=float(args.weight_power),
        snapshot_dir=snapshot_dir,
        visualize=visualize,
    )
    print(f"Max pull force metric: {metric:.4f}")
    print(f"Simulated time: {total_time_to_reach:.4f}")
    print(f"Average power: {average_power:.2f}W")
    print(f"Simulation failed: {sim_failed}")
