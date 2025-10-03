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
# ART Vehicle on CRM terrain pulling a load
#
# =============================================================================

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
import pychrono.vsg3d as vsg
import os
import csv

# Terrain dimensions
terrain_length = 2.5
terrain_width = 1
terrain_height = 0.10

# Wheel type enumeration
class WheelType:
    SIMPLE = 0
    MESH = 1
    BCE_MARKERS = 2

wheel_BCE_csvfile = "vehicle/artcar/wheel_straight.csv"
wheel_type = WheelType.BCE_MARKERS

def CreateFSIWheels(vehicle, terrain, wheel_type):
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
            
            if wheel_type == WheelType.SIMPLE:
                # Create cylindrical wheel representation
                wheel_radius = wheel.GetTire().GetRadius()
                wheel_width = wheel.GetTire().GetWidth()
                
                print(f"Wheel radius: {wheel_radius} Wheel width: {wheel_width}")
                
                wheel_mass = wheel.GetTire().GetTireMass()
                cylinder = chrono.ChCylinder(wheel_radius, wheel_width)
                mass = wheel_mass * cylinder.GetVolume()
                inertia = wheel_mass * cylinder.GetGyration()
                
                geometry_simple = chrono.ChBodyGeometry()
                geometry_simple.materials.append(chrono.ChContactMaterialData())
                geometry_simple.coll_cylinders.append(
                    chrono.ChBodyGeometry.CylinderShape(chrono.VNULL, chrono.Q_ROTATE_Z_TO_Y, cylinder, 0))
                terrain.AddRigidBody(wheel.GetSpindle(), geometry_simple, False, False)
                
            elif wheel_type == WheelType.MESH:
                # Use mesh representation
                mesh_filename = veh.GetDataFile("artcar/tire.obj")
                geometry = chrono.ChBodyGeometry()
                geometry.materials.append(chrono.ChContactMaterialData())
                geometry.coll_meshes.append(
                    chrono.ChBodyGeometry.TrimeshShape(chrono.VNULL, chrono.QUNIT, mesh_filename, chrono.VNULL))
                terrain.AddRigidBody(wheel.GetSpindle(), geometry, False, False)
                
            else:  # BCE_MARKERS
                print(f"Adding wheel BCE markers from csv file: {wheel_BCE_csvfile}")
                BCE_wheel = []
                
                # Read BCE markers from CSV file
                with open(chrono.GetChronoDataFile(wheel_BCE_csvfile), 'r') as file:
                    lines = file.readlines()
                    for line in lines[1:]:  # Skip header
                        values = [float(val.strip()) for val in line.split(',')]
                        BCE_wheel.append(chrono.ChVector3d(values[0], values[1], values[2]))
                
                sysFSI = terrain.GetFsiSystemSPH()
                fsi_body = sysFSI.AddFsiBody(wheel.GetSpindle(), BCE_wheel, 
                                           chrono.ChFramed(chrono.VNULL, chrono.Q_ROTATE_Z_TO_Y), False)
                fsi_bodies.append(fsi_body)
                print("Added wheel BCE markers to FSI system")
    
    return fsi_bodies

class PullForceFunctor(chrono.ForceFunctor):
    """
    Custom force functor for TSDA to apply pulling force.
    """
    def __init__(self, force_func):
        super().__init__()
        self.m_force_func = force_func
    
    def evaluate(self, time, rest_length, length, vel, link):
        return self.m_force_func.GetVal(time)

def main():
    # Set the data path for vehicle models
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Problem settings
    tend = 3
    verbose = True
    
    # Visualization settings
    render = True
    render_fps = 200
    visualization_sph = True
    visualization_bndry_bce = False
    visualization_rigid_bce = False
    
    # CRM material properties
    density = 1700
    cohesion = 0
    friction = 0.8
    youngs_modulus = 1e6
    poisson_ratio = 0.3
    
    # CRM active box dimension
    active_box_dim = 0.5
    settling_time = 0
    
    # SPH spacing
    spacing = 0.005
    
    # SPH integration scheme
    integration_scheme = fsi.IntegrationScheme_RK2
    
    # Create vehicle
    print("Create vehicle...")
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
    sysFSI = terrain.GetFsiSystemSPH()
    terrain.SetVerbose(verbose)
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    terrain.SetStepSizeCFD(1e-4)
    
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
    sph_params.use_consistent_gradient_discretization = False
    sph_params.use_consistent_laplacian_discretization = False
    sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_BILATERAL
    sph_params.boundary_method = fsi.BoundaryMethod_ADAMI
    sph_params.kernel_type = fsi.KernelType_WENDLAND
    terrain.SetSPHParameters(sph_params)
    
    # Set output level from SPH simulation
    terrain.SetOutputLevel(fsi.OutputLevel_STATE)
    
    # Add vehicle wheels as FSI solids
    fsi_bodies = CreateFSIWheels(artCar, terrain, wheel_type)
    terrain.SetActiveDomain(chrono.ChVector3d(active_box_dim))
    terrain.SetActiveDomainDelay(settling_time)
    
    # Construct the terrain
    print("Create terrain...")
    terrain.Construct(chrono.ChVector3d(terrain_length, terrain_width, terrain_height),
                      chrono.ChVector3d(terrain_length / 2, 0, 0),
                      (fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS))
    
    # Initialize the terrain system
    terrain.Initialize()
    
    aabb = terrain.GetSPHBoundingBox()
    print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
    print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
    print(f"  SPH AABB:          {aabb.min}   {aabb.max}")
    
    # Set maximum vehicle X location
    x_max = aabb.max.x - 0.5
    
    # Create the path-following driver
    print("Create path...")
    path = veh.StraightLinePath(chrono.ChVector3d(0, 0, vehicle_init_height),
                               chrono.ChVector3d(terrain_length, 0, vehicle_init_height), 1)
    
    target_speed = 1.0
    driver = veh.ChPathFollowerDriver(artCar.GetVehicle(), path, "straight_path", target_speed)
    driver.GetSteeringController().SetLookAheadDistance(1.0)
    driver.GetSteeringController().SetGains(1.0, 0, 0)
    driver.GetSpeedController().SetGains(0.6, 0.05, 0)
    driver.Initialize()
    
    # Set up TSDA to apply pulling force
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
    vis = None
    if render:
        # FSI plugin
        col_callback = fsi.ParticleHeightColorCallback(aabb.min.z, aabb.max.z)
        visFSI = fsi.ChSphVisualizationVSG(sysFSI)
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
        visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, -1, 0.5))
        
        visVSG.Initialize()
        vis = visVSG
    
    # Simulation loop
    time = 0
    sim_frame = 0
    render_frame = 0
    braking = False
    
    print("Start simulation...")
    
    with open(out_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=' ')
        
        while time < tend:
            veh_loc = artCar.GetVehicle().GetPos()
            
            # Get driver inputs from path follower
            driver_inputs = driver.GetInputs()
            
            # Override throttle control with custom logic
            if time < zero_force_duration - 0.1:
                driver_inputs.m_throttle = 0
                driver_inputs.m_braking = 1
            elif time < zero_force_duration and time > zero_force_duration - 0.1:
                driver_inputs.m_throttle = -(time - zero_force_duration) / 0.1
                driver_inputs.m_braking = 0
            else:
                driver_inputs.m_throttle = 1.0
                driver_inputs.m_braking = 0
            
            # Stop vehicle before reaching end of terrain patch
            if veh_loc.x > x_max:
                break
            
            # Run-time visualization
            if render and time >= render_frame / render_fps:
                if not vis.Run():
                    break
                vis.Render()
                render_frame += 1
            
            if not render:
                print(f"{time}  {terrain.GetRtfCFD()}  {terrain.GetRtfMBD()}")
            
            # Synchronize systems
            driver.Synchronize(time)
            if vis:
                vis.Synchronize(time, driver_inputs)
            terrain.Synchronize(time)
            artCar.GetVehicle().Synchronize(time, driver_inputs, terrain)
            
            # Advance system state
            driver.Advance(step_size)
            if vis:
                vis.Advance(step_size)
            terrain.Advance(step_size)
            
            # Output results
            csvwriter.writerow([time, veh_loc.x, veh_loc.y, veh_loc.z, artCar.GetVehicle().GetSpeed()])
            
            # Log per-tire forces, torque, and RPM
            tire_idx = 0
            for ia in range(num_axles):
                for is_side in range(2):
                    side = veh.LEFT if is_side == 0 else veh.RIGHT
                    tire = artCar.GetVehicle().GetTire(ia, side)
                    if tire and tire_idx < len(fsi_bodies):
                        force = sysFSI.GetFsiBodyForce(tire_idx)
                        torque = sysFSI.GetFsiBodyTorque(tire_idx)
                        omega = artCar.GetVehicle().GetSpindleOmega(ia, side)
                        rpm = omega * 60.0 / (2.0 * chrono.CH_PI)
                        
                        tire_writer = csv.writer(tire_writers[ia][is_side], delimiter=',')
                        tire_writer.writerow([time, force.x, force.y, force.z, torque.z, rpm])
                        tire_counts[ia][is_side] += 1
                        tire_idx += 1
            
            time += step_size
            sim_frame += 1
    
    # Close tire CSV files
    for ia in range(num_axles):
        for is_side in range(2):
            if tire_writers[ia][is_side]:
                tire_writers[ia][is_side].close()
    
    print("Simulation complete. Results written to:", out_file)

if __name__ == "__main__":
    main()
