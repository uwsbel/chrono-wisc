#!/usr/bin/env python3
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
# Racetrack simulation for ARTcar on CRM terrain.
# - Terrain particles/BCE markers come from race-track-gen/out.
# - Path comes from race-track-gen/convex_racetrack.csv (no sine path).
#
# =============================================================================

import argparse
import csv
import math
import os
import time
import shutil

import pychrono.core as chrono
import pychrono.fsi as fsi
import pychrono.vehicle as veh

from cuda_error_checker import check_cuda_error, safe_advance, safe_initialize, safe_synchronize
from simple_wheel_gen import GenSimpleWheelPointCloud


# Optional pullback tether (implemented via a TSDA with a constant force functor).
# This matches the behavior in `SineTestSideSlip_sim.py`, but is disabled by default.
SAVE_OUTPUT_HZ = 10

USE_PULLBACK_SPRING = True
PULLBACK_FORCE_N = 25.0
USE_STEERING_FILTER = True
STEERING_FILTER_ALPHA = 0.15


class Params:
    rad = 50  # Radius is rad * particle_spacing (inner/body radius)
    width = 40  # Width is width * particle_spacing
    g_height = 10  # Grouser height is g_height * particle_spacing
    g_density = 8  # Number of grousers per revolution
    particle_spacing = 0.005  # Particle spacing (m)
    fan_theta_deg = 60.0  # Only for Straight
    steering_kp = 5.0
    steering_ki = 0.0
    steering_kd = 0.5
    speed_kp = 0.5
    speed_kd = 0.1


class SimParams:
    density = 1700.0
    cohesion = 0.0
    friction = 0.8
    target_speed = 2.0


vehicle_init_height = 0.3
vehicle_init_time = 0.5
step_size = 2e-4
control_period = 1e-2


def _here(*parts):
    return os.path.join(os.path.dirname(__file__), *parts)


DEFAULT_PARTICLES_FILE = _here("race-track-gen", "out", "particles_5mm.txt")
DEFAULT_BCE_FILE = _here("race-track-gen", "out", "bce_5mm.txt")
DEFAULT_WAYPOINTS_FILE = _here("race-track-gen", "convex_racetrack.csv")


def _reset_output_dir(path):
    if path is None:
        return
    if os.path.isdir(path):
        shutil.rmtree(path)
    elif os.path.exists(path):
        os.remove(path)
    os.makedirs(path, exist_ok=True)


def numpy_to_ChVector3d(numpy_array):
    return [chrono.ChVector3d(row[0], row[1], row[2]) for row in numpy_array]


class PullForceFunctor(chrono.ForceFunctor):
    def __init__(self, force_func):
        super().__init__()
        self.m_force_func = force_func

    def evaluate(self, time, rest_length, length, vel, link):
        return self.m_force_func.GetVal(float(time))



def CreateFSIWheels(vehicle, terrain, params):
    fsi_bodies = []
    for axle in vehicle.GetVehicle().GetAxles():
        for wheel in axle.GetWheels():
            sysFSI = terrain.GetFsiSystemSPH()

            grouser_type = "straight"
            radius = params.rad * params.particle_spacing
            width = params.width * params.particle_spacing
            g_height = params.g_height * params.particle_spacing
            g_width = 2 * params.particle_spacing
            g_density = params.g_density
            fan_theta_deg = params.fan_theta_deg

            bce_wheel = GenSimpleWheelPointCloud(
                rad=radius,
                width=width,
                cp_deviation=0.0,
                g_height=g_height,
                g_width=g_width,
                g_density=g_density,
                particle_spacing=params.particle_spacing,
                grouser_type=grouser_type,
                fan_theta_deg=fan_theta_deg,
            )
            bce_wheel = numpy_to_ChVector3d(bce_wheel)
            fsi_body = sysFSI.AddFsiBody(
                wheel.GetSpindle(),
                bce_wheel,
                chrono.ChFramed(chrono.VNULL, chrono.Q_ROTATE_Z_TO_Y),
                False,
            )
            fsi_bodies.append(fsi_body)
    return fsi_bodies


def _load_waypoints_xy(csv_path):
    pts = []
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
            except Exception:
                continue
            pts.append((x, y))
    if len(pts) < 2:
        raise ValueError(f"Need at least 2 waypoints in {csv_path}")
    return pts


def _make_bezier_from_waypoints(waypoints_xyz, control_offset=0.1, closed=False):
    n = len(waypoints_xyz)
    if n < 2:
        raise ValueError("Need at least 2 waypoints")

    in_cvs = []
    out_cvs = []

    for i in range(n):
        if closed:
            prev_i = (i - 1) % n
            next_i = (i + 1) % n
            dirx = (waypoints_xyz[next_i].x - waypoints_xyz[prev_i].x) * 0.5
            diry = (waypoints_xyz[next_i].y - waypoints_xyz[prev_i].y) * 0.5
        else:
            if i == 0:
                dirx = waypoints_xyz[i + 1].x - waypoints_xyz[i].x
                diry = waypoints_xyz[i + 1].y - waypoints_xyz[i].y
            elif i == n - 1:
                dirx = waypoints_xyz[i].x - waypoints_xyz[i - 1].x
                diry = waypoints_xyz[i].y - waypoints_xyz[i - 1].y
            else:
                dirx = (waypoints_xyz[i + 1].x - waypoints_xyz[i - 1].x) * 0.5
                diry = (waypoints_xyz[i + 1].y - waypoints_xyz[i - 1].y) * 0.5

        norm = math.sqrt(dirx * dirx + diry * diry)
        if norm > 1e-9:
            dirx /= norm
            diry /= norm
        in_cvs.append(
            chrono.ChVector3d(
                waypoints_xyz[i].x - dirx * control_offset,
                waypoints_xyz[i].y - diry * control_offset,
                waypoints_xyz[i].z,
            )
        )
        out_cvs.append(
            chrono.ChVector3d(
                waypoints_xyz[i].x + dirx * control_offset,
                waypoints_xyz[i].y + diry * control_offset,
                waypoints_xyz[i].z,
            )
        )

    return chrono.ChBezierCurve(waypoints_xyz, in_cvs, out_cvs)


def _estimate_polyline_length_xy(pts_xy):
    length = 0.0
    for i in range(1, len(pts_xy)):
        dx = pts_xy[i][0] - pts_xy[i - 1][0]
        dy = pts_xy[i][1] - pts_xy[i - 1][1]
        length += math.sqrt(dx * dx + dy * dy)
    return length


def _nearest_point_distance_xy(point_xy, seg_start_xy, seg_end_xy):
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
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    closest_x = seg_start_xy[0] + t * vx
    closest_y = seg_start_xy[1] + t * vy
    dx = point_xy[0] - closest_x
    dy = point_xy[1] - closest_y
    return math.sqrt(dx * dx + dy * dy)


def _compute_cross_track_error_xy(vehicle_xy, path_points_xy):
    best = None
    for i in range(1, len(path_points_xy)):
        d = _nearest_point_distance_xy(vehicle_xy, path_points_xy[i - 1], path_points_xy[i])
        if best is None or d < best:
            best = d
    return best if best is not None else 0.0


def _clean_marker_csv(src_path, dst_path):
    os.makedirs(os.path.dirname(dst_path), exist_ok=True)
    with open(src_path, "r", newline="") as src, open(dst_path, "w", newline="") as dst:
        dst.write("x,y,z\n")
        for line in src:
            s = line.strip()
            if not s:
                continue
            if s.lower().startswith("x"):
                continue
            parts = [p.strip() for p in s.split(",") if p.strip() != ""]
            if len(parts) < 3:
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            except Exception:
                continue
            dst.write(f"{x},{y},{z}\n")


def sim(
    params,
    sim_params,
    particles_file=DEFAULT_PARTICLES_FILE,
    bce_file=DEFAULT_BCE_FILE,
    waypoints_file=DEFAULT_WAYPOINTS_FILE,
    weight_time=1.0,
    weight_power=0.0,
    snapshot_dir=None,
    particle_sph_dir=None,
    particle_fsi_dir=None,
    blender_dir=None,
    visualize=False,
):
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    pts_xy = _load_waypoints_xy(waypoints_file)
    start_xy = pts_xy[0]
    next_xy = pts_xy[1]
    yaw = math.atan2(next_xy[1] - start_xy[1], next_xy[0] - start_xy[0])
    init_pos = chrono.ChVector3d(start_xy[0], start_xy[1], vehicle_init_height)
    init_rot = chrono.QuatFromAngleZ(yaw)

    path_len = _estimate_polyline_length_xy(pts_xy)
    tend = max(10.0, 2.5 * path_len / max(0.1, sim_params.target_speed))

    render_fps = 200
    visualization_sph = True
    visualization_bndry_bce = False
    visualization_rigid_bce = True

    density = sim_params.density
    cohesion = sim_params.cohesion
    friction = sim_params.friction
    youngs_modulus = 1e6
    poisson_ratio = 0.3

    active_box_dim = 0.6
    settling_time = 0.0
    spacing = params.particle_spacing
    integration_scheme = fsi.IntegrationScheme_RK2

    artCar = veh.ARTcar()
    artCar.SetContactMethod(chrono.ChContactMethod_SMC)
    artCar.SetChassisFixed(False)
    artCar.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
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

    sysMBS.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sysMBS.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    sysMBS.SetNumThreads(8, 1, 7)

    terrain = veh.CRMTerrain(sysMBS, spacing)
    terrain.SetVerbose(False)
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    terrain.SetStepSizeCFD(step_size)
    terrain.GetFluidSystemSPH().EnableCudaErrorCheck(True)

    terrain.RegisterVehicle(artCar.GetVehicle())

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
    terrain.SetOutputLevel(fsi.OutputLevel_STATE)

    CreateFSIWheels(artCar, terrain, params)
    terrain.SetActiveDomain(chrono.ChVector3d(active_box_dim))
    terrain.SetActiveDomainDelay(settling_time)
    print(f"Constructing terrain with particles file: {particles_file} and bce file: {bce_file}")
    start_time = time.time()
    # try:
    terrain.Construct(particles_file, bce_file, chrono.ChVector3d(0, 0, 0), False)
    # except BaseException:
    #     cleaned_particles = _here("race-track-gen", "out", f"_clean_particles_{int(spacing*1000)}mm.csv")
    #     cleaned_bce = _here("race-track-gen", "out", f"_clean_bce_{int(spacing*1000)}mm.csv")
    #     _clean_marker_csv(particles_file, cleaned_particles)
    #     _clean_marker_csv(bce_file, cleaned_bce)
    #     terrain.Construct(cleaned_particles, cleaned_bce, chrono.ChVector3d(0, 0, 0), False)
    end_time = time.time()  
    print(f"Terrain construction time: {end_time - start_time} seconds")

    ok, msg = safe_initialize(terrain)
    if not ok:
        return float("inf"), tend * 2, 0.0, 0.0, True

    aabb = terrain.GetSPHBoundingBox()

    waypoints_xyz = [chrono.ChVector3d(x, y, vehicle_init_height) for (x, y) in pts_xy]
    bezier_path = _make_bezier_from_waypoints(waypoints_xyz, control_offset=0.25, closed=False)

    driver = veh.ChPathFollowerDriver(
        artCar.GetVehicle(), bezier_path, "racetrack_path", sim_params.target_speed, vehicle_init_time, 2.0
    )
    driver.GetSteeringController().SetLookAheadDistance(0.5)
    driver.GetSteeringController().SetGains(params.steering_kp, params.steering_ki, params.steering_kd)
    driver.GetSpeedController().SetGains(params.speed_kp, 0.2, params.speed_kd)
    driver.Initialize()

    if USE_PULLBACK_SPRING:
        max_force = float(PULLBACK_FORCE_N)
        if max_force < 0:
            raise ValueError(f"PULLBACK_FORCE_N must be non-negative, got {PULLBACK_FORCE_N}")

        tsda = chrono.ChLinkTSDA()
        zero_force_duration = vehicle_init_time + 0.5
        ramp_duration = 0.1
        max_force_duration = max(0.0, tend - zero_force_duration - ramp_duration)

        seq = chrono.ChFunctionSequence()
        seq.InsertFunct(chrono.ChFunctionConst(0), zero_force_duration)
        seq.InsertFunct(chrono.ChFunctionRamp(0, -max_force / max(1e-9, ramp_duration)), ramp_duration)
        if max_force_duration > 0.0:
            seq.InsertFunct(chrono.ChFunctionConst(-max_force), max_force_duration)
        seq.Setup()

        chassis_body = artCar.GetVehicle().GetChassisBody()

        spring_box = chrono.ChBody()
        spring_box.SetFixed(True)
        terrain_height = 0.10
        anchor_xy = pts_xy[0]
        if len(pts_xy) >= 3:
            d1x = pts_xy[1][0] - pts_xy[0][0]
            d1y = pts_xy[1][1] - pts_xy[0][1]
            d2x = pts_xy[2][0] - pts_xy[1][0]
            d2y = pts_xy[2][1] - pts_xy[1][1]
            dirx = d1x + d2x
            diry = d1y + d2y
        else:
            dirx = pts_xy[1][0] - pts_xy[0][0]
            diry = pts_xy[1][1] - pts_xy[0][1]
        dir_len = math.sqrt(dirx * dirx + diry * diry)
        if dir_len > 1e-9:
            dirx /= dir_len
            diry /= dir_len
        anchor_x = pts_xy[0][0] - dirx * 0.5
        anchor_y = pts_xy[0][1] - diry * 0.5
        anchor_xy = (anchor_x, anchor_y)
        spring_box.SetPos(chrono.ChVector3d(anchor_xy[0], anchor_xy[1], terrain_height / 2 + 0.1))
        sysMBS.AddBody(spring_box)

        rear_wheel_x = -0.32264 - 0.05
        cg_y = -0.0014
        cg_z = -0.048 + 0.1
        tsda.Initialize(
            spring_box,
            chassis_body,
            True,
            chrono.ChVector3d(0, 0, 0),
            chrono.ChVector3d(rear_wheel_x, cg_y, cg_z),
        )
        tsda.SetSpringCoefficient(0)
        tsda.SetDampingCoefficient(0)
        pullback_force_functor = PullForceFunctor(seq)
        tsda.RegisterForceFunctor(pullback_force_functor)
        sysMBS.AddLink(tsda)

        if visualize:
            vis_sphere = chrono.ChVisualShapeSphere(0.05)
            vis_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
            chassis_body.AddVisualShape(vis_sphere, chrono.ChFramed(chrono.ChVector3d(rear_wheel_x, cg_y, cg_z)))

            vis_box = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
            vis_box.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
            spring_box.AddVisualShape(vis_box, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

            tsda.AddVisualShape(chrono.ChVisualShapeSpring(0.1, 80, 1))

    vis = None
    if visualize:
        try:
            import pychrono.vsg3d as vsg  # noqa: F401
        except Exception:
            vsg = None
        col_callback = fsi.ParticleHeightColorCallback(aabb.min.z, aabb.max.z)
        visFSI = fsi.ChSphVisualizationVSG(terrain.GetFsiSystemSPH())
        visFSI.EnableFluidMarkers(visualization_sph)
        visFSI.EnableBoundaryMarkers(visualization_bndry_bce)
        visFSI.EnableRigidBodyMarkers(visualization_rigid_bce)
        visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_BROWN)

        visVSG = veh.ChWheeledVehicleVisualSystemVSG()
        visVSG.AttachVehicle(artCar.GetVehicle())
        visVSG.AttachPlugin(visFSI)
        visVSG.SetWindowTitle("ARTcar racetrack on CRM deformable terrain")
        visVSG.SetWindowSize(1280, 800)
        visVSG.SetWindowPosition(100, 100)
        visVSG.EnableSkyBox()
        visVSG.SetLightIntensity(1.0)
        visVSG.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
        visVSG.SetCameraAngleDeg(40)
        visVSG.SetChaseCamera(chrono.VNULL, 1.0, 0.0)
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

    t_sim = 0.0
    sim_failed = False
    total_time_to_reach = tend * 2
    net_power = 0.0
    power_count = 0
    render_frame = 0
    output_frame = 0

    sum_squared_error = 0.0
    error_samples = 0
    control_delay = vehicle_init_time

    last_control_time = -control_period
    driver_inputs = veh.DriverInputs()
    last_steering = 0.0

    goal_xy = pts_xy[-1]
    goal_tol = 0.5

    start_wall = time.time()
    while t_sim < tend:
        veh_loc = artCar.GetVehicle().GetPos()

        engine_speed = artCar.GetVehicle().GetEngine().GetMotorSpeed()
        engine_torque = artCar.GetVehicle().GetEngine().GetOutputMotorshaftTorque()
        if t_sim > 0.5:
            net_power += engine_torque * engine_speed
            power_count += 1

        if t_sim >= last_control_time + control_period:
            if t_sim < vehicle_init_time:
                driver_inputs = veh.DriverInputs()
                driver_inputs.m_throttle = 0.0
                driver_inputs.m_steering = 0.0
                driver_inputs.m_braking = 0.0
                last_steering = 0.0
            else:
                driver_inputs = driver.GetInputs()
                if USE_STEERING_FILTER:
                    alpha = float(STEERING_FILTER_ALPHA)
                    alpha = 0.0 if alpha < 0.0 else 1.0 if alpha > 1.0 else alpha
                    filtered = alpha * driver_inputs.m_steering + (1.0 - alpha) * last_steering
                    driver_inputs.m_steering = filtered
                    last_steering = filtered
                else:
                    last_steering = driver_inputs.m_steering
                driver_inputs.m_braking = 0.0
            last_control_time = t_sim

        if t_sim >= control_delay:
            e = _compute_cross_track_error_xy((veh_loc.x, veh_loc.y), pts_xy)
            sum_squared_error += e * e
            error_samples += 1

        dxg = veh_loc.x - goal_xy[0]
        dyg = veh_loc.y - goal_xy[1]
        if t_sim > vehicle_init_time and (dxg * dxg + dyg * dyg) <= goal_tol * goal_tol:
            total_time_to_reach = t_sim
            break

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
            driver.Synchronize(t_sim)
            if vis is not None:
                vis.Synchronize(t_sim, driver_inputs)
            terrain.Synchronize(t_sim)

            ok, msg = safe_synchronize(artCar.GetVehicle(), t_sim, driver_inputs, terrain)
            if not ok:
                sim_failed = True
                break

            driver.Advance(step_size)
            if vis is not None:
                vis.Advance(step_size)

            ok, msg = safe_advance(terrain, step_size)
            if not ok:
                sim_failed = True
                break
        except BaseException:
            sim_failed = True
            break

        t_sim += step_size

    wall_time = time.time() - start_wall
    print(f"Simulation wall time: {wall_time} seconds")

    err, err_msg = check_cuda_error()
    if err:
        print(f"CUDA error detected: {err_msg}")
        sim_failed = True

    rms_error = math.sqrt(sum_squared_error / error_samples) if error_samples > 0 else 0.0
    average_power = (net_power / power_count) if power_count > 0 else 0.0
    metric = weight_time * total_time_to_reach + weight_power * average_power

    return metric, total_time_to_reach, rms_error, average_power, sim_failed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Racetrack simulation runner")
    parser.add_argument("--rad", type=int, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--g_height", type=int, required=True)
    parser.add_argument("--g_density", type=int, required=True)
    parser.add_argument("--particle_spacing", type=float, default=0.005)
    parser.add_argument("--fan_theta_deg", type=float, default=60.0)
    parser.add_argument("--steering_kp", type=float, default=None)
    parser.add_argument("--steering_ki", type=float, default=None)
    parser.add_argument("--steering_kd", type=float, default=None)
    parser.add_argument("--speed_kp", type=float, default=None)
    parser.add_argument("--speed_kd", type=float, default=None)

    parser.add_argument("--density", type=float, default=1700.0)
    parser.add_argument("--cohesion", type=float, default=0.0)
    parser.add_argument("--friction", type=float, default=0.8)
    parser.add_argument("--target_speed", type=float, default=2.0)

    parser.add_argument("--particles-file", type=str, default=DEFAULT_PARTICLES_FILE)
    parser.add_argument("--bce-file", type=str, default=DEFAULT_BCE_FILE)
    parser.add_argument("--waypoints-file", type=str, default=DEFAULT_WAYPOINTS_FILE)

    parser.add_argument("--weight_time", type=float, default=1.0)
    parser.add_argument("--weight_power", type=float, default=0.0)

    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--snapshots", action="store_true", help="Enable snapshot saving (implies visualize)")
    parser.add_argument("--save-blender", action="store_true", help="Enable Blender output")
    parser.add_argument("--save-particles", action="store_true", help="Enable particle output")
    parser.add_argument("--output-dir", type=str, default=None)

    args = parser.parse_args()

    p = Params()
    p.rad = int(args.rad)
    p.width = int(args.width)
    p.g_height = int(args.g_height)
    p.g_density = int(args.g_density)
    p.particle_spacing = float(args.particle_spacing)
    p.fan_theta_deg = float(args.fan_theta_deg)
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
    sp.target_speed = float(args.target_speed)

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

    metric, total_time_to_reach, rms_error, average_power, sim_failed = sim(
        p,
        sp,
        particles_file=args.particles_file,
        bce_file=args.bce_file,
        waypoints_file=args.waypoints_file,
        weight_time=float(args.weight_time),
        weight_power=float(args.weight_power),
        snapshot_dir=snapshot_dir,
        particle_sph_dir=particle_sph_dir,
        particle_fsi_dir=particle_fsi_dir,
        blender_dir=blender_dir,
        visualize=visualize,
    )

    print(f"Metric: {metric:.4f}")
    print(f"Total time to reach goal: {total_time_to_reach:.4f} s")
    print(f"RMS cross-track error: {rms_error:.4f} m")
    print(f"Average power: {average_power:.2f} W")
    print(f"Simulation failed: {sim_failed}")
