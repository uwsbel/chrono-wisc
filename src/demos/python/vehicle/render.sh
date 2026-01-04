#!/usr/bin/env bash
set -euo pipefail

# /home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender -b -P /home/huzaifa/chrono-wisc/src/demos/python/vehicle/chrono_blender_render.py -- \
#   --assets /home/huzaifa/chrono-wisc/src/demos/python/vehicle/Pull_global_0.005_0_0.8_25_2/rank_01_trial_1777/blender/exported.assets.py \
#   --output /home/huzaifa/chrono-wisc/src/demos/python/vehicle/Pull_global_0.005_0_0.8_25_2/rank_01_trial_1777/blender/output \
#   --terrain_path /home/huzaifa/chrono-wisc/src/demos/python/vehicle/Pull_global_0.005_0_0.8_25_2/rank_01_trial_1777/particle_files/particles \
#   --terrain_type PARTICLES \
#   --terrain_prefix fluid \
#   --wheel_bce_radius 0.0025 \
#   --frame_start 0 \
#   --frame_end 25 \
#   --camera TOP \
#   --camera_padding 1.0 \
#   --camera_lens 55 \
#   --scene_yaw_deg 0 \

# Sine track
# Seperate case
# /home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender -b -P /home/huzaifa/chrono-wisc/src/demos/python/vehicle/chrono_blender_render.py -- \
#   --assets /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/best_runs_snapshots_20260101T172832Z/optimized_controller/blender/exported.assets.py \
#   --output /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/best_runs_snapshots_20260101T172832Z/optimized_controller/blender/output \
#   --terrain_path /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/best_runs_snapshots_20260101T172832Z/optimized_controller/particle_files/particles \
#   --terrain_type PARTICLES \
#   --terrain_prefix fluid \
#   --wheel_bce_radius 0.0025 \
#   --frame_start 0 \
#   --frame_end 100 \
#   --camera TOP \
#   --camera_padding 1.0 \
#   --camera_lens 55 \
#   --scene_yaw_deg 0 \

# #Default case
# /home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender -b -P /home/huzaifa/chrono-wisc/src/demos/python/vehicle/chrono_blender_render.py -- \
#   --assets /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/best_runs_snapshots_20260101T174107Z/default_controller/blender/exported.assets.py \
#   --output /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/best_runs_snapshots_20260101T174107Z/default_controller/blender/output \
#   --terrain_path /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/best_runs_snapshots_20260101T174107Z/default_controller/particle_files/particles \
#   --terrain_type PARTICLES \
#   --terrain_prefix fluid \
#   --wheel_bce_radius 0.0025 \
#   --frame_start 0 \
#   --frame_end 100 \
#   --camera TOP \
#   --camera_padding 1.0 \
#   --camera_lens 55 \
#   --scene_yaw_deg 0 \


# #Joint case
# /home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender -b -P /home/huzaifa/chrono-wisc/src/demos/python/vehicle/chrono_blender_render.py -- \
#   --assets /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_wControl_Dec22/best_runs_snapshots_20260101T174848Z/joint/blender/exported.assets.py \
#   --output /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_wControl_Dec22/best_runs_snapshots_20260101T174848Z/joint/blender/output \
#   --terrain_path /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_wControl_Dec22/best_runs_snapshots_20260101T174848Z/joint/particle_files/particles \
#   --terrain_type PARTICLES \
#   --terrain_prefix fluid \
#   --wheel_bce_radius 0.0025 \
#   --frame_start 0 \
#   --frame_end 100 \
#   --camera TOP \
#   --camera_padding 1.0 \
#   --camera_lens 55 \
#   --scene_yaw_deg 0 \


#Race track
#seperate case
/home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender -b -P /home/huzaifa/chrono-wisc/src/demos/python/vehicle/chrono_blender_render.py -- \
  --assets /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/racetrack_best_runs_20260101T181203Z/optimized_controller/blender/exported.assets.py \
  --output /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/racetrack_best_runs_20260101T181203Z/optimized_controller/blender/output \
  --terrain_path /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/racetrack_best_runs_20260101T181203Z/optimized_controller/particle_files/particles \
  --terrain_type PARTICLES \
  --terrain_prefix fluid \
  --wheel_bce_radius 0.0025 \
  --frame_start 0 \
  --frame_end 50 \
  --camera TOP \
  --camera_padding 1.1 \
  --camera_lens 55 \
  --scene_yaw_deg 0 \

#Default case
/home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender -b -P /home/huzaifa/chrono-wisc/src/demos/python/vehicle/chrono_blender_render.py -- \
  --assets /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/racetrack_best_runs_20260101T175203Z/default_controller/blender/exported.assets.py \
  --output /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/racetrack_best_runs_20260101T175203Z/default_controller/blender/output \
  --terrain_path /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_global_1700_0_0.6_25_3_5_24Dec/racetrack_best_runs_20260101T175203Z/default_controller/particle_files/particles \
  --terrain_type PARTICLES \
  --terrain_prefix fluid \
  --wheel_bce_radius 0.0025 \
  --frame_start 0 \
  --frame_end 50 \
  --camera TOP \
  --camera_padding 1.1 \
  --camera_lens 55 \
  --scene_yaw_deg 0 \


#Joint case
/home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender -b -P /home/huzaifa/chrono-wisc/src/demos/python/vehicle/chrono_blender_render.py -- \
  --assets /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_wControl_Dec22/racetrack_best_runs_20260102T142642Z/joint/blender/exported.assets.py \
  --output /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_wControl_Dec22/racetrack_best_runs_20260102T142642Z/joint/blender/output \
  --terrain_path /home/huzaifa/chrono-wisc/src/demos/python/vehicle/SineSideSlip_wControl_Dec22/racetrack_best_runs_20260102T142642Z/joint/particle_files/particles \
  --terrain_type PARTICLES \
  --terrain_prefix fluid \
  --wheel_bce_radius 0.0025 \
  --frame_start 0 \
  --frame_end 50 \
  --camera TOP \
  --camera_padding 1.1 \
  --camera_lens 55 \
  --scene_yaw_deg 0 \