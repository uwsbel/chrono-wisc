import bpy
import sys
import os
import math
import mathutils
import argparse
import time
import glob
import gc

# Camera and rendering configuration constants
# Adjust these values to control camera positioning and behavior
CAMERA_DEFAULT_DISTANCE = 10.0         # Default distance when no objects are found
CAMERA_SCENE_DISTANCE_FACTOR = 0.5     # Multiplier for scene size to camera distance

# Revolving camera specific settings
REVOLVING_DISTANCE_FACTOR_X = 1.2      # Horizontal distance multiplier (higher = further away)
REVOLVING_DISTANCE_FACTOR_Y = 1.2      # Horizontal distance multiplier (higher = further away)
REVOLVING_HEIGHT_FACTOR = 0.5          # Height multiplier (higher = higher camera)
REVOLVING_ROTATION_SPEED = 0.5         # Rotation speed (lower = slower rotation, 1.0 = one full rotation)

# Camera stability settings
CAMERA_STABLE_REVOLVING = True         # Use stable camera path for revolving camera
CAMERA_STABLE_ISOMETRIC = True         # Use stable camera for isometric view

# First, ensure we're running in background mode
# This script should be called with: blender -b -P chrono_blender_render.py -- [args]

# Import the necessary functions from chrono_import.py
# We need to add the directory containing chrono_import.py to sys.path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'importer_blender'))
import chrono_import

def delete_default_objects():
    """Delete default objects in Blender scene (cube, light, camera)"""
    # Select all objects
    bpy.ops.object.select_all(action='SELECT')
    # Delete selected objects
    bpy.ops.object.delete()
    # Clear default collections as well
    for collection in bpy.data.collections:
        if collection.name not in ['chrono_assets', 'chrono_frame_assets', 'chrono_frame_objects', 'chrono_cameras']:
            # Safely remove objects in collection first
            for obj in collection.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.collections.remove(collection)

def create_terrain_material():
    """Create a nice material for terrain visualization"""
    mat_name = "TerrainMaterial"
    if mat_name in bpy.data.materials:
        return bpy.data.materials[mat_name]
    
    terrain_mat = bpy.data.materials.new(name=mat_name)
    terrain_mat.use_nodes = True

    nodes = terrain_mat.node_tree.nodes
    links = terrain_mat.node_tree.links
    nodes.clear()

    # Material Output
    out = nodes.new("ShaderNodeOutputMaterial")
    out.location = (400, 0)

    # Principled BSDF
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (200, 0)
    bsdf.inputs['Base Color'].default_value = (0.18, 0.17, 0.16, 1.0)
    bsdf.inputs['Roughness'].default_value = 0.8

    # Noise Texture (for bump height)
    noise = nodes.new("ShaderNodeTexNoise")
    noise.location = (-400, 200)
    noise.inputs['Scale'].default_value = 10.0
    noise.inputs['Detail'].default_value = 2.0
    noise.inputs['Roughness'].default_value = 0.5

    # Bump node
    bump = nodes.new("ShaderNodeBump")
    bump.location = (-100, 0)
    bump.inputs['Strength'].default_value = 0.3
    bump.inputs['Distance'].default_value = 0.1

    # Wire them up
    links.new(noise.outputs['Fac'], bump.inputs['Height'])
    links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    
    return terrain_mat

def create_particle_material():
    """Create a material for terrain particles"""
    mat_name = "ParticleMaterial"
    if mat_name in bpy.data.materials:
        return bpy.data.materials[mat_name]
    
    particle_mat = bpy.data.materials.new(name=mat_name)
    particle_mat.use_nodes = True

    nodes = particle_mat.node_tree.nodes
    links = particle_mat.node_tree.links
    nodes.clear()

    # Material Output
    out = nodes.new("ShaderNodeOutputMaterial")
    out.location = (400, 0)

    # Principled BSDF
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (200, 0)
    bsdf.inputs['Base Color'].default_value = (0.1, 0.1, 0.1, 1.0)  # Dark gray
    bsdf.inputs['Roughness'].default_value = 0.6
    bsdf.inputs['Metallic'].default_value = 0.1

    # Connect nodes
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    
    return particle_mat

def load_terrain_for_frame(terrain_folder, file_prefix, file_suffix, frame, terrain_collection):
    """Load terrain OBJ for specific frame"""
    # Clear any previous terrain objects
    for obj in terrain_collection.objects:
        bpy.data.objects.remove(obj, do_unlink=True)
    
    # Construct filename
    filename = f"{file_prefix}{frame}{file_suffix}"
    filepath = os.path.join(terrain_folder, filename)
    
    if not os.path.exists(filepath):
        print(f"Warning: Terrain file not found: {filepath}")
        return False
    
    print(f"Loading terrain from: {filepath}")
    try:
        # Import OBJ
        bpy.ops.wm.obj_import(
            filepath=filepath,
            forward_axis='Y',
            up_axis='Z'
        )
        
        # Create material
        terrain_mat = create_terrain_material()
        
        # Move objects to terrain collection and apply material
        for obj in bpy.context.selected_objects:
            if obj.type == 'MESH':
                # Move to terrain collection
                for coll in obj.users_collection:
                    coll.objects.unlink(obj)
                terrain_collection.objects.link(obj)
                
                # Apply material
                obj.data.materials.clear()
                obj.data.materials.append(terrain_mat)
                
                # Rename
                obj.name = f"terrain_{frame}"
                print(f"Added terrain object: {obj.name}")
        
        return True
    except Exception as e:
        print(f"Error loading terrain: {e}")
        return False

def read_terrain_particles(particle_file, particle_radius=0.005):
    """
    Read terrain particles from a CSV file.
    Returns a list of particle positions (x, y, z).
    """
    print(f"Reading terrain particles from: {particle_file}")
    positions = []
    
    try:
        with open(particle_file) as f:
            line_count = 0
            count = 0
            for line in f:
                if line_count == 0:
                    line_count += 1
                    continue
                else:
                    line_count += 1
                    # Parse x, y, z from the line
                    line_seg = line.split(",")
                    if len(line_seg) < 3:
                        continue
                    try:
                        x, y, z = float(line_seg[0]), float(line_seg[1]), float(line_seg[2])
                        positions.append((x, y, z))
                        count += 1
                    except (ValueError, IndexError):
                        continue
        print(f"Total number of terrain particles: {count}")
    except FileNotFoundError:
        print(f"ERROR: Terrain particle file not found: {particle_file}")
        print("Continuing with empty particles list.")
    
    return positions

def setup_particle_system(positions, particle_radius, collection):
    """
    Set up a particle system to display terrain particles
    """
    if not positions:
        print("No particles to display")
        return
    
    # Create a small icosphere as particle instance
    bpy.ops.mesh.primitive_ico_sphere_add(radius=particle_radius, location=(50, 50, 50))
    ico_sphere = bpy.context.object
    ico_sphere.name = "ParticleInstance"
    
    # Apply material
    particle_mat = create_particle_material()
    ico_sphere.data.materials.append(particle_mat)
    
    # Create empty object to host particles
    bpy.ops.mesh.primitive_cube_add(size=0.0001)
    host_object = bpy.context.object
    host_object.name = "ParticleHost"
    
    # Move objects to the terrain collection
    for obj in [ico_sphere, host_object]:
        for coll in obj.users_collection:
            coll.objects.unlink(obj)
        collection.objects.link(obj)
    
    # Set up particle system
    particle_system = host_object.modifiers.new("TerrainParticles", 'PARTICLE_SYSTEM').particle_system
    ps_name = particle_system.name
    
    # Configure particle system
    settings = particle_system.settings
    settings.count = len(positions)
    settings.lifetime = 1000
    settings.frame_start = settings.frame_end = 1
    settings.render_type = "OBJECT"
    settings.instance_object = ico_sphere
    
    # Define particle handler function to position particles
    def particle_handler(scene, depsgraph):
        ob = depsgraph.objects.get(host_object.name)
        if ob:
            ps = ob.particle_systems[ps_name]
            for m, particle in enumerate(ps.particles):
                if m < len(positions):
                    setattr(particle, "location", positions[m])
                    setattr(particle, "size", particle_radius)
    
    # Register the handler
    bpy.app.handlers.frame_change_post.append(particle_handler)
    
    # Force update to initialize particles
    bpy.context.scene.frame_current = 2
    
    return particle_handler

def load_terrain_particles_for_frame(terrain_folder, file_prefix, frame, terrain_collection, particle_radius):
    """Load terrain particles for a specific frame"""
    # Clear any previous terrain objects and handlers
    for obj in terrain_collection.objects:
        bpy.data.objects.remove(obj, do_unlink=True)
    
    # Remove any existing particle handlers
    for handler in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(handler)
    
    # Construct filename
    filename = f"{file_prefix}{frame}.csv"
    filepath = os.path.join(terrain_folder, filename)
    
    if not os.path.exists(filepath):
        print(f"Warning: Terrain particle file not found: {filepath}")
        return False
    
    # Read particle positions
    positions = read_terrain_particles(filepath, particle_radius)
    
    if not positions:
        print("No terrain particles found")
        return False
    
    # Setup particle system
    handler = setup_particle_system(positions, particle_radius, terrain_collection)
    
    return True

def add_debug_objects():
    """Add debug objects to the scene to verify rendering is working"""
    # Add a simple debug sphere at origin
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0, location=(0, 0, 0))
    debug_sphere = bpy.context.active_object
    debug_sphere.name = "DebugSphere"
    debug_material = bpy.data.materials.new(name="DebugRed")
    debug_material.diffuse_color = (1.0, 0.0, 0.0, 1.0)
    debug_sphere.data.materials.append(debug_material)
    
    # Add coordinate axes for reference
    for axis, color in [((5, 0, 0), (1, 0, 0, 1)), 
                        ((0, 5, 0), (0, 1, 0, 1)), 
                        ((0, 0, 5), (0, 0, 1, 1))]:
        bpy.ops.mesh.primitive_cylinder_add(radius=0.1, depth=10, location=(axis[0]/2, axis[1]/2, axis[2]/2))
        cyl = bpy.context.active_object
        cyl.rotation_euler = (0, math.radians(90), 0) if axis[0] > 0 else \
                             (math.radians(90), 0, 0) if axis[1] > 0 else \
                             (0, 0, 0)
        axis_material = bpy.data.materials.new(name=f"Debug{color[0]}{color[1]}{color[2]}")
        axis_material.diffuse_color = color
        cyl.data.materials.append(axis_material)

def precalculate_camera_path(center, distance, start_frame, end_frame, camera_type):
    """
    Precalculate camera positions for stable camera movement
    
    Args:
        center: Center point for camera rotation
        distance: Distance from center
        start_frame: First frame number
        end_frame: Last frame number
        camera_type: Type of camera ("REVOLVING", "ISOMETRIC", etc.)
        
    Returns:
        List of camera positions
    """
    positions = []
    total_frames = max(1, end_frame - start_frame + 1)
    
    if camera_type == "REVOLVING":
        # Calculate positions for revolving camera
        for f in range(total_frames):
            angle = 2 * math.pi * REVOLVING_ROTATION_SPEED * (f / total_frames)
            
            x = center.x + distance * REVOLVING_DISTANCE_FACTOR_X * math.cos(angle)
            y = center.y + distance * REVOLVING_DISTANCE_FACTOR_Y * math.sin(angle)
            z = center.z + distance * REVOLVING_HEIGHT_FACTOR
            
            positions.append(mathutils.Vector((x, y, z)))
    
    elif camera_type == "ISOMETRIC":
        # For isometric, create fixed position
        position = center + mathutils.Vector((distance * 0.3, distance * 0.9, distance * 0.4))
        # Fill list with the same position for all frames
        positions = [position] * total_frames
        
    return positions

def setup_camera(camera_type, target_collections=None, frame=0, total_frames=100, camera_data=None):
    """
    Set up camera based on specified type, with improved positioning for tracked vehicle and terrain
    
    Args:
        target_collections: List of collections to include in framing
        camera_data: Optional pre-calculated camera data for stable paths
    """
    # Create camera if it doesn't exist
    if not bpy.data.cameras.get('ChronoCamera'):
        cam_data = bpy.data.cameras.new('ChronoCamera')
    else:
        cam_data = bpy.data.cameras['ChronoCamera']
    
    # Create camera object if it doesn't exist
    if not bpy.data.objects.get('ChronoCamera'):
        cam_obj = bpy.data.objects.new('ChronoCamera', cam_data)
        bpy.context.scene.collection.objects.link(cam_obj)
    else:
        cam_obj = bpy.data.objects['ChronoCamera']
    
    # Make this the active camera
    bpy.context.scene.camera = cam_obj
    
    # If we have pre-calculated camera data, use it
    if camera_data and camera_data.get('positions'):
        # Use pre-calculated position
        if frame < len(camera_data['positions']):
            cam_obj.location = camera_data['positions'][frame]
        else:
            print(f"Warning: Frame {frame} exceeds pre-calculated camera positions")
            cam_obj.location = camera_data['positions'][-1]
            
        # Point camera at fixed center
        direction = camera_data['center'] - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam_obj.rotation_euler = rot_quat.to_euler()
        
        # Set common camera parameters
        cam_data.lens = 55
        cam_data.clip_start = 0.1
        cam_data.clip_end = 10000
        
        print(f"Using pre-calculated camera position at {cam_obj.location}")
        return cam_obj
    
    # Calculate center of target collections
    object_found = False
    # Default to center of expected tracked vehicle position
    center = mathutils.Vector((0, 3.75, 1))  # Default center of vehicle
    distance = CAMERA_DEFAULT_DISTANCE
    
    # Initialize min/max coordinates
    min_co = mathutils.Vector((float('inf'), float('inf'), float('inf')))
    max_co = mathutils.Vector((float('-inf'), float('-inf'), float('-inf')))
    
    # Separate processing for chrono_frame_objects and terrain
    vehicle_center = None
    vehicle_size = None
    
    if target_collections:
        for collection in target_collections:
            if collection and collection.objects:
                print(f"Examining collection '{collection.name}' with {len(collection.objects)} objects")
                
                # Only use chrono_frame_objects for camera targeting, ignore terrain
                if collection.name == 'chrono_frame_objects' and collection.objects:
                    # Reset min/max for vehicle collection
                    vehicle_min_co = mathutils.Vector((float('inf'), float('inf'), float('inf')))
                    vehicle_max_co = mathutils.Vector((float('-inf'), float('-inf'), float('-inf')))
                    vehicle_found = False
                    
                    obj_count = 0
                    for obj in collection.objects:
                        if obj.type == 'MESH':
                            vehicle_found = True
                            obj_count += 1
                            if obj_count <= 3:
                                print(f"Found vehicle object: {obj.name} at {obj.location}")
                            for v in obj.bound_box:
                                world_co = obj.matrix_world @ mathutils.Vector(v)
                                vehicle_min_co.x = min(vehicle_min_co.x, world_co.x)
                                vehicle_min_co.y = min(vehicle_min_co.y, world_co.y)
                                vehicle_min_co.z = min(vehicle_min_co.z, world_co.z)
                                vehicle_max_co.x = max(vehicle_max_co.x, world_co.x)
                                vehicle_max_co.y = max(vehicle_max_co.y, world_co.y)
                                vehicle_max_co.z = max(vehicle_max_co.z, world_co.z)
                    
                    if vehicle_found:
                        vehicle_center = (vehicle_min_co + vehicle_max_co) / 2
                        vehicle_size = vehicle_max_co - vehicle_min_co
                        print(f"Vehicle center: {vehicle_center}, bounds: min={vehicle_min_co}, max={vehicle_max_co}")
                
                # Still collect all objects for overall bounding box (for distance calculation)
                obj_count = 0
                for obj in collection.objects:
                    if obj.type == 'MESH':
                        object_found = True
                        obj_count += 1
                        for v in obj.bound_box:
                            world_co = obj.matrix_world @ mathutils.Vector(v)
                            min_co.x = min(min_co.x, world_co.x)
                            min_co.y = min(min_co.y, world_co.y)
                            min_co.z = min(min_co.z, world_co.z)
                            max_co.x = max(max_co.x, world_co.x)
                            max_co.y = max(max_co.y, world_co.y)
                            max_co.z = max(max_co.z, world_co.z)
    
    # Prioritize vehicle center if available
    if vehicle_center:
        center = vehicle_center
        print(f"Using vehicle center: {center}")
    elif object_found:
        center = (min_co + max_co) / 2
        print(f"Using overall scene center: {center}")
    else:
        print(f"WARNING: No mesh objects found in collections - using default camera position")
    
    # Calculate distance based on all objects including terrain
    if object_found:
        size = max_co - min_co
        max_dim = max(size.x, size.y, size.z)
        # Calculate distance using the configurable factor
        distance = max_dim * CAMERA_SCENE_DISTANCE_FACTOR
        print(f"Calculated scene size: {size}, max dimension: {max_dim}, camera distance: {distance}")
    
    # Set up camera based on type
    if camera_type == "ISOMETRIC":
        # Move camera for isometric view - more from the side
        cam_obj.location = center + mathutils.Vector((distance * 0.3, distance * 0.9, distance * 0.4))
        # Adjust camera rotation to point at the center of the scene
        direction = center - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam_obj.rotation_euler = rot_quat.to_euler()
    
    elif camera_type == "SIDE":
        cam_obj.location = center + mathutils.Vector((distance * 0.6, 0, 0))
        # Point camera directly at center
        direction = center - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam_obj.rotation_euler = rot_quat.to_euler()
    
    elif camera_type == "TOP":
        cam_obj.location = center + mathutils.Vector((0, 0, distance * 0.6))
        # Point camera directly at center
        direction = center - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam_obj.rotation_euler = rot_quat.to_euler()
    
    elif camera_type == "REVOLVING":
        # Calculate angle based on current frame and speed factor
        angle = 2 * math.pi * REVOLVING_ROTATION_SPEED * (frame / total_frames)
        
        # Create a circle around the center with configurable factors
        x = center.x + distance * REVOLVING_DISTANCE_FACTOR_X * math.cos(angle)
        y = center.y + distance * REVOLVING_DISTANCE_FACTOR_Y * math.sin(angle)
        z = center.z + distance * REVOLVING_HEIGHT_FACTOR
        
        cam_obj.location = mathutils.Vector((x, y, z))
        
        # Point camera directly at center
        direction = center - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam_obj.rotation_euler = rot_quat.to_euler()
    
    # Set common camera parameters
    cam_data.lens = 55  # Increased from 45mm to 55mm for even tighter framing
    cam_data.clip_start = 0.1
    cam_data.clip_end = 10000  # Keep large clip end to ensure terrain is visible
    
    print(f"Camera positioned at {cam_obj.location}, rotation: {cam_obj.rotation_euler}")
    
    # If we're setting up a camera for the first time and stable mode is enabled, return calculated data
    should_return_data = (
        (camera_type == "REVOLVING" and CAMERA_STABLE_REVOLVING) or
        (camera_type == "ISOMETRIC" and CAMERA_STABLE_ISOMETRIC)
    ) and frame == 0
    
    if should_return_data:
        return cam_obj, {
            'center': center,
            'distance': distance,
            'positions': precalculate_camera_path(center, distance, 0, total_frames, camera_type)
        }
    
    return cam_obj

def setup_lighting(light_setup="THREE_POINT"):
    """
    Set up lighting based on specified setup
    """
    # Delete existing lights
    for obj in bpy.data.objects:
        if obj.type == 'LIGHT':
            bpy.data.objects.remove(obj)
    
    # Brighter lights for better visibility
    if light_setup == "THREE_POINT":
        # Key light
        key_data = bpy.data.lights.new(name="Key", type='SUN')
        key_data.energy = 3.0  # Increased brightness
        key_obj = bpy.data.objects.new(name="Key", object_data=key_data)
        bpy.context.scene.collection.objects.link(key_obj)
        key_obj.rotation_euler = mathutils.Euler((math.radians(45), 0, math.radians(45)), 'XYZ')
        
        # Fill light
        fill_data = bpy.data.lights.new(name="Fill", type='SUN')
        fill_data.energy = 2.0  # Increased brightness
        fill_obj = bpy.data.objects.new(name="Fill", object_data=fill_data)
        bpy.context.scene.collection.objects.link(fill_obj)
        fill_obj.rotation_euler = mathutils.Euler((math.radians(45), 0, math.radians(-45)), 'XYZ')
        
        # Back light
        back_data = bpy.data.lights.new(name="Back", type='SUN')
        back_data.energy = 2.5  # Increased brightness
        back_obj = bpy.data.objects.new(name="Back", object_data=back_data)
        bpy.context.scene.collection.objects.link(back_obj)
        back_obj.rotation_euler = mathutils.Euler((math.radians(-45), 0, math.radians(180)), 'XYZ')
        
        # Add ambient light
        ambient_data = bpy.data.lights.new(name="Ambient", type='SUN')
        ambient_data.energy = 0.5
        ambient_obj = bpy.data.objects.new(name="Ambient", object_data=ambient_data)
        bpy.context.scene.collection.objects.link(ambient_obj)
        ambient_obj.rotation_euler = mathutils.Euler((0, 0, 0), 'XYZ')
    
    elif light_setup == "STUDIO":
        # Area light from above
        area_data = bpy.data.lights.new(name="Area", type='AREA')
        area_data.energy = 8.0  # Increased brightness
        area_data.size = 10.0  # Larger light
        area_obj = bpy.data.objects.new(name="Area", object_data=area_data)
        bpy.context.scene.collection.objects.link(area_obj)
        area_obj.location = (0, 0, 10)
        area_obj.rotation_euler = mathutils.Euler((0, 0, 0), 'XYZ')
        
        # Fill light
        fill_data = bpy.data.lights.new(name="Fill", type='AREA')
        fill_data.energy = 4.0  # Increased brightness
        fill_data.size = 5.0
        fill_obj = bpy.data.objects.new(name="Fill", object_data=fill_data)
        bpy.context.scene.collection.objects.link(fill_obj)
        fill_obj.location = (10, 0, 5)
        fill_obj.rotation_euler = mathutils.Euler((0, math.radians(90), 0), 'XYZ')
        
        # Second fill light
        fill2_data = bpy.data.lights.new(name="Fill2", type='AREA')
        fill2_data.energy = 3.0
        fill2_data.size = 5.0
        fill2_obj = bpy.data.objects.new(name="Fill2", object_data=fill2_data)
        bpy.context.scene.collection.objects.link(fill2_obj)
        fill2_obj.location = (0, 10, 5)
        fill2_obj.rotation_euler = mathutils.Euler((0, 0, math.radians(90)), 'XYZ')
    
    elif light_setup == "OUTDOOR":
        # Sun light
        sun_data = bpy.data.lights.new(name="Sun", type='SUN')
        sun_data.energy = 5.0  # Increased brightness
        sun_obj = bpy.data.objects.new(name="Sun", object_data=sun_data)
        bpy.context.scene.collection.objects.link(sun_obj)
        sun_obj.rotation_euler = mathutils.Euler((math.radians(45), math.radians(10), math.radians(45)), 'XYZ')
        
        # Add environment lighting
        world = bpy.context.scene.world
        if not world:
            world = bpy.data.worlds.new("World")
            bpy.context.scene.world = world
        world.use_nodes = True
        bg_node = world.node_tree.nodes.get('Background')
        if bg_node:
            bg_node.inputs[0].default_value = (0.8, 0.9, 1.0, 1.0)  # Sky blue
            bg_node.inputs[1].default_value = 2.0  # Increased strength

def setup_render_settings(output_path, resolution_x=1920, resolution_y=1080, use_gpu=True):
    """
    Set up rendering settings with more conservative Cycles options
    """
    scene = bpy.context.scene
    
    # Set render engine to Cycles
    scene.render.engine = 'CYCLES'
    
    # Configure GPU rendering if requested and available
    if use_gpu:
        try:
            # More conservative GPU settings
            cycles_prefs = bpy.context.preferences.addons['cycles'].preferences
            # Clear device list first
            cycles_prefs.get_devices()
            
            # Try CUDA first, then fall back to other options
            compute_devices = ['CUDA', 'OPTIX', 'HIP', 'ONEAPI']
            for device_type in compute_devices:
                try:
                    cycles_prefs.compute_device_type = device_type
                    cycles_prefs.get_devices()
                    device_found = False
                    for device in cycles_prefs.devices:
                        if device.type == device_type and device.use:
                            device_found = True
                            break
                    if device_found:
                        print(f"Using {device_type} for GPU rendering")
                        bpy.context.scene.cycles.device = 'GPU'
                        break
                except:
                    continue
            print(f"Final rendering device: {bpy.context.scene.cycles.device}")
        except Exception as e:
            print(f"Error setting up GPU rendering: {e}")
            print("Falling back to CPU rendering")
            bpy.context.scene.cycles.device = 'CPU'
    else:
        # Use CPU rendering
        bpy.context.scene.cycles.device = 'CPU'
        print("Using CPU for rendering as requested")
    
    # Set samples to 128
    scene.cycles.samples = 128
    scene.cycles.preview_samples = 32
    
    # Disable OptiX denoising which can cause crashes
    scene.cycles.use_denoising = True
    scene.cycles.denoiser = 'OPENIMAGEDENOISE'  # More stable than OptiX
    
    # Use more conservative settings
    scene.cycles.max_bounces = 4       # Reduced from 8
    scene.cycles.diffuse_bounces = 2   # Reduced from 3
    scene.cycles.glossy_bounces = 2    # Reduced from 3
    scene.cycles.transmission_bounces = 4  # Reduced from 6
    
    # Ensure we're not running out of memory
    scene.render.use_persistent_data = False  # Don't keep render data in memory
    
    # Set resolution
    scene.render.resolution_x = resolution_x
    scene.render.resolution_y = resolution_y
    scene.render.resolution_percentage = 100
    
    # Output settings
    scene.render.filepath = output_path
    scene.render.image_settings.file_format = 'PNG'
    
    # Enable transparent background if needed
    scene.render.film_transparent = False

def cleanup_resources():
    """Clean up resources to prevent memory leaks"""
    # Clear particle handlers
    bpy.app.handlers.frame_change_post.clear()
    
    # Force garbage collection
    gc.collect()

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Render Chrono simulation in Blender")
    parser.add_argument("--assets", required=True, help="Path to the .assets.py file")
    parser.add_argument("--output", required=True, help="Output directory for rendered images")
    parser.add_argument("--camera", dest="camera", default="ISOMETRIC", 
                        choices=["ISOMETRIC", "SIDE", "TOP", "REVOLVING"],
                        help="Camera type")
    parser.add_argument("--view", dest="camera", 
                        choices=["ISOMETRIC", "SIDE", "TOP", "REVOLVING"],
                        help=argparse.SUPPRESS)
    parser.add_argument("--lighting", default="THREE_POINT", 
                        choices=["THREE_POINT", "STUDIO", "OUTDOOR"],
                        help="Lighting setup")
    parser.add_argument("--resolution_x", type=int, default=1920, help="Horizontal resolution")
    parser.add_argument("--resolution_y", type=int, default=1080, help="Vertical resolution")
    parser.add_argument("--frame_start", type=int, default=1, help="Start frame")
    parser.add_argument("--frame_end", type=int, default=100, help="End frame")
    parser.add_argument("--debug", action="store_true", help="Add debug objects to scene") 
    parser.add_argument("--cpu", action="store_true", help="Force CPU rendering instead of GPU")
    
    # Terrain-related arguments
    parser.add_argument("--terrain_path", help="Path to folder containing terrain files")
    parser.add_argument("--terrain_type", choices=["OBJ", "PARTICLES"], default="OBJ", 
                        help="Type of terrain representation (OBJ meshes or particle CSV files)")
    parser.add_argument("--terrain_prefix", default="fluid", help="Prefix for terrain filenames")
    parser.add_argument("--terrain_suffix", default="_surface.obj", help="Suffix for terrain OBJ filenames (only used with --terrain_type OBJ)")
    parser.add_argument("--particle_radius", type=float, default=0.005, help="Radius for terrain particles (only used with --terrain_type PARTICLES)")
    
    # Camera stability options
    parser.add_argument("--stable_camera", action="store_true", help="Use stable camera path for revolving and isometric views")
    
    # Extract arguments after "--"
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    
    args = parser.parse_args(argv)
    
    # First, delete all default objects
    delete_default_objects()
    
    # Set up rendering parameters
    bpy.context.scene.frame_start = args.frame_start
    bpy.context.scene.frame_end = args.frame_end
    
    # Create output directory if it doesn't exist
    os.makedirs(args.output, exist_ok=True)
    
    # Create view-specific subfolder
    view_folder = os.path.join(args.output, args.camera)
    os.makedirs(view_folder, exist_ok=True)
    print(f"Saving renders to: {view_folder}")
    
    # Create a collection for terrain if terrain path is provided
    terrain_collection = None
    if args.terrain_path:
        # Check if terrain path exists
        if not os.path.exists(args.terrain_path):
            print(f"WARNING: Terrain path not found: {args.terrain_path}")
        else:
            print(f"Terrain path: {args.terrain_path}")
            print(f"Terrain type: {args.terrain_type}")
            terrain_collection = bpy.data.collections.get('chrono_terrain')
            if not terrain_collection:
                terrain_collection = bpy.data.collections.new('chrono_terrain')
                bpy.context.scene.collection.children.link(terrain_collection)
    
    # Import Chrono simulation
    print(f"Importing Chrono simulation from {args.assets}")
    chrono_import.read_chrono_simulation(bpy.context, args.assets, True, False)
    
    # Find the chrono_frame_objects collection for camera targeting
    chrono_frame_objects = bpy.data.collections.get('chrono_frame_objects')
    
    # Add debug objects if requested
    if args.debug:
        print("Adding debug objects to scene")
        add_debug_objects()
    
    # Set up lighting
    print(f"Setting up {args.lighting} lighting")
    setup_lighting(args.lighting)
    
    # Print collections information
    print("Collections in scene:")
    for collection in bpy.data.collections:
        print(f" - {collection.name}: {len(collection.objects)} objects")
        
    # Check for specific collections
    for name in ['chrono_assets', 'chrono_frame_assets', 'chrono_frame_objects']:
        collection = bpy.data.collections.get(name)
        if collection:
            print(f"Collection {name} has {len(collection.objects)} objects")
        else:
            print(f"Collection {name} not found!")
    
    # Set up render settings - use view folder for base file path
    render_base_path = os.path.join(view_folder, "frame_")
    setup_render_settings(render_base_path, args.resolution_x, args.resolution_y, not args.cpu)
    
    # Determine which collections to include in camera framing
    target_collections = [chrono_frame_objects]
    if terrain_collection:
        target_collections.append(terrain_collection)
    
    # Track if we had to switch to CPU rendering
    switched_to_cpu = False
    
    # Pre-calculate camera path if using a camera type that needs stabilization
    camera_data = None
    should_precalculate = (
        (args.camera == "REVOLVING" and (args.stable_camera or CAMERA_STABLE_REVOLVING)) or
        (args.camera == "ISOMETRIC" and (args.stable_camera or CAMERA_STABLE_ISOMETRIC))
    )
    
    if should_precalculate:
        print(f"Pre-calculating stable camera path for {args.camera} view")
        
        # Force update before calculating camera path
        bpy.context.scene.frame_set(args.frame_start)
        chrono_import.callback_post(bpy.context.scene)
        
        # Load initial terrain state if needed
        if args.terrain_path and terrain_collection:
            if args.terrain_type == "OBJ":
                load_terrain_for_frame(
                    args.terrain_path, args.terrain_prefix, args.terrain_suffix, 
                    args.frame_start, terrain_collection
                )
            else:  # PARTICLES
                load_terrain_particles_for_frame(
                    args.terrain_path, args.terrain_prefix, 
                    args.frame_start, terrain_collection, args.particle_radius
                )
            
            # Force updates
            bpy.context.view_layer.update()
        
        # Calculate initial camera setup
        _, camera_data = setup_camera(
            args.camera,
            target_collections,
            0,  # Initial frame
            args.frame_end - args.frame_start + 1
        )
        print(f"Created stable camera path with {len(camera_data['positions'])} positions")
    
    # Render each frame
    for frame in range(args.frame_start, args.frame_end + 1):
        print(f"\nProcessing frame {frame}")
        bpy.context.scene.frame_set(frame)
        
        # Force update to ensure the callback is triggered for Chrono objects
        chrono_import.callback_post(bpy.context.scene)
        
        # Load terrain for this frame if path was provided
        if args.terrain_path and terrain_collection:
            # Clear particle handlers before loading new terrain
            bpy.app.handlers.frame_change_post.clear()
            
            if args.terrain_type == "OBJ":
                # Load terrain as OBJ mesh
                load_terrain_for_frame(
                    args.terrain_path, 
                    args.terrain_prefix, 
                    args.terrain_suffix, 
                    frame, 
                    terrain_collection
                )
            else:  # PARTICLES
                # Load terrain as particles
                load_terrain_particles_for_frame(
                    args.terrain_path,
                    args.terrain_prefix,
                    frame,
                    terrain_collection,
                    args.particle_radius
                )
            
            # Give Blender a moment to process the terrain import
            if bpy.app.background:
                # In background mode, we need to force updates
                bpy.context.view_layer.update()
            else:
                # In interactive mode, we can use the GUI update
                bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        
        # Update camera for this frame
        print("Setting up camera")
        setup_camera(
            args.camera, 
            target_collections,
            frame - args.frame_start, 
            args.frame_end - args.frame_start + 1,
            camera_data
        )
        
        # Render with error handling
        print(f"Rendering frame {frame}")
        bpy.context.scene.render.filepath = os.path.join(view_folder, f"frame_{frame:04d}.png")
        
        try:
            bpy.ops.render.render(write_still=True)
        except Exception as e:
            print(f"Error rendering frame {frame}: {e}")
            
            # If we haven't already switched to CPU, try that
            if not switched_to_cpu and bpy.context.scene.cycles.device == 'GPU':
                print("Switching to CPU rendering and trying again...")
                bpy.context.scene.cycles.device = 'CPU'
                switched_to_cpu = True
                
                try:
                    bpy.ops.render.render(write_still=True)
                    print("CPU rendering successful!")
                except Exception as e2:
                    print(f"CPU rendering also failed: {e2}")
                    print(f"Skipping frame {frame}")
            else:
                print(f"Skipping frame {frame}")
        
        # Clean up terrain objects after rendering to free memory
        if terrain_collection:
            for obj in terrain_collection.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
        
        # Clean up resources after each frame
        cleanup_resources()
    
    print("Rendering complete!")

if __name__ == "__main__":
    main()