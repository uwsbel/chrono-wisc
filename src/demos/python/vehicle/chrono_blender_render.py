import os
import sys

# Force headless mode for systems without displays
# Set these before importing bpy
if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
    print("Detected headless environment - forcing GPU rendering without display")
    os.environ["BLENDER_SKIP_PYTHON_PATH_CLEANUP"] = "1"
    os.environ["PYOPENGL_PLATFORM"] = "egl"  # Use EGL instead of GLX

import bpy
import math
import mathutils
import argparse
import re
import gc

# Try different import approaches for chrono_import
try:
    # Try direct import first (when chrono_import.py is in the current directory)
    import chrono_import
    print("Successfully imported chrono_import from current directory")
except ImportError:
    # Fall back to the standard path approach
    print("Direct import failed, trying standard path...")
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        sys.path.append(os.path.join(script_dir, '..', 'src', 'importer_blender'))
        import chrono_import
        print("Successfully imported chrono_import from standard path")
    except ImportError:
        print("ERROR: Could not import chrono_import module!")
        print("Current sys.path:", sys.path)
        sys.exit(1)  # Exit with error code

# Camera and rendering configuration constants
CAMERA_DEFAULT_DISTANCE = 5.0          # Default distance when no objects are found
CAMERA_PADDING = 0.7                  # Extra margin when fitting bounds into view
CAMERA_LENS = 55                       # Camera lens in mm (higher = zoom in)
CAMERA_LENS_OVERRIDE = None            # Set when user overrides lens via CLI
CHRONO_ALIGN_UP = True                 # Align Chrono camera roll to world up
CAMERA_USE_CHRONO_REFERENCE = True     # Use Chrono camera pose to define view directions
SCENE_YAW_DEG = 0.0                    # Rotate scene around Z (degrees)
OUTPUT_DPI = 150                       # PNG metadata DPI

# Background settings
USE_BACKGROUND_TEXTURE = False          # Toggle to enable/disable background texture
BACKGROUND_TYPE = "WHITE"           # Options: "GRADIENT", "SKY", "NONE", "WHITE"
BACKGROUND_COLOR1 = (0.2, 0.3, 0.4, 1.0)  # Bottom/horizon color for gradient
BACKGROUND_COLOR2 = (0.5, 0.7, 0.9, 1.0)  # Top/zenith color for gradient

# Revolving camera settings
REVOLVING_ROTATION_SPEED = 0.5         # Rotation speed (1.0 = one full rotation)
REVOLVING_PITCH = 0.35                 # Height ratio for revolving view (0 = level)

# Particle material presets
WHEEL_BCE_COLOR = (0.02, 0.02, 0.02, 1.0)  # Dark for rigid BCE particles
WHEEL_BCE_RADIUS_DEFAULT = 0.0025         # Default radius for wheel BCE particles

# First, ensure we're running in background mode
# This script should be called with: blender -b -P chrono_blender_render.py -- [args]

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

def create_particle_material(name="ParticleMaterial", base_color=(0.1, 0.8, 0.2, 1.0)):
    """Create a material for terrain particles"""
    mat_name = name
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
    # Use a bright color for visibility against dark background
    bsdf.inputs['Base Color'].default_value = base_color
    bsdf.inputs['Roughness'].default_value = 0.3
    bsdf.inputs['Metallic'].default_value = 0.5
    
    # Check if Emission properties exist before setting them
    if 'Emission Color' in bsdf.inputs:
        bsdf.inputs['Emission Color'].default_value = (0.1, 0.7, 0.2, 1.0)
    
    if 'Emission Strength' in bsdf.inputs:
        bsdf.inputs['Emission Strength'].default_value = 0.2
    elif 'Emission' in bsdf.inputs:
        bsdf.inputs['Emission'].default_value = 0.2
    
    print(f"Created particle material '{mat_name}' with base color {base_color}")

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

def read_terrain_particles(particle_file):
    """Read terrain particles from a CSV file and return positions."""
    print(f"Reading terrain particles from: {particle_file}")
    positions = []
    
    # Track position ranges for debugging
    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')
    min_z = float('inf')
    max_z = float('-inf')
    
    try:
        with open(particle_file) as f:
            line_count = 0
            count = 0
            for line in f:
                if line_count == 0:
                    line_count += 1
                    continue
                line_count += 1
                line_seg = line.split(",")
                if len(line_seg) < 3:
                    continue
                try:
                    x, y, z = float(line_seg[0]), float(line_seg[1]), float(line_seg[2])

                    # Update position ranges
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
                    min_z = min(min_z, z)
                    max_z = max(max_z, z)

                    positions.append((x, y, z))
                    count += 1
                except (ValueError, IndexError):
                    continue
        print(f"Total number of terrain particles: {count}")
        
        # Print position ranges for debugging
        if positions:
            print(f"Particle position ranges:")
            print(f"  X: {min_x:.3f} to {max_x:.3f}")
            print(f"  Y: {min_y:.3f} to {max_y:.3f}")
            print(f"  Z: {min_z:.3f} to {max_z:.3f}")
            
            # Check if particles are likely outside camera view
            if min_z < -10 or max_z > 10 or min_x < -10 or max_x > 10:
                print("WARNING: Particles may be outside expected range - check scale/units")
                
    except FileNotFoundError:
        print(f"ERROR: Terrain particle file not found: {particle_file}")
        print("Continuing with empty particles list.")
    
    return positions

def setup_particle_system(positions, particle_radius, collection, material=None):
    """Set up a particle system to display terrain particles."""
    if not positions:
        print("No particles to display")
        return {'handler': None, 'host': None, 'instance': None}
    
    # Debug: Show particle info
    print(f"Setting up particle system with {len(positions)} particles and radius {particle_radius}")
    
    # Limit number of particles
    max_particles = 5000000 
    original_count = len(positions)
    
    if len(positions) > max_particles:
        print(f"WARNING: Too many particles ({original_count}). Limiting to {max_particles} particles.")
        skip_factor = original_count // max_particles
        positions = positions[::skip_factor]
        print(f"Downsampled from {original_count} to {len(positions)} particles (factor: {skip_factor})")

    # Create a small icosphere as particle instance
    bpy.ops.mesh.primitive_ico_sphere_add(radius=particle_radius, location=(50, 50, 50)) # Off-screen
    ico_sphere = bpy.context.object
    ico_sphere.name = "ParticleInstance"
    
    # Apply material
    if material is None:
        material = create_particle_material()
    ico_sphere.data.materials.append(material)
    
    # Create empty object to host particles
    bpy.ops.mesh.primitive_cube_add(size=0.0001, location=(-50,-50,-50)) # Off-screen
    host_object = bpy.context.object
    host_object.name = "ParticleHost"
    
    # Move objects to the terrain collection
    for obj in [ico_sphere, host_object]:
        if obj.name in bpy.context.view_layer.objects: # Check if obj is in current view layer
            for coll in obj.users_collection: # Unlink from all collections
                coll.objects.unlink(obj)
        if not obj.name in collection.objects: # Link to target collection if not already there
            collection.objects.link(obj)
    
    # Set up particle system
    particle_system_modifier = host_object.modifiers.new("TerrainParticles", 'PARTICLE_SYSTEM')
    if not particle_system_modifier: # Should not happen with .new
        print("ERROR: Could not create particle system modifier.")
        return {'handler': None, 'host': host_object, 'instance': ico_sphere}

    ps = particle_system_modifier.particle_system
    ps_name = ps.name # Should be "TerrainParticles"
    
    settings = ps.settings
    settings.count = len(positions) # Based on (potentially downsampled) positions
    settings.lifetime = 10000 # Very long lifetime to cover all frames
    settings.frame_start = settings.frame_end = 0 # Emit all at frame 0 so they exist at all frames
    settings.render_type = "OBJECT"
    settings.instance_object = ico_sphere
    settings.particle_size = 1.0 # Use the actual radius of the ico_sphere
    # settings.use_dynamic_size = False # if ico_sphere radius is particle_radius

    if len(positions) > 0:
        print(f"Sample particle positions (first 5):")
        for i in range(min(5, len(positions))):
            print(f"  Particle {i}: {positions[i]}")
    
    # Ensure particles are updated for the handler
    # It's important that the depsgraph is evaluated correctly when the handler runs.
    # Setting the frame can help, but sometimes a full depsgraph update is needed.
    bpy.context.view_layer.update()


    def particle_handler(scene, depsgraph):
        # Check if our host object and particle system exist in the depsgraph
        # It's crucial to get the evaluated object from depsgraph for handlers
        evaluated_host = depsgraph.objects.get(host_object.name)
        if not evaluated_host:
            print(f"Particle handler: Host object '{host_object.name}' not in depsgraph (frame {scene.frame_current}).")
            return

        # Particle systems are on the evaluated object
        if not evaluated_host.particle_systems or not ps_name in evaluated_host.particle_systems:
            print(f"Particle handler: Particle system '{ps_name}' not on evaluated host (frame {scene.frame_current}).")
            return
            
        current_ps = evaluated_host.particle_systems[ps_name]
        num_depsgraph_particles = len(current_ps.particles)

        print(f"Particle handler for frame {scene.frame_current}: Setting up {num_depsgraph_particles} particles (expected {len(positions)}) on host '{host_object.name}'")

        positions_set = 0
        for i, p in enumerate(current_ps.particles):
            if i < len(positions): # Ensure we don't go out of bounds for positions list
                p.location = positions[i]
                positions_set += 1
        
        if scene.frame_current <= 2:
            print(f"  Set {positions_set} particle positions. Sample: {positions[:3] if len(positions) >= 3 else positions}")

    bpy.app.handlers.frame_change_post.append(particle_handler)
    
    # Force an update to try and initialize particles correctly
    # Particles emit at frame 0, so going to frame 0 then 1 initializes them
    bpy.context.scene.frame_set(0) # Go to emission frame - particles are born
    bpy.context.scene.frame_set(1) # Advance one frame so particles are fully active
    bpy.context.view_layer.update() # Update depsgraph

    return {
        'handler': particle_handler,
        'host': host_object,
        'instance': ico_sphere,
    }

def load_terrain_particles_for_frame(terrain_folder, file_prefix, frame, terrain_collection, particle_radius,
                                     y_min=None, y_max=None, y_tol=0.0, clear_collection=True,
                                     material=None, transform_fn=None):
    """
    Load terrain particles for a specific frame.
    
    Args:
        terrain_folder: Folder containing particle CSV files
        file_prefix: Prefix for CSV filenames
        frame: Frame number to load
        terrain_collection: Collection to add particles to
        particle_radius: Radius for particles
    Returns:
        Tuple: (success, particle_system_info, positions)
    """
    # Clear any previous terrain objects and handlers
    if clear_collection:
        for obj in terrain_collection.objects:
            bpy.data.objects.remove(obj, do_unlink=True)

        handlers_to_remove = [h for h in bpy.app.handlers.frame_change_post if hasattr(h, '__name__') and 'particle_handler' in h.__name__]
        for h in handlers_to_remove:
            bpy.app.handlers.frame_change_post.remove(h)
            
    filename = f"{file_prefix}{frame}.csv"
    filepath = os.path.join(terrain_folder, filename)
    
    if not os.path.exists(filepath):
        print(f"Warning: Terrain particle file not found: {filepath}")
        return False, None, []
    
    positions = read_terrain_particles(filepath)
    if transform_fn:
        positions = transform_fn(positions)
    
    # Apply optional Y-slice filtering for cross-section view with tolerance
    if y_min is not None or y_max is not None:
        total_before = len(positions)
        mask = []
        for (x, y, z) in positions:
            keep = True
            # Allow a small tolerance so near-boundary particles are included
            if y_min is not None and y < (y_min - (y_tol or 0.0)):
                keep = False
            if y_max is not None and y > (y_max + (y_tol or 0.0)):
                keep = False
            mask.append(keep)

        # Filter positions
        positions = [p for p, m in zip(positions, mask) if m]
        print(f"Applied Y-slice filter (y_min={y_min}, y_max={y_max}, y_tol={y_tol}). Kept {len(positions)} / {total_before} particles.")

    if not positions:
        print("No terrain particles found")
        return False, None, []
    
    if particle_radius <= 0.001:
        print(f"WARNING: Particle radius {particle_radius} is too small - using default 0.005")
        particle_radius = 0.005
    print(f"Using particle radius: {particle_radius}")
    
    particle_system_info = setup_particle_system(
        positions, 
        particle_radius, 
        terrain_collection,
        material=material
    )

    if particle_system_info and particle_system_info['handler']:
        print("Particle system successfully created and handler registered.")
        return True, particle_system_info, positions
    else:
        print("WARNING: Failed to create particle system or register handler.")
        return False, particle_system_info, positions

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

def create_force_arrow_material(name="ForceArrowMaterial", color=(1.0, 0.5, 0.0, 1.0)):
    """Create a material for the force arrow."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()
    
    # Material Output
    out = nodes.new("ShaderNodeOutputMaterial")
    out.location = (400, 0)
    
    # Principled BSDF with emission for visibility
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (200, 0)
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Roughness'].default_value = 0.3
    bsdf.inputs['Metallic'].default_value = 0.8
    
    # Add emission for better visibility
    if 'Emission Color' in bsdf.inputs:
        bsdf.inputs['Emission Color'].default_value = color
    if 'Emission Strength' in bsdf.inputs:
        bsdf.inputs['Emission Strength'].default_value = 0.5
    
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return mat

def create_force_arrow(start_pos, end_pos, name="ForceArrow", shaft_radius=0.015, head_radius=0.04, 
                       head_length=0.1, color=(1.0, 0.5, 0.0, 1.0), fixed_length=0.3):
    """
    Create an arrow mesh pointing from start_pos toward end_pos.

    Args:
        start_pos: Starting position (tuple or Vector) - arrow base
        end_pos: Ending position (tuple or Vector) - arrow points toward this
        name: Name for the arrow object
        shaft_radius: Radius of the arrow shaft
        head_radius: Radius of the arrow head base
        head_length: Length of the arrow head cone
        color: RGBA color tuple for the arrow
        fixed_length: Fixed length for the arrow (None = use actual distance)

    Returns:
        The arrow object
    """
    start = mathutils.Vector(start_pos)
    end = mathutils.Vector(end_pos)

    direction = end - start
    actual_distance = direction.length

    if actual_distance < 0.001:
        print(f"Warning: Arrow '{name}' has zero length, skipping")
        return None

    # Use fixed length for the arrow (not the actual distance)
    if fixed_length is not None:
        length = fixed_length
    else:
        length = actual_distance

    # Ensure head doesn't exceed total length
    actual_head_length = min(head_length, length * 0.3)
    shaft_length = length - actual_head_length
    
    # Remove existing arrow if present
    if name in bpy.data.objects:
        old_obj = bpy.data.objects[name]
        bpy.data.objects.remove(old_obj, do_unlink=True)
    if name + "_head" in bpy.data.objects:
        old_obj = bpy.data.objects[name + "_head"]
        bpy.data.objects.remove(old_obj, do_unlink=True)
    
    # Create arrow shaft (cylinder)
    bpy.ops.mesh.primitive_cylinder_add(radius=shaft_radius, depth=shaft_length, location=(0, 0, shaft_length/2))
    shaft = bpy.context.active_object
    shaft.name = name
    
    # Create arrow head (cone)
    bpy.ops.mesh.primitive_cone_add(radius1=head_radius, radius2=0, depth=actual_head_length, location=(0, 0, shaft_length + actual_head_length/2))
    head = bpy.context.active_object
    head.name = name + "_head"
    
    # Join head to shaft
    bpy.ops.object.select_all(action='DESELECT')
    shaft.select_set(True)
    head.select_set(True)
    bpy.context.view_layer.objects.active = shaft
    bpy.ops.object.join()
    
    arrow = bpy.context.active_object
    arrow.name = name
    
    # Apply material
    mat = create_force_arrow_material(name + "_Material", color)
    arrow.data.materials.clear()
    arrow.data.materials.append(mat)
    
    # Calculate rotation to align with direction
    # Default cylinder points in +Z, we need to rotate to point along direction
    dir_normalized = direction.normalized()
    up = mathutils.Vector((0, 0, 1))
    if abs(dir_normalized.dot(up)) > 0.999:
        # Direction is nearly parallel to Z, use a different reference
        rot_axis = mathutils.Vector((1, 0, 0))
        rot_angle = 0 if direction.z > 0 else math.pi
    else:
        rot_axis = up.cross(dir_normalized)
        rot_angle = math.acos(up.dot(dir_normalized))
    
    rot_matrix = mathutils.Matrix.Rotation(rot_angle, 4, rot_axis)
    arrow.matrix_world = rot_matrix
    
    # The arrow was created with its center at (0, 0, shaft_length/2), so after joining
    # the origin is at the shaft center, not at the base (z=0).
    # We need to offset the position so the BASE is at start, not the center.
    # Move forward along the direction by shaft_length/2 so base ends up at start.
    base_offset = dir_normalized * (shaft_length / 2)
    arrow.location = start + base_offset
    
    return arrow

def update_force_arrow(arrow_name, start_pos, end_pos):
    """
    Update an existing force arrow to point from start_pos to end_pos.
    
    Args:
        arrow_name: Name of the arrow object
        start_pos: New starting position
        end_pos: New ending position
    """
    arrow = bpy.data.objects.get(arrow_name)
    if not arrow:
        return None
    
    start = mathutils.Vector(start_pos)
    end = mathutils.Vector(end_pos)
    direction = end - start
    length = direction.length
    
    if length < 0.001:
        arrow.hide_render = True
        arrow.hide_viewport = True
        return arrow
    
    arrow.hide_render = False
    arrow.hide_viewport = False
    
    # Calculate rotation to align with direction
    up = mathutils.Vector((0, 0, 1))
    if abs(direction.normalized().dot(up)) > 0.999:
        rot_axis = mathutils.Vector((1, 0, 0))
        rot_angle = 0 if direction.z > 0 else math.pi
    else:
        rot_axis = up.cross(direction.normalized())
        rot_angle = math.acos(max(-1, min(1, up.dot(direction.normalized()))))
    
    rot_matrix = mathutils.Matrix.Rotation(rot_angle, 4, rot_axis)
    
    # Scale to match new length (original arrow was created with a specific length)
    # We'll recreate instead of scaling for accuracy
    arrow.matrix_world = rot_matrix
    arrow.location = start
    
    return arrow

def find_red_sphere_and_anchor():
    """
    Find the red sphere (vehicle attachment) and anchor (fixed block) positions.
    
    The structure in chrono is:
    - 'Chassis body' parent object has the vehicle world position
    - Red sphere shape 'shape_100981430595456' is a child of Chassis body
    - Anchor is an unnamed parent object '' at position (0, -0.15, 0) in Chrono coords
    
    Returns:
        Tuple of (red_sphere_world_pos, anchor_world_pos) or (None, None)
    """
    # Force complete scene update
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    
    # Known shape name for the red sphere
    RED_SPHERE_SHAPE = 'shape_100981430595456'
    # Fallback local offset if we can't find the shape directly
    RED_SPHERE_LOCAL_OFFSET = mathutils.Vector((-0.37264, -0.0014, 0.052))
    
    chassis_body = None
    anchor_body = None
    red_sphere_shape = None
    
    print("  Searching for force arrow endpoints...")
    
    # Get fresh reference to chrono_frame_objects collection
    cfo = bpy.data.collections.get('chrono_frame_objects')
    if not cfo:
        print("    ERROR: chrono_frame_objects collection not found")
        return None, None
    
    # Search directly in the chrono_frame_objects collection
    for obj in cfo.objects:
        name = obj.name if obj.name else ""
        
        # Find the red sphere shape directly
        if RED_SPHERE_SHAPE in name:
            red_sphere_shape = obj
        
        # Find Chassis body
        if name == 'Chassis body':
            chassis_body = obj
        
        # Find anchor - unnamed parent object (first one with 'Object' in name at y=-0.15)
        elif (name == '' or name == 'Object') and obj.parent is None:
            pos = obj.location
            # The anchor should be at y=-0.15 (Chrono coords)
            if abs(pos.y + 0.15) < 0.1:
                anchor_body = obj
    
    # Fallback anchor search
    if not anchor_body:
        for obj in cfo.objects:
            if (obj.name == '' or obj.name.startswith('Object')) and obj.parent is None:
                anchor_body = obj
                break
    
    if not chassis_body:
        print("    ERROR: Could not find 'Chassis body' object")
        return None, None
    
    if not anchor_body:
        print("    ERROR: Could not find anchor object")
        return None, None
    
    # Get red sphere world position
    if red_sphere_shape:
        # The shape is a child of chassis_body, compute its world position
        # World pos = parent_world_matrix @ child_local_pos
        parent_matrix = chassis_body.matrix_world
        child_local = red_sphere_shape.location
        red_sphere_world = parent_matrix @ child_local
        print(f"    Found red sphere shape: {red_sphere_shape.name}, local={child_local}")
    else:
        # Fallback: compute from chassis position and hardcoded offset
        print(f"    Red sphere shape not found, using hardcoded offset")
        chassis_pos = chassis_body.location.copy()
        chassis_rot = chassis_body.rotation_quaternion.copy()
        red_sphere_world = chassis_pos + (chassis_rot @ RED_SPHERE_LOCAL_OFFSET)
    
    # Get anchor position
    anchor_pos = anchor_body.location.copy()
    
    print(f"  Force arrow endpoints:")
    print(f"    Red sphere world: {red_sphere_world}")
    print(f"    Anchor: {anchor_pos}")
    
    return red_sphere_world, anchor_pos


def find_force_endpoints(chrono_frame_objects, chrono_frame_assets=None):
    """
    Find the force arrow endpoints for the pull test visualization.
    
    Returns:
        Tuple of (vehicle_attachment_pos, anchor_pos) as world coordinates, or (None, None)
    """
    return find_red_sphere_and_anchor()


def get_force_arrow_endpoints(chrono_frame_objects):
    """
    Get the current world positions for the force arrow endpoints.
    
    Returns:
        Tuple of (start_pos, end_pos) for the arrow, or (None, None) if not found.
        Arrow points FROM vehicle attachment (red sphere) TO anchor (block).
    """
    return find_red_sphere_and_anchor()

def unhide_all_collections():
    """Ensure collections are visible and included in the active view layer."""
    def walk_layer(layer):
        layer.exclude = False
        if hasattr(layer, "hide_viewport"):
            layer.hide_viewport = False
        if hasattr(layer, "hide_render"):
            layer.hide_render = False
        for child in layer.children:
            walk_layer(child)
    walk_layer(bpy.context.view_layer.layer_collection)

    for collection in bpy.data.collections:
        if hasattr(collection, "hide_viewport"):
            collection.hide_viewport = False
        if hasattr(collection, "hide_render"):
            collection.hide_render = False

    for obj in bpy.data.objects:
        if hasattr(obj, "hide_viewport"):
            obj.hide_viewport = False
        if hasattr(obj, "hide_render"):
            obj.hide_render = False

def compute_scene_bounds(target_collections=None, extra_points=None):
    """Compute bounds for camera framing from collections and optional points."""
    object_found = False
    min_co = mathutils.Vector((float('inf'), float('inf'), float('inf')))
    max_co = mathutils.Vector((float('-inf'), float('-inf'), float('-inf')))

    if target_collections:
        for collection in target_collections:
            if not collection:
                continue
            for obj in collection.objects:
                if obj.type != 'MESH':
                    continue
                if obj.name.startswith("ParticleHost") or obj.name.startswith("ParticleInstance"):
                    continue
                object_found = True
                for v in obj.bound_box:
                    world_co = obj.matrix_world @ mathutils.Vector(v)
                    min_co = mathutils.Vector(map(min, min_co, world_co))
                    max_co = mathutils.Vector(map(max, max_co, world_co))

    if extra_points:
        for p in extra_points:
            pt = mathutils.Vector(p)
            min_co = mathutils.Vector(map(min, min_co, pt))
            max_co = mathutils.Vector(map(max, max_co, pt))
            object_found = True

    if not object_found:
        center = mathutils.Vector((0, 0, 0))
        return {
            'center': center,
            'max_dim': 1.0,
            'radius': 1.0,
            'corners': []
        }

    center = (min_co + max_co) / 2
    size = max_co - min_co
    max_dim = max(size.x, size.y, size.z, 0.1)
    radius = 0.5 * size.length
    corners = [
        mathutils.Vector((x, y, z))
        for x in (min_co.x, max_co.x)
        for y in (min_co.y, max_co.y)
        for z in (min_co.z, max_co.z)
    ]

    return {
        'center': center,
        'max_dim': max_dim,
        'radius': max(radius, 0.1),
        'corners': corners
    }

def compute_centroid(points):
    if not points:
        return mathutils.Vector((0.0, 0.0, 0.0))
    total = mathutils.Vector((0.0, 0.0, 0.0))
    for p in points:
        total += mathutils.Vector(p)
    return total / len(points)

def compute_bounds(points):
    if not points:
        return None
    min_co = mathutils.Vector((float('inf'), float('inf'), float('inf')))
    max_co = mathutils.Vector((float('-inf'), float('-inf'), float('-inf')))
    for p in points:
        pt = mathutils.Vector(p)
        min_co = mathutils.Vector(map(min, min_co, pt))
        max_co = mathutils.Vector(map(max, max_co, pt))
    return min_co, max_co

def rotate_vector(vec, axis, angle_rad):
    if vec.length == 0 or axis.length == 0:
        return vec
    rot = mathutils.Matrix.Rotation(angle_rad, 3, axis.normalized())
    return rot @ vec

def read_fsi_body_states(fsi_folder, frame_index):
    """Read FSI body positions/quaternions for a frame index."""
    if not fsi_folder or not os.path.isdir(fsi_folder):
        return []
    states = []
    paths = sorted(
        p for p in os.listdir(fsi_folder)
        if p.lower().startswith("fsi_body") and p.lower().endswith(".csv")
    )
    for name in paths:
        path = os.path.join(fsi_folder, name)
        try:
            with open(path) as f:
                header = next(f, None)
                if header is None:
                    continue
                rows = list(f)
        except OSError:
            continue
        if not rows:
            continue
        idx = min(max(frame_index, 0), len(rows) - 1)
        row = rows[idx].strip().split(',')
        if len(row) < 8:
            continue
        try:
            pos = mathutils.Vector((float(row[1]), float(row[2]), float(row[3])))
            quat = mathutils.Quaternion((float(row[4]), float(row[5]), float(row[6]), float(row[7])))
        except ValueError:
            continue
        states.append({'pos': pos, 'quat': quat})
    return states

def should_transform_bce(points, fsi_states):
    """Heuristic to decide if BCE points need body-space transforms."""
    if not points or not fsi_states:
        return False
    bounds = compute_bounds(points)
    if not bounds:
        return False
    min_co, max_co = bounds
    extent = max((max_co - min_co).length, 1e-6)
    bce_center = compute_centroid(points)
    fsi_center = compute_centroid([s['pos'] for s in fsi_states])
    center_delta = (bce_center - fsi_center).length
    return center_delta > max(0.1, 0.5 * extent)

def transform_bce_points(points, fsi_states):
    """Transform local BCE points into world space using FSI body poses."""
    if not points or not fsi_states:
        return points
    transformed = []
    for state in fsi_states:
        quat = state['quat']
        try:
            quat.normalize()
        except Exception:
            pass
        rot = quat.to_matrix()
        pos = state['pos']
        for p in points:
            local = mathutils.Vector(p)
            transformed.append(pos + rot @ local)
    return transformed

def get_chrono_camera_reference(target_collections=None, extra_points=None):
    """Return chrono camera reference basis and settings."""
    chrono_cam = bpy.data.objects.get("default_camera")
    if not chrono_cam:
        cam_collection = bpy.data.collections.get("chrono_cameras")
        if cam_collection:
            for obj in cam_collection.objects:
                if obj.type == 'CAMERA':
                    chrono_cam = obj
                    break
    if not chrono_cam:
        return None

    bounds = compute_scene_bounds(target_collections, extra_points=extra_points)
    center = bounds['center']
    cam_loc = chrono_cam.matrix_world.translation
    forward = (center - cam_loc).normalized()
    if CHRONO_ALIGN_UP:
        up = mathutils.Vector((0.0, 0.0, 1.0))
        if abs(forward.dot(up)) > 0.98:
            up = mathutils.Vector((0.0, 1.0, 0.0))
        right = forward.cross(up).normalized()
        up = right.cross(forward).normalized()
    else:
        cam_quat = chrono_cam.matrix_world.to_quaternion()
        right = (cam_quat @ mathutils.Vector((1.0, 0.0, 0.0))).normalized()
        up = (cam_quat @ mathutils.Vector((0.0, 1.0, 0.0))).normalized()
        right = forward.cross(up).normalized()
        up = right.cross(forward).normalized()

    return {
        'camera': chrono_cam,
        'center': center,
        'view_dir': (cam_loc - center).normalized(),
        'forward': forward,
        'right': right,
        'up': up,
        'lens': chrono_cam.data.lens if chrono_cam.data else CAMERA_LENS,
        'location': cam_loc
    }

def get_view_direction(camera_type, frame, total_frames, chrono_ref=None):
    """Return a view direction vector (from center to camera).
    
    Note: These directions are in Blender coordinates after the Chrono-to-Blender transform.
    The transform maps Chrono (x, y, z) to Blender (x, -z, y), meaning:
      - Chrono +X (vehicle forward) → Blender +X
      - Chrono +Y (vehicle left/right) → Blender +Z  
      - Chrono +Z (up) → Blender -Y
    """
    if chrono_ref and CAMERA_USE_CHRONO_REFERENCE:
        iso_dir = chrono_ref['view_dir']
        if camera_type == "ISOMETRIC":
            return iso_dir
        if camera_type == "SIDE":
            return rotate_vector(iso_dir, chrono_ref['up'], math.radians(90))
        if camera_type == "FRONT":
            return rotate_vector(iso_dir, chrono_ref['up'], math.radians(-90))
        if camera_type == "TOP":
            return rotate_vector(iso_dir, chrono_ref['right'], math.radians(-90))
    
    # View directions in Blender coordinates (after Chrono-to-Blender transform)
    # These assume the vehicle drives along +X, with -Y as "up"
    if camera_type == "ISOMETRIC":
        # Looking from front-right-above: Chrono (1, 1, 0.7) → Blender (1, -0.7, 1)
        return mathutils.Vector((1.0, -0.7, 1.0))
    if camera_type == "SIDE":
        # Looking from the side (Chrono +Y → Blender +Z)
        return mathutils.Vector((0.0, -0.1, 1.0))
    if camera_type == "FRONT":
        # Looking from front (Chrono +X → Blender +X)
        return mathutils.Vector((1.0, -0.05, 0.0))
    if camera_type == "TOP":
        # Looking from above (Chrono +Z → Blender -Y)
        return mathutils.Vector((0.0, -1.0, 0.0))
    if camera_type == "REVOLVING":
        denom = max(1, total_frames)
        angle = 2 * math.pi * REVOLVING_ROTATION_SPEED * (frame / denom)
        # Revolve in XZ plane (the horizontal plane in Blender after transform)
        return mathutils.Vector((math.cos(angle), -REVOLVING_PITCH, math.sin(angle)))
    return mathutils.Vector((0.0, -1.0, 0.3))

def compute_camera_basis(view_dir):
    """Build a camera basis (forward, right, up) from a view direction.
    
    In Blender coordinates (after Chrono transform), "up" is -Y direction.
    """
    if view_dir.length == 0:
        view_dir = mathutils.Vector((0.0, -1.0, 0.0))
    forward = (-view_dir).normalized()
    # After Chrono-to-Blender transform, "up" is -Y (former Chrono Z)
    up_world = mathutils.Vector((0.0, -1.0, 0.0))
    if abs(forward.dot(up_world)) > 0.98:
        # Fallback if looking straight up/down
        up_world = mathutils.Vector((0.0, 0.0, 1.0))
    right = forward.cross(up_world).normalized()
    up = right.cross(forward).normalized()
    return forward, right, up

def set_camera_rotation_from_view(cam_obj, view_dir):
    """Align camera to look along view_dir with world-up stabilization."""
    forward, right, up = compute_camera_basis(view_dir)
    rot_mat = mathutils.Matrix((right, up, -forward)).transposed()
    cam_obj.rotation_euler = rot_mat.to_euler()

def rotate_points_z(points, angle_rad):
    if not points or abs(angle_rad) < 1e-8:
        return points
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    rotated = []
    for x, y, z in points:
        rotated.append((x * c - y * s, x * s + y * c, z))
    return rotated

def chrono_to_blender_coords(points):
    """
    Transform particle positions from Chrono's coordinate system to Blender's.
    
    Chrono uses a coordinate system that, when exported to Blender, has a 90-degree 
    rotation around the X-axis applied to all objects (quaternion = (0.707107, 0.707107, 0, 0)).
    
    This transformation applies the same rotation to particle data:
    (x, y, z) -> (x, -z, y)
    
    Args:
        points: List of (x, y, z) tuples in Chrono coordinates
        
    Returns:
        List of (x, y, z) tuples in Blender coordinates
    """
    if not points:
        return points
    transformed = []
    for x, y, z in points:
        # 90-degree rotation around X-axis: (x, y, z) -> (x, -z, y)
        transformed.append((x, -z, y))
    return transformed

# Global flag for coordinate transformation (set from CLI args)
USE_CHRONO_COORDS_TRANSFORM = True

def make_particle_transform(bce_transform_fn=None, yaw_rad=0.0):
    """
    Create a transform function for particle coordinates.
    
    Args:
        bce_transform_fn: Optional BCE transform function (body-local to world)
        yaw_rad: Optional scene yaw rotation in radians
        
    Returns:
        A transform function that takes points and returns transformed points
    """
    def transform(pts):
        # Apply BCE transform if provided (body-local to Chrono world)
        if bce_transform_fn:
            pts = bce_transform_fn(pts)
        
        # Apply Chrono-to-Blender coordinate transform if enabled
        if USE_CHRONO_COORDS_TRANSFORM:
            pts = chrono_to_blender_coords(pts)
        
        # Apply scene yaw rotation
        if abs(yaw_rad) > 1e-8:
            pts = rotate_points_z(pts, yaw_rad)
        
        return pts
    
    return transform

def apply_scene_yaw_to_collections(collections, angle_rad):
    if abs(angle_rad) < 1e-8:
        return
    rot = mathutils.Matrix.Rotation(angle_rad, 4, 'Z')
    for collection in collections:
        if not collection:
            continue
        for obj in collection.objects:
            if obj.type == 'CAMERA':
                continue
            if obj.name.startswith("ParticleHost") or obj.name.startswith("ParticleInstance"):
                continue
            obj.matrix_world = rot @ obj.matrix_world

def fit_distance_to_bounds(bounds, camera, view_dir, use_sphere=False):
    """Compute camera distance to fit bounds in view."""
    if not bounds or not bounds.get('center'):
        return CAMERA_DEFAULT_DISTANCE

    if use_sphere or not bounds.get('corners'):
        fov = min(camera.angle_x, camera.angle_y)
        if fov <= 0:
            return CAMERA_DEFAULT_DISTANCE
        return max(CAMERA_DEFAULT_DISTANCE, bounds['radius'] / math.tan(fov / 2) * CAMERA_PADDING)

    _, right, up = compute_camera_basis(view_dir)
    max_x = 0.0
    max_y = 0.0
    center = bounds['center']
    for corner in bounds['corners']:
        delta = corner - center
        max_x = max(max_x, abs(delta.dot(right)))
        max_y = max(max_y, abs(delta.dot(up)))

    fov_x = camera.angle_x
    fov_y = camera.angle_y
    dist_x = max_x / math.tan(fov_x / 2) if fov_x > 0 else CAMERA_DEFAULT_DISTANCE
    dist_y = max_y / math.tan(fov_y / 2) if fov_y > 0 else CAMERA_DEFAULT_DISTANCE
    return max(CAMERA_DEFAULT_DISTANCE, max(dist_x, dist_y) * CAMERA_PADDING)

def setup_camera(camera_type, target_collections=None, frame=0, total_frames=100,
                 camera_data=None, extra_points=None, stable_distance=False):
    """
    Set up camera based on specified type with auto-fit framing.
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

    chrono_ref = get_chrono_camera_reference(target_collections, extra_points=extra_points)
    if camera_type == "CHRONO" and chrono_ref:
        bounds = compute_scene_bounds(target_collections, extra_points=extra_points)
        center = bounds['center']
        if CAMERA_LENS_OVERRIDE is not None:
            cam_data.lens = CAMERA_LENS_OVERRIDE
        else:
            cam_data.lens = chrono_ref['lens']

        view_dir = (chrono_ref['location'] - center).normalized()
        distance = fit_distance_to_bounds(bounds, cam_data, view_dir, use_sphere=stable_distance)
        cam_obj.location = center + view_dir * distance
        direction = center - cam_obj.location
        if CHRONO_ALIGN_UP:
            set_camera_rotation_from_view(cam_obj, view_dir)
        else:
            cam_obj.rotation_euler = chrono_ref['camera'].matrix_world.to_euler()
        cam_data.clip_start = max(0.1, distance / 1000.0)
        cam_data.clip_end = max(1000.0, distance * 10.0)
        print(f"Using Chrono camera reference '{chrono_ref['camera'].name}'.")
        print(f"Chrono camera view_dir: {view_dir}, center: {center}, distance: {distance:.3f}")
        return cam_obj, {
            'center': bounds['center'],
            'max_dim': bounds['max_dim'],
            'radius': bounds['radius'],
            'corners': bounds['corners'],
            'distance': distance
        }
    if camera_type == "CHRONO":
        print("Warning: Chrono camera not found, falling back to auto-fit.")

    if camera_data:
        bounds = camera_data
    else:
        bounds = compute_scene_bounds(target_collections, extra_points=extra_points)

    view_dir = get_view_direction(camera_type, frame, total_frames, chrono_ref=chrono_ref)
    if chrono_ref and CAMERA_USE_CHRONO_REFERENCE:
        print(f"Chrono camera basis: view_dir={chrono_ref['view_dir']}, up={chrono_ref['up']}, right={chrono_ref['right']}")
        print(f"View direction for {camera_type}: {view_dir}")
    distance = fit_distance_to_bounds(
        bounds,
        cam_data,
        view_dir,
        use_sphere=stable_distance or camera_type == "REVOLVING"
    )
    if chrono_ref and CAMERA_USE_CHRONO_REFERENCE:
        chrono_distance = (chrono_ref['location'] - bounds['center']).length
        distance = max(distance, chrono_distance)

    center = bounds['center']
    if view_dir.length == 0:
        view_dir = mathutils.Vector((0.0, -1.0, 0.0))
    cam_obj.location = center + view_dir.normalized() * distance

    direction = center - cam_obj.location
    set_camera_rotation_from_view(cam_obj, view_dir)
    
    if CAMERA_LENS_OVERRIDE is not None:
        cam_data.lens = CAMERA_LENS_OVERRIDE
    elif chrono_ref and CAMERA_USE_CHRONO_REFERENCE:
        cam_data.lens = chrono_ref['lens']
    else:
        cam_data.lens = CAMERA_LENS
    cam_data.clip_start = max(0.1, distance / 1000.0)
    cam_data.clip_end = max(1000.0, distance * 10.0)

    print(f"Camera positioned for '{camera_type}'. Center: {center}, Distance: {distance:.3f}")

    return cam_obj, {
        'center': center,
        'max_dim': bounds['max_dim'],
        'radius': bounds['radius'],
        'corners': bounds['corners'],
        'distance': distance
    }

def setup_lighting(light_setup="THREE_POINT", camera_type=None, background_type=None):
    """
    Set up lighting based on specified setup
    Args:
        light_setup: The type of lighting to set up (e.g., "THREE_POINT")
        camera_type: The current camera view type (e.g., "SIDE"), used to adjust lighting.
        background_type: The background type to set up
    """
    # Delete existing lights
    for obj in bpy.data.objects:
        if obj.type == 'LIGHT':
            bpy.data.objects.remove(obj)
    
    # Brighter lights for better visibility
    if light_setup == "THREE_POINT":
        # Define default rotations
        key_rot_euler = mathutils.Euler((math.radians(45), 0, math.radians(45)), 'XYZ')
        fill_rot_euler = mathutils.Euler((math.radians(45), 0, math.radians(-45)), 'XYZ')
        back_rot_euler = mathutils.Euler((math.radians(-45), 0, math.radians(180)), 'XYZ')
        ambient_rot_euler = mathutils.Euler((0, 0, 0), 'XYZ')

        if camera_type == "SIDE":
            print("Adjusting THREE_POINT lighting for SIDE view: swapping key and fill Z-rotation component.")
            # Swap the Z-axis rotation component for key and fill lights
            key_rot_euler = mathutils.Euler((math.radians(45), 0, math.radians(-45)), 'XYZ') # Key light Z rot becomes -45 deg
            fill_rot_euler = mathutils.Euler((math.radians(45), 0, math.radians(45)), 'XYZ')  # Fill light Z rot becomes 45 deg
            # Back light and ambient light rotations remain the same as default for THREE_POINT
        elif camera_type == "FRONT":
            print("Adjusting THREE_POINT lighting for FRONT view: optimizing for front-facing camera.")
            # Adjust lighting for front view - keep key light from front-left, fill from front-right
            key_rot_euler = mathutils.Euler((math.radians(30), 0, math.radians(30)), 'XYZ')   # Shallower angle from front-left
            fill_rot_euler = mathutils.Euler((math.radians(30), 0, math.radians(-30)), 'XYZ') # Shallower angle from front-right
            back_rot_euler = mathutils.Euler((math.radians(-30), 0, math.radians(180)), 'XYZ') # Back light at shallower angle

        # Key light
        key_data = bpy.data.lights.new(name="Key", type='SUN')
        key_data.energy = 3.0  # Increased brightness
        key_obj = bpy.data.objects.new(name="Key", object_data=key_data)
        bpy.context.scene.collection.objects.link(key_obj)
        key_obj.rotation_euler = key_rot_euler
        
        # Fill light
        fill_data = bpy.data.lights.new(name="Fill", type='SUN')
        fill_data.energy = 2.0  # Increased brightness
        fill_obj = bpy.data.objects.new(name="Fill", object_data=fill_data)
        bpy.context.scene.collection.objects.link(fill_obj)
        fill_obj.rotation_euler = fill_rot_euler
        
        # Back light
        back_data = bpy.data.lights.new(name="Back", type='SUN')
        back_data.energy = 2.5  # Increased brightness
        back_obj = bpy.data.objects.new(name="Back", object_data=back_data)
        bpy.context.scene.collection.objects.link(back_obj)
        back_obj.rotation_euler = back_rot_euler
        
        # Add ambient light
        ambient_data = bpy.data.lights.new(name="Ambient", type='SUN')
        ambient_data.energy = 0.5
        ambient_obj = bpy.data.objects.new(name="Ambient", object_data=ambient_data)
        bpy.context.scene.collection.objects.link(ambient_obj)
        ambient_obj.rotation_euler = ambient_rot_euler
    
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
    
    # Set up background
    setup_background(background_type)
    # setup_background()

def setup_background(background_type=None):
    """Set up background based on settings"""
    # Use provided background_type or fall back to global constant
    bg_type = background_type if background_type is not None else BACKGROUND_TYPE
    
    # Ensure we have a world
    world = bpy.context.scene.world
    if not world:
        world = bpy.data.worlds.new("World")
        bpy.context.scene.world = world
    
    # Set up world nodes
    world.use_nodes = True
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    
    # Clear existing nodes
    nodes.clear()
    
    # Add output node
    output = nodes.new(type='ShaderNodeOutputWorld')
    output.location = (600, 0)
    
    if bg_type == "TRANSPARENT":
        # Transparent background: set alpha to 0, color doesn't matter
        background = nodes.new(type='ShaderNodeBackground')
        background.location = (400, 0)
        background.inputs[0].default_value = (0.0, 0.0, 0.0, 0.0)  # Black, fully transparent
        background.inputs[1].default_value = 1.0
        links.new(background.outputs[0], output.inputs[0])
        print("Background set up as TRANSPARENT (alpha=0)")
        return
    
    if not USE_BACKGROUND_TEXTURE or bg_type == "NONE":
        # Simple solid background
        background = nodes.new(type='ShaderNodeBackground')
        background.location = (400, 0)
        background.inputs[0].default_value = (0.05, 0.05, 0.05, 1.0)  # Dark gray
        background.inputs[1].default_value = 1.0  # Strength
        links.new(background.outputs[0], output.inputs[0])
        
    elif bg_type == "GRADIENT":
        # Create a gradient background
        background = nodes.new(type='ShaderNodeBackground')
        background.location = (400, 0)
        
        # Add gradient texture
        gradient = nodes.new(type='ShaderNodeTexGradient')
        gradient.location = (0, 0)
        gradient.gradient_type = 'LINEAR'
        
        # Add mapping to control gradient orientation
        mapping = nodes.new(type='ShaderNodeMapping')
        mapping.location = (-200, 0)
        # Rotate to make gradient vertical (bottom to top)
        mapping.inputs[2].default_value = (0, 0, -1.57)  # Rotate around X axis
        
        # Add texture coordinate
        tex_coord = nodes.new(type='ShaderNodeTexCoord')
        tex_coord.location = (-400, 0)
        
        # Add color ramp for gradient colors
        color_ramp = nodes.new(type='ShaderNodeValToRGB')
        color_ramp.location = (200, 0)
        # Set gradient colors
        color_ramp.color_ramp.elements[0].color = BACKGROUND_COLOR1  # Bottom color
        color_ramp.color_ramp.elements[1].color = BACKGROUND_COLOR2  # Top color
        
        # Link nodes
        links.new(tex_coord.outputs['Generated'], mapping.inputs[0])
        links.new(mapping.outputs[0], gradient.inputs[0])
        links.new(gradient.outputs[0], color_ramp.inputs[0])
        links.new(color_ramp.outputs[0], background.inputs[0])
        links.new(background.outputs[0], output.inputs[0])
        
    elif bg_type == "SKY":
        # Create a sky texture
        background = nodes.new(type='ShaderNodeBackground')
        background.location = (400, 0)
        
        # Add sky texture
        sky = nodes.new(type='ShaderNodeTexSky')
        sky.location = (200, 0)
        sky.sky_type = 'HOSEK_WILKIE'
        sky.turbidity = 3.0
        sky.ground_albedo = 0.3
        
        # Link nodes
        links.new(sky.outputs[0], background.inputs[0])
        links.new(background.outputs[0], output.inputs[0])
    
    elif bg_type == "WHITE":
        # Create a white background
        background = nodes.new(type='ShaderNodeBackground')
        background.location = (400, 0)
        background.inputs[0].default_value = (1.0, 1.0, 1.0, 1.0)  # Pure white
        background.inputs[1].default_value = 1.0  # Strength
        links.new(background.outputs[0], output.inputs[0])
    
    print(f"Background set up with type: {bg_type if USE_BACKGROUND_TEXTURE else 'NONE'}")

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
            # Check if we're in a headless environment
            is_headless = not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY')
            if is_headless:
                print("Setting up headless GPU rendering")
                # Force Cycles to use CUDA or OptiX
                bpy.context.preferences.addons['cycles'].preferences.compute_device_type = 'CUDA'
                
                # Enable all CUDA devices
                bpy.context.preferences.addons['cycles'].preferences.get_devices()
                for device in bpy.context.preferences.addons['cycles'].preferences.devices:
                    if device.type == 'CUDA':
                        device.use = True
                        print(f"Enabled CUDA device: {device.name}")
                
                # Set device to GPU
                bpy.context.scene.cycles.device = 'GPU'
                print("Headless GPU rendering configured")
            else:
                # More conservative GPU settings for interactive mode
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
    scene.cycles.samples = 96
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
    # Check if TRANSPARENT background is requested
    if bpy.context.scene.get("background_type", None) == "TRANSPARENT":
        scene.render.film_transparent = True
    else:
        scene.render.film_transparent = False

def apply_png_dpi(path, dpi):
    """Inject a pHYs chunk to set PNG DPI metadata."""
    if dpi <= 0:
        return
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        return
    if not data.startswith(b"\x89PNG\r\n\x1a\n"):
        return
    try:
        import struct
        import zlib
        ppm = int(round(dpi / 0.0254))
        chunk_type = b"pHYs"
        chunk_data = struct.pack(">IIB", ppm, ppm, 1)
        chunk_crc = zlib.crc32(chunk_type + chunk_data) & 0xffffffff
        chunk = struct.pack(">I", len(chunk_data)) + chunk_type + chunk_data + struct.pack(">I", chunk_crc)
        # Insert after IHDR
        offset = 8
        ihdr_len = struct.unpack(">I", data[offset:offset+4])[0]
        offset += 8 + ihdr_len + 4
        data = data[:offset] + chunk + data[offset:]
        with open(path, "wb") as f:
            f.write(data)
    except Exception:
        return

def cleanup_resources():
    """Clean up resources to prevent memory leaks"""
    remove_particle_handlers()
    
    # Force garbage collection
    gc.collect()

def find_particle_prefixes_for_frame(terrain_folder, frame, keywords):
    """Return prefixes for files that match keyword(s) and frame number."""
    prefixes = set()
    pattern = re.compile(r"^(?P<prefix>.*?)(?P<frame>\\d+)\\.csv$", re.IGNORECASE)
    try:
        for name in os.listdir(terrain_folder):
            match = pattern.match(name)
            if not match:
                continue
            if int(match.group("frame")) != frame:
                continue
            prefix = match.group("prefix")
            lower = prefix.lower()
            if any(k in lower for k in keywords):
                prefixes.add(prefix)
    except FileNotFoundError:
        return []
    return sorted(prefixes)

def remove_particle_handlers():
    """Remove only handlers created by this script."""
    handlers_to_remove = [
        h for h in bpy.app.handlers.frame_change_post
        if hasattr(h, '__name__') and 'particle_handler' in h.__name__
    ]
    if handlers_to_remove:
        print(f"Removing {len(handlers_to_remove)} particle handlers")
    for h in handlers_to_remove:
        bpy.app.handlers.frame_change_post.remove(h)
    print(f"Total handlers after removal: {len(bpy.app.handlers.frame_change_post)}")

def ensure_blender4_assets_compat(assets_path):
    """Patch Blender assets for Blender 4 compatibility if needed."""
    try:
        with open(assets_path, "r", encoding="utf-8") as f:
            text = f.read()
    except OSError:
        print(f"WARNING: Could not read assets file: {assets_path}")
        return

    if "shade_auto_smooth" not in text:
        return

    helper = (
        "def apply_auto_smooth(obj, angle=0.8):\n"
        "    if not obj or not getattr(obj, \"data\", None):\n"
        "        return\n"
        "    obj.data.use_auto_smooth = True\n"
        "    obj.data.auto_smooth_angle = angle\n"
        "    try:\n"
        "        obj.data.polygons.foreach_set(\"use_smooth\", [True] * len(obj.data.polygons))\n"
        "    except Exception:\n"
        "        pass\n\n"
    )

    if "def apply_auto_smooth" not in text:
        insert_at = text.find("\n\n")
        if insert_at == -1:
            text = helper + text
        else:
            text = text[:insert_at + 2] + helper + text[insert_at + 2:]

    old = "with bpy.context.temp_override(selected_editable_objects=[new_object]):\n    bpy.ops.object.shade_auto_smooth(angle=0.8)"
    text = text.replace(old, "apply_auto_smooth(new_object, angle=0.8)")
    text = text.replace("bpy.ops.object.shade_auto_smooth(angle=0.8)", "apply_auto_smooth(new_object, angle=0.8)")

    try:
        with open(assets_path, "w", encoding="utf-8") as f:
            f.write(text)
        print("Patched assets file for Blender 4 compatibility (auto smooth).")
    except OSError:
        print(f"WARNING: Could not update assets file: {assets_path}")

def hide_all_except_tires():
    """TEMPORARY HACK: Hide all objects except tires"""
    print("TEMPORARY: Hiding all objects except tires...")
    
    # Get the chrono_frame_objects collection
    chrono_frame_objects = bpy.data.collections.get('chrono_frame_objects')
    
    if chrono_frame_objects:
        for obj in chrono_frame_objects.objects:
            # Check if object name contains tire-related keywords or the specific shape name
            is_tire = (
                "tire" in obj.name.lower() or
                "wheel" in obj.name.lower() or
                "shape_107857183842752" in obj.name or
                obj.name in ["Object", "Object.001"]  # Fallback names you mentioned
            )
            
            if is_tire:
                print(f"  Keeping tire object visible: {obj.name}")
                obj.hide_viewport = False
                obj.hide_render = False
            else:
                print(f"  Hiding non-tire object: {obj.name}")
                obj.hide_viewport = True
                obj.hide_render = True
    
    # Also hide objects in other collections if needed
    for collection_name in ['chrono_frame_assets', 'chrono_assets']:
        collection = bpy.data.collections.get(collection_name)
        if collection:
            for obj in collection.objects:
                is_tire = (
                    "tire" in obj.name.lower() or
                    "wheel" in obj.name.lower() or
                    "shape_107857183842752" in obj.name
                )
                
                if not is_tire:
                    print(f"  Hiding {collection_name} object: {obj.name}")
                    obj.hide_viewport = True
                    obj.hide_render = True

def main():
    global CAMERA_PADDING, CAMERA_LENS, CAMERA_LENS_OVERRIDE
    global CHRONO_ALIGN_UP, CAMERA_USE_CHRONO_REFERENCE
    global SCENE_YAW_DEG
    global USE_CHRONO_COORDS_TRANSFORM

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Render Chrono simulation in Blender")
    parser.add_argument("--assets", required=True, help="Path to the .assets.py file")
    parser.add_argument("--output", required=True, help="Output directory for rendered images")
    parser.add_argument("--camera", dest="camera", default="ISOMETRIC", 
                        choices=["ISOMETRIC", "SIDE", "TOP", "REVOLVING", "FRONT", "CHRONO"],
                        help="Camera type")
    parser.add_argument("--view", dest="camera", 
                        choices=["ISOMETRIC", "SIDE", "TOP", "REVOLVING", "FRONT", "CHRONO"],
                        help=argparse.SUPPRESS)
    parser.add_argument("--lighting", default="THREE_POINT", 
                        choices=["THREE_POINT", "STUDIO", "OUTDOOR"],
                        help="Lighting setup")
    parser.add_argument("--background", default="GRADIENT", 
                        choices=["GRADIENT", "SKY", "NONE", "WHITE", "TRANSPARENT"],
                        help="Background type")
    parser.add_argument("--resolution_x", type=int, default=1920, help="Horizontal resolution")
    parser.add_argument("--resolution_y", type=int, default=1080, help="Vertical resolution")
    parser.add_argument("--frame_start", type=int, default=1, help="Start frame")
    parser.add_argument("--frame_end", type=int, default=100, help="End frame")
    parser.add_argument("--debug", action="store_true", help="Add debug objects to scene") 
    parser.add_argument("--cpu", action="store_true", help="Force CPU rendering instead of GPU")
    parser.add_argument("--camera_padding", type=float, default=CAMERA_PADDING,
                        help="Extra margin when fitting bounds (lower = zoom in)")
    parser.add_argument("--camera_lens", type=float, default=None,
                        help="Camera lens in mm (higher = zoom in). Only overrides when set.")
    parser.add_argument("--chrono_align_up", action=argparse.BooleanOptionalAction, default=CHRONO_ALIGN_UP,
                        help="Align Chrono camera roll to world up (default: true)")
    parser.add_argument("--use_chrono_reference", action=argparse.BooleanOptionalAction, default=CAMERA_USE_CHRONO_REFERENCE,
                        help="Use Chrono camera pose to define view directions and lens (default: true)")
    parser.add_argument("--scene_yaw_deg", type=float, default=SCENE_YAW_DEG,
                        help="Rotate imported scene and particles around Z (degrees)")
    
    # Terrain-related arguments
    parser.add_argument("--terrain_path", help="Path to folder containing terrain files")
    parser.add_argument("--terrain_type", choices=["OBJ", "PARTICLES"], default="OBJ", 
                        help="Type of terrain representation (OBJ meshes or particle CSV files)")
    parser.add_argument("--terrain_prefix", default="fluid", help="Prefix for terrain filenames")
    parser.add_argument("--terrain_suffix", default="_surface.obj", help="Suffix for terrain OBJ filenames (only used with --terrain_type OBJ)")
    parser.add_argument("--particle_radius", type=float, default=0.0025, help="Radius for terrain particles (only used with --terrain_type PARTICLES)")
    parser.add_argument("--wheel_bce_radius", type=float, default=WHEEL_BCE_RADIUS_DEFAULT,
                        help="Radius for wheel BCE particles (only used with --terrain_type PARTICLES)")
    
    # Cross-section filtering (Y-slice)
    parser.add_argument("--y_min", type=float, help="Keep particles with y >= y_min (cross-section lower bound)")
    parser.add_argument("--y_max", type=float, help="Keep particles with y <= y_max (cross-section upper bound)")
    parser.add_argument("--y_tol", type=float, default=0.0, help="Tolerance added to Y bounds (extends range by ±y_tol)")
    
    # Coordinate transformation
    parser.add_argument("--chrono_coords", action="store_true", default=True,
                        help="Apply Chrono-to-Blender coordinate transform to particles (default: enabled)")
    parser.add_argument("--no_chrono_coords", dest="chrono_coords", action="store_false",
                        help="Disable Chrono-to-Blender coordinate transform (use raw particle coordinates)")
    
    # Camera stability options
    parser.add_argument("--stable_camera", action="store_true", help="Freeze camera bounds to the first frame")
    
    # Force arrow visualization (enabled by default for pull test simulations)
    parser.add_argument("--no_force_arrow", action="store_true", 
                        help="Disable the force arrow visualization")

    # Extract arguments after "--"
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    
    args = parser.parse_args(argv)
    CAMERA_PADDING = args.camera_padding
    if args.camera_lens is not None:
        CAMERA_LENS = args.camera_lens
        CAMERA_LENS_OVERRIDE = args.camera_lens
    else:
        CAMERA_LENS_OVERRIDE = None
    CHRONO_ALIGN_UP = args.chrono_align_up
    CAMERA_USE_CHRONO_REFERENCE = args.use_chrono_reference
    SCENE_YAW_DEG = args.scene_yaw_deg
    USE_CHRONO_COORDS_TRANSFORM = args.chrono_coords
    
    if USE_CHRONO_COORDS_TRANSFORM:
        print("Chrono-to-Blender coordinate transform enabled (particles: x,y,z -> x,-z,y)")
    else:
        print("Chrono-to-Blender coordinate transform disabled (using raw particle coordinates)")
    
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
    ensure_blender4_assets_compat(args.assets)
    chrono_import.read_chrono_simulation(bpy.context, args.assets, True, False)
    # unhide_all_collections()
    
    # Find the chrono collections for camera targeting
    chrono_frame_objects = bpy.data.collections.get('chrono_frame_objects')
    chrono_assets = bpy.data.collections.get('chrono_assets')
    chrono_frame_assets = bpy.data.collections.get('chrono_frame_assets')

    if SCENE_YAW_DEG:
        scene_yaw_rad = math.radians(SCENE_YAW_DEG)
        apply_scene_yaw_to_collections([chrono_assets, chrono_frame_assets], scene_yaw_rad)
    
    # Add debug objects if requested
    if args.debug:
        print("Adding debug objects to scene")
        add_debug_objects()
    
    # Set up lighting
    print(f"Setting up {args.lighting} lighting")
    setup_lighting(args.lighting, args.camera, args.background)
    
    # Store background type in scene for render settings
    bpy.context.scene["background_type"] = args.background
    
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
    target_collections = []
    if chrono_frame_objects and chrono_frame_objects.objects:
        target_collections.append(chrono_frame_objects)
    elif chrono_assets and chrono_assets.objects:
        target_collections.append(chrono_assets)
    if terrain_collection and args.terrain_type == "OBJ":
        target_collections.append(terrain_collection)
    
    # Track if we had to switch to CPU rendering
    switched_to_cpu = False
    
    stable_camera_bounds = None
    wheel_prefix_keywords = ["rigidbce"]
    fluid_material = create_particle_material(name="ParticleMaterial_Fluid", base_color=(0.3, 0.25, 0.2, 1.0))
    wheel_material = create_particle_material(name="ParticleMaterial_WheelBCE", base_color=WHEEL_BCE_COLOR)
    fsi_folder = None
    if args.terrain_path:
        fsi_candidate = os.path.join(os.path.dirname(args.terrain_path), "fsi")
        if os.path.isdir(fsi_candidate):
            fsi_folder = fsi_candidate
            print(f"Found FSI body data: {fsi_folder}")
    if args.stable_camera:
        print("Computing stable camera bounds from the first frame")
        bpy.context.scene.frame_set(args.frame_start)
        chrono_import.callback_post(bpy.context.scene)
        if SCENE_YAW_DEG:
            apply_scene_yaw_to_collections([chrono_frame_objects], math.radians(SCENE_YAW_DEG))

        terrain_positions = None
        if args.terrain_path and terrain_collection:
            if args.terrain_type == "OBJ":
                load_terrain_for_frame(
                    args.terrain_path, args.terrain_prefix, args.terrain_suffix,
                    args.frame_start, terrain_collection
                )
            else:
                # Create particle transform for fluid particles (no BCE transform needed)
                fluid_transform = make_particle_transform(yaw_rad=math.radians(SCENE_YAW_DEG))
                success, _, terrain_positions = load_terrain_particles_for_frame(
                    args.terrain_path, args.terrain_prefix, args.frame_start,
                    terrain_collection, args.particle_radius,
                    y_min=args.y_min, y_max=args.y_max, y_tol=args.y_tol,
                    clear_collection=True, material=fluid_material,
                    transform_fn=fluid_transform
                )
                if not success:
                    terrain_positions = None
                extra_positions = []
                extra_prefixes = find_particle_prefixes_for_frame(
                    args.terrain_path, args.frame_start, wheel_prefix_keywords
                )
                if not extra_prefixes:
                    fallback_prefix = "rigidBCE"
                    fallback_path = os.path.join(args.terrain_path, f"{fallback_prefix}{args.frame_start}.csv")
                    if os.path.exists(fallback_path):
                        extra_prefixes = [fallback_prefix]
                if extra_prefixes:
                    print(f"Loading rigid BCE particles (frame {args.frame_start}): {', '.join(extra_prefixes)}")
                fsi_states = read_fsi_body_states(fsi_folder, args.frame_start)
                bce_transform_fn = None
                if fsi_states:
                    def bce_transform_fn(points):
                        if should_transform_bce(points, fsi_states):
                            print("Applying FSI body transforms to BCE points.")
                            return transform_bce_points(points, fsi_states)
                        print("BCE points appear to already be in world space.")
                        return points
                # Create particle transform for BCE particles
                bce_particle_transform = make_particle_transform(bce_transform_fn=bce_transform_fn, yaw_rad=math.radians(SCENE_YAW_DEG))
                for prefix in extra_prefixes:
                    success, _, positions = load_terrain_particles_for_frame(
                        args.terrain_path, prefix, args.frame_start,
                        terrain_collection, args.wheel_bce_radius,
                        y_min=args.y_min, y_max=args.y_max, y_tol=args.y_tol,
                        clear_collection=False, material=wheel_material,
                        transform_fn=bce_particle_transform
                    )
                    if success and positions:
                        extra_positions.extend(positions)
                if extra_positions:
                    terrain_positions = (terrain_positions or []) + extra_positions
            bpy.context.view_layer.update()

        stable_camera_bounds = compute_scene_bounds(
            target_collections,
            extra_points=terrain_positions
        )

        if terrain_collection:
            for obj in terrain_collection.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
        remove_particle_handlers()

    # Set up force arrow (enabled by default for pull test visualization)
    force_arrow_enabled = not args.no_force_arrow
    force_arrow_color = (1.0, 0.4, 0.0, 1.0)  # Orange color for visibility
    force_arrow_radius = 0.015
    
    if force_arrow_enabled:
        print("Force arrow visualization enabled (use --no_force_arrow to disable)")
        
        # Load the first frame to detect force endpoints
        bpy.context.scene.frame_set(args.frame_start)
        chrono_import.callback_post(bpy.context.scene)
        
        # Try to find force endpoints
        vehicle_pos, anchor_pos = find_force_endpoints(chrono_frame_objects, chrono_frame_assets)
        
        if vehicle_pos and anchor_pos:
            print(f"  Creating initial force arrow...")
            force_arrow_obj = create_force_arrow(
                vehicle_pos, anchor_pos,
                name="PullForceArrow",
                shaft_radius=force_arrow_radius,
                head_radius=force_arrow_radius * 2.5,
                head_length=0.08,
                color=force_arrow_color,
                fixed_length=0.4  # Fixed arrow length regardless of distance
            )
            if force_arrow_obj:
                print(f"  Force arrow created successfully")
        else:
            print("  Warning: Could not detect force endpoints. Force arrow disabled.")
            force_arrow_enabled = False

    # Render each frame
    for frame_num_abs in range(args.frame_start, args.frame_end + 1):
        print(f"\nProcessing frame {frame_num_abs}")
        bpy.context.scene.frame_set(frame_num_abs)
        chrono_import.callback_post(bpy.context.scene)
        if SCENE_YAW_DEG:
            apply_scene_yaw_to_collections([chrono_frame_objects], math.radians(SCENE_YAW_DEG))
        
        current_frame_index_for_stable_path = frame_num_abs - args.frame_start
        terrain_positions = None

        if args.terrain_path and terrain_collection:
            remove_particle_handlers()
            if args.terrain_type == "OBJ":
                load_terrain_for_frame(
                    args.terrain_path, args.terrain_prefix, args.terrain_suffix, 
                    frame_num_abs, terrain_collection
                )
            else:  # PARTICLES
                # Create particle transform for fluid particles
                fluid_transform = make_particle_transform(yaw_rad=math.radians(SCENE_YAW_DEG))
                success, _, terrain_positions = load_terrain_particles_for_frame(
                    args.terrain_path, args.terrain_prefix, frame_num_abs,
                    terrain_collection, args.particle_radius,
                    y_min=args.y_min, y_max=args.y_max, y_tol=args.y_tol,
                    clear_collection=True, material=fluid_material,
                    transform_fn=fluid_transform
                )
                
                # Debug check for particle handler
                if success:
                    print(f"Registered {len(bpy.app.handlers.frame_change_post)} frame handlers")
                    # Force multiple frame updates to ensure particles are properly positioned
                    for _ in range(3):
                        bpy.context.scene.frame_set(frame_num_abs)
                else:
                    print("WARNING: No particles loaded for this frame")

                extra_positions = []
                extra_prefixes = find_particle_prefixes_for_frame(
                    args.terrain_path, frame_num_abs, wheel_prefix_keywords
                )
                if not extra_prefixes:
                    fallback_prefix = "rigidBCE"
                    fallback_path = os.path.join(args.terrain_path, f"{fallback_prefix}{frame_num_abs}.csv")
                    if os.path.exists(fallback_path):
                        extra_prefixes = [fallback_prefix]
                if extra_prefixes:
                    print(f"Loading rigid BCE particles (frame {frame_num_abs}): {', '.join(extra_prefixes)}")
                fsi_states = read_fsi_body_states(fsi_folder, frame_num_abs)
                bce_transform_fn = None
                if fsi_states:
                    def bce_transform_fn(points):
                        if should_transform_bce(points, fsi_states):
                            print("Applying FSI body transforms to BCE points.")
                            return transform_bce_points(points, fsi_states)
                        print("BCE points appear to already be in world space.")
                        return points
                # Create particle transform for BCE particles
                bce_particle_transform = make_particle_transform(bce_transform_fn=bce_transform_fn, yaw_rad=math.radians(SCENE_YAW_DEG))
                for prefix in extra_prefixes:
                    success, _, positions = load_terrain_particles_for_frame(
                        args.terrain_path, prefix, frame_num_abs,
                        terrain_collection, args.wheel_bce_radius,
                        y_min=args.y_min, y_max=args.y_max, y_tol=args.y_tol,
                        clear_collection=False, material=wheel_material,
                        transform_fn=bce_particle_transform
                    )
                    if success and positions:
                        extra_positions.extend(positions)
                if extra_positions:
                    terrain_positions = (terrain_positions or []) + extra_positions
            
            # Give Blender a moment to process the terrain import
            if bpy.app.background:
                # In background mode, we need to force updates
                bpy.context.view_layer.update()
            else:
                # In interactive mode, we can use the GUI update
                bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        
        print("Setting up camera")
        setup_camera(
            args.camera,
            target_collections,
            current_frame_index_for_stable_path,
            args.frame_end - args.frame_start + 1,
            camera_data=stable_camera_bounds,
            extra_points=terrain_positions,
            stable_distance=bool(stable_camera_bounds)
        )
        
        # TEMPORARY HACK: Hide all objects except tires
        # hide_all_except_tires()
        
        # Set Blender frame to simulation frame
        # Particles emit at frame 0, so they exist at all frames ≥ 0
        # chrono_import.callback_post will load the correct state for this frame
        bpy.context.scene.frame_set(frame_num_abs)
        chrono_import.callback_post(bpy.context.scene)
        if SCENE_YAW_DEG:
            apply_scene_yaw_to_collections([chrono_frame_objects], math.radians(SCENE_YAW_DEG))
        
        # Update force arrow position if enabled
        if force_arrow_enabled:
            # Force depsgraph update after callback_post
            bpy.context.view_layer.update()
            
            print(f"  Looking for force arrow endpoints (frame {bpy.context.scene.frame_current})...")
            vehicle_pos, anchor_pos = get_force_arrow_endpoints(chrono_frame_objects)

            if vehicle_pos and anchor_pos:
                print(f"  Force arrow: vehicle={vehicle_pos}, anchor={anchor_pos}")
                
                # Remove old arrow and create new one with updated positions
                if "PullForceArrow" in bpy.data.objects:
                    old_arrow = bpy.data.objects["PullForceArrow"]
                    bpy.data.objects.remove(old_arrow, do_unlink=True)

                # Arrow starts at red sphere (vehicle) and points to anchor (block)
                # Use fixed length so arrow doesn't grow as vehicle moves away
                force_arrow_obj = create_force_arrow(
                    vehicle_pos, anchor_pos,
                    name="PullForceArrow",
                    shaft_radius=force_arrow_radius,
                    head_radius=force_arrow_radius * 2.5,
                    head_length=0.08,
                    color=force_arrow_color,
                    fixed_length=0.4  # Fixed arrow length regardless of distance
                )
        
        bpy.context.view_layer.update()
        
        # Render with error handling
        print(f"Rendering frame {frame_num_abs}")
        bpy.context.scene.render.filepath = os.path.join(view_folder, f"frame_{frame_num_abs:04d}.png")
        
        try:
            bpy.ops.render.render(write_still=True)
            apply_png_dpi(bpy.context.scene.render.filepath, OUTPUT_DPI)
        except Exception as e:
            print(f"Error rendering frame {frame_num_abs}: {e}")
            
            # If we haven't already switched to CPU, try that
            if not switched_to_cpu and bpy.context.scene.cycles.device == 'GPU':
                print("Switching to CPU rendering and trying again...")
                bpy.context.scene.cycles.device = 'CPU'
                switched_to_cpu = True
                
                try:
                    bpy.ops.render.render(write_still=True)
                    apply_png_dpi(bpy.context.scene.render.filepath, OUTPUT_DPI)
                    print("CPU rendering successful!")
                except Exception as e2:
                    print(f"CPU rendering also failed: {e2}")
                    print(f"Skipping frame {frame_num_abs}")
            else:
                print(f"Skipping frame {frame_num_abs}")
        
        # Clean up terrain objects after rendering to free memory
        if terrain_collection:
            for obj in terrain_collection.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
        
        # Clean up resources after each frame
        cleanup_resources()
    
    print("Rendering complete!")

if __name__ == "__main__":
    main()
