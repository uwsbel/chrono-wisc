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
import time
import glob
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
# Adjust these values to control camera positioning and behavior
CAMERA_DEFAULT_DISTANCE = 5.0          # Default distance when no objects are found
CAMERA_SCENE_DISTANCE_FACTOR = 3    # Multiplier for scene size to camera distance

# Background settings
USE_BACKGROUND_TEXTURE = True          # Toggle to enable/disable background texture
BACKGROUND_TYPE = "TRANSPARENT"           # Options: "GRADIENT", "SKY", "NONE", "WHITE"
# BACKGROUND_COLOR1 = (0.2, 0.3, 0.4, 1.0)  # Bottom/horizon color for gradient
# BACKGROUND_COLOR2 = (0.5, 0.7, 0.9, 1.0)  # Top/zenith color for gradient

BACKGROUND_COLOR1 = (1.0, 1.0, 1.0, 0.0)  # Bottom/horizon color for gradient
BACKGROUND_COLOR2 = (1.0, 1.0, 1.0, 0.0)  # Top/zenith color for gradient

# Revolving camera specific settings
REVOLVING_DISTANCE_FACTOR_X = 0.2      # Horizontal distance multiplier (higher = further away)
REVOLVING_DISTANCE_FACTOR_Y = 0.2      # Horizontal distance multiplier (higher = further away)
REVOLVING_HEIGHT_FACTOR = 0.1          # Height multiplier (higher = higher camera)
REVOLVING_ROTATION_SPEED = 0.5         # Rotation speed (lower = slower rotation, 1.0 = one full rotation)
REVOLVING_TARGET_OFFSET_X = 0.0        # X offset for revolving camera target
REVOLVING_TARGET_OFFSET_Y = 5.0        # Y offset for revolving camera target

# Isometric view specific settings
ISOMETRIC_DISTANCE_FACTOR_X = 0.1      # X-axis distance factor (higher = further right)
ISOMETRIC_DISTANCE_FACTOR_Y = 0.1     # Y-axis distance factor (higher = further back)
ISOMETRIC_DISTANCE_FACTOR_Z = 0.05      # Z-axis height factor (higher = higher camera)
ISOMETRIC_TARGET_OFFSET_X = 0.0        # X offset for camera target
ISOMETRIC_TARGET_OFFSET_Y = 0.0        # Y offset for camera target

# Side view specific settings
SIDE_DISTANCE_FACTOR = 0.2          # Distance multiplier for side view (higher = further away)
SIDE_HEIGHT_FACTOR = 0.01              # Height factor for side view (relative to center)
SIDE_X_OFFSET = 0.3                  # X offset for side view (positive moves camera right to center leftward objects)
SIDE_TARGET_OFFSET = 0.0            # Offset the camera target point along Y axis (shows more in front of vehicle)

# Front view specific settings
FRONT_DISTANCE_FACTOR = 0.15        # Distance multiplier for front view (much closer)
FRONT_HEIGHT_FACTOR = -0.1         # Height factor for front view (at same level as center)
FRONT_Y_OFFSET = 0.5              # Y offset for front view
FRONT_TARGET_OFFSET = 0.0           # Offset the camera target point along X axis

# Camera stability settings
CAMERA_STABLE_REVOLVING = True         # Use stable camera path for revolving camera
CAMERA_STABLE_ISOMETRIC = False         # Use stable camera for isometric view

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
    # Use bright green for better visibility against dark background
    bsdf.inputs['Base Color'].default_value = (0.1, 0.8, 0.2, 1.0)  # Bright green
    bsdf.inputs['Roughness'].default_value = 0.3
    bsdf.inputs['Metallic'].default_value = 0.5
    
    # Check if Emission properties exist before setting them
    if 'Emission Color' in bsdf.inputs:
        bsdf.inputs['Emission Color'].default_value = (0.1, 0.7, 0.2, 1.0)
    
    if 'Emission Strength' in bsdf.inputs:
        bsdf.inputs['Emission Strength'].default_value = 0.2
    elif 'Emission' in bsdf.inputs:
        bsdf.inputs['Emission'].default_value = 0.2
    
    print("Created enhanced particle material with bright green color")

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
    Returns a list of particle positions (x, y, z) and a dictionary of additional data columns.
    """
    print(f"Reading terrain particles from: {particle_file}")
    positions = []
    data_columns = {}  # Dictionary to store additional data columns
    column_names = []  # List to store the header names
    
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
                    # Parse header to get column names
                    column_names = [name.strip() for name in line.split(",")]
                    # Initialize data columns
                    for name in column_names[3:]:  # Skip x,y,z which are handled separately
                        data_columns[name] = []
                    print(f"Found data columns: {column_names[3:]}")
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
                        
                        # Update position ranges
                        min_x = min(min_x, x)
                        max_x = max(max_x, x)
                        min_y = min(min_y, y)
                        max_y = max(max_y, y)
                        min_z = min(min_z, z)
                        max_z = max(max_z, z)
                        
                        positions.append((x, y, z))
                        
                        # Store additional column data if available
                        for idx, col_name in enumerate(column_names[3:], 3):
                            if idx < len(line_seg):
                                try:
                                    data_columns[col_name].append(float(line_seg[idx]))
                                except ValueError:
                                    # If cannot convert to float, store None
                                    data_columns[col_name].append(None)
                            else:
                                # If column doesn't exist in this row
                                data_columns[col_name].append(None)
                                
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
                
        # Print data ranges for data columns
        for col_name, values in data_columns.items():
            if values and all(v is not None for v in values):
                print(f"  {col_name}: {min(values):.3f} to {max(values):.3f}")
    except FileNotFoundError:
        print(f"ERROR: Terrain particle file not found: {particle_file}")
        print("Continuing with empty particles list.")
    
    return positions, data_columns, column_names

def create_color_ramp_material(name="ParticleColorRamp", min_val=0.0, max_val=1.0, color_scheme="VIRIDIS", color_attribute="v_x"):
    """
    Create a material with a color ramp for data visualization, driven by particle attributes.
    
    Args:
        name: Name for the material
        min_val: Minimum value in the data range for normalization
        max_val: Maximum value in the data range for normalization
        color_scheme: Name of the color scheme to use (VIRIDIS, PLASMA, etc.)
        color_attribute: The particle attribute to use for coloring ('v_x', 'v_y', 'v_z', '|U|')
        
    Returns:
        The created material
    """
    # Create a new material
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    
    # Clear all nodes
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()
    
    # Add output node
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    output_node.location = (800, 0)
    
    # Add Principled BSDF node
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf_node.location = (600, 0)
    
    # Add ColorRamp node
    color_ramp_node = nodes.new(type='ShaderNodeValToRGB')
    color_ramp_node.location = (400, 100)
    
    # Configure color ramp based on scheme
    if color_scheme == "VIRIDIS":
        color_ramp_node.color_ramp.elements[0].position = 0.0
        color_ramp_node.color_ramp.elements[0].color = (0.267, 0.005, 0.329, 1.0)
        pos1 = color_ramp_node.color_ramp.elements.new(0.25)
        pos1.color = (0.275, 0.194, 0.496, 1.0)
        pos2 = color_ramp_node.color_ramp.elements.new(0.5)
        pos2.color = (0.163, 0.395, 0.523, 1.0)
        pos3 = color_ramp_node.color_ramp.elements.new(0.75)
        pos3.color = (0.47, 0.683, 0.28, 1.0)
        color_ramp_node.color_ramp.elements[1].position = 1.0
        color_ramp_node.color_ramp.elements[1].color = (0.993, 0.906, 0.144, 1.0)
    elif color_scheme == "PLASMA":
        color_ramp_node.color_ramp.elements[0].position = 0.0
        color_ramp_node.color_ramp.elements[0].color = (0.05, 0.03, 0.54, 1.0)
        pos1 = color_ramp_node.color_ramp.elements.new(0.25)
        pos1.color = (0.417, 0.0, 0.723, 1.0)
        pos2 = color_ramp_node.color_ramp.elements.new(0.5)
        pos2.color = (0.798, 0.0, 0.627, 1.0)
        pos3 = color_ramp_node.color_ramp.elements.new(0.75)
        pos3.color = (0.996, 0.39, 0.39, 1.0)
        color_ramp_node.color_ramp.elements[1].position = 1.0
        color_ramp_node.color_ramp.elements[1].color = (0.94, 0.975, 0.131, 1.0)
    elif color_scheme == "RAINBOW":
        color_ramp_node.color_ramp.elements[0].position = 0.0
        color_ramp_node.color_ramp.elements[0].color = (0.0, 0.0, 1.0, 1.0)
        pos1 = color_ramp_node.color_ramp.elements.new(0.25)
        pos1.color = (0.0, 1.0, 1.0, 1.0)
        pos2 = color_ramp_node.color_ramp.elements.new(0.5)
        pos2.color = (0.0, 1.0, 0.0, 1.0)
        pos3 = color_ramp_node.color_ramp.elements.new(0.75)
        pos3.color = (1.0, 1.0, 0.0, 1.0)
        color_ramp_node.color_ramp.elements[1].position = 1.0
        color_ramp_node.color_ramp.elements[1].color = (1.0, 0.0, 0.0, 1.0)
    else:  # Default to BLUE_RED
        color_ramp_node.color_ramp.elements[0].position = 0.0
        color_ramp_node.color_ramp.elements[0].color = (0.0, 0.0, 1.0, 1.0)
        color_ramp_node.color_ramp.elements[1].position = 1.0
        color_ramp_node.color_ramp.elements[1].color = (1.0, 0.0, 0.0, 1.0)

    # Shader nodes for getting and normalizing particle data
    particle_info_node = nodes.new(type='ShaderNodeParticleInfo')
    particle_info_node.location = (-200, 0)

    map_range_node = nodes.new(type='ShaderNodeMapRange')
    map_range_node.location = (200, 0)
    map_range_node.inputs['From Min'].default_value = min_val
    map_range_node.inputs['From Max'].default_value = max_val
    map_range_node.inputs['To Min'].default_value = 0.0
    map_range_node.inputs['To Max'].default_value = 1.0
    map_range_node.clamp = True

    # Connect based on color_attribute
    if color_attribute == 'v_x':
        separate_xyz_node = nodes.new(type='ShaderNodeSeparateXYZ')
        separate_xyz_node.location = (0, -50)
        links.new(particle_info_node.outputs['Velocity'], separate_xyz_node.inputs['Vector'])
        links.new(separate_xyz_node.outputs['X'], map_range_node.inputs['Value'])
    elif color_attribute == 'v_y':
        separate_xyz_node = nodes.new(type='ShaderNodeSeparateXYZ')
        separate_xyz_node.location = (0, -50)
        links.new(particle_info_node.outputs['Velocity'], separate_xyz_node.inputs['Vector'])
        links.new(separate_xyz_node.outputs['Y'], map_range_node.inputs['Value'])
    elif color_attribute == 'v_z':
        separate_xyz_node = nodes.new(type='ShaderNodeSeparateXYZ')
        separate_xyz_node.location = (0, -50)
        links.new(particle_info_node.outputs['Velocity'], separate_xyz_node.inputs['Vector'])
        links.new(separate_xyz_node.outputs['Z'], map_range_node.inputs['Value'])
    elif color_attribute == '|U|':
        vector_math_node = nodes.new(type='ShaderNodeVectorMath')
        vector_math_node.operation = 'LENGTH'
        vector_math_node.location = (0, 50)
        links.new(particle_info_node.outputs['Velocity'], vector_math_node.inputs[0])
        links.new(vector_math_node.outputs['Value'], map_range_node.inputs['Value'])
    elif color_attribute is not None: # Handles 'pressure' and other custom scalar attributes
        # Assumes custom scalar attributes are passed via angular_velocity.x
        print(f"Using attribute '{color_attribute}' via angular_velocity.x for color ramp.")
        separate_xyz_node = nodes.new(type='ShaderNodeSeparateXYZ')
        separate_xyz_node.location = (0, -100) # Adjusted Y location
        links.new(particle_info_node.outputs['Angular Velocity'], separate_xyz_node.inputs['Vector'])
        links.new(separate_xyz_node.outputs['X'], map_range_node.inputs['Value'])
    else:
        # Fallback for None or truly unsupported attributes
        print(f"Warning: Unsupported or None color_attribute ('{color_attribute}'). Using default color (start of ramp).")
        # Using a Value node to feed a constant to Map Range, resulting in To Min (0.0) -> first color.
        constant_val_node = nodes.new(type='ShaderNodeValue')
        constant_val_node.location = (0,0)
        constant_val_node.outputs[0].default_value = 0.0 # Default to the start of the color ramp
        links.new(constant_val_node.outputs['Value'], map_range_node.inputs['Value'])


    # Connect nodes: Normalized Value -> ColorRamp -> BSDF -> Output
    links.new(map_range_node.outputs['Result'], color_ramp_node.inputs['Fac'])
    links.new(color_ramp_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    links.new(bsdf_node.outputs['BSDF'], output_node.inputs['Surface'])
    
    # Store min/max and scheme as custom properties for legend or debugging
    mat["data_min_used_for_norm"] = min_val
    mat["data_max_used_for_norm"] = max_val
    mat["color_scheme"] = color_scheme
    mat["color_attribute"] = color_attribute
    
    print(f"Created color ramp material '{name}' for attribute '{color_attribute}' with data range [{min_val:.6f}, {max_val:.6f}] using {color_scheme} scheme")
    
    return mat

def setup_particle_system(positions, particle_radius, collection, 
                         all_data_columns=None, # Changed from data_values
                         color_by_column=None,  # Changed from column_name
                         color_min_val=None,    # Changed from color_min
                         color_max_val=None,    # Changed from color_max
                         color_scheme="VIRIDIS"):
    """
    Set up a particle system to display terrain particles with optional coloring based on data
    
    Args:
        positions: List of particle positions (x,y,z)
        particle_radius: Radius for particles
        collection: Collection to add particles to
        all_data_columns: Dictionary of all data columns read from CSV (e.g., {'v_x': [], 'v_y': [], ...})
        color_by_column: Name of the data column used for coloring (e.g., "v_x", "|U|")
        color_min_val: Minimum value for color mapping normalization
        color_max_val: Maximum value for color mapping normalization
        color_scheme: Color scheme to use for mapping (default: VIRIDIS)
    """
    if not positions:
        print("No particles to display")
        return {'handler': None, 'host': None, 'instance': None, 'has_colors': False} # Return dict
    
    # Debug: Show particle info
    print(f"Setting up particle system with {len(positions)} particles and radius {particle_radius}")
    
    # Limit number of particles
    max_particles = 5000000 
    original_count = len(positions)
    
    # Prepare a copy of all_data_columns for potential downsampling
    # This ensures the original data passed to the function is not modified if it's used elsewhere.
    current_data_columns = {}
    if all_data_columns:
        for key, values in all_data_columns.items():
            current_data_columns[key] = list(values) # Create copies of lists

    if len(positions) > max_particles:
        print(f"WARNING: Too many particles ({original_count}). Limiting to {max_particles} particles.")
        skip_factor = original_count // max_particles
        positions = positions[::skip_factor]
        
        # Downsample current_data_columns consistently
        if current_data_columns:
            for key in current_data_columns:
                # Ensure the column exists and had the original full length before downsampling
                if current_data_columns[key] and len(current_data_columns[key]) == original_count:
                    current_data_columns[key] = current_data_columns[key][::skip_factor]
        print(f"Downsampled from {original_count} to {len(positions)} particles (factor: {skip_factor})")

    # Data values for the specific column used for coloring (after potential downsampling)
    coloring_data_values = None
    if color_by_column and current_data_columns and color_by_column in current_data_columns:
        coloring_data_values = current_data_columns[color_by_column]

    # Create a small icosphere as particle instance
    bpy.ops.mesh.primitive_ico_sphere_add(radius=particle_radius, location=(50, 50, 50)) # Off-screen
    ico_sphere = bpy.context.object
    ico_sphere.name = "ParticleInstance"
    
    # Apply material - either color-mapped or default
    has_color_data_for_material = False
    if color_by_column and coloring_data_values and any(v is not None for v in coloring_data_values):
        print(f"Coloring particles by '{color_by_column}' data")
        
        # Determine color range if not specified by user (using the actual data from the chosen column)
        # This ensures min/max are based on the actual values being used for coloring.
        # Filter out None values before calculating min/max for the range.
        valid_coloring_values = [v for v in coloring_data_values if v is not None]
        
        actual_data_min = min(valid_coloring_values) if valid_coloring_values else 0.0
        actual_data_max = max(valid_coloring_values) if valid_coloring_values else 1.0

        # Use user-provided min/max for normalization if available, otherwise use data's actual min/max
        norm_min = color_min_val if color_min_val is not None else actual_data_min
        norm_max = color_max_val if color_max_val is not None else actual_data_max
            
        if norm_min == norm_max: # Ensure valid range for MapRange node
            print(f"WARNING: Normalization range min ({norm_min}) equals max ({norm_max}). Offsetting max slightly.")
            norm_max = norm_min + 0.001 # Add a small epsilon, or 1.0 if norm_min is 0
            if norm_min == 0.0 and norm_max == 0.0: norm_max = 1.0


        print(f"  Actual data range for '{color_by_column}': [{actual_data_min:.6f}, {actual_data_max:.6f}]")
        print(f"  Using normalization range for material: [{norm_min:.6f}, {norm_max:.6f}]")
        
        material = create_color_ramp_material(
            name=f"ColorMap_{color_by_column}", 
            min_val=norm_min, 
            max_val=norm_max,
            color_scheme=color_scheme,
            color_attribute=color_by_column # Pass the type of attribute, e.g. "v_x"
        )
        has_color_data_for_material = True
    else:
        # Use default material
        material = bpy.data.materials.new(name="ParticleDefaultMaterial")
        material.diffuse_color = (0.1, 0.1, 0.1, 1.0)  # Gray color
        if color_by_column:
             print(f"Warning: No valid data found for column '{color_by_column}' for coloring, using default material.")

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
        return {'handler': None, 'host': host_object, 'instance': ico_sphere, 'has_colors': has_color_data_for_material}

    ps = particle_system_modifier.particle_system
    ps_name = ps.name # Should be "TerrainParticles"
    
    settings = ps.settings
    settings.count = len(positions) # Based on (potentially downsampled) positions
    settings.lifetime = 1000 # Long lifetime
    settings.frame_start = settings.frame_end = 1 # Emit all at frame 1
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
            if scene.frame_current == 1: print(f"Particle handler: Host object '{host_object.name}' not in depsgraph.")
            return

        # Particle systems are on the evaluated object
        if not evaluated_host.particle_systems or not ps_name in evaluated_host.particle_systems:
            if scene.frame_current == 1: print(f"Particle handler: Particle system '{ps_name}' not on evaluated host.")
            return
            
        current_ps = evaluated_host.particle_systems[ps_name]
        num_depsgraph_particles = len(current_ps.particles)

        if scene.frame_current == 1 or scene.frame_current % 50 == 0 : # Reduce log spam
            print(f"Particle handler for frame {scene.frame_current}: Setting up {num_depsgraph_particles} particles (expected {len(positions)})")

        for i, p in enumerate(current_ps.particles):
            if i < len(positions): # Ensure we don't go out of bounds for positions list
                p.location = positions[i]
                # p.size is controlled by settings.particle_size and ico_sphere radius

                # Set particle velocity based on v_x, v_y, v_z columns if they exist.
                # This is used by the shader if color_by_column is 'v_x', 'v_y', 'v_z', or '|U|'.
                vx = current_data_columns.get('v_x')[i] if 'v_x' in current_data_columns and i < len(current_data_columns['v_x']) and current_data_columns['v_x'][i] is not None else 0.0
                vy = current_data_columns.get('v_y')[i] if 'v_y' in current_data_columns and i < len(current_data_columns['v_y']) and current_data_columns['v_y'][i] is not None else 0.0
                vz = current_data_columns.get('v_z')[i] if 'v_z' in current_data_columns and i < len(current_data_columns['v_z']) and current_data_columns['v_z'][i] is not None else 0.0
                p.velocity = mathutils.Vector((float(vx), float(vy), float(vz)))

                # If coloring by a custom scalar attribute (e.g., 'pressure'),
                # store its value in angular_velocity.x.
                # The shader will then be configured to read this via ParticleInfo node.
                if color_by_column and color_by_column not in ['v_x', 'v_y', 'v_z', '|U|']:
                    custom_scalar_val = 0.0
                    if color_by_column in current_data_columns and \
                       i < len(current_data_columns[color_by_column]) and \
                       current_data_columns[color_by_column][i] is not None:
                        try:
                            custom_scalar_val = float(current_data_columns[color_by_column][i])
                        except (ValueError, TypeError):
                            if scene.frame_current == 1: # Log error only once per problematic setup
                                print(f"Warning: Could not convert value for '{color_by_column}' at index {i} to float. Using 0.0.")
                            custom_scalar_val = 0.0
                    
                    p.angular_velocity = mathutils.Vector((custom_scalar_val, 0.0, 0.0))
                else:
                    # If not using a custom scalar for coloring, zero out angular_velocity
                    # or ensure it's reset if it was used in a previous frame/setup.
                    p.angular_velocity = mathutils.Vector((0.0, 0.0, 0.0))
            # else:
                # This case (i >= len(positions)) shouldn't happen if settings.count is correct
                # and particle emission is handled as expected (all at frame 1).
                # if scene.frame_current == 1: print(f"Warning: Particle index {i} out of range for positions data (len {len(positions)})")

    # Clear any previous handlers for safety, though load_terrain_particles_for_frame should do this
    for h in bpy.app.handlers.frame_change_post:
        if h.__name__ == particle_handler.__name__: # A bit fragile, but ok for now
            bpy.app.handlers.frame_change_post.remove(h)
            
    bpy.app.handlers.frame_change_post.append(particle_handler)
    
    # Force an update to try and initialize particles correctly for frame 1
    # This sequence is sometimes needed to ensure particles are born and handler can find them.
    bpy.context.scene.frame_set(0) # Go to frame before emission
    bpy.context.scene.frame_set(1) # Go to emission frame
    bpy.context.view_layer.update() # Update depsgraph

    return {
        'handler': particle_handler,
        'host': host_object,
        'instance': ico_sphere,
        'has_colors': has_color_data_for_material # Use the flag determined earlier
    }

def create_color_legend(material, width=0.3, height=2.0, 
                        position=mathutils.Vector((0,0,0)), camera=None, 
                        anchor_center=None, anchor_scale_ref=1.0):
    """Create a legend showing the color map with labels, oriented towards the camera, positioned via anchor."""
    # Determine final position based on anchor_center if provided
    # The `position` argument can serve as a fallback or fine-tuning offset if anchor_center is primary.
    # For this implementation, `position` IS the calculated position based on anchor.
    final_pos = position 

    # Create a plane for the legend
    # Initial rotation makes its local Y axis point up (world Z if plane is upright) and X along world X.
    bpy.ops.mesh.primitive_plane_add(size=1, location=final_pos, rotation=(math.radians(90), 0, 0)) 
    legend_plane = bpy.context.object
    legend_plane.name = "ColorLegend_Plane"
    
    # Scale the legend plane itself based on anchor_scale_ref for relative sizing.
    # Ensure width and height are scaled proportionally.
    # Example: if anchor_scale_ref is large, legend is larger.
    # Base width/height parameters are now relative to a typical scene scale of 1.
    effective_width = width * (anchor_scale_ref / 1.0) # Adjust base width for scene scale
    effective_height = height * (anchor_scale_ref / 1.0)
    legend_plane.dimensions.x = effective_width
    legend_plane.dimensions.y = effective_height # This is along local Y (becomes height)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    legend_mat = bpy.data.materials.new(name="LegendGradientMaterial")
    legend_mat.use_nodes = True
    nodes = legend_mat.node_tree.nodes
    links = legend_mat.node_tree.links
    nodes.clear()
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    output_node.location = (400, 0)
    emission_node = nodes.new(type='ShaderNodeEmission') 
    emission_node.location = (200, 0)
    links.new(emission_node.outputs['Emission'], output_node.inputs['Surface'])
    tex_coord_node = nodes.new(type='ShaderNodeTexCoord')
    tex_coord_node.location = (-400, 100)
    separate_xyz_node = nodes.new(type='ShaderNodeSeparateXYZ')
    separate_xyz_node.location = (-200, 100)
    links.new(tex_coord_node.outputs['UV'], separate_xyz_node.inputs['Vector'])
    color_ramp_node = nodes.new(type='ShaderNodeValToRGB')
    color_ramp_node.location = (0, 0)
    links.new(separate_xyz_node.outputs['Y'], color_ramp_node.inputs['Fac']) 
    links.new(color_ramp_node.outputs['Color'], emission_node.inputs['Color'])
    src_color_ramp_data = None
    if material and material.node_tree:
        for n in material.node_tree.nodes:
            if n.type == 'VALTORGB': 
                src_color_ramp_data = n.color_ramp
                break
    legend_ramp = color_ramp_node.color_ramp
    if src_color_ramp_data:
        legend_ramp.color_mode = src_color_ramp_data.color_mode
        legend_ramp.hue_interpolation = src_color_ramp_data.hue_interpolation
        legend_ramp.interpolation = src_color_ramp_data.interpolation
        while len(legend_ramp.elements) < len(src_color_ramp_data.elements):
            legend_ramp.elements.new(0) 
        while len(legend_ramp.elements) > len(src_color_ramp_data.elements):
            legend_ramp.elements.remove(legend_ramp.elements[-1]) 
        for i, src_el in enumerate(src_color_ramp_data.elements):
            target_el = legend_ramp.elements[i]
            target_el.position = src_el.position
            target_el.color = src_el.color
    else:
        print("Warning: Source color ramp not found for legend. Using default black-to-white.")
        if len(legend_ramp.elements) > 0: legend_ramp.elements[0].color = (0,0,0,1)
        if len(legend_ramp.elements) > 1: legend_ramp.elements[1].color = (1,1,1,1)
        while len(legend_ramp.elements) < 2:
             legend_ramp.elements.new(1.0 if len(legend_ramp.elements) > 0 else 0.0)
        if len(legend_ramp.elements) == 2: 
            legend_ramp.elements[0].position = 0.0
            legend_ramp.elements[1].position = 1.0
            legend_ramp.elements[0].color = (0,0,0,1)
            legend_ramp.elements[1].color = (1,1,1,1)
    legend_plane.data.materials.clear()
    legend_plane.data.materials.append(legend_mat)
    
    # Text properties also need to be scaled according to the legend plane's new size
    base_text_size = 0.15 # This is now relative to a legend of height ~2.0
    effective_text_size = base_text_size * (effective_height / 2.0) # Scale text with legend height
    text_extrude = 0.01 * (effective_height / 2.0)
    text_mat = bpy.data.materials.new(name="LegendTextMaterial")
    text_mat.diffuse_color = (0.9, 0.9, 0.9, 1.0) 

    data_min_val = material.get("data_min_used_for_norm", 0.0)
    data_max_val = material.get("data_max_used_for_norm", 1.0)
    data_col_name = material.get("color_attribute", "Value")

    # Text positions are local to the legend_plane (parent)
    # The plane's local Y is its height, local X is its width.
    # (0,0,0) is center of plane. Text Z offset to be slightly in front.
    min_text_y_offset = -effective_height/2 - effective_text_size * 0.7
    max_text_y_offset =  effective_height/2 + effective_text_size * 0.7
    title_text_y_offset = effective_height/2 + effective_text_size * 2.5
    text_z_offset = text_extrude * 2 # Slightly in front of the plane

    bpy.ops.object.text_add(location=(0, min_text_y_offset, text_z_offset)) 
    min_text = bpy.context.object
    min_text.name = "LegendMinText"
    min_text.data.body = f"{data_min_val:.2f}"
    min_text.data.size = effective_text_size
    min_text.data.extrude = text_extrude
    min_text.data.align_x = 'CENTER'
    min_text.data.align_y = 'TOP' 
    min_text.data.materials.append(text_mat)
    min_text.parent = legend_plane
    min_text.matrix_parent_inverse = legend_plane.matrix_world.inverted()

    bpy.ops.object.text_add(location=(0, max_text_y_offset, text_z_offset))
    max_text = bpy.context.object
    max_text.name = "LegendMaxText"
    max_text.data.body = f"{data_max_val:.2f}"
    max_text.data.size = effective_text_size
    max_text.data.extrude = text_extrude
    max_text.data.align_x = 'CENTER'
    max_text.data.align_y = 'BOTTOM'
    max_text.data.materials.append(text_mat)
    max_text.parent = legend_plane
    max_text.matrix_parent_inverse = legend_plane.matrix_world.inverted()
    
    bpy.ops.object.text_add(location=(0, title_text_y_offset, text_z_offset))
    title_text = bpy.context.object
    title_text.name = "LegendTitleText"
    title_text.data.body = str(data_col_name) 
    title_text.data.size = effective_text_size * 1.1 
    title_text.data.extrude = text_extrude
    title_text.data.align_x = 'CENTER'
    title_text.data.align_y = 'BOTTOM'
    title_text.data.materials.append(text_mat)
    title_text.parent = legend_plane
    title_text.matrix_parent_inverse = legend_plane.matrix_world.inverted()

    if camera:
        direction_to_cam = camera.location - legend_plane.location
        rot_quat = direction_to_cam.to_track_quat('-Z', 'Y') 
        legend_plane.rotation_euler = rot_quat.to_euler()
    else:
        legend_plane.rotation_euler = (math.radians(90), 0, 0)

    bpy.context.view_layer.objects.active = legend_plane
    legend_plane.select_set(True)
    # ... (deselect text objects)
    min_text.select_set(False)
    max_text.select_set(False)
    title_text.select_set(False)

    print(f"Created legend: {legend_plane.name} with text labels, parented and oriented.")
    return [legend_plane, min_text, max_text, title_text]

def load_terrain_particles_for_frame(terrain_folder, file_prefix, frame, terrain_collection, particle_radius, 
                                color_by=None, color_min=None, color_max=None, color_scheme="VIRIDIS", 
                                show_legend=True, legend_anchor_center=None, legend_anchor_scale_ref=1.0):
    """
    Load terrain particles for a specific frame with optional color mapping
    
    Args:
        terrain_folder: Folder containing particle CSV files
        file_prefix: Prefix for CSV filenames
        frame: Frame number to load
        terrain_collection: Collection to add particles to
        particle_radius: Radius for particles
        color_by: Name of the column to use for coloring (None for no coloring)
        color_min: Minimum value for color mapping (optional)
        color_max: Maximum value for color mapping (optional)
        color_scheme: Color scheme to use (VIRIDIS, PLASMA, RAINBOW, BLUE_RED)
        show_legend: Whether to show a color legend
        legend_anchor_center: Center of the legend relative to the camera
        legend_anchor_scale_ref: Scale reference for the legend
    
    Returns:
        Boolean indicating if particles were loaded successfully
    """
    # Clear any previous terrain objects and handlers
    for obj in terrain_collection.objects:
        bpy.data.objects.remove(obj, do_unlink=True)
    
    handlers_to_remove = [h for h in bpy.app.handlers.frame_change_post if hasattr(h, '__name__') and 'particle_handler' in h.__name__]
    for h in handlers_to_remove:
        bpy.app.handlers.frame_change_post.remove(h)
            
    filename = f"{file_prefix}{frame}.csv"
    filepath = os.path.join(terrain_folder, filename)
    
    if not os.path.exists(filepath):
        print(f"Warning: Terrain particle file not found: {filepath}")
        return False, None 
    
    positions, all_data_columns, column_names = read_terrain_particles(filepath, particle_radius)
    
    if not positions:
        print("No terrain particles found")
        return False, None
    
    if particle_radius <= 0.001:
        print(f"WARNING: Particle radius {particle_radius} is too small - using default 0.005")
        particle_radius = 0.005
    print(f"Using particle radius: {particle_radius}")
    
    selected_column_for_coloring = None
    actual_data_min_for_info = None
    actual_data_max_for_info = None

    if color_by:
        if color_by not in all_data_columns or not all_data_columns[color_by] or all(v is None for v in all_data_columns[color_by]):
            print(f"WARNING: Requested color column '{color_by}' not found or has no valid data. No coloring will be applied.")
        else:
            selected_column_for_coloring = color_by
            valid_values = [v for v in all_data_columns[selected_column_for_coloring] if v is not None]
            if valid_values:
                actual_data_min_for_info = min(valid_values)
                actual_data_max_for_info = max(valid_values)
                print(f"Coloring particles by '{selected_column_for_coloring}' column.")
                print(f"  Actual data range in CSV for '{selected_column_for_coloring}': [{actual_data_min_for_info:.6f}, {actual_data_max_for_info:.6f}]")
                if color_min is not None or color_max is not None:
                    user_min = color_min if color_min is not None else actual_data_min_for_info
                    user_max = color_max if color_max is not None else actual_data_max_for_info
                    print(f"  Using user-defined normalization range for material: [{user_min:.6f}, {user_max:.6f}]")
            else:
                print(f"WARNING: Column '{color_by}' has no valid numeric data. No coloring.")
                selected_column_for_coloring = None 

    particle_system_info = setup_particle_system(
        positions, 
        particle_radius, 
        terrain_collection, 
        all_data_columns=all_data_columns, 
        color_by_column=selected_column_for_coloring,
        color_min_val=color_min, 
        color_max_val=color_max, 
        color_scheme=color_scheme
    )
    
    if particle_system_info['has_colors'] and show_legend and particle_system_info['instance']:
        instance_obj = particle_system_info['instance']
        if instance_obj.data.materials:
            material_for_legend = instance_obj.data.materials[0]
            cam = bpy.context.scene.camera # Get current camera for orientation
            
            # Use provided anchor points for legend positioning
            anchor_c = legend_anchor_center if legend_anchor_center is not None else mathutils.Vector((0,0,0))
            anchor_s_ref = legend_anchor_scale_ref if legend_anchor_scale_ref > 0.01 else 1.0 # Ensure some scale

            # Calculate offset from the anchor center. 
            # This example places it to the right (world +X) and slightly up, scaled by scene size.
            # You might want to make this offset direction dependent on camera view (e.g., always to camera's right)
            # For now, a world-space offset relative to the scene center:
            legend_offset = mathutils.Vector((anchor_s_ref * 0.6, 0, anchor_s_ref * 0.1)) 
            final_legend_pos = anchor_c + legend_offset
            
            create_color_legend(material_for_legend, 
                                width=0.3 * (anchor_s_ref / 5.0), # Scale legend size too
                                height=2.0 * (anchor_s_ref / 5.0),
                                position=final_legend_pos, 
                                camera=cam,
                                anchor_center=anchor_c, # Pass for potential future use within legend
                                anchor_scale_ref=anchor_s_ref)
            print(f"Created color legend for '{material_for_legend.name}' at {final_legend_pos}.")
        else:
            print("Warning: Instance object for legend has no materials.")
            
    if particle_system_info and particle_system_info['handler']:
        print("Particle system successfully created and handler registered.")
        return True, particle_system_info
    else:
        print("WARNING: Failed to create particle system or register handler.")
        return False, particle_system_info

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
        position = center + mathutils.Vector((distance * ISOMETRIC_DISTANCE_FACTOR_X, distance * ISOMETRIC_DISTANCE_FACTOR_Y, distance * ISOMETRIC_DISTANCE_FACTOR_Z))
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
    
    # Initialize scene metrics
    current_scene_center = mathutils.Vector((0, 3.75, 1)) # Default
    current_scene_max_dim = CAMERA_DEFAULT_DISTANCE / CAMERA_SCENE_DISTANCE_FACTOR # Estimate from default
    current_scene_distance = CAMERA_DEFAULT_DISTANCE

    if camera_data and camera_data.get('positions') and camera_data.get('center'):
        if frame < len(camera_data['positions']):
            cam_obj.location = camera_data['positions'][frame]
        else:
            print(f"Warning: Frame {frame} exceeds pre-calculated camera positions")
            cam_obj.location = camera_data['positions'][-1]
        
        direction = camera_data['center'] - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam_obj.rotation_euler = rot_quat.to_euler()
        
        cam_data.lens = 55
        cam_data.clip_start = 0.1
        cam_data.clip_end = 10000
        
        current_scene_center = camera_data['center']
        current_scene_distance = camera_data.get('distance', CAMERA_DEFAULT_DISTANCE)
        current_scene_max_dim = camera_data.get('max_dim', current_scene_distance / CAMERA_SCENE_DISTANCE_FACTOR)
        
        print(f"Using pre-calculated camera. Scene center: {current_scene_center}, Max dim: {current_scene_max_dim}")
        return cam_obj, {
            'center': current_scene_center, 
            'max_dim': current_scene_max_dim, 
            'distance': current_scene_distance
        }

    # Dynamically calculate center and size if not using precalculated full path
    object_found = False
    min_co = mathutils.Vector((float('inf'), float('inf'), float('inf')))
    max_co = mathutils.Vector((float('-inf'), float('-inf'), float('-inf')))
    vehicle_center = None

    if target_collections:
        for collection in target_collections:
            if collection and collection.objects:
                if collection.name == 'chrono_frame_objects' and collection.objects:
                    vehicle_min_co = mathutils.Vector((float('inf'), float('inf'), float('inf')))
                    vehicle_max_co = mathutils.Vector((float('-inf'), float('-inf'), float('-inf')))
                    vehicle_found_in_collection = False
                    for obj in collection.objects:
                        if obj.type == 'MESH':
                            vehicle_found_in_collection = True
                            for v in obj.bound_box:
                                world_co = obj.matrix_world @ mathutils.Vector(v)
                                vehicle_min_co = mathutils.Vector(map(min, vehicle_min_co, world_co))
                                vehicle_max_co = mathutils.Vector(map(max, vehicle_max_co, world_co))
                    if vehicle_found_in_collection:
                        vehicle_center = (vehicle_min_co + vehicle_max_co) / 2
                
                for obj in collection.objects:
                    if obj.type == 'MESH':
                        object_found = True
                        for v in obj.bound_box:
                            world_co = obj.matrix_world @ mathutils.Vector(v)
                            min_co = mathutils.Vector(map(min, min_co, world_co))
                            max_co = mathutils.Vector(map(max, max_co, world_co))
    
    if vehicle_center:
        current_scene_center = vehicle_center
    elif object_found:
        current_scene_center = (min_co + max_co) / 2
    else:
        print(f"WARNING: No mesh objects found in collections - using default camera position and scene center")

    if object_found:
        size = max_co - min_co
        current_scene_max_dim = max(size.x, size.y, size.z, 0.1) # Ensure not zero
        current_scene_distance = current_scene_max_dim * CAMERA_SCENE_DISTANCE_FACTOR
    else: # Fallback if no objects found
        current_scene_max_dim = CAMERA_DEFAULT_DISTANCE / CAMERA_SCENE_DISTANCE_FACTOR 
        current_scene_distance = CAMERA_DEFAULT_DISTANCE
    
    # Camera positioning logic (ISOMETRIC, SIDE, TOP, REVOLVING) using current_scene_center and current_scene_distance
    # ... (This part remains largely the same, just uses current_scene_center and current_scene_distance)
    if camera_type == "ISOMETRIC":
        cam_obj.location = current_scene_center + mathutils.Vector((
            current_scene_distance * ISOMETRIC_DISTANCE_FACTOR_X, 
            current_scene_distance * ISOMETRIC_DISTANCE_FACTOR_Y, 
            current_scene_distance * ISOMETRIC_DISTANCE_FACTOR_Z
        ))
        target_point = current_scene_center + mathutils.Vector((ISOMETRIC_TARGET_OFFSET_X, ISOMETRIC_TARGET_OFFSET_Y, 0))
    elif camera_type == "SIDE":
        cam_obj.location = current_scene_center + mathutils.Vector((
            SIDE_X_OFFSET, 
            current_scene_distance * SIDE_DISTANCE_FACTOR, 
            current_scene_distance * SIDE_HEIGHT_FACTOR
        ))
        target_point = current_scene_center + mathutils.Vector((SIDE_TARGET_OFFSET, 0, 0))
    elif camera_type == "FRONT":
        cam_obj.location = current_scene_center + mathutils.Vector((
            current_scene_distance * FRONT_DISTANCE_FACTOR + FRONT_Y_OFFSET,  # Position along X-axis for front view
            0,  # Centered on Y-axis
            current_scene_distance * FRONT_HEIGHT_FACTOR
        ))
        target_point = current_scene_center + mathutils.Vector((FRONT_TARGET_OFFSET, 0, 0))
    elif camera_type == "TOP":
        cam_obj.location = current_scene_center + mathutils.Vector((0, 0, current_scene_distance * 0.6))
        target_point = current_scene_center
    elif camera_type == "REVOLVING":
        angle = 2 * math.pi * REVOLVING_ROTATION_SPEED * (frame / total_frames)
        x = current_scene_center.x + current_scene_distance * REVOLVING_DISTANCE_FACTOR_X * math.cos(angle)
        y = current_scene_center.y + current_scene_distance * REVOLVING_DISTANCE_FACTOR_Y * math.sin(angle)
        z = current_scene_center.z + current_scene_distance * REVOLVING_HEIGHT_FACTOR
        cam_obj.location = mathutils.Vector((x, y, z))
        target_point = current_scene_center + mathutils.Vector((REVOLVING_TARGET_OFFSET_X, REVOLVING_TARGET_OFFSET_Y, 0))
    else: # Default/fallback
        target_point = current_scene_center
        cam_obj.location = current_scene_center + mathutils.Vector((0, -current_scene_distance, current_scene_distance *0.4))

    direction = target_point - cam_obj.location
    rot_quat = direction.to_track_quat('-Z', 'Y')
    cam_obj.rotation_euler = rot_quat.to_euler()
    
    cam_data.lens = 55
    cam_data.clip_start = 0.1
    cam_data.clip_end = 10000
    
    print(f"Camera positioned for '{camera_type}'. Scene center: {current_scene_center}, Max dim: {current_scene_max_dim}")

    scene_details = {
        'center': current_scene_center,
        'max_dim': current_scene_max_dim,
        'distance': current_scene_distance
    }

    should_precalculate_now = (
        (camera_type == "REVOLVING" and (CAMERA_STABLE_REVOLVING)) or
        (camera_type == "ISOMETRIC" and (CAMERA_STABLE_ISOMETRIC))
    ) and frame == 0 and not camera_data # Only precalculate if not already using camera_data

    if should_precalculate_now:
        print(f"Pre-calculating stable camera path for {camera_type} view for {total_frames} frames")
        positions = precalculate_camera_path(current_scene_center, current_scene_distance, 0, total_frames, camera_type)
        scene_details['positions'] = positions # Add positions to the returned dict
        # This scene_details (including positions, center, distance, max_dim) will become camera_data for subsequent frames.
        print(f"Created stable camera path with {len(positions)} positions. Center: {current_scene_center}, Max Dim: {current_scene_max_dim}")
    
    return cam_obj, scene_details

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
    # Check if TRANSPARENT background is requested
    if bpy.context.scene.get("background_type", None) == "TRANSPARENT":
        scene.render.film_transparent = True
    else:
        scene.render.film_transparent = False

def cleanup_resources():
    """Clean up resources to prevent memory leaks"""
    # Clear particle handlers
    bpy.app.handlers.frame_change_post.clear()
    
    # Force garbage collection
    gc.collect()

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
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Render Chrono simulation in Blender")
    parser.add_argument("--assets", required=True, help="Path to the .assets.py file")
    parser.add_argument("--output", required=True, help="Output directory for rendered images")
    parser.add_argument("--camera", dest="camera", default="ISOMETRIC", 
                        choices=["ISOMETRIC", "SIDE", "TOP", "REVOLVING", "FRONT"],
                        help="Camera type")
    parser.add_argument("--view", dest="camera", 
                        choices=["ISOMETRIC", "SIDE", "TOP", "REVOLVING", "FRONT"],
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
    
    # Terrain-related arguments
    parser.add_argument("--terrain_path", help="Path to folder containing terrain files")
    parser.add_argument("--terrain_type", choices=["OBJ", "PARTICLES"], default="OBJ", 
                        help="Type of terrain representation (OBJ meshes or particle CSV files)")
    parser.add_argument("--terrain_prefix", default="fluid", help="Prefix for terrain filenames")
    parser.add_argument("--terrain_suffix", default="_surface.obj", help="Suffix for terrain OBJ filenames (only used with --terrain_type OBJ)")
    parser.add_argument("--particle_radius", type=float, default=0.005, help="Radius for terrain particles (only used with --terrain_type PARTICLES)")
    
    # Particle color mapping arguments
    parser.add_argument("--color_by", help="Column name to use for particle coloring (e.g., '|U|', 'pressure')")
    parser.add_argument("--color_min", type=float, help="Minimum value for color mapping (if not specified, uses data min)")
    parser.add_argument("--color_max", type=float, help="Maximum value for color mapping (if not specified, uses data max)")
    parser.add_argument("--color_scheme", choices=["VIRIDIS", "PLASMA", "RAINBOW", "BLUE_RED"], default="VIRIDIS",
                       help="Color scheme to use for mapping")
    parser.add_argument("--no_legend", action="store_true", help="Hide color legend when coloring particles")
    
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
    target_collections = [chrono_frame_objects]
    if terrain_collection:
        target_collections.append(terrain_collection)
    
    # Track if we had to switch to CPU rendering
    switched_to_cpu = False
    
    # Pre-calculate camera path if using a camera type that needs stabilization
    # This `camera_data` will now be the `scene_details` dictionary from `setup_camera`
    camera_data_for_stable_path = None 
    should_precalculate = (
        (args.camera == "REVOLVING" and (args.stable_camera or CAMERA_STABLE_REVOLVING)) or
        (args.camera == "ISOMETRIC" and (args.stable_camera or CAMERA_STABLE_ISOMETRIC))
    )
    
    if should_precalculate:
        print(f"Pre-calculating stable camera path for {args.camera} view")
        bpy.context.scene.frame_set(args.frame_start)
        chrono_import.callback_post(bpy.context.scene)
        
        initial_legend_anchor_center = mathutils.Vector((0,0,0))
        initial_legend_anchor_scale_ref = 1.0

        if args.terrain_path and terrain_collection:
            # Load initial terrain for accurate bounding box for first camera setup
            if args.terrain_type == "OBJ":
                load_terrain_for_frame(
                    args.terrain_path, args.terrain_prefix, args.terrain_suffix, 
                    args.frame_start, terrain_collection
                )
            else:  # PARTICLES
                # For particles, the first call to setup_camera will use default scene center, 
                # then particles loaded, then legend positioned based on that first camera setup.
                # This is a bit of a chicken-and-egg, but the legend will use the first frame's overall scene estimate.
                pass # Particles will be loaded per-frame anyway
            bpy.context.view_layer.update()
        
        # Initial camera setup to get scene details for precalculation
        # The returned scene_details here will become camera_data_for_stable_path
        _, camera_data_for_stable_path = setup_camera(
            args.camera,
            target_collections,
            0,  # Frame index for precalculation (0 to total_frames-1)
            args.frame_end - args.frame_start + 1,
            None # No camera_data provided for this initial call
        )
        initial_legend_anchor_center = camera_data_for_stable_path['center']
        initial_legend_anchor_scale_ref = camera_data_for_stable_path['max_dim']
        print(f"Initial scene details for stable path: center={initial_legend_anchor_center}, max_dim={initial_legend_anchor_scale_ref}")

    # Render each frame
    for frame_num_abs in range(args.frame_start, args.frame_end + 1):
        print(f"\nProcessing frame {frame_num_abs}")
        bpy.context.scene.frame_set(frame_num_abs)
        chrono_import.callback_post(bpy.context.scene)
        
        current_frame_index_for_stable_path = frame_num_abs - args.frame_start

        # Setup camera for the current frame, potentially using precalculated data
        # `scene_details` will contain center, max_dim, etc. for *this* frame
        cam_obj, current_scene_details = setup_camera(
            args.camera, 
            target_collections,
            current_frame_index_for_stable_path, 
            args.frame_end - args.frame_start + 1,
            camera_data_for_stable_path # Pass the precalculated data if available
        )
        
        # Use the scene details from the current frame's camera setup for legend positioning
        legend_anchor_center = current_scene_details['center']
        legend_anchor_scale_ref = current_scene_details['max_dim']

        if args.terrain_path and terrain_collection:
            bpy.app.handlers.frame_change_post.clear() # Clear old particle handlers
            if args.terrain_type == "OBJ":
                load_terrain_for_frame(
                    args.terrain_path, args.terrain_prefix, args.terrain_suffix, 
                    frame_num_abs, terrain_collection
                )
            else:  # PARTICLES
                success, particle_system_info = load_terrain_particles_for_frame(
                    args.terrain_path, args.terrain_prefix, frame_num_abs,
                    terrain_collection, args.particle_radius,
                    color_by=args.color_by, color_min=args.color_min, 
                    color_max=args.color_max, color_scheme=args.color_scheme,
                    show_legend=not args.no_legend,
                    legend_anchor_center=legend_anchor_center, # New
                    legend_anchor_scale_ref=legend_anchor_scale_ref # New
                )
                
                # Debug check for particle handler
                if success:
                    print(f"Registered {len(bpy.app.handlers.frame_change_post)} frame handlers")
                    # Force multiple frame updates to ensure particles are properly positioned
                    for _ in range(3):
                        bpy.context.scene.frame_set(frame_num_abs)
                else:
                    print("WARNING: No particles loaded for this frame")
            
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
            current_frame_index_for_stable_path, 
            args.frame_end - args.frame_start + 1,
            camera_data_for_stable_path # Pass the precalculated data if available
        )
        
        # TEMPORARY HACK: Hide all objects except tires
        hide_all_except_tires()
        
        # Render with error handling
        print(f"Rendering frame {frame_num_abs}")
        bpy.context.scene.render.filepath = os.path.join(view_folder, f"frame_{frame_num_abs:04d}.png")
        
        try:
            bpy.ops.render.render(write_still=True)
        except Exception as e:
            print(f"Error rendering frame {frame_num_abs}: {e}")
            
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