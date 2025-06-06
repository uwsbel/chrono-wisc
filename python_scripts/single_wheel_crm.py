import bpy
import math
import random
import os
import sys
import mathutils
import gc

# define different camera settings, this will be used to create different camera location, pointed at different location, and write different image name
# there will be four types of camera settings: side view, focused view, top view, and isometric view
camera_settings = {
    "side_view": {
        "location": (0, -2, 0),
        "point_at": (0, 0, 0),
        "file_name": "side_view"
    },
    "focused_view": {
        "location": (-0.2, 0.15, 0.35),
        "point_at": (-0.5, 0.15, -0.3),
        "file_name": "focused_view"
    },
    "top_view": {
        "location": (0, 0, 2),
        "point_at": (0, 0, 0),
        "file_name": "top_view"
    },
    "isometric_view": {
        "location": (1, -1, 1),
        "point_at": (0, 0, 0),
        "file_name": "isometric_view"
    },
    "side_view_no_shadow": {
        "location": (0, -2, 0),
        "point_at": (0, 0, 0),
        "file_name": "side_view_no_shadow"
    },
    "rotating_view": {
        "location": (1, -1, 1),  # Initial position (same as isometric)
        "point_at": (0, 0, 0),
        "file_name": "rotating_view"
    }
}

# Camera view mapping
camera_view_mapping = {
    1: 'side_view',
    2: 'focused_view',
    3: 'top_view',
    4: 'isometric_view',
    5: 'side_view_no_shadow',
    6: 'rotating_view'
}

# Global constants
half_drum_width = 1
step_format = "%04d"
ground_plane_pos = -0.05
particle_radius = 0.005

# Global variables to track resources
current_particle_handler = None


def point_at(obj, target, roll=0):
    """
    Rotate obj to look at target
    :arg obj: the object to be rotated. Usually the camera
    :arg target: the location (3-tuple or Vector) to be looked at
    :arg roll: The angle of rotation about the axis from obj to target in radians. 
    """
    if not isinstance(target, mathutils.Vector):
        target = mathutils.Vector(target)
    loc = obj.location
    # direction points from the object to the target
    direction = target - loc

    quat = direction.to_track_quat('-Z', 'Y')

    # /usr/share/blender/scripts/addons/add_advanced_objects_menu/arrange_on_curve.py
    quat = quat.to_matrix().to_4x4()
    rollMatrix = mathutils.Matrix.Rotation(roll, 4, 'Z')

    # remember the current location, since assigning to obj.matrix_world changes it
    loc = loc.to_tuple()
    # obj.matrix_world = quat * rollMatrix
    # in blender 2.8 and above @ is used to multiply matrices
    # using * still works but results in unexpected behaviour!
    obj.matrix_world = quat @ rollMatrix
    obj.location = loc


def get_chassis_right_center(positions_drum):
    """Find the center right position of the chassis"""
    # given position drum, find the bounding box of the drum
    x_min = 1000
    x_max = -1000
    z_min = 1000
    z_max = -1000
    y_min = 1000
    y_max = -1000

    for pos in positions_drum:
        x, y, z = pos
        if x < x_min:
            x_min = x
        if x > x_max:
            x_max = x
        if z < z_min:
            z_min = z
        if z > z_max:
            z_max = z
        if y < y_min:
            y_min = y
        if y > y_max:
            y_max = y

    # Default values if no particles are found
    if len(positions_drum) == 0:
        return 0, 0, 0
    
    # get the center of the drum, this should be the position camera is pointing at
    vehicle_center_x = (x_min + x_max) / 2
    vehicle_center_z = (z_min + z_max) / 2
    vehicle_y_min = y_min

    return vehicle_center_x, vehicle_y_min, vehicle_center_z


def read_particle_file_half_drum(particle_dir, k):
    """Read particle files and create both soil and drum particles"""
    in_particle_file = particle_dir + "rigidBCE" + str(k) + ".csv"
    print("read drum file: " + in_particle_file)

    positions_drum = []
    positions_soil = []
    # in_particle_file has x, y, z compoenents, find the minimum of y
    y_min = 1000

    try:
        with open(in_particle_file) as f:
            line_count = 0
            count = 0
            for line in f:
                # if count > 9000000: break
                if line_count == 0:
                    line_count += 1
                    continue
                else:
                    line_count += 1
                    # you have to parse "x", "y", "z" and "r" from the variable "line"
                    line_seg = line.split(",")
                    # print(line_seg)
                    if len(line_seg) < 3:
                        continue
                    try:
                        y = float(line_seg[1])
                        if y < y_min:
                            y_min = y
                    except (ValueError, IndexError):
                        continue

            print("y_min: " + str(y_min))
            f.seek(0)
            line_count = 0
            count = 0
            for line in f:
                # if count > 9000000: break
                if line_count == 0:
                    line_count += 1
                    continue
                else:
                    line_count += 1
                    # you have to parse "x", "y", "z" and "r" from the variable "line"
                    line_seg = line.split(",")
                    if len(line_seg) < 3:
                        continue
                    try:
                        x, y, z = float(line_seg[0]), float(
                            line_seg[1]), float(line_seg[2])
                        if y > y_min + 0.02:
                            positions_drum.append((x, y, z))
                            count = count + 1
                    except (ValueError, IndexError):
                        continue
        print("total number of drum particles " + str(count))
    except FileNotFoundError:
        print(f"ERROR: Drum particle file not found: {in_particle_file}")
        print("Continuing with empty drum particles list.")

    # read fluid particles, ignore everything with y coordinates larger than 0.045
    in_particle_file = particle_dir + "fluid" + str(k) + ".csv"
    print("read particle file: " + in_particle_file)

    try:
        with open(in_particle_file) as f:
            line_count = 0
            count = 0
            for line in f:
                # if count > 9000000: break
                if line_count == 0:
                    line_count += 1
                    continue
                else:
                    line_count += 1
                    # you have to parse "x", "y", "z" and "r" from the variable "line"
                    line_seg = line.split(",")
                    if len(line_seg) < 3:
                        continue
                    try:
                        x, y, z = float(line_seg[0]), float(
                            line_seg[1]), float(line_seg[2])

                        # do not count fluid particles outside the drum
                        if abs(y) > half_drum_width:
                            continue

                        positions_soil.append((x, y, z))
                        count = count + 1
                    except (ValueError, IndexError):
                        continue
        print("total number of fluid particles " + str(count))
    except FileNotFoundError:
        print(f"ERROR: Fluid particle file not found: {in_particle_file}")
        print("Continuing with empty soil particles list.")

    return positions_drum, positions_soil


def setup_lights(view_type):
    """Set up lighting based on the view type"""
    # Remove existing lights
    for obj in bpy.data.objects:
        if obj.type == 'LIGHT':
            bpy.data.objects.remove(obj, do_unlink=True)

    if view_type == 'side_view_no_shadow':
        # Create a single, strong area light for even illumination
        light_data = bpy.data.lights.new(name="even_light", type='AREA')
        light_data.energy = 5000  # High energy for strong, even lighting
        light_data.size = 10  # Large size for soft shadows

        light_object = bpy.data.objects.new(
            name="even_light", object_data=light_data)
        bpy.context.collection.objects.link(light_object)

        # Position the light directly in front of the scene
        light_object.location = (0, -5, 0)
        light_object.rotation_euler = (math.radians(90), 0, 0)

    elif view_type in ['top_view']:
        # Create an area light for softer, more diffused lighting
        light_data = bpy.data.lights.new(name="main_light", type='AREA')
        light_data.energy = 1000  # Adjust as needed
        light_data.size = 5  # Larger size for softer shadows

        light_object = bpy.data.objects.new(
            name="main_light", object_data=light_data)
        bpy.context.collection.objects.link(light_object)

        light_object.location = (0, 0, 5)  # Directly above
        light_object.rotation_euler = (0, 0, 0)
        
        # Add a weak fill light to soften shadows
        fill_light_data = bpy.data.lights.new(name="fill_light", type='SUN')
        fill_light_data.energy = 0.5  # Very low energy
        fill_light_object = bpy.data.objects.new(
            name="fill_light", object_data=fill_light_data)
        bpy.context.collection.objects.link(fill_light_object)
        fill_light_object.rotation_euler = (
            math.radians(30), math.radians(30), 0)
    
    elif view_type in ['isometric_view', 'rotating_view']:
        # Create a more balanced three-point lighting setup for isometric and rotating views
        
        # Key light - main source of illumination
        key_light_data = bpy.data.lights.new(name="key_light", type='AREA')
        key_light_data.energy = 600  # Reduced energy to avoid overexposure
        key_light_data.size = 4  # Soft shadows
        key_light_data.color = (1.0, 0.98, 0.95)  # Slightly warm tint
        
        key_light_object = bpy.data.objects.new(
            name="key_light", object_data=key_light_data)
        bpy.context.collection.objects.link(key_light_object)
        key_light_object.location = (2, -2, 3)
        key_light_object.rotation_euler = (math.radians(60), math.radians(-15), math.radians(-45))
        
        # Fill light - softens shadows created by key light
        fill_light_data = bpy.data.lights.new(name="fill_light", type='AREA')
        fill_light_data.energy = 300  # Lower energy than key light
        fill_light_data.size = 5
        fill_light_data.color = (0.9, 0.9, 1.0)  # Slightly cool tint
        
        fill_light_object = bpy.data.objects.new(
            name="fill_light", object_data=fill_light_data)
        bpy.context.collection.objects.link(fill_light_object)
        fill_light_object.location = (-2, -2, 1.5)
        fill_light_object.rotation_euler = (math.radians(45), math.radians(30), math.radians(45))
        
        # Rim/back light - creates separation from background
        rim_light_data = bpy.data.lights.new(name="rim_light", type='SPOT')
        rim_light_data.energy = 400
        rim_light_data.spot_size = math.radians(60)
        rim_light_data.spot_blend = 0.5
        
        rim_light_object = bpy.data.objects.new(
            name="rim_light", object_data=rim_light_data)
        bpy.context.collection.objects.link(rim_light_object)
        rim_light_object.location = (0, 2, 3)
        rim_light_object.rotation_euler = (math.radians(120), 0, 0)
        
        # Ambient fill - very soft overall illumination
        ambient_light_data = bpy.data.lights.new(name="ambient_light", type='SUN')
        ambient_light_data.energy = 0.2  # Very low energy
        ambient_light_data.color = (0.8, 0.85, 1.0)  # Slightly blue tint for ambient light
        
        ambient_light_object = bpy.data.objects.new(
            name="ambient_light", object_data=ambient_light_data)
        bpy.context.collection.objects.link(ambient_light_object)
        ambient_light_object.rotation_euler = (math.radians(15), math.radians(45), 0)
        
    elif view_type == 'focused_view':
        # Softer lighting for focused view
        main_light_data = bpy.data.lights.new(name="main_light", type='AREA')
        main_light_data.energy = 800  # Lower energy
        main_light_data.size = 3  # Medium size for softer shadows
        
        main_light_object = bpy.data.objects.new(
            name="main_light", object_data=main_light_data)
        bpy.context.collection.objects.link(main_light_object)
        main_light_object.location = (1, -1, 2)
        main_light_object.rotation_euler = (
            math.radians(30), 0, math.radians(-45))
        
        # Add a fill light to soften shadows
        fill_light_data = bpy.data.lights.new(name="fill_light", type='AREA')
        fill_light_data.energy = 300
        fill_light_data.size = 2
        
        fill_light_object = bpy.data.objects.new(
            name="fill_light", object_data=fill_light_data)
        bpy.context.collection.objects.link(fill_light_object)
        fill_light_object.location = (-1, -1, 1)
        fill_light_object.rotation_euler = (
            math.radians(30), math.radians(30), math.radians(45))
            
        # Ambient light for overall scene illumination
        ambient_light_data = bpy.data.lights.new(name="ambient_light", type='SUN')
        ambient_light_data.energy = 0.2
        
        ambient_light_object = bpy.data.objects.new(
            name="ambient_light", object_data=ambient_light_data)
        bpy.context.collection.objects.link(ambient_light_object)
        ambient_light_object.rotation_euler = (
            math.radians(30), math.radians(30), 0)
    else:
        # For side view, keep the original two-point lighting setup but with improved balance
        light_data = bpy.data.lights.new(name="light_2.80", type='POINT')
        light_data.energy = 1500  # Reduced from 2000

        light_object = bpy.data.objects.new(
            name="light_2.80", object_data=light_data)
        bpy.context.collection.objects.link(light_object)
        light_object.location = (2, -5., 3.)

        second_light_data = bpy.data.lights.new(
            name="second_light", type='POINT')
        second_light_data.energy = 1500  # Reduced from 2000

        second_light_object = bpy.data.objects.new(
            name="second_light", object_data=second_light_data)
        bpy.context.collection.objects.link(second_light_object)
        second_light_object.location = (-2, -5., 3.)


def calculate_rotating_camera_position(frame, start_frame, end_frame):
    """
    Calculate the position of a camera that rotates around the wheel.
    
    Args:
        frame: Current frame number
        start_frame: First frame of the sequence
        end_frame: Last frame of the sequence
        
    Returns:
        Tuple of (x, y, z) camera position
    """
    # Calculate progress from 0 to 1
    if end_frame == start_frame:
        progress = 0
    else:
        progress = (frame - start_frame) / (end_frame - start_frame)
    
    # Calculate angle in radians (0 to 2π)
    angle = progress * 2 * math.pi
    
    # Distance from center (radius of circular path)
    distance = 2.0
    
    # Height of camera
    height = 1.0
    
    # Calculate camera position
    x = distance * math.sin(angle)
    y = distance * math.cos(angle)
    z = height
    
    return (x, y, z)


def setup_scene_for_frame(k, particle_dir, mesh_dir, CAMERA_VIEW, start_frame=0, end_frame=1000):
    """Set up the scene for rendering a specific frame"""
    global current_particle_handler
    
    # Clear any existing frame handlers
    bpy.app.handlers.frame_change_post.clear()
    current_particle_handler = None
    
    # Read particles
    positions_drum, positions_soil = read_particle_file_half_drum(
        particle_dir, k)

    vehicle_center_x, vehicle_y_min, vehicle_center_z = get_chassis_right_center(
        positions_drum)

    bpy.ops.wm.read_factory_settings(use_empty=True)

    scene = bpy.context.scene
    scene.objects.keys()

    # Add ground plane
    bpy.ops.mesh.primitive_plane_add(size=20.0, calc_uvs=True, enter_editmode=False, align='WORLD',
                                     location=(0.0, 0.0, ground_plane_pos))

    # Set up particle system
    context = bpy.context

    # Create materials
    material_drum = bpy.data.materials.new(name="BlueMaterial")
    material_drum.diffuse_color = (0.2, 0.2, 0.6, 1)  # purple color

    material_soil = bpy.data.materials.new(name="GrayMaterial")
    material_soil.diffuse_color = (0.1, 0.1, 0.1, 0.1)  # Gray color

    # Create instance objects - only if we have soil particles
    if len(positions_soil) > 0:
        bpy.ops.mesh.primitive_ico_sphere_add(radius=1, location=(50, 50, 50))
        ico_gray = context.object
        ico_gray.data.materials.append(material_soil)

        bpy.ops.mesh.primitive_cube_add(size=0.0001)
        gray_cube = context.object

        # Set up particle system
        gray_ps = gray_cube.modifiers.new(
            "SomeName", 'PARTICLE_SYSTEM').particle_system
        gray_psname = gray_ps.name

        gray_ps.settings.count = len(positions_soil)
        gray_ps.settings.lifetime = 1000
        gray_ps.settings.frame_start = gray_ps.settings.frame_end = 1
        gray_ps.settings.render_type = "OBJECT"
        gray_ps.settings.instance_object = ico_gray

        # Define particle handler
        def particle_handler_soil(scene, depsgraph):
            ob = depsgraph.objects.get(gray_cube.name)
            if ob and len(positions_soil) > 0:
                ps = ob.particle_systems[gray_psname]
                for m, particle in enumerate(ps.particles):
                    if m < len(positions_soil):
                        setattr(particle, "location", positions_soil[m])
                        setattr(particle, "size", particle_radius)

        # Register frame handler
        current_particle_handler = particle_handler_soil
        bpy.app.handlers.frame_change_post.append(particle_handler_soil)
        bpy.context.scene.frame_current = 2

    # Import wheel model - change to use OBJ instead of VTK
    in_wheel_file = mesh_dir + "wheel." + str(k) + ".obj"  # Change extension to .obj
    print("Reading wheel obj file: " + in_wheel_file)

    mesh_object = None
    
    # Check if the wheel file exists
    if not os.path.isfile(in_wheel_file):
        print(f"ERROR: Wheel file not found: {in_wheel_file}")
        # Create a dummy wheel instead of exiting
        bpy.ops.mesh.primitive_cylinder_add(radius=0.3, depth=0.2, location=(0, 0, 0))
        mesh_object = bpy.context.active_object
        mesh_object.name = "Wheel"
        print("Created placeholder wheel")
    else:
        # Try to import the OBJ file
        try:
            # Try to load OBJ with appropriate operator based on Blender version
            if hasattr(bpy.ops.wm, 'obj_import'):
                # Newer Blender versions
                bpy.ops.wm.obj_import(filepath=in_wheel_file)
                if len(bpy.context.selected_objects) > 0:
                    mesh_object = bpy.context.selected_objects[0]
                    mesh_object.name = "Wheel"
            elif hasattr(bpy.ops.import_scene, 'obj'):
                # Older Blender versions
                bpy.ops.import_scene.obj(filepath=in_wheel_file)
                if len(bpy.context.selected_objects) > 0:
                    mesh_object = bpy.context.selected_objects[0]
                    mesh_object.name = "Wheel"
            else:
                # Fallback: Create a dummy wheel
                bpy.ops.mesh.primitive_cylinder_add(radius=0.3, depth=0.2, location=(0, 0, 0))
                mesh_object = bpy.context.active_object
                mesh_object.name = "Wheel"
                print("Created placeholder wheel - no OBJ import operator available")
            
        except Exception as e:
            print(f"Error importing wheel file: {e}")
            # Create a dummy wheel instead of exiting
            bpy.ops.mesh.primitive_cylinder_add(radius=0.3, depth=0.2, location=(0, 0, 0))
            mesh_object = bpy.context.active_object
            mesh_object.name = "Wheel"
            print("Created placeholder wheel due to import error")

    # Apply material to wheel if we have one
    if mesh_object:
        if not mesh_object.material_slots:
            material = bpy.data.materials.new(name="TitaniumMaterial")
            mesh_object.data.materials.append(material)
        else:
            material = mesh_object.material_slots[0].material

        material.use_nodes = True
        material.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (
            0.0, 0.3, 0.0, 1)
        material.node_tree.nodes["Principled BSDF"].inputs["Metallic"].default_value = 0.8
        material.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.5

        # Rotate the wheel - check if we can access VIEW_3D areas
        try:
            areas = [area for area in bpy.context.screen.areas if area.type == 'VIEW_3D']
            if areas:
                area = areas[0]
                with bpy.context.temp_override(area=area):
                    bpy.ops.transform.rotate(value=(math.pi * 0.5), orient_axis='X')
        except Exception as e:
            print(f"Warning: Could not rotate wheel: {e}")

    # Update view layer
    bpy.context.view_layer.update()

    # Add camera
    bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', rotation=(
        1.4, 0.0, 0.0), scale=(5.0, 5.0, 5.0))

    # Set up camera
    cam = bpy.data.objects["Camera"]
    
    # Handle rotating camera view separately
    if CAMERA_VIEW == 'rotating_view':
        cam_pos = calculate_rotating_camera_position(k, start_frame, end_frame)
        cam.location = (vehicle_center_x + cam_pos[0], vehicle_y_min + cam_pos[1], vehicle_center_z + cam_pos[2])
        # Always point at the wheel
        point_at(cam, (vehicle_center_x, vehicle_y_min, vehicle_center_z), roll=math.radians(0))
    else:
        # Normal camera setup for other views
        cam_relative_loc = camera_settings[CAMERA_VIEW]["location"]
        cam_point_at_loc = camera_settings[CAMERA_VIEW]["point_at"]
        cam.location = (vehicle_center_x + cam_relative_loc[0], vehicle_y_min +
                        cam_relative_loc[1], vehicle_center_z + cam_relative_loc[2])
        point_at(cam, (vehicle_center_x + cam_point_at_loc[0], vehicle_y_min +
                cam_point_at_loc[1], vehicle_center_z + cam_point_at_loc[2]), roll=math.radians(0))

    # Set active camera
    scene.camera = bpy.context.object

    # Set up GPU rendering
    scene.cycles.device = 'GPU'
    prefs = bpy.context.preferences
    
    # Handle cycles preferences with try-except to prevent issues with different Blender versions
    try:
        cprefs = prefs.addons['cycles'].preferences

        # Try to set GPU device type
        for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
            try:
                cprefs.compute_device_type = compute_device_type
                break
            except (TypeError, AttributeError):
                pass

        # Enable all devices
        try:
            cprefs.get_devices()
            for device in cprefs.devices:
                device.use = True
        except (AttributeError, RuntimeError):
            pass
    except (KeyError, AttributeError):
        print("Warning: Could not configure cycles GPU rendering preferences")

    # Set up lighting
    setup_lights(CAMERA_VIEW)

    # Configure camera and render settings
    bpy.context.scene.camera.data.clip_end = 100
    
    # Configure render settings with safeguards
    try:
        bpy.context.scene.render.engine = 'CYCLES'
        if hasattr(bpy.context.scene, 'cycles'):
            bpy.context.scene.cycles.device = 'GPU'
            bpy.context.scene.cycles.samples = 80
        bpy.context.scene.render.resolution_x = 1080
        bpy.context.scene.render.resolution_y = 960
    except Exception as e:
        print(f"Warning: Could not fully configure render settings: {e}")

    return scene


def cleanup_resources():
    """Clean up resources to prevent memory leaks and crashes"""
    global current_particle_handler
    
    # Clear frame handlers
    if current_particle_handler in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(current_particle_handler)
    bpy.app.handlers.frame_change_post.clear()
    current_particle_handler = None
    
    # Remove particles and objects
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and obj.name not in ['Camera', 'Wheel']:
            bpy.data.objects.remove(obj, do_unlink=True)
    
    # Force garbage collection
    gc.collect()


def main():
    print("Here")
    # Check arguments
    if len(sys.argv) < 2:
        print(
            "Usage: blender --background --python single_wheel_crm.py -- <camera_view_number> [simulation_folder] [start_frame] [end_frame]")
        return

    # Get camera view from command line
    try:
        camera_view_number = int(sys.argv[1])
        CAMERA_VIEW = camera_view_mapping.get(
            camera_view_number, 'side_view_no_shadow')
    except (IndexError, ValueError):
        print("Camera view must be a number between 1-6")
        CAMERA_VIEW = 'side_view_no_shadow'

    print("camera view:", CAMERA_VIEW)

    # Check if simulation directory is provided
    if len(sys.argv) >= 3:
        sim_folder = sys.argv[2]
        if not sim_folder.endswith('/'):
            sim_folder += '/'
    else:
        # Default simulation folder if not provided
        ps = 1
        s = 0.010
        d0 = 1.2
        av = 0.5
        slope = 1
        sim_folder = "../DEMO_OUTPUT/FSI_Viper_Single_Wheel2/ps_" + \
            str(ps) + "_s_" + f"{s:.3f}" + "_d0_" + str(d0) + \
            "_av_" + str(av) + "/" + str(slope) + "/"

    print("Using simulation folder:", sim_folder)

    # Set up directories
    particle_dir = sim_folder + "particles/"
    mesh_dir = sim_folder + "obj/"
    out_dir = sim_folder + "images/"

    # Create output directories
    os.makedirs(out_dir, exist_ok=True)
    for view in camera_settings.keys():
        os.makedirs(os.path.join(out_dir, view), exist_ok=True)

    # Get start and end frames from command line if provided
    my_start_frame = 0
    my_end_frame = 499

    if len(sys.argv) >= 4:
        try:
            my_start_frame = int(sys.argv[3])
        except (IndexError, ValueError):
            print(f"Invalid start frame, using default: {my_start_frame}")

    if len(sys.argv) >= 5:
        try:
            my_end_frame = int(sys.argv[4])
        except (IndexError, ValueError):
            print(f"Invalid end frame, using default: {my_end_frame}")

    print(f"Processing frames {my_start_frame} to {my_end_frame}")

    # Find available wheel files to determine valid frame range
    available_frames = []
    if os.path.exists(mesh_dir):
        for filename in os.listdir(mesh_dir):
            # Check for both OBJ and VTK files
            if filename.startswith("wheel.") and (filename.endswith(".obj") or filename.endswith(".vtk")):
                try:
                    frame_num = int(filename.split(".")[1])
                    available_frames.append(frame_num)
                except (IndexError, ValueError):
                    continue

    if available_frames:
        available_frames.sort()
        print(f"Found {len(available_frames)} wheel files")
        print(
            f"Available frame range: {min(available_frames)} to {max(available_frames)}")

        # Adjust frame range based on available files
        if not (my_start_frame in available_frames):
            closest_start = min(
                available_frames, key=lambda x: abs(x - my_start_frame))
            print(
                f"Adjusting start frame from {my_start_frame} to {closest_start} based on available files")
            my_start_frame = closest_start

        if not (my_end_frame in available_frames) or my_end_frame > max(available_frames):
            closest_end = min(available_frames,
                              key=lambda x: abs(x - my_end_frame))
            print(
                f"Adjusting end frame from {my_end_frame} to {closest_end} based on available files")
            my_end_frame = closest_end
    else:
        print("WARNING: No wheel files found. Will use placeholder wheels for all frames.")

    # Process frames
    for k in range(my_start_frame, my_end_frame + 1, 1):
        try:
            # Clean up resources from previous frame
            cleanup_resources()
            
            # Set up the scene for this frame
            scene = setup_scene_for_frame(
                k, particle_dir, mesh_dir, CAMERA_VIEW, my_start_frame, my_end_frame)

            # Configure output path
            output_path = os.path.join(
                out_dir, CAMERA_VIEW, camera_settings[CAMERA_VIEW]["file_name"] + "_" + step_format % k + ".png")
            bpy.context.scene.render.filepath = output_path

            # Set render format
            bpy.context.scene.render.image_settings.color_mode = 'RGBA'
            bpy.context.scene.render.image_settings.file_format = 'PNG'

            # Render the frame
            print(f"Rendering frame {k} to {output_path}")
            bpy.ops.render.render(write_still=True)
            
            # Explicitly update and redraw to ensure frame has completed
            bpy.context.view_layer.update()
            
            # Clean up resources after rendering
            cleanup_resources()
            
        except Exception as e:
            print(f"Error rendering frame {k}:", e)
            cleanup_resources()
            continue
    
    # Final cleanup
    cleanup_resources()
    print("Rendering complete")


if __name__ == "__main__":
    main()
