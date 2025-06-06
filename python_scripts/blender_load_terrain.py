import bpy
import os

# === Configuration ===
folder_path   = "/home/harry/chrono-wisc/build/DEMO_OUTPUT/0.400000/soil_leveling_1"
file_prefix   = "fluid"
file_suffix   = "_surface.obj"
frame_rate    = 50
start_frame   = 200
total_frames  = 300

# --- Create (or get) a gray material with bump ---
mat_name = "Gray_Bumpy"
if mat_name in bpy.data.materials:
    gray_mat = bpy.data.materials[mat_name]
else:
    gray_mat = bpy.data.materials.new(name=mat_name)
    gray_mat.use_nodes = True

    nodes = gray_mat.node_tree.nodes
    links = gray_mat.node_tree.links
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

# --- Scene setup ---
scene = bpy.context.scene
scene.render.fps     = frame_rate
scene.frame_start    = start_frame
scene.frame_end      = start_frame + total_frames - 1

# --- Clear existing objects ---
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# === Import & animate sequence ===
for frame in range(start_frame,total_frames):
    filename = f"{file_prefix}{frame}{file_suffix}"
    filepath = os.path.join(folder_path, filename)
    if not os.path.exists(filepath):
        continue

    # import OBJ
    bpy.ops.wm.obj_import(
        filepath=filepath,
        forward_axis='Y',
        up_axis='Z'
    )

    # assign material + visibility keyframes
    for obj in bpy.context.selected_objects:
        if obj.type == 'MESH':
            # give it our bumpy gray material
            obj.data.materials.clear()
            obj.data.materials.append(gray_mat)

        obj.name = f"{file_prefix}{frame}"

        # hide before this frame
        obj.hide_render = True
        obj.hide_viewport = True
        obj.keyframe_insert(data_path="hide_render",   frame=frame-1)
        obj.keyframe_insert(data_path="hide_viewport", frame=frame-1)

        # show on this frame
        obj.hide_render = False
        obj.hide_viewport = False
        obj.keyframe_insert(data_path="hide_render",   frame=frame)
        obj.keyframe_insert(data_path="hide_viewport", frame=frame)

        # hide after this frame
        obj.hide_render = True
        obj.hide_viewport = True
        obj.keyframe_insert(data_path="hide_render",   frame=frame+1)
        obj.keyframe_insert(data_path="hide_viewport", frame=frame+1)
