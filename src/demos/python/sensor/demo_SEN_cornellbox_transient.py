import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time
import numpy as np
import matplotlib.pyplot as plt
import cv2



# -----------------
# Camera parameters
# -----------------


# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model="CONST_NORMAL"      # Gaussian noise with constant mean and standard deviation
# noise_model="PIXEL_DEPENDENT"   # Pixel dependent gaussian noise
# noise_model="RESPONSE_FUNCTION" # Noise model based on camera's response and parameters
noise_model = "NONE"              # No noise model

# Camera lens model
# Either PINHOLE or FOV_LENS
lens_model = sens.PINHOLE

# Update rate in Hz
update_rate = 30
transCam_update_rate = 1

# Image width and height
image_width = 500
image_height = 500

# Camera's horizontal field of view
fov = 1.408

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure (in seconds) of each image
exposure_time = 0

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 20.0

# Save camera images
save = True

# Render camera images
vis = False

# Output directory
out_dir = "SENSOR_OUTPUT/CornellBoxTransientPychrono/"


# -----------------
def main():
    mphysicalSystem = chrono.ChSystemNSC()


    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(
        "sensor/geometries/box.obj"), False, True)
    

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Cornell Box")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    manager = sens.ChSensorManager(mphysicalSystem)
    b = sens.Background()
    b.model = sens.BackgroundMode_SOLID_COLOR
    b.zenith  = chrono.ChVector3f(0,0,0)
    manager.scene.SetBackground(b)

    manager.SetRayRecursions(4)

    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVector3f(0,0,3.8), chrono.ChColor(intensity, intensity, intensity), 5.0)

    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-7, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(
        mesh_body,              # body camera is attached to
        update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                    # camera's horizontal field of view
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)
    cam.SetIntegrator(sens.Integrator_PATH)
    cam.SetUseGI(True)

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            image_width, image_height, "Steady State Image"))
        
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "steady_state/"))
    cam.PushFilter(sens.ChFilterRGBA8Access()) 
    manager.AddSensor(cam)

    # Add transient camera
    tmin = 6
    tmax = 25
    tbins = 128
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-7, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    transCam = sens.ChTransientSensor(
        mesh_body,              # body camera is attached to
        transCam_update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov,                   # camera's horizontal field of view
        tmin,
        tmax,
        tbins
    )
    transCam.SetIntegrator(sens.Integrator_TRANSIENT)
    transCam.SetSampleFactor(64)
    

    if save:
        transCam.PushFilter(sens.ChFilterSave(out_dir + "transience/"))

    transCam.PushFilter(sens.ChFilterRGBA8Access()) # Get access to transience buffer

    manager.AddSensor(transCam)
    ch_time = 0.0
    end_time = 1.0
    t1 = time.time()
    rgba8_buffer = None
    while (ch_time < end_time):
        
       
        manager.Update()

        mphysicalSystem.DoStepDynamics(step_size)

        rgba8_buffer = transCam.GetMostRecentRGBA8Buffer()
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

    # processing transient buffer
    print(rgba8_buffer)
    if (rgba8_buffer.HasData()):
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            rgba8_data = np.array(rgba8_data)
            rgba8_data = rgba8_data.ravel()
            rgba8_data = rgba8_data.reshape((tbins, image_width, image_height, 4))
            rgba8_data = rgba8_data[:, :, :, :3]
            rgba8_data = np.flip(rgba8_data, 1)
     
            print(f"Buffer Size: {rgba8_data.shape}") # Buffer size: (H, W * Tbins, 4)
    
            # write to video
            out = cv2.VideoWriter(out_dir + 'cornell_box.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (image_width, image_height))
            for i in range(tbins):
                out.write(cv2.cvtColor(rgba8_data[i], cv2.COLOR_RGB2BGR))
            out.release()

if __name__ == "__main__":
    main()