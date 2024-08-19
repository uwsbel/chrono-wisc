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
image_width = 640
image_height = 640

# Camera's horizontal field of view
fov = 90 * (np.pi / 180)

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
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/NLOS/"

alias_factor = 2

def rgb_to_grayscale(image):
    return np.dot(image[..., :3], [0.30, 0.59, 0.11]).astype(np.uint8)
# -----------------
def main():
    mphysicalSystem = chrono.ChSystemNSC()
    phys_mat = chrono.ChContactMaterialNSC()
    
    white = chrono.ChVisualMaterial()
    white.SetDiffuseColor(chrono.ChColor(1,1,1))
    white.SetBSDF(sens.BSDFType_DIFFUSE)

    green = chrono.ChVisualMaterial()
    green.SetDiffuseColor(chrono.ChColor(0,1,0))
    green.SetBSDF(sens.BSDFType_DIFFUSE)

    manager = sens.ChSensorManager(mphysicalSystem)
    b = sens.Background()
    b.model = sens.BackgroundMode_SOLID_COLOR
    b.zenith  = chrono.ChVector3f(0,0,0)
    manager.scene.SetBackground(b)

    floor = chrono.ChBodyEasyBox(1,1,1,1000,False,False,phys_mat)
    floor.SetPos(chrono.ChVector3d(0,0,0))
    floor.SetFixed(True)
    mphysicalSystem.Add(floor)

    relay_wall = chrono.ChBodyEasyBox(.1,10,5,1000,True,False,phys_mat)
    relay_wall.SetPos(chrono.ChVector3d(5, 0, 2.5))
    relay_wall.SetFixed(True)
    relay_wall.GetVisualShape(0).SetMaterial(0, white)
    mphysicalSystem.Add(relay_wall)

    separator_wall = chrono.ChBodyEasyBox(5, .1, 5,1000,True,False,phys_mat)
    separator_wall.SetPos(chrono.ChVector3d(-2.5, 0, 2.5))
    separator_wall.SetFixed(True)
    separator_wall.GetVisualShape(0).SetMaterial(0, white)
    #mphysicalSystem.Add(separator_wall)

    obj = chrono.ChBodyEasyBox(1,1,1,1000,True,False,phys_mat)
    obj.SetPos(chrono.ChVector3d(-1, 2.5, 3))
    obj.SetFixed(True)
    obj.GetVisualShape(0).SetMaterial(0, green)
    #mphysicalSystem.Add(obj)


    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/geometries/Z.obj"), False, True)
    

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Z")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetRot(chrono.QuatFromAngleZ(np.pi/2)*chrono.QuatFromAngleX(np.pi/2))
    mesh_body.SetPos(chrono.ChVector3d(0, 2.5, 2))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)
    

    manager.SetRayRecursions(4)

    intensity = 1e5
    manager.scene.AddSpotLight(chrono.ChVector3f(0, -2.5, 2), chrono.ChVector3f(5, 1.7, 2), chrono.ChColor(intensity,intensity,intensity), 5, 1*(np.pi/180), .5*(np.pi/180)) # (position, lookat, intensity, range, beam width, beam fall off)
    #manager.scene.AddPointLight(chrono.ChVector3f(0,0, 100), chrono.ChColor(10,10,10), 500.0)
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0,-2.5,2), chrono.QuatFromAngleAxis(40 * (np.pi / 180), chrono.ChVector3d(0,0,1)))
    cam = sens.ChCameraSensor(
        floor,              # body camera is attached to
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
    cam.SetUseDenoiser(True)
    cam.SetSampleFactor(4)

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            image_width, image_height, "Steady State Image"))
        
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "steady_state/"))
    cam.PushFilter(sens.ChFilterRGBA8Access()) 
    manager.AddSensor(cam)

    # Add transient camera
    tmin = 2
    tmax = 30
    tbins = 300
    transCam = sens.ChTransientSensor(
        floor,              # body camera is attached to
        transCam_update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                 # camera's horizontal field of view
    )
    transCam.SetIntegrator(sens.Integrator_MITRANSIENT)
    transCam.SetSampleFactor(alias_factor)
    transCam.SetTmin(tmin)
    transCam.SetTmax(tmax)
    transCam.SetNumBins(tbins)
    transCam.SetUseDenoiser(False)
    transCam.SetName("Transient Camera Sensor")
    transCam.SetLag(lag)
    transCam.SetNLOSaserSamples(True)
    transCam.SetDiscardDirectPaths(False)
    transCam.SetNLOSHiddenGeometrySampling(True)
    transCam.SetFilterBounces(-1)
    transCam.SetCollectionWindow(exposure_time)
    

    # if save:
    #     transCam.PushFilter(sens.ChFilterSave(out_dir + "transience/"))

    transCam.PushFilter(sens.ChFilterRGBA8Access()) # Get access to transience buffer
    manager.AddSensor(transCam)

        # Add time gated camera
    window_size = .1
    target_dist = 25
    tof_mode = sens.TIMEGATED_MODE_EXPONENTIAL
    timegateCam = sens.ChTransientSensor(
        floor,              # body camera is attached to
        transCam_update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                 # camera's horizontal field of view
    )
    timegateCam.SetSampleFactor(alias_factor)
    timegateCam.SetTargetDist(target_dist)
    timegateCam.SetWindowSize(window_size)
    timegateCam.SetTimeGatedMode(tof_mode)
    timegateCam.SetUseDenoiser(False)
    timegateCam.SetIntegrator(sens.Integrator_TIMEGATED)
    timegateCam.SetName("Time Gated Camera Sensor")
    timegateCam.SetLag(lag)
    timegateCam.SetCollectionWindow(exposure_time)
    
    if vis:
        timegateCam.PushFilter(sens.ChFilterVisualize(
            image_width, image_height, "Time Gated Image"))
    if save:
        timegateCam.PushFilter(sens.ChFilterSave(out_dir + "timegate/"))
    #timegateCam.PushFilter(sens.ChFilterRGBA8Access())
    #manager.AddSensor(timegateCam)

    ch_time = 0.0
    end_time = 1.0
    t1 = time.time()
    transient_buffer = None
    while (ch_time < end_time):
        
       
        manager.Update()

        mphysicalSystem.DoStepDynamics(step_size)

        #transient_buffer_rgb = transCam.GetMostRecentRGBA8Buffer()
        transient_buffer = transCam.GetMostRecentFloat4Buffer()

        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

    print(transient_buffer)
    if (transient_buffer.HasData()):
            transient_data = transient_buffer.GetFloat4Data()
            transient_data = np.array(transient_data)
            print(f"Buffer Size: {transient_data.shape}") # Buffer size: (H, W * Tbins, 4)
            transient_data = transient_data.ravel()
            transient_data = transient_data.reshape((tbins, image_width, image_height, 4))
            transient_data = transient_data[:, :, :, :3]
            transient_data = np.flip(transient_data, 1)
            print(f"Buffer Size: {transient_data.shape}") # Buffer size: (H, W * Tbins, 4)
            transient_data_uint8 = (transient_data * 255).astype(np.uint8)

      
    
            # write to video
            out = cv2.VideoWriter(out_dir + 'nlos.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (image_width, image_height))
            for i in range(tbins):
                out.write(cv2.cvtColor(transient_data_uint8[i], cv2.COLOR_RGB2BGR))
            out.release()

            # gray_data = rgb_to_grayscale(transient_data_uint8)
     
            # print(f"Buffer Size: {transient_data.shape}") # Buffer size: (H, W * Tbins, 4)
            # print(f"Gray Buffer Size: {gray_data.shape}")
            # # write gray buffer to video
            # out = cv2.VideoWriter(out_dir + 'nlos_gray.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (image_width, image_height))
            # for i in range(tbins):
            #     out.write(cv2.cvtColor(gray_data[i], cv2.COLOR_GRAY2BGR))
            # out.release()

            bin_labels =  np.linspace(tmin, tmax, tbins)
            label_indices = np.linspace(0, tbins - 1, 10, dtype=int)
            for i in range(0, image_width, 1):
                for j in range(0, image_height, 1):
                    if np.sum(transient_data[:, i, j]) == 0:
                        continue
                    plt.figure()
                    plt.plot(bin_labels,transient_data[:, i, j])
                    plt.xlabel('Path Length (m)')
                    #plt.xticks(label_indices, [f"{bin_labels[i]:.2f}" for i in label_indices], rotation=45)
                    plt.ylabel('Intensity')
                    plt.yscale('log')
                    plt.title(f'Transience at Pixel {i}, {j}')
                    plt.savefig(out_dir + 'histograms/' + f'pixel_{i}_{j}.png')
                    plt.close()
                    print(f'Pixel {i * image_height + j}/{image_width*image_height} done', end='\r')

if __name__ == "__main__":
    main()