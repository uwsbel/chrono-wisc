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
image_width = 8
image_height = 8

# Camera's horizontal field of view
fov = 45 * (np.pi / 180)

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

alias_factor = 64

def rgb_to_grayscale(image):
    return np.dot(image[..., :3], [0.30, 0.59, 0.11])
# -----------------
def main():
    mphysicalSystem = chrono.ChSystemNSC()
    phys_mat = chrono.ChContactMaterialNSC()
    
    white = chrono.ChVisualMaterial()
    white.SetDiffuseColor(chrono.ChColor(1,1,1))
    white.SetBSDF(sens.BSDFType_DIFFUSE)

    relay_wall_mat = chrono.ChVisualMaterial()
    relay_wall_mat.SetDiffuseColor(chrono.ChColor(1,1,1))
    relay_wall_mat.SetIsHiddenObject(False)
    relay_wall_mat.SetBSDF(sens.BSDFType_DIFFUSE)

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
    relay_wall.SetPos(chrono.ChVector3d(0, 0, 0))
    relay_wall.SetFixed(True)
    relay_wall.GetVisualShape(0).SetMaterial(0, white)
    mphysicalSystem.Add(relay_wall)

    separator_wall = chrono.ChBodyEasyBox(5, .1, 5,1000,True,False,phys_mat)
    separator_wall.SetPos(chrono.ChVector3d(-2.5, 0, 2.5))
    separator_wall.SetFixed(True)
    separator_wall.GetVisualShape(0).SetMaterial(0, white)
    #mphysicalSystem.Add(separator_wall)

    obj = chrono.ChBodyEasyBox(.1,1,1,1000,True,False,phys_mat)
    obj.SetPos(chrono.ChVector3d(-1, 1, 0))
    obj.SetFixed(True)
    obj.GetVisualShape(0).SetMaterial(0, white)
    #mphysicalSystem.Add(obj)


    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/geometries/Z.obj"), False, True)
    

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Z")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetRot(chrono.QuatFromAngleZ(np.pi/2)*chrono.QuatFromAngleX(np.pi/2))
    mesh_body.SetPos(chrono.ChVector3d(-1,0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mesh_body.GetVisualShape(0).SetMaterial(0, white)
    mphysicalSystem.Add(mesh_body)
    

    manager.SetRayRecursions(4)

    intensity = 1e1
    camera_pos = chrono.ChVector3d(-0.25,0.5,0)
    manager.scene.AddSpotLight(chrono.ChVector3f(-0.25,0.5,0), chrono.ChVector3f(0,0.5,0), chrono.ChColor(intensity,intensity,intensity), 5, fov, fov * 0.5) # (position, lookat, intensity, range, beam width, beam fall off)
    #manager.scene.AddPointLight(chrono.ChVector3f(-1,0,0), chrono.ChColor(1,1,1), 500.0)
    offset_pose = chrono.ChFramed(
        camera_pos, chrono.QuatFromAngleAxis(0* (np.pi / 180), chrono.ChVector3d(0,0,1)))
    cam = sens.ChCameraSensor(
        floor,              # body camera is attached to
        update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        500,            # image width
        500,           # image height
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
    tmin = 0.2 # 2 cm
    tmax = 4
    tbins = 300
    transCam = sens.ChTransientSensor(
        floor,              # body camera is attached to
        transCam_update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                 # camera's horizontal field of view
    )
    transCam.SetIntegrator(sens.Integrator_TRANSIENT)
    transCam.SetSampleFactor(alias_factor)
    transCam.SetTmin(tmin)
    transCam.SetTmax(tmax)
    transCam.SetNumBins(tbins)
    transCam.SetUseDenoiser(False)
    transCam.SetName("Transient Camera Sensor")
    transCam.SetLag(lag)
    transCam.SetNLOSaserSamples(True)
    transCam.SetDiscardDirectPaths(False)
    transCam.SetNLOSHiddenGeometrySampling(False)
    transCam.SetFilterBounces(-1)
    transCam.SetCollectionWindow(exposure_time)
    transCam.SetGamma(1)
    

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
    print(transCam.GetNLOSHiddenGeometrySampling())
    if (transient_buffer is not None and transient_buffer.HasData()):
            transient_data = transient_buffer.GetFloat4Data()
            transient_data = np.array(transient_data)
            print(f"Buffer Size: {transient_data.shape}") # Buffer size: (H, W * Tbins, 4)
            transient_data = transient_data.ravel()
            transient_data = transient_data.reshape((tbins, image_width, image_height, 4))
            transient_data = transient_data[:, :, :, :3]
            transient_data = np.flip(transient_data, 1)
            print(f"Buffer Size: {transient_data.shape}") # Buffer size: (H, W * Tbins, 4)
            # gamma correction
            transient_data_rgba = np.power(transient_data, 1/2.2)
            transient_data_rgba = (transient_data_rgba * 255).astype(np.uint8)

      
    
            # write to video
            out = cv2.VideoWriter(out_dir + 'nlos.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (image_width, image_height))
            for i in range(tbins):
                out.write(cv2.cvtColor(transient_data_rgba[i], cv2.COLOR_RGB2BGR))
            out.release()

            transient_data = rgb_to_grayscale(transient_data)
            # supersample transient data from 8x8 to 4x4
            transient_data_tmp = np.zeros((tbins, image_width//2, image_height//2))
            for i in range(0, image_width, 2):
                for j in range(0, image_height, 2):
                    transient_data_tmp[:, i//2, j//2] = np.mean(transient_data[:, i:i+2, j:j+2], axis=(1, 2))
            transient_data = transient_data_tmp
     
            print(f"Buffer Size: {transient_data.shape}")
            # print(f"Gray Buffer Size: {gray_data.shape}")
            # # write gray buffer to video
            # out = cv2.VideoWriter(out_dir + 'nlos_gray.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (image_width, image_height))
            # for i in range(tbins):
            #     out.write(cv2.cvtColor(gray_data[i], cv2.COLOR_GRAY2BGR))
            # out.release()
            image_height_1, image_width_1 = transient_data.shape[1], transient_data.shape[2]
            bin_labels =  np.linspace(tmin, tmax, tbins)
            label_indices = np.linspace(0, tbins - 1, 10, dtype=int)
            for i in range(0, image_height_1, 1):
                for j in range(0, image_width_1, 1):
                    plt.figure()
                    plt.plot(bin_labels, transient_data[:, i, j])
                    plt.xlabel('Path Length (m)')
                    #plt.xticks(label_indices, [f"{bin_labels[i]:.2f}" for i in label_indices], rotation=45)
                    plt.ylabel('Intensity')
                    plt.yscale('log')
                    plt.title(f'Transience at Pixel {i}, {j}')
                    plt.savefig(out_dir + 'histograms/' + f'pixel_{i}_{j}.png')
                    plt.close()
                    print(f'Pixel {i * image_height_1 + j}/{transient_data.shape[1]*transient_data.shape[2]} done', end='\r')

if __name__ == "__main__":
    main()