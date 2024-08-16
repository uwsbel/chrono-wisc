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
image_width = 32
image_height = 32

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
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/NLOS/"

alias_factor = 2048

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
    mphysicalSystem.Add(separator_wall)

    obj = chrono.ChBodyEasyBox(1,1,1,1000,True,False,phys_mat)
    obj.SetPos(chrono.ChVector3d(-1, 2.5, 3))
    obj.SetFixed(True)
    obj.GetVisualShape(0).SetMaterial(0, green)
    mphysicalSystem.Add(obj)
    

    manager.SetRayRecursions(4)

    intensity = 1e5
    manager.scene.AddSpotLight(chrono.ChVector3f(0, -2.5, 2), chrono.ChVector3f(5, 1.5, 2.5), chrono.ChColor(intensity,intensity,intensity), 5, 1*(np.pi/180), .5*(np.pi/180)) # (position, lookat, intensity, range, beam width, beam fall off)

    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0, -2.5, 2), chrono.QuatFromAngleAxis(40 * (np.pi / 180), chrono.ChVector3d(0,0,1)))
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

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            image_width, image_height, "Steady State Image"))
        
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "steady_state/"))
    cam.PushFilter(sens.ChFilterRGBA8Access()) 
    #manager.AddSensor(cam)

    # Add transient camera
    tmin = 2
    tmax = 30
    tbins = 512
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
    transCam.SetCollectionWindow(exposure_time)
    

    if save:
        transCam.PushFilter(sens.ChFilterSave(out_dir + "transience/"))

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
    timegateCam.PushFilter(sens.ChFilterRGBA8Access())
    #manager.AddSensor(timegateCam)

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

            gray_data = rgb_to_grayscale(rgba8_data)
     
            print(f"Buffer Size: {rgba8_data.shape}") # Buffer size: (H, W * Tbins, 4)
            print(f"Gray Buffer Size: {gray_data.shape}")
    
            # write to video
            out = cv2.VideoWriter(out_dir + 'nlos.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (image_width, image_height))
            for i in range(tbins):
                out.write(cv2.cvtColor(rgba8_data[i], cv2.COLOR_RGB2BGR))
            out.release()

            # write gray buffer to video
            out = cv2.VideoWriter(out_dir + 'nlos_gray.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (image_width, image_height))
            for i in range(tbins):
                out.write(cv2.cvtColor(gray_data[i], cv2.COLOR_GRAY2BGR))
            out.release()

            bin_labels =  np.linspace(tmin, tmax, tbins)
            label_indices = np.linspace(0, tbins - 1, 10, dtype=int)
            for i in range(0, image_width, 1):
                for j in range(0, image_height, 1):
                    if np.sum(gray_data[:, i, j]) == 0:
                        continue
                    plt.figure()
                    plt.plot(bin_labels,gray_data[:, i, j])
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