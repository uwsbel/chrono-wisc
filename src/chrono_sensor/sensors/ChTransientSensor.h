// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu M. Batagoda
// =============================================================================
//
// Container class for a camera sensor. This specifies a default ray tracing
// for cameras.
//
// =============================================================================

#ifndef ChTransientSensor_H
#define ChTransientSensor_H

#include "chrono_sensor/sensors/ChCameraSensor.h"


namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Camera class
class CH_SENSOR_API ChTransientSensor : public ChOptixSensor {
  public:
    /// @brief Constructor for the base camera class that defaults to a pinhole lens model
    /// @param parent A shared pointer to a body on which the sensor should be attached.
    /// @param updateRate The desired update rate of the sensor in Hz.
    /// @param offsetPose The desired relative position and orientation of the sensor on the body.
    /// @param w The width of the image the camera should generate.
    /// @param h The height of the image the camera should generate.
    /// @param hFOV The horizontal field of view of the camera lens.
    /// @param supersample_factor The number of rays that should be sampled per pixel for antialiasing.
    /// @param lens_model A enum specifying the desired lens model.
    /// @param use_gi Enable the global illumination, with significant decrease in performace
    /// @param gamma correction of the image, 1 for linear color space, 2.2 for sRGB
    /// @param use_fog whether to use fog on this camera
    ChTransientSensor(std::shared_ptr<chrono::ChBody> parent,  // object to which the sensor is attached
                   float updateRate,                        // rate at which the sensor updates
                   chrono::ChFrame<double> offsetPose,      // position of sensor relative to parent object
                   unsigned int w,                          // image width
                   unsigned int h,                          // image height
                   float hFOV,                              // horizontal field of view,
                   float tmin,
                   float tmax,
                   float tbins,
                   unsigned int supersample_factor = 1,     // number of samples per pixel for antialiasing
                   CameraLensModelType lens_model = CameraLensModelType::PINHOLE,
                   Integrator integrator = Integrator::TRANSIENT,
                   float gamma = 2.2,     // gamma correction value
                   bool use_fog = true   // whether to use fog
                   ); 

    /// camera class destructor
    ~ChTransientSensor();

   /// returns the camera's horizontal field of view. Vertical field of view is determined by the image aspect
    /// ratio and the lens model
    /// @return The horizontal field of view of the camera lens
    float GetHFOV() const { return m_hFOV; }

    /// returns the lens model type used for rendering
    /// @return An enum specifying which lens model is being used. (0: PINHOLE, 1: FOV, 2: Radial)
    CameraLensModelType GetLensModelType() const { return m_lens_model_type; }
    void SetLensModelType(CameraLensModelType lens_model) { m_lens_model_type = lens_model; }

    /// returns the lens model parameters
    /// @return LensParams struct of lens parameters. Will default to zeros for any terms not used. These are coverted
    /// for the inverse model
    LensParams GetLensParameters() const { return m_lens_parameters; }

    /// @briefSets the parameters for a radial lens distortion model.
    /// Parameters should be given for the forward model.
    /// The backward distortion model will the used and calculated from the forward parameters given.
    /// @param params the set of 3 radial parameters (k1,k2,k3)
    void SetRadialLensParameters(ChVector3f params);

    /// returns if the cemera requesting global illumination
    /// @return True if it does request
    bool GetUseGI() { return m_use_gi; }
    void SetUseGI(bool use_gi) { m_use_gi = use_gi; }

    /// returns the gamma correction value of this camera.
    /// 1 means no correction and the image is in linear color space. Useful for other ML applications
    /// 2.2 means the image is in sRGB color space. Useful for display
    /// @return Gamma value of the image
    float GetGamma() { return m_gamma; }
    void SetGamm(float gamma) { m_gamma = gamma; }

    /// Gets the number of samples per pixels in each direction used for super sampling
    /// @return the number of samples per pixel
    unsigned int GetSampleFactor() { return m_supersample_factor; }
    void SetSampleFactor(unsigned int sample_factor) { m_supersample_factor = sample_factor; }

    /// returns if the cemera should use fog as dictated by the scene
    /// @return True if it does request
    bool GetUseFog() { return m_use_fog; }
    void SetUseFog(bool use_fog) { m_use_fog = use_fog; }
    /// returns the  3x3 Intrinsic Matrix(K) of the camera
    /// @return 3x3 ChMatrix33<flaot> Intrinsic Matrix(K) of the camera
    ChMatrix33<float> GetCameraIntrinsicMatrix();

    /// returns the camera distortion coefficients k1, k2, k3
    /// @return ChVector3f of the camera distortion coefficients k1, k2, k3
    ChVector3f GetCameraDistortionCoefficients() { return m_distortion_params; }

    /// calculate the parameters for the inverse polynomial model
    static LensParams CalcInvRadialModel(ChVector3f params);

    float GetNumBins() { return m_tbins; }
    float GetTmin() { return m_tmin; }
    float GetTmax() { return m_tmax; }




  private:
    float m_hFOV;                           ///< the horizontal field of view of the sensor
    unsigned int m_supersample_factor;      ///< super sampling factor for antialiasing
    CameraLensModelType m_lens_model_type;  ///< lens model used by the camera
    bool m_use_gi;                          ///< to hold reference to whether user what to use GI or not
    float m_gamma;                          ///< holds the camera's gamma value
    bool m_use_fog;                         ///< holds whether the camera follows the scene fog model
    LensParams m_lens_parameters;           ///< lens parameters when applicable
    float m_width; // width of the image formed
    float m_height; // height of the image formed
    ChVector3f m_distortion_params = {0.f, 0.f, 0.f};
    float m_tmin;
    float m_tmax;
    float m_tbins;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
