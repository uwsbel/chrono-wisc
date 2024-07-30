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
// Container class for a camera sensor
//
// =============================================================================

#include "chrono_sensor/sensors/ChTransientSensor.h"
#include "chrono_sensor/optix/ChFilterOptixRender.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"


namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChTransientSensor::ChTransientSensor(std::shared_ptr<chrono::ChBody> parent,
                                             float updateRate,
                                             chrono::ChFrame<double> offsetPose,
                                             unsigned int w,                   // image width
                                             unsigned int h,                   // image height
                                             float hFOV,                       // horizontal field of view
                                             float tmin,
                                             float tmax,
                                             float tbins,
                                             unsigned int supersample_factor,  // super sampling factor
                                             CameraLensModelType lens_model,   // lens model to use
                                             bool use_gi,                      // 1 to use Global Illumination
                                             float gamma,                      // 1 for linear color space, 2.2 for sRGB
                                             bool use_fog                      // whether to use fog on this camera
                                             )
    : m_hFOV(hFOV),
      m_supersample_factor(supersample_factor),
      m_lens_model_type(lens_model),
      m_use_gi(use_gi),
      m_gamma(gamma),
      m_use_fog(use_fog),
      m_lens_parameters({}),
      m_width(w),
      m_height(h),
      m_tmin(tmin),
      m_tmax(tmax),
      m_tbins(tbins),
      ChOptixSensor(parent, updateRate, offsetPose, w, h) {

    m_pipeline_type = PipelineType::TANSIENT;


    m_filters.push_back(chrono_types::make_shared<ChFilterImageHalf4ToRGBA8>());

    // if (m_supersample_factor > 1) {
    //     m_filters.push_back(
    //         chrono_types::make_shared<ChFilterImgAlias>(m_supersample_factor, "Image antialias filter"));
    // }

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChTransientSensor::~ChTransientSensor() {}

void ChTransientSensor::SetRadialLensParameters(ChVector3f params) {
    m_distortion_params = params;
    m_lens_parameters = CalcInvRadialModel(params);
}

 ChMatrix33<float> ChTransientSensor::GetCameraIntrinsicMatrix() {
    ChMatrix33<float> I;
    float focal_length = (m_width / 2) * tanf(m_hFOV / 2);
    I(0, 0) = focal_length;
    I(0, 1) = 0.f;
    I(0, 2) = m_width / 2;
    I(1, 0) = 0.f;
    I(1, 1) = focal_length;
    I(1, 2) = m_height / 2;
    I(2, 0) = 0.f;
    I(2, 1) = 0.f;
    I(2, 2) = 1.f;

    return I;
};
 LensParams ChTransientSensor::CalcInvRadialModel(ChVector3f params) {
    // coefficients directory without algorithm from
    // Drap, P., & Lefevre, J. (2016).
    // An Exact Formula for Calculating Inverse Radial Lens Distortions.
    // Sensors (Basel, Switzerland), 16(6), 807. https://doi.org/10.3390/s16060807

    LensParams p = {};

    double k1 = params.x();
    double k21 = k1 * k1;
    double k31 = k21 * k1;
    double k41 = k21 * k21;
    double k51 = k31 * k21;
    double k61 = k31 * k31;
    double k71 = k41 * k31;
    double k81 = k41 * k41;
    double k91 = k51 * k41;

    double k2 = params.y();
    double k22 = k2 * k2;
    double k32 = k2 * k22;
    double k42 = k22 * k22;

    double k3 = params.z();
    double k23 = k3 * k3;
    double k33 = k23 * k3;

    // k4 = 0

    p.a0 = -k1;
    p.a1 = (float)(3 * k21 - k2);
    p.a2 = (float)(-12 * k31 + 8 * k1 * k2 - k3);
    p.a3 = (float)(55 * k41 - 55 * k21 * k2 + 5 * k22 + 10 * k1 * k3);
    p.a4 = (float)(-273 * k51 + 364 * k31 * k2 - 78 * k1 * k22 - 78 * k21 * k3 + 12 * k2 * k3);
    p.a5 = (float)(1428 * k61 - 2380 * k41 * k2 + 840 * k21 * k22 - 35 * k32 + 560 * k31 * k3 - 210 * k1 * k2 * k3 +
                   7 * k23);
    p.a6 = (float)(-7752 * k71 + 15504 * k51 * k2 - 7752 * k31 * k22 + 816 * k1 * k32 - 3876 * k41 * k3 +
                   2448 * k21 * k2 * k3 - 136 * k22 * k3 - 136 * k1 * k23);
    p.a7 = (float)(43263 * k81 - 100947 * k61 * k2 + 65835 * k41 * k22 - 11970 * k21 * k32 + 285 * k42 +
                   26334 * k51 * k3 - 23940 * k31 * k2 * k3 + 3420 * k1 * k22 * k3 + 1710 * k21 * k23 - 171 * k2 * k23);
    p.a8 = (float)(-246675 * k91 + 657800 * k71 * k2 - 531300 * k51 * k22 + 141680 * k31 * k32 - 8855 * k1 * k42 -
                   177100 * k61 * k3 + 212520 * k41 * k2 * k3 - 53130 * k21 * k22 * k3 + 1540 * k32 * k3 -
                   17710 * k31 * k23 + 4620 * k1 * k2 * k23 - 70 * k33);

    return p;
}


/*ChMatrix33<float> GetCameraIntrinsciMatrix() {
    
}*/

}  // namespace sensor
}  // namespace chrono
