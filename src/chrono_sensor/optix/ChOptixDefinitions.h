// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist, Han Wang
// =============================================================================
//
// =============================================================================

#ifndef CHOPTIXDEFINITIONS_H
#define CHOPTIXDEFINITIONS_H

#include <vector_types.h>
#include <optix_types.h>
#include <cuda_runtime_api.h>
#include <curand_kernel.h>
#include <cuda_fp16.h>

#ifdef USE_SENSOR_NVDB
    #include <nanovdb/NanoVDB.h>
    #include <nanovdb/util/GridHandle.h>
    #include <nanovdb/util/cuda/CudaDeviceBuffer.h>
#endif

struct half4 {
    __half x;
    __half y;
    __half z;
    __half w;
};

/// @addtogroup sensor_optix
/// @{

/// Ray types, used to determine the shading and miss functions for populating ray information
enum RayType {
    CAMERA_RAY_TYPE = 0,       /// camera rays
    SHADOW_RAY_TYPE = 1,       /// shadow rays
    LIDAR_RAY_TYPE = 2,        /// lidar rays
    RADAR_RAY_TYPE = 3,        /// radar rays
    SEGMENTATION_RAY_TYPE = 4, /// semantic camera rays 
    DEPTH_RAY_TYPE = 5,        /// depth camera rays
    PHYS_CAMERA_RAY_TYPE = 6,  /// physics-based camera rays
    NORMAL_RAY_TYPE = 7        /// normal camera rays
};

/// The type of lens model that camera can use for rendering
enum CameraLensModelType {
    PINHOLE,   ///< traditional computer graphics ideal camera model.
    FOV_LENS,  ///< Wide angle lens model based on single spherical lens.
    RADIAL     ///< Wide angle lens model based on polynomial fit
};

/// The different types of area lights that can exist in our current models
/*enum AreaLightType {
    CIRCLE,     ///< circular-disk area light with a radius value
    SQUARE,     ///< square-shaped area light with a side length value
    RECTANGLE   ///< retangular area light with a length and width value
}
*/

/// Parameters for an area light
struct AreaLight {
    float3 pos;         ///< the centre position of the area light from a global perspective
    float3 color;       ///< the light's color and intensity
    float max_range;    ///< the range at which 10% of the light's intensity remains
    float3 du;          ///< the vector signifying the length of the area light
    float3 dv;          ///< The vector signifying the width of the area light
};

/// Packed parameters of a point light
struct PointLight {
    float3 pos;       ///< the light's global position
    float3 color;     ///< the light's color and intensity
    float max_range;  ///< the range at which 10% of the light's intensity remains
};

/// Type of background to be spherically mapped when rays miss all objects in the scene
enum class BackgroundMode {
    SOLID_COLOR,     ///< single solid color defined by RGB
    GRADIENT,        ///< color gradient used for upper hemisphere
    ENVIRONMENT_MAP  ///< image used for spherical sky map
};

/// The parameters associated with camera miss data. A.K.A background data
struct CameraMissParameters {
    BackgroundMode mode;          ///< the mode to determine type of miss shading
    float3 color_zenith;          ///< the color at the zenith (ignored when using a sky map)
    float3 color_horizon;         ///< the color at the horizon (only used for GRADIENT)
    cudaTextureObject_t env_map;  ///< the texture object of the sky map (ignored when using a color mode)
};

/// The parameters for a camera miss record
struct MissParameters {
    CameraMissParameters camera_miss;
};

/// Inverse lens param for modeling polynomial forward model
struct LensParams {
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
    float a5;
    float a6;
    float a7;
    float a8;
};

struct PhysCameraGainParams {
    float defocus_gain;
    float defocus_bias;
    float vignetting_gain;
    float aggregator_gain;
    float3 expsr2dv_gains;
    float3 expsr2dv_biases;
    float expsr2dv_gamma;
    // std::string expsr2dv_crf_type;
    int expsr2dv_crf_type; // camera response function type, 0: gamma_correct, 1: sigmoid, 2: linear
};

struct PhysCameraNoiseParams {
    float3 dark_currents;   // dark currents and hot-pixel noises, [electron/sec]
    float3 noise_gains;     // temporal noise gains, [1/1]
    float3 STD_reads;       // STDs of FPN and readout noises, [electron]
    int FPN_rng_seed;       // seed of random number generator for readout and FPN noises
};

/// The parameters needed to define a camera
struct CameraParameters {
    float hFOV;                        ///< horizontal field of view
    CameraLensModelType lens_model;    ///< lens model to use
    LensParams lens_parameters;        ///< lens fitting parameters (if applicable)
    unsigned int super_sample_factor;  ///< number of samples per pixel in each dimension
    float gamma;                       ///< camera's gamma value
    bool use_gi;                       ///< whether to use global illumination
    bool use_fog;                      ///< whether to use the scene fog model
    half4* frame_buffer;               ///< buffer of camera pixels
    half4* albedo_buffer;  ///< the material color of the first hit. Only initialized if using global illumination
    half4* normal_buffer;  ///< The screen-space normal of the first hit. Only initialized if using global illumination
                           ///< (screenspace normal)
    curandState_t* rng_buffer;  ///< The random number generator object. Only initialized if using global illumination
};

/// The parameters needed to define a physics-based camera
struct PhysCameraParameters {
    float hFOV;                         ///< horizontal field of view
    CameraLensModelType lens_model;     ///< lens model to use
    LensParams lens_parameters;         ///< lens fitting parameters (if applicable)
    unsigned int super_sample_factor;   ///< number of samples per pixel in each dimension
    float gamma;                        ///< camera's gamma value
    bool use_gi;                        ///< whether to use global illumination
    bool use_fog;                       ///< whether to use the scene fog model
    float aperture_num;          		///< F-number (or aperture number) = focal_length / aperture_diameter, [1/1]
    float expsr_time;                   ///< exposure time, [sec]
    float ISO;                          ///< ISO exposure gain, [1/1]
    float focal_length;                 ///< focal length, [m]
    float focus_dist;                   ///< focus distance, [m]
    float max_scene_light_amount;		///< maximum light amount in the scene, [lumen]
    float sensor_width;                 ///< equivalent width of the image sensor, [m]
    float pixel_size;                   ///< length of a pixel, [m]
    PhysCameraGainParams gain_params;	///< gain parameters in camera model, [1/1]
    PhysCameraNoiseParams noise_params;	///< noise model parameters in camera model
    half4* rgbd_buffer;                 ///< buffer in RGBD format for camera pixels and distance map
    half4* albedo_buffer;               ///< the material color of the first hit. Only initialized if using global illumination
    half4* normal_buffer;               ///< The screen-space normal of the first hit. Only initialized if using global illumination
                                        ///< (screenspace normal)
    // float* distance_buffer;             ///< buffer of distances between the camera pixel to the first hit point
    curandState_t* rng_buffer;          ///< The random number generator object. Only initialized if using global illumination
};

// Parameters for specifying a depth camera
struct DepthCameraParameters {
    float hFOV;                      ///< horizontal field of view
    CameraLensModelType lens_model;  ///< lens model to use
    LensParams lens_parameters;      ///< lens fitting parameters (if applicable)
    float* frame_buffer;             ///< buffer of class and instance ids
    curandState_t* rng_buffer;       ///< only initialized if using global illumination
    float max_depth;                 ///< maximum depth value for the depth camera
};

// Parameters for specifying a normal camera
struct NormalCameraParameters {
    float hFOV;                      ///< horizontal field of view
    CameraLensModelType lens_model;  ///< lens model to use
    LensParams lens_parameters;      ///< lens fitting parameters (if applicable)
    float3* frame_buffer;             ///< buffer of class and instance ids to put normal vector
    curandState_t* rng_buffer;       ///< only initialized if using global illumination
};

/// Parameters need to define a camera that generates semantic segmentation data
struct SemanticCameraParameters {
    float hFOV;                      ///< horizontal field of view
    CameraLensModelType lens_model;  ///< lens model to use
    LensParams lens_parameters;      ///< lens fitting parameters (if applicable)
    ushort2* frame_buffer;           ///< buffer of class and instance ids
    curandState_t* rng_buffer;       ///< only initialized if using global illumination
};

/// The shape of a lidar beam
enum class LidarBeamShape {
    RECTANGULAR,  ///< rectangular beam (inclusive of square beam)
    ELLIPTICAL    ///< elliptical beam (inclusive of circular beam)
};

/// Parameters used to define a lidar
struct LidarParameters {
    float max_vert_angle;          ///< angle of the top-most lidar channel
    float min_vert_angle;          ///< angle of the bottom-most lidar channel
    float hFOV;                    ///< horizontal field of view of the lidar
    float max_distance;            ///< maximum distance measureable by the lidar
    float clip_near;               ///< near clipping distance to geometric considerations
    unsigned short sample_radius;  ///< radius of samples for discretizing the lidar beams
    LidarBeamShape beam_shape;     ///< the beam shape
    float horiz_div_angle;         ///< divergence angle of the beam horizontally (in radians)
    float vert_div_angle;          ///< divergence angle of the beam vertically (in radians)
    float2* frame_buffer;          ///< buffer where the lidar data will be placed when generated
};

/// The mode used when determining what data the radar should return
enum class RadarReturnMode {
    RETURN,  ///< raw data mode
    TRACK    ///< object tracking mode
};

/// Parameters used to define a radar
struct RadarParameters {
    float vFOV;
    float hFOV;
    float max_distance;
    float clip_near;
    float horiz_div_angle;
    float vert_div_angle;
    float3 velocity;      ///< the velocity of the sensor
    float* frame_buffer;  ///< buffer where the radar data will be placed when generated
};

/// Parameters for specifying raygen programs
struct RaygenParameters {
    float t0;     ///< time of the first ray
    float t1;     ///< time of the last ray
    float3 pos0;  ///< sensor position at t0
    float4 rot0;  ///< sensor rotation at t0
    float3 pos1;  ///< sensor position at t1
    float4 rot1;  ///< sensor rotation at t1
    union {
        CameraParameters camera;                ///< the specific data when modeling a camera
        PhysCameraParameters phys_camera;       ///< the specific data when modeling a physics-based camera
        SemanticCameraParameters segmentation;  ///< the specific data when modeling a semantic segementation camera
        LidarParameters lidar;                  ///< the specific data when modeling a lidar
        RadarParameters radar;                  ///< the specific data when modeling a radar
        DepthCameraParameters depthCamera;      /// < the specific data when modeling a depth camera
        NormalCameraParameters normalCamera;    /// < the specific data when modeling a normal camera
    } specific;                                 ///< the data for the specific sensor
};

/// All the data to specific a triangle mesh
struct MeshParameters {              // pad to align 16 (swig doesn't support explicit alignment calls)
    float4* vertex_buffer;           ///< a device pointer to the mesh's vertices // size 8
    float4* normal_buffer;           ///< a device pointer to the mesh's normals //size 8
    float2* uv_buffer;               ///< a device pointer to the mesh's uv coordinates // size 8
    uint4* vertex_index_buffer;      ///< a device pointer to the mesh's vertex indices // size 8
    uint4* normal_index_buffer;      ///< a device pointer to the mesh's normal indices // size 8
    uint4* uv_index_buffer;          ///< a device pointer to the mesh's uv indices // size 8
    unsigned int* mat_index_buffer;  ///< a device pointer to the mesh's materials on a per face basis // size 8
    double pad;                      ///< padding to ensure 16 byte alignment // size 8
};

/// All parameters for specifying a material in optix
struct MaterialParameters {      // pad to align 16 (swig doesn't support explicit alignment calls)
    float3 Kd;                   ///< the diffuse color // size 12
    float3 Ks;                   ///< the specular color // size 12
    float3 Ke;                   /// < the emissive color // size 12>
    float anisotropy;            ///< the anisotropic value of material (0-1) // size 4
    float fresnel_exp;           ///< the fresnel exponent // size 4
    float fresnel_min;           ///< the minimum fresnel value (0-1) // size 4
    float fresnel_max;           ///< maximum fresnel value (0-1) // size 4
    float transparency;          ///< transparency value (0-1) // size 4
    float roughness;             ///< roughness value (0-1) // size 4
    float metallic;              ///< metallic value (0-1) // size 4
    float lidar_intensity;       ///< reflectivity in a lidar's wavelength (0-1) // size 4
    float radar_backscatter;     ///< reflectivity in a radar's wavelength (0-1) // size 4
    int use_specular_workflow;   ///< toggle between a specular workflow or a metallic/roughness PBR workflow // size 4
    cudaTextureObject_t kd_tex;  ///< a diffuse color texture // size 8
    cudaTextureObject_t kn_tex;  ///< a normal perterbation texture // size 8
    cudaTextureObject_t ks_tex;  ///< a specular color texture // size 8
    cudaTextureObject_t ke_tex; /// <an emissive color texture // size 8
    cudaTextureObject_t metallic_tex;   ///< a metalic color texture // size 8
    cudaTextureObject_t roughness_tex;  ///< a roughness texture // size 8
    cudaTextureObject_t opacity_tex;    ///< an opacity texture // size 8
    cudaTextureObject_t weight_tex;     ///< a weight texture for blended textures // size 8
    float2 tex_scale;                   ///< texture scaling // size 8
    unsigned short int class_id;        ///< a class id of an object // size 2
    unsigned short int instance_id;     ///< an instance id of an object // size 2
   
    int use_hapke;                      // toggle between disney and hapke shader // size 4
    float emissive_power;               // size 4

    // hapke parameters
    float w; // average single scattering albedo
    float b; // shape controlling parameter for the amplitude of backward and forward scatter of particles
    float c; // weighting factor that controls the contribution of backward and forward scatter.
    float B_s0;
    float h_s;
    float phi; 
    float theta_p;
    float3 pad; // padding to ensure 16 byte alignment
    
};

/// Parameters associated with the entire optix scene
struct ContextParameters {
    AreaLight* arealights;              ///< device pointer to the set of area lights in the scene
    PointLight* lights;                 ///< device pointer to set of point lights in the scene
    int num_arealights;                ///< the number of area lights in the scene
    int num_lights;                     ///< the number of point lights in the scene
    float3 ambient_light_color;         ///< the ambient light color and intensity
    float3 fog_color;                   ///< color of fog in the scene
    float fog_scattering;               ///< scattering coefficient of fog in the scene
    int max_depth;                      ///< maximum traversable depth
    float scene_epsilon;                ///< an epsilon value used for detecting self intersections
    float importance_cutoff;            ///< mimumum value before killing rays
    OptixTraversableHandle root;        ///< a handle to the root node in the scene
    MaterialParameters* material_pool;  ///< device pointer to list of materials to use for shading
    MeshParameters* mesh_pool;          ///< device pointer to list of meshes for instancing
    
    #ifdef USE_SENSOR_NVDB
    //nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>* handle_ptr; // NanoVDB grid handle
    //nanovdb::NanoGrid<float>* handle_ptr;
        nanovdb::NanoGrid<nanovdb::Point>* handle_ptr;
    #else
        int handle_ptr;
    #endif
};

/// Parameters associated with a single object in the scene. Padding added during record creation
struct MaterialRecordParameters {
    unsigned int material_pool_id;       ///< material id of first material to be
    unsigned int num_blended_materials;  ///< number of blended materials on this object (can use a weight file per
                                         ///< material to blend)
    unsigned int mesh_pool_id;           ///< mesh id of object
    float3 translational_velocity;       ///< translational velocity of object, used for radars
    float3 angular_velocity;             ///< angular velocity of object, used for radar
    float objectId;                      ///< object id, used for tracking and clustering in radar
};

/// Data associated with a single camera ray
struct PerRayData_camera {
    float3 color;             ///< color packed on the ray
    float3 contrib_to_pixel;  ///< the current contribution to the pixel
    curandState_t rng;        ///< a random number generator. Only valid if use_gi is true
    int depth;                ///< the current depth of the ray
    bool use_gi;              ///< whether global illumination is on
    float3 albedo;            ///< the albed of the first hit
    float3 normal;            ///< the global normal of the first hit
    bool use_fog;             ///< whether to use fog on this prd
};

/// Data associated with a single physics-based camera ray
struct PerRayData_phys_camera : PerRayData_camera {
    float distance;           ///< the distance from the camera to the first hit
};

struct PerRayData_depthCamera {
    float depth;
    float max_depth;
};

/// Data associated with a single normal camera ray
struct PerRayData_normalCamera {
    float3 normal;            ///< the global normal vector of the first hit
};

/// Data associated with a single segmentation camera ray
struct PerRayData_semantic {
    unsigned short int class_id;     ///< the class id of the first hit
    unsigned short int instance_id;  ///< the instance id of the first hit
};

/// Data associated with a single shadow ray
struct PerRayData_shadow {
    float3 attenuation;    ///< the current attenuation of the light
    int depth;             ///< the current traversal depth of the ray
    float ramaining_dist;  ///< the remaining distance to the light
};

/// Data associated with a single lidar ray
struct PerRayData_lidar {
    float range;      ///< the distance to the first hit
    float intensity;  ///< the intensity of the first hit
};

/// Data associated with a single radar ray
struct PerRayData_radar {
    float range;
    float rcs;
    float3 velocity;
    float objectId;
};

/// @} sensor_optix

#endif
