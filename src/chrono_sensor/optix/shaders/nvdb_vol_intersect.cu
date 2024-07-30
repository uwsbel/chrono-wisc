#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"

#ifdef USE_SENSOR_NVDB

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/Ray.h>
#include <nanovdb/util/HDDA.h>
#include <nanovdb/util/Stencils.h>

#include <iostream>

template <typename RayT, typename AccT>
inline __hostdev__ bool ZeroCrossingPoint(RayT& ray, AccT& acc, nanovdb::Coord& ijk, typename AccT::ValueType& v, float& t, nanovdb::CoordBBox& bbox) {
    //printf("ZCrossing::ray.start(): (%f,%f,%f)\n", ray.start()[0], ray.start()[1], ray.start()[2]);
    bbox = acc.root().bbox();
    //printf("BBox: min:(%d,%d,%d)| max:(%d,%d,%d)\n", bbox.min()[0], bbox.min()[1], bbox.min()[2], bbox.max()[0],bbox.max()[1], bbox.max()[2]);
    if (!ray.clip(bbox) || ray.t1() > 1e20) //!ray.clip(acc.root().bbox()) 
        return false;  // clip ray to bbox
    static const float Delta = 1.0001f;
    ijk = nanovdb::RoundDown<nanovdb::Coord>(ray.start());  // first hit of bbox
    //printf("ZCrossing::ray.start(): (%f,%f,%f) | ray.dir(): (%f,%f,%f)\n", ray.start()[0], ray.start()[1], ray.start()[2], ray.dir()[0],ray.dir()[1],ray.dir()[2]);
    nanovdb::HDDA<RayT, nanovdb::Coord> hdda(ray, acc.getDim(ijk, ray));
    const auto v0 = acc.getValue(ijk);
    if (v0 > 0) {
        v = v0;
        t = ray.t0();
        return true;
    }
    //printf("Start Value: %f | Start Idx: %f,%f,%f\n", v0, ijk.asVec3d()[0], ijk.asVec3d()[1], ijk.asVec3d()[2]);
    while (hdda.step()) {
        ijk = nanovdb::RoundDown<nanovdb::Coord>(ray(hdda.time() + Delta));
        hdda.update(ray, acc.getDim(ijk, ray));
        bool b1 = (hdda.dim() > 1);
        bool b2 = (!acc.isActive(ijk));
        //printf("v0: %f | hadd.dim() > 1? %d | !acc.isActive(ijk)? %d\n", v0, b1, b2);
        if (hdda.dim() > 1 || !acc.isActive(ijk))
            continue;                                        // either a tile value or an inactive voxel
       
        while (hdda.step() && acc.isActive(hdda.voxel())) {  // in the narrow band
            v = acc.getValue(hdda.voxel());
            //printf("v: %f, v0: %f\n", v, v0);
            if (v * v0 < 0) {  // zero crossing
                ijk = hdda.voxel();
                t = hdda.time();
                return true;
            }
        }
    }
    return false;
}

static __device__ float3 box_normal(float t, float3 t0, float3 t1) {
    float3 normal_pos = make_float3(t == t0.x ? 1 : 0, t == t0.y ? 1 : 0, t == t0.z ? 1 : 0);
    float3 normal_neg = make_float3(t == t1.x ? 1 : 0, t == t1.y ? 1 : 0, t == t1.z ? 1 : 0);
    return normal_neg - normal_pos;
}

extern "C" __global__ void __intersection__nvdb_vol_intersect() {
    const float3 ray_orig = optixGetObjectRayOrigin();
    const float3 ray_dir = optixGetObjectRayDirection();
    const float ray_tmin = optixGetRayTmin();
    const float ray_tmax = optixGetRayTmax();
    
    // Print ray info
    //printf("ray_orig: %f %f %f\n", ray_orig.x, ray_orig.y, ray_orig.z);
    //printf("ray_dir: %f %f %f\n", ray_dir.x, ray_dir.y, ray_dir.z);
    bool renderAsBox = false;

    if (renderAsBox) {
        // calculate potential intersections with the box
        float3 t0 = (make_float3(-.5f) - ray_orig) / ray_dir;
        float3 t1 = (make_float3(.5f) - ray_orig) / ray_dir;
        float3 near = fminf(t0, t1);
        float3 far = fmaxf(t0, t1);
        // dist_near and dist_far are the distances to the potential intsection points
        float dist_near = fmaxf(near);
        float dist_far = fminf(far);

        
        //printf("NVDBVolIS: orig: (%f,%f,%f), dir:(%f,%f,%f)\n", ray_orig.x,ray_orig.y, ray_orig.z, ray_dir.x, ray_dir.y, ray_dir.z);
        //printf("NVDBVolIS: orig: (%f,%f,%f), dir:(%f,%f,%f)\n", ray_orig.x,ray_orig.y, ray_orig.z, ray_dir.x, ray_dir.y, ray_dir.z);
        // check if near is less than far
        if (dist_near <= dist_far) {
            float3 p = make_float3(0);

            if (dist_near > ray_tmin && dist_near < ray_tmax) {
                float3 shading_normal = box_normal(dist_near, t0, t1);
                float2 texcoord;
                float3 tangent_vector;
                p = ray_orig + dist_near * ray_dir;

                if (abs(shading_normal.x) > 0.5) {
                    texcoord = make_float2((p.y + 0.5), (p.z + 0.5) * shading_normal.x);
                    tangent_vector = make_float3(0, 1, 0);
                } else if (abs(shading_normal.y) > 0.5) {
                    texcoord = make_float2((p.x + 0.5), -(p.z + 0.5) * shading_normal.y);
                    tangent_vector = make_float3(1, 0, 0);
                } else {
                    texcoord = make_float2((p.x + 0.5), (p.y + 0.5) * shading_normal.z);
                    tangent_vector = make_float3(1, 0, 0);
                }
                //printf("Box intersection\n");
                optixReportIntersection(
                    dist_near, 0, reinterpret_cast<unsigned int&>(shading_normal.x),
                    reinterpret_cast<unsigned int&>(shading_normal.y),
                    reinterpret_cast<unsigned int&>(shading_normal.z), reinterpret_cast<unsigned int&>(texcoord.x),
                    reinterpret_cast<unsigned int&>(texcoord.y), reinterpret_cast<unsigned int&>(tangent_vector.x),
                    reinterpret_cast<unsigned int&>(tangent_vector.y),
                    reinterpret_cast<unsigned int&>(tangent_vector.z));
            } else if (dist_far > ray_tmin && dist_far < ray_tmax) {
                float3 shading_normal = box_normal(dist_far, t0, t1);
                float2 texcoord;
                float3 tangent_vector;
                p = ray_orig + dist_far * ray_dir;

                // calculate uvs and tangent vector
                if (abs(shading_normal.x) > 0.5) {
                    texcoord = make_float2((p.y + 0.5), (p.z + 0.5) * shading_normal.x);
                    tangent_vector = make_float3(0, 1, 0);
                } else if (abs(shading_normal.y) > 0.5) {
                    texcoord = make_float2((p.x + 0.5), -(p.z + 0.5) * shading_normal.y);
                    tangent_vector = make_float3(1, 0, 0);
                } else {
                    texcoord = make_float2((p.x + 0.5), (p.y + 0.5) * shading_normal.z);
                    tangent_vector = make_float3(1, 0, 0);
                }
                //printf("Box intersection\n");
                optixReportIntersection(
                    dist_far, 0, reinterpret_cast<unsigned int&>(shading_normal.x),
                    reinterpret_cast<unsigned int&>(shading_normal.y),
                    reinterpret_cast<unsigned int&>(shading_normal.z), reinterpret_cast<unsigned int&>(texcoord.x),
                    reinterpret_cast<unsigned int&>(texcoord.y), reinterpret_cast<unsigned int&>(tangent_vector.x),
                    reinterpret_cast<unsigned int&>(tangent_vector.y),
                    reinterpret_cast<unsigned int&>(tangent_vector.z));
            }
        }
    }
    else {
        /// NanoVDB stuff

        using Vec3T = nanovdb::Vec3f;
        using BuildT = float;

      
        /*const Vec3T ray_orig_v(ray_orig.x, ray_orig.y, ray_orig.z);
        const Vec3T ray_dir_v(ray_dir.x, ray_dir.y, ray_dir.z);*/
        // nanovdb::NanoGrid<nanovdb::Point>* grid = params.handle_ptr;
        nanovdb::NanoGrid<BuildT>* grid = params.handle_ptr;

        // Get grid accessor

        nanovdb::DefaultReadAccessor<BuildT> acc = grid->tree().getAccessor();
        // nanovdb::PointAccessor<Vec3T, BuildT> pacc(*grid);
        float wBBoxDimX = 400.f;//(float)grid->worldBBox().dim()[0]*10;
        float wBBoxDimY = 400.f;//(float)grid->worldBBox().dim()[1]*10;
        float wBBoxDimZ = 400.f;//(float)grid->worldBBox().dim()[2]*10;

         // convert to nanovdb types
        const Vec3T ray_orig_v(ray_orig.x * wBBoxDimX, ray_orig.y * wBBoxDimY, ray_orig.z * wBBoxDimZ);
        const Vec3T ray_dir_v(ray_dir.x * wBBoxDimX, ray_dir.y * wBBoxDimY, ray_dir.z * wBBoxDimZ);


        // nanovdb::DefaultReadAccessor<BuildT>::ValueType v1;
    
        nanovdb::Vec3d wMax = grid->indexToWorld(grid->worldBBox().max());
        nanovdb::Vec3d wMin = grid->indexToWorld(grid->worldBBox().min());
        nanovdb::Coord wMinCoord(static_cast<int32_t>(wMin[0]), static_cast<int32_t>(wMin[1]), static_cast<int32_t>(wMin[2]));
        nanovdb::Coord wMaxCoord(static_cast<int32_t>(wMax[0]), static_cast<int32_t>(wMax[1]), static_cast<int32_t>(wMax[2]));

        //nanovdb::Coord wMinCoord(-43,-32,-15);
        //nanovdb::Coord wMaxCoord(43,33,17);
        
        //printf("wMax: %f %f %f | wMin: %f %f %f\n",wMax[0], wMax[1], wMax[2], wMin[0], wMin[1], wMin[2]);
        //printf("wMaxCoord: %f %f %f | wMinCoord: %f %f %f\n", wMaxCoord[0], wMaxCoord[1], wMaxCoord[2], wMinCoord[0],wMinCoord[1], wMinCoord[2]);
        //nanovdb::CoordBBox IdxBBox(nanovdb::Coord(-43, -32, -15), nanovdb::Coord(43, 33, 17));
        //printf("IdxBBox: min:(%d,%d,%d)| max:(%d,%d,%d)\n", IdxBBox.min()[0], IdxBBox.min()[1], IdxBBox.min()[2], IdxBBox.max()[0],IdxBBox.max()[1], IdxBBox.max()[2]);
        // Get ray origin in grid space
        const Vec3T eye = grid->worldToIndex(ray_orig_v);
        const Vec3T dir = grid->worldToIndex(ray_dir_v);
        

        // check if ray intersects grid
        nanovdb::Ray<float> iRay(eye, dir, ray_tmin, ray_tmax);
        float t0;
        nanovdb::Coord ijk;
        nanovdb::DefaultReadAccessor<BuildT>::ValueType v;

       
  

        // Implement ZeroCrossing for PointType
        float3 shading_normal = make_float3(0.f, 0.f, 1.f);
        float2 texcoord = make_float2(0.f, 0.f);
        float3 tangent_vector = make_float3(0.f, 0.f, 0.f);
       
        nanovdb::BoxStencil<nanovdb::FloatGrid> stencil(*grid);
        bool hitting = iRay.clip(acc.root().bbox());
        if (hitting && iRay.t1() < 1e20) {
            nanovdb::Vec3f rayStart = iRay.start();
            nanovdb::Vec3f rayDir = iRay.dir();
            /*printf("VolIS: rayStart: %f,%f,%f, rayDir: %f,%f,%f\n", rayStart[0], rayStart[1], rayStart[2], rayDir[0],
                   rayDir[1], rayDir[2]);*/
            float t0 = iRay.t0();
            float wT0 = t0*float(grid->voxelSize()[0]);
            //nanovdb::Coord ijk = nanovdb::RoundDown<nanovdb::Coord>(iRay.start());
            optixReportIntersection(
                    t0, 0, reinterpret_cast<unsigned int&>(shading_normal.x), reinterpret_cast<unsigned
                int&>(shading_normal.y), reinterpret_cast<unsigned int&>(shading_normal.z),
                reinterpret_cast<unsigned int&>(texcoord.x), reinterpret_cast<unsigned int&>(texcoord.y),
                reinterpret_cast<unsigned int&>(tangent_vector.x), reinterpret_cast<unsigned
                int&>(tangent_vector.y), reinterpret_cast<unsigned int&>(tangent_vector.z));
    /*printf(
                "Hit: %d, t1: %f | NVDBVolIS: orig: (%f,%f,%f), dir:(%f,%f,%f), IdxEye:(%f,%f,%f), IdxDir: "
                "(%f,%f,%f)\n",
                hitting, iRay.t1(), ray_orig_v[0], ray_orig_v[1], ray_orig_v[2], ray_dir_v[0], ray_dir_v[1],
                ray_dir_v[2], eye[0], eye[1], eye[2], dir[0], dir[1], dir[2]);*/
        }
        //if (ZeroCrossingPoint(iRay, acc, ijk, v, t0, IdxBBox)) {
        //    float wT0 = t0 * float(grid->voxelSize()[0]);
        //    //printf("ijk: %d %d %d, v: %f, wT0: %f\n", ijk[0], ijk[1], ijk[2], v, wT0);
        //    float v100 = acc.getValue(ijk.offsetBy(1, 0, 0));
        //    float v010 = acc.getValue(ijk.offsetBy(0, 1, 0));
        //    nanovdb::Vec3d grad(v100 - v, v010 - v, v);
        //    grad = grid->map().applyIJT(grad);
        //    nanovdb::Vec3d norm = grid->indexToWorld(grad);
        //    norm.normalize();
        //  /*  stencil.moveTo(ijk);
        //    nanovdb::Vec3d norm = stencil.gradient(ijk.asVec3s());
        //    norm.normalize();*/
        //    //printf("Normal: (%f,%f,%f) | v100: %f, v010: %f\n", norm[0], norm[1], norm[2], v100, v010);
        //    shading_normal = make_float3((float)norm[0], (float)norm[1], (float)norm[2]);
        //    optixReportIntersection(
        //        t0, 0, reinterpret_cast<unsigned int&>(shading_normal.x), reinterpret_cast<unsigned int&>(shading_normal.y),
        //        reinterpret_cast<unsigned int&>(shading_normal.z),
        //        reinterpret_cast<unsigned int&>(texcoord.x), reinterpret_cast<unsigned int&>(texcoord.y),
        //        reinterpret_cast<unsigned int&>(tangent_vector.x), reinterpret_cast<unsigned int&>(tangent_vector.y),
        //        reinterpret_cast<unsigned int&>(tangent_vector.z));
        //} else {
        //    //printf("ray does not intersect grid\n\n");
        //}
    }
}

#endif
