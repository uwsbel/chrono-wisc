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
#include <Eigen/Dense>
#define EIGEN_USE_GPU

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

static __device__ void computePCA(const float3* neighbors, int count, float3& normal) {
    // Compute the centroid
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < count; ++i) {
        float3 neighbor = neighbors[i];
        centroid += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z);
    }
    centroid /= float(count);

    // Compute the covariance matrix
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int i = 0; i < count; ++i) {
        float3 neighbor = neighbors[i];
        Eigen::Vector3f diff = Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z) - centroid;
        covariance += diff * diff.transpose();
    }
    covariance /= float(count);

    // Perform eigen decomposition to get the normal (smallest eigenvalue)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f  n = solver.eigenvectors().col(0);  // Eigenvector corresponding to smallest eigenvalue
    n.normalize();

    normal = make_float3(n.x(), n.y(), n.z());
}

extern "C" __global__ void __intersection__nvdb_vol_intersect() {
    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir = optixGetWorldRayDirection();
    const float ray_tmin = optixGetRayTmin();
    const float ray_tmax = optixGetRayTmax();
    
    // Print ray info
    //printf("ray_orig: %f %f %f\n", ray_orig.x, ray_orig.y, ray_orig.z);
    //printf("ray_dir: %f %f %f\n", ray_dir.x, ray_dir.y, ray_dir.z);
    bool renderAsBox = false;

    if (renderAsBox) {
        const float3 ray_orig1 = optixGetObjectRayOrigin();
        const float3 ray_dir1 = optixGetObjectRayDirection();
        // calculate potential intersections with the box
        float3 t0 = (make_float3(-.5f) - ray_orig1) / ray_dir1;
        float3 t1 = (make_float3(.5f) - ray_orig1) / ray_dir1;
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
                p = ray_orig1 + dist_near * ray_dir1;

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
                p = ray_orig1 + dist_far * ray_dir1;

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
        bool renderAsDensity = true;
        using Vec3T = nanovdb::Vec3f;
        const Vec3T ray_orig_v = make_nanovec3f(ray_orig);
        const Vec3T ray_dir_v = make_nanovec3f(ray_dir);


        float3 shading_normal = make_float3(0.f, 0.f, 1.f);
        float2 texcoord = make_float2(0.f, 0.f);
        float3 tangent_vector = make_float3(0.f, 0.f, 0.f);


        if (!renderAsDensity) {
            using BuildT = nanovdb::Point;

            nanovdb::NanoGrid<BuildT>* grid = params.handle_ptr;
            nanovdb::NanoGrid<Vec3T>* normal_grid = params.normal_handle_ptr;

            // Get grid accessor
            nanovdb::DefaultReadAccessor<BuildT> drAcc = grid->tree().getAccessor();
            nanovdb::DefaultReadAccessor<Vec3T> nAcc = normal_grid->tree().getAccessor();
           
            
            const Vec3T eye = grid->worldToIndex(ray_orig_v);
            const Vec3T dir = grid->worldToIndexDir(ray_dir_v);

            // check if ray intersects grid
            nanovdb::Ray<float> iRay(eye, dir, ray_tmin, ray_tmax);
            float t0;
            nanovdb::Coord ijk;
            //const Vec3T* data = grid->getBlindData<Vec3T>(0);

            bool hitting = iRay.clip(drAcc.root().bbox());
            bool foundIS = false;

            if (hitting && iRay.t1() < 1e20) {
                nanovdb::Vec3f rayStart = iRay.start();
                nanovdb::Vec3f rayDir = iRay.dir();
                /*printf("VolIS: rayStart: %f,%f,%f, rayDir: %f,%f,%f\n", rayStart[0], rayStart[1], rayStart[2], rayDir[0],
                       rayDir[1], rayDir[2]);*/
                ijk = nanovdb::RoundDown<nanovdb::Coord>(rayStart);
                //nanovdb::PointAccessor<Vec3T, BuildT> acc(*grid);

                nanovdb::HDDA<nanovdb::Ray<float>, nanovdb::Coord> hdda(iRay, nAcc.getDim(ijk, iRay));
                int nsteps = 0;
                int inactiveSteps = 0;
                //uint64_t v;
                static const float Delta = 1e-3;
                while (!foundIS && hdda.step()) {
                    ijk = nanovdb::RoundDown<nanovdb::Coord>(iRay(hdda.time() + Delta));
                    hdda.update(iRay, nAcc.getDim(ijk, iRay));
                    if (hdda.dim() > 1 || !nAcc.isActive(ijk)) {
                        inactiveSteps++;
                        if (inactiveSteps > 10000) {
                              break;
                        }
                        continue;  // either a tile value or an inactive voxel
                    }
               
                    const Vec3T *start = nullptr, *stop = nullptr;
                    while (!foundIS && hdda.step() && nAcc.isActive(hdda.voxel())) {
                        //v = drAcc.getValue(hdda.voxel());
                        ijk = hdda.voxel();
                        float t = hdda.time();
                        float wT = t * float(grid->voxelSize()[0]);
                      
                      
                        float3 hp = ray_orig + t*ray_dir;
                        float3 ijkW = make_float3(grid->indexToWorld(ijk.asVec3s()));
                        float3 normal = normalize(make_float3(nAcc.getValue(ijk)));
                        float3 ref_dir = make_float3(0,0,1);
                        float NdC = Dot(normal, -ray_dir);
                        float NdRef = Dot(normal, ref_dir);
                        //printf("Normal: (%f,%f,%f)\n", normal.x, normal.y, normal.z);
                        //if (NdC < 0)  // flip normals
                        //    normal = -normal;
                        //printf("VolIS |t: %f | o: (%f,%f,%f) | d: (%f,%f,%f) | hp: (%f,%f,%f) | n: (%f,%f,%f) | ijkW: (%f,%f,%f)\n",t, ray_orig.x, ray_orig.y, ray_orig.z, ray_dir.x, ray_dir.y, ray_dir.z,hp.x, hp.y, hp.z, normal.x, normal.y, normal.z, ijkW.x, ijkW.y, ijkW.z);
                         //t = t + float(grid->voxelSize()[0]);
                         foundIS = true;
                         optixReportIntersection(
                            t, 0, reinterpret_cast<unsigned int&>(normal.x), reinterpret_cast<unsigned int&>(normal.y),
                            reinterpret_cast<unsigned int&>(normal.z),
                            reinterpret_cast<unsigned int&>(texcoord.x), reinterpret_cast<unsigned int&>(texcoord.y),
                            reinterpret_cast<unsigned int&>(tangent_vector.x), reinterpret_cast<unsigned
                            int&>(tangent_vector.y), reinterpret_cast<unsigned int&>(tangent_vector.z));
                          //break;
                    }
                }
            }
        } else {
        
             nanovdb::NanoGrid<float>* density_grid = params.density_grid_ptr;
             nanovdb::DefaultReadAccessor<float> drAcc = density_grid->tree().getAccessor();

             const Vec3T eye = density_grid->worldToIndex(ray_orig_v);
             const Vec3T dir = density_grid->worldToIndexDir(ray_dir_v);

             // check if ray intersects grid
             nanovdb::Ray<float> iRay(eye, dir, ray_tmin, ray_tmax);
             float t0;
             nanovdb::Coord ijk;
             // const Vec3T* data = grid->getBlindData<Vec3T>(0);

             bool hitting = iRay.clip(drAcc.root().bbox());
             bool foundIS = false;

              if (hitting && iRay.t1() < 1e20) {
                 nanovdb::Vec3f rayStart = iRay.start();
                 nanovdb::Vec3f rayDir = iRay.dir();
               
                 ijk = nanovdb::RoundDown<nanovdb::Coord>(rayStart);

                 nanovdb::HDDA<nanovdb::Ray<float>, nanovdb::Coord> hdda(iRay, drAcc.getDim(ijk, iRay));
                 int nsteps = 0;
                 int inactiveSteps = 0;
                 // uint64_t v;
                 static const float Delta = 1e-3;
                 while (!foundIS && hdda.step()) {
                     ijk = nanovdb::RoundDown<nanovdb::Coord>(iRay(hdda.time() + Delta));
                     hdda.update(iRay, drAcc.getDim(ijk, iRay));
                     if (hdda.dim() > 1 || !drAcc.isActive(ijk)) {
                         inactiveSteps++;
                         if (inactiveSteps > 10000) {
                             break;
                         }
                         continue;  // either a tile value or an inactive voxel
                     }

                     const Vec3T *start = nullptr, *stop = nullptr;
                     while (!foundIS && hdda.step() && drAcc.isActive(hdda.voxel())) {
                         // v = drAcc.getValue(hdda.voxel());
                         ijk = hdda.voxel();
                         float t = hdda.time();
                         float wT = t * float(density_grid->voxelSize()[0]);
                         //printf("VolIS: t: %f i: %f, j:%f, k:%f \n", t, ijk[0], ijk[1], ijk[2]);
                         foundIS = true;
                         optixReportIntersection(t, 0, reinterpret_cast<unsigned int&>(shading_normal.x),
                                                 reinterpret_cast<unsigned int&>(shading_normal.y),
                                                 reinterpret_cast<unsigned int&>(shading_normal.z),
                                                 reinterpret_cast<unsigned int&>(texcoord.x),
                             reinterpret_cast<unsigned int&>(texcoord.y),
                             reinterpret_cast<unsigned int&>(tangent_vector.x),
                             reinterpret_cast<unsigned int&>(tangent_vector.y),
                             reinterpret_cast<unsigned int&>(tangent_vector.z));
                         // break;
                     }
                 }
             }
        }
    }   
}

#endif
