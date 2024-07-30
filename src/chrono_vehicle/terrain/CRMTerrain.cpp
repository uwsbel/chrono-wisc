// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Continuum representation (SPH-based) deformable terrain model.
//
// Reference frame is ISO (X forward, Y left, Z up).
// All units SI.
//
// =============================================================================

#include <fstream>
#include <iostream>
#include <sstream>
#include <queue>

#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_thirdparty/stb/stb.h"
#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

CRMTerrain::CRMTerrain(ChSystem& sys, double spacing)
    : m_sys(sys), m_spacing(spacing), m_initialized(false), m_offset(VNULL), m_angle(0.0), m_verbose(false) {
    // Create ground body
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetFixed(true);
    sys.AddBody(m_ground);

    // Associate MBS system with underlying FSI system
    m_sysFSI.SetVerbose(m_verbose);
    m_sysFSI.AttachSystem(&sys);
    m_sysFSI.SetInitialSpacing(spacing);
    m_sysFSI.SetKernelLength(spacing);
}

void CRMTerrain::SetVerbose(bool verbose) {
    m_sysFSI.SetVerbose(verbose);
    m_verbose = verbose;
}

void CRMTerrain::AddRigidObstacle(const std::string& obj_file,
                                  double scale,
                                  double density,
                                  const ChContactMaterialData& cmat,
                                  const ChFrame<>& pos,
                                  const ChVector3d& interior_point) {
    //// TODO: calculate OOBB (for possible use with "moving patch")

    if (m_initialized)
        throw std::runtime_error("CRMTerrain: obstacles cannot be added after initialization");

    if (m_verbose) {
        cout << "Add obstacle" << endl;
        cout << "  Mesh filename: " << obj_file << endl;
        cout << "  Scale: " << scale << endl;
    }

    RigidObstacle o;
    o.density = density;
    o.cmat = cmat;
    o.point = interior_point;

    // Create trimesh
    o.trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    o.trimesh->LoadWavefrontMesh(GetChronoDataFile(obj_file), true, true);

    // Scale trimesh
    for (auto& v : o.trimesh->GetCoordsVertices()) {
        v *= scale;
    }

    // Calculate mass and inertia properties
    double mass;
    ChVector3d baricenter;
    ChMatrix33<> inertia;
    o.trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

    auto o_name = filesystem::path(obj_file).stem();

    // Create the obstacle body
    o.body = chrono_types::make_shared<ChBody>();
    o.body->SetName("obstacle_" + o_name);
    o.body->SetPos(pos.GetPos());
    o.body->SetRot(pos.GetRot());
    o.body->SetMass(mass * density);
    o.body->SetInertia(inertia * density);
    o.body->SetFixed(false);
    o.body->EnableCollision(true);

    // Create obstacle visualization geometry
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(o.trimesh);
    trimesh_shape->SetName(o_name);
    o.body->AddVisualShape(trimesh_shape, ChFrame<>());

    // Create obstacle collision geometry
    auto mat = o.cmat.CreateMaterial(m_sys.GetContactMethod());
    auto thickness = m_spacing / 2;
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, o.trimesh, false, false, thickness);
    o.body->AddCollisionShape(ct_shape);
    o.body->GetCollisionModel()->SetFamily(2);

    // Create the obstacle BCE points (relative coordinates)
    m_sysFSI.CreateMeshPoints(*o.trimesh, m_spacing, o.point_cloud);

    m_sys.AddBody(o.body);

    m_obstacles.push_back(o);

    if (m_verbose) {
        cout << "  Num. BCE markers: " << o.point_cloud.size() << endl;
    }
}

void CRMTerrain::Construct(const std::string& sph_file,
                           const std::string& bce_file,
                           const ChVector3d& pos,
                           double yaw_angle) {
    if (m_verbose) {
        cout << "Construct CRMTerrain from data files" << endl;
    }

    std::string line;
    int x, y, z;

    std::ifstream sph(sph_file, std::ios_base::in);
    while (std::getline(sph, line)) {
        std::istringstream iss(line, std::ios_base::in);
        iss >> x >> y >> z;
        m_sph.insert(ChVector3i(x, y, z));
    }

    std::ifstream bce(bce_file, std::ios_base::in);
    while (std::getline(bce, line)) {
        std::istringstream iss(line, std::ios_base::in);
        iss >> x >> y >> z;
        m_bce.insert(ChVector3i(x, y, z));
    }

    if (m_verbose) {
        cout << "  SPH particles filename: " << sph_file << "  [" << m_sph.size() << "]" << endl;
        cout << "  BCE markers filename: " << bce_file << "  [" << m_bce.size() << "]" << endl;
    }

    // Complete construction of CRM terrain and obstacles
    m_offset = pos;
    m_angle = yaw_angle;
    CompleteConstruct();
}

void CRMTerrain::Construct(double length,
                           double width,
                           double depth,
                           int bce_layers,
                           const ChVector3d& pos,
                           double yaw_angle,
                           bool side_walls) {
    if (m_verbose) {
        cout << "Construct rectangular patch CRMTerrain" << endl;
    }

    // Number of particles in each direction
    int Nx = (int)std::floor(length / m_spacing);
    int Ny = (int)std::floor(width / m_spacing);
    int Nz = (int)std::floor(depth / m_spacing);
    ////double Dx = length / (Nx - 1);
    ////double Dy = width / (Ny - 1);

    // Number of SPH particles and BCE markers
    int num_sph = Nx * Ny * Nz;
    int num_bce = Nx * Ny * bce_layers + (Nx + Ny + 2 * bce_layers) * 2 * bce_layers * Nz;

    // Reserve space for containers
    std::vector<ChVector3i> sph;
    std::vector<ChVector3i> bce;
    sph.reserve(num_sph);
    bce.reserve(num_bce);

    // Generate SPH and bottom BCE points
    for (int Ix = 0; Ix < Nx; Ix++) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Iz = 0; Iz < Nz; Iz++) {
                sph.push_back(ChVector3i(Ix, Iy, Iz));  // SPH particles above 0
            }
            for (int Iz = 1; Iz <= bce_layers; Iz++) {
                bce.push_back(ChVector3d(Ix, Iy, -Iz));  // BCE markers below 0
            }
        }
    }

    // Generate side BCE points
    if (side_walls) {
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int Iz = -bce_layers; Iz < Nz; Iz++) {
                    bce.push_back(ChVector3d(Ix, Iy, Iz));
                    bce.push_back(ChVector3d(Ix, Ny - 1 - Iy, Iz));
                }
            }
        }
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int Iz = -bce_layers; Iz < Nz; Iz++) {
                    bce.push_back(ChVector3d(Ix, Iy, Iz));
                    bce.push_back(ChVector3d(Nx - 1 - Ix, Iy, Iz));
                }
            }
        }
    }

    // Insert in cached sets
    for (auto& p : sph) {
        m_sph.insert(p);
    }
    for (auto& p : bce) {
        p.y() = (Ny - 1) - p.y();
        m_bce.insert(p);
    }

    if (m_verbose) {
        cout << "  Particle grid size: " << Nx << " " << Ny << " " << Nz << endl;
        cout << "  Num. SPH particles: " << m_sph.size() << endl;
        cout << "  Num. BCE markers: " << m_bce.size() << endl;
    }

    // Complete construction of CRM terrain and obstacles
    m_offset = pos - ChVector3d(length / 2, width / 2, 0);
    m_angle = yaw_angle;
    CompleteConstruct();
}

void CRMTerrain::Construct(const std::string& heightmap_file,
                           double length,
                           double width,
                           const ChVector2d& height_range,
                           double depth,
                           int bce_layers,
                           const ChVector3d& pos,
                           double yaw_angle,
                           bool side_walls) {
    // Read the image file (request only 1 channel) and extract number of pixels
    STB hmap;
    if (!hmap.ReadFromFile(heightmap_file, 1)) {
        throw std::invalid_argument("Cannot open height map image file");
    }

    if (m_verbose) {
        cout << "Construct CRMTerrain from heightmap file" << endl;
    }

    int nx = hmap.GetWidth();   // number of pixels in x direction
    int ny = hmap.GetHeight();  // number of pixels in y direction
    double dx = length / (nx - 1);
    double dy = width / (ny - 1);

    // Create a matrix with gray levels (g = 0x0000 is black, g = 0xffff is white)
    unsigned short g_min = +std::numeric_limits<unsigned short>::max();
    unsigned short g_max = -std::numeric_limits<unsigned short>::max();
    ChMatrixDynamic<unsigned short> gmap(nx + 1, ny + 1);
    for (int ix = 0; ix <= nx; ix++) {
        for (int iy = 0; iy <= ny; iy++) {
            auto gray = hmap.Gray(ix < nx ? ix : nx - 1, iy < ny ? iy : ny - 1);
            gmap(ix, iy) = gray;
            g_min = std::min(g_min, gray);
            g_max = std::max(g_max, gray);
        }
    }

    // Calculate height scaling (black->hMin, white->hMax)
    double h_scale = (height_range[1] - height_range[0]) / hmap.GetRange();

    // Number of particles in each direction
    int Nx = (int)std::floor(length / m_spacing);
    int Ny = (int)std::floor(width / m_spacing);
    int Nz = (int)std::floor(depth / m_spacing);
    double Dx = length / (Nx - 1);
    double Dy = width / (Ny - 1);

    // Number of SPH particles and BCE markers
    int num_sph = Nx * Ny * Nz;
    int num_bce = bce_layers * (Nx * Ny);  // underestimate if side_walls

    // Reserve space for containers
    std::vector<ChVector3i> sph;
    std::vector<ChVector3i> bce;
    sph.reserve(num_sph);
    bce.reserve(num_bce);

    // Generate SPH and bottom BCE points
    int Izmin = std::numeric_limits<int>::max();
    int Izmax = -std::numeric_limits<int>::max();
    for (int Ix = 0; Ix < Nx; Ix++) {
        double x = Ix * Dx;
        int ix = (int)std::round(x / dx);       // x location between pixels ix and ix+1
        double wx1 = ((ix + 1) * dx - x) / dx;  // weight for bi-linear interpolation
        double wx2 = (x - ix * dx) / dx;        // weight for bi-linear interpolation
        for (int Iy = 0; Iy < Ny; Iy++) {
            double y = Iy * Dy;
            int iy = (int)std::round(y / dy);       // y location between pixels iy and iy+1
            double wy1 = ((iy + 1) * dy - y) / dy;  // weight for bi-linear interpolation
            double wy2 = (y - iy * dy) / dy;        // weight for bi-linear interpolation
            
            // Calculate surface height at current location (bi-linear interpolation)
            auto h1 = wx1 * gmap(ix + 0, iy + 0) + wx2 * gmap(ix + 1, iy + 0);
            auto h2 = wx1 * gmap(ix + 0, iy + 1) + wx2 * gmap(ix + 1, iy + 1);
            auto z = height_range[0] + (wy1 * h1 + wy2 * h2) * h_scale;
            int Iz = (int)std::round(z / m_spacing);

            Izmin = std::min(Izmin, Iz);
            Izmax = std::max(Izmax, Iz);

            // Load SPH and BCE location below current point
            for (int k = 0; k < Nz; k++) {
                sph.push_back(ChVector3i(Ix, Iy, Iz));
                Iz--;
            }
            for (int k = 0; k < bce_layers; k++) {
                bce.push_back(ChVector3d(Ix, Iy, Iz));
                Iz--;
            }
        }
    }

    // Generate side BCE points
    if (side_walls) {
        Izmin -= Nz + bce_layers;
        Izmax += bce_layers;
        Nz += 2 * bce_layers;
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int k = 0; k < Nz; k++) {
                    bce.push_back(ChVector3d(Ix, Iy, Izmin + k));
                    bce.push_back(ChVector3d(Ix, Ny - 1 - Iy, Izmin + k));
                }
            }
        }
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int k = 0; k < Nz; k++) {
                    bce.push_back(ChVector3d(Ix, Iy, Izmin + k));
                    bce.push_back(ChVector3d(Nx - 1 - Ix, Iy, Izmin + k));
                }
            }
        }
    }

    // Note that pixels in image start at top-left.
    // Modify y coordinates so that particles start at bottom-left before inserting in cached sets.
    for (auto& p : sph) {
        p.y() = (Ny - 1) - p.y();
        m_sph.insert(p);
    }
    for (auto& p : bce) {
        p.y() = (Ny - 1) - p.y();
        m_bce.insert(p);
    }

    if (m_verbose) {
        cout << "  Heightmap filename: " << heightmap_file << endl;
        cout << "  Num. SPH particles: " << m_sph.size() << endl;
        cout << "  Num. BCE markers: " << m_bce.size() << endl;
    }

    // Complete construction of CRM terrain and obstacles
    m_offset = pos - ChVector3d(length / 2, width / 2, 0);
    m_angle = yaw_angle;
    CompleteConstruct();
}

void CRMTerrain::CompleteConstruct() {
    // Prune SPH particles at grid locations that overlap with obstacles
    if (!m_obstacles.empty()) {
        if (m_verbose)
            cout << "Remove SPH particles inside obstacle volumes" << endl;

        for (auto& o : m_obstacles)
            ProcessObstacleMesh(o);

        if (m_verbose)
            cout << "Num. SPH particles: " << m_sph.size() << endl;
    }

    // Convert SPH and BCE grid points to real coordinates and apply patch transformation.
    //// TODO: yaw rotation
    std::vector<ChVector3d> sph_points;
    sph_points.reserve(m_sph.size());
    for (const auto& p : m_sph) {
        ChVector3d point(m_spacing * p.x(), m_spacing * p.y(), m_spacing * p.z());
        sph_points.push_back(point + m_offset);
    }
    std::vector<ChVector3d> bce_points;
    bce_points.reserve(m_bce.size());
    for (const auto& p : m_bce) {
        ChVector3d point(m_spacing * p.x(), m_spacing * p.y(), m_spacing * p.z());
        bce_points.push_back(point + m_offset);
    }

    // Create SPH particles and calculate AABB.
    ChVector3d tau(0);
    for (const auto& p : sph_points) {
        m_sysFSI.AddSPHParticle(p, m_sysFSI.GetDensity(), 0.0, m_sysFSI.GetViscosity(), VNULL, tau, VNULL);
        m_aabb.min = Vmin(m_aabb.min, p);
        m_aabb.max = Vmax(m_aabb.max, p);
    }

    if (m_verbose) {
        cout << "AABB of SPH particles" << endl;
        cout << "  min: " << m_aabb.min << endl;
        cout << "  max: " << m_aabb.max << endl;
    }

    // Set computational domain
    ChVector3d aabb_dim = m_aabb.Size();
    aabb_dim.z() *= 50;
    m_sysFSI.SetBoundaries(m_aabb.min - 0.1 * aabb_dim, m_aabb.max + 0.1 * aabb_dim);

    // Create BCE markers for terrain patch
    // (ATTENTION: BCE markers must be created after the SPH particles!)
    m_sysFSI.AddPointsBCE(m_ground, bce_points, ChFrame<>(), false);

    // Create the BCE markers for the obstacles
    // (ATTENTION: BCE markers for moving objects must be created after the fixed BCE markers!)
    for (const auto& o : m_obstacles) {
        // Create the BCE markers for the obstacle
        m_sysFSI.AddFsiBody(o.body);
        m_sysFSI.AddPointsBCE(o.body, o.point_cloud, ChFrame<>(), true);
    }
}

void CRMTerrain::Initialize() {
    m_sysFSI.Initialize();
    m_initialized = true;
}

ChVector3i Snap2Grid(const ChVector3d point, double spacing) {
    return ChVector3i((int)std::round(point.x() / spacing),  //
                      (int)std::round(point.y() / spacing),  //
                      (int)std::round(point.z() / spacing));
}

// Offsets for the 6 neighbors of an integer grid node
static const std::vector<ChVector3i> nbr3D{
    ChVector3i(-1, 0, 0),  //
    ChVector3i(+1, 0, 0),  //
    ChVector3i(0, -1, 0),  //
    ChVector3i(0, +1, 0),  //
    ChVector3i(0, 0, -1),  //
    ChVector3i(0, 0, +1)   //
};

//// TODO:
////  - Include yaw angle

void CRMTerrain::ProcessObstacleMesh(RigidObstacle& o) {
    // Create BCE markers for the *transformed* obstacle mesh
    // (to address any roundoff issues that may result in a set of BCE markers that are not watertight)
    auto trimesh = *o.trimesh;
    for (auto& v : trimesh.GetCoordsVertices()) {
        auto v_abs = o.body->TransformPointLocalToParent(v);  // vertex in absolute frame
        v = v_abs - m_offset;                                 // vertex in CRMTerrain frame
    }

    // BCE marker locations (in CRMTerrain frame)
    std::vector<ChVector3d> point_cloud;
    m_sysFSI.CreateMeshPoints(trimesh, m_spacing, point_cloud);

    // Express the points in the obstacle BCE point cloud in CRMTerrain grid coordinates
    o.bce.reserve(point_cloud.size());
    for (const auto& p : point_cloud) {
        o.bce.insert(Snap2Grid(p, m_spacing));  // point in CRMTerrain grid coordinates
    }

    // Express the provided interior point in CRMTerrain grid coordinates
    auto c_abs = o.body->TransformPointLocalToParent(o.point);  // point in absolute frame
    auto c_sph = c_abs - m_offset;                              // point in CRMTerrain frame
    auto c = Snap2Grid(c_sph, m_spacing);                       // point in CRMTerrain grid coordinates

    // Calculate the (integer) obstacle AABB
    ChVector3i aabb_min(+std::numeric_limits<int>::max());
    ChVector3i aabb_max(-std::numeric_limits<int>::max());
    for (const auto& p : o.bce) {
        aabb_min = Vmin(aabb_min, p);
        aabb_max = Vmax(aabb_max, p);
    }

    // Collect all grid points contained in the BCE volume in a set (initialized with the obstacle BCEs)
    Points list = o.bce;

    // Use a queue-based flood-filling algorithm to find all points interior to the obstacle volume
    std::queue<ChVector3i> todo;

    // Add the provided interior point to the work queue then iterate until the queue is empty
    todo.push({c.x(), c.y(), c.z()});
    while (!todo.empty()) {
        // Get first element in queue, add it to the set, then remove from queue
        auto crt = todo.front();
        list.insert(crt);
        todo.pop();

        // Safeguard -- stop as soon as we spill out of the obstacle AABB
        if (!(crt > aabb_min && crt < aabb_max)) {
            std::cout << "Obstacle BCE set is NOT watertight!" << std::endl;
            throw std::invalid_argument("Obstacle BCE set is NOT watertight!");
        }

        // Loop through all 6 neighbors of the current node and add them to the end of the work queue
        // if not already in the set
        for (int k = 0; k < 6; k++) {
            auto nbr = crt + nbr3D[k];
            if (list.find(nbr) == list.end())
                todo.push(nbr);
        }
    }

    // Loop through the set of nodes and remove any SPH particle at one of these locations
    size_t num_removed = 0;
    for (const auto& p : list) {
        auto iter = m_sph.find(p);
        if (iter != m_sph.end()) {
            m_sph.erase(iter);
            num_removed++;
        }
    }

    if (m_verbose) {
        cout << "Obstacle mesh name: " << o.trimesh->GetFileName() << endl;
        cout << "  Num. BCE markers: " << o.bce.size() << endl;
        cout << "  Num. grid points in obstacle volume: " << list.size() << endl;
        cout << "  Num. SPH particles removed: " << num_removed << endl;
    }
}

void CRMTerrain::SaveMarkers(const std::string& out_dir) const {
    // SPH particle grid locations
    std::ofstream sph_grid(out_dir + "/sph_grid.txt", std::ios_base::out);
    for (const auto& p : m_sph)
        sph_grid << p << std::endl;

    // Fixed BCE marker grid locations
    std::ofstream bce_grid(out_dir + "/bce_grid.txt", std::ios_base::out);
    for (const auto& p : m_bce)
        bce_grid << p << std::endl;

    // Obstacle BCE marker grid locations
    std::ofstream obs_bce_grid(out_dir + "/obs_bce_grid.txt", std::ios_base::out);
    for (const auto& o : m_obstacles) {
        for (const auto& p : o.bce)
            obs_bce_grid << p << std::endl;
    }

    // Obstacle BCE marker locations
    std::ofstream obs_bce(out_dir + "/obs_bce.txt", std::ios_base::out);
    for (const auto& o : m_obstacles) {
        for (const auto& p : o.point_cloud)
            obs_bce << p << std::endl;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
