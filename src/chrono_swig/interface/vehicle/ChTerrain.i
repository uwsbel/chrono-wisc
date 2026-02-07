%{
#include <string>
#include <vector>

#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChFrame.h"

#include "chrono/geometry/ChGeometry.h"
#include "chrono/geometry/ChVolume.h"
#include "chrono/geometry/ChSurface.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChSphere.h"
#include "chrono/geometry/ChCylinder.h"
#include "chrono/geometry/ChCapsule.h"
#include "chrono/geometry/ChCone.h"
#include "chrono/geometry/ChEllipsoid.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLinePoly.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineBSpline.h"
#include "chrono/geometry/ChLineCam.h"
#include "chrono/geometry/ChSurface.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/geometry/ChVolume.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/geometry/ChRoundedCylinder.h"
#include "chrono/geometry/ChRoundedBox.h"

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColormap.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"
#if defined(SWIGCSHARP) && defined(CHRONO_HAS_OPENCRG)
    #include "chrono_vehicle/terrain/CRGTerrain.h"
#endif

#if defined(SWIGPYTHON) || defined(SWIGCSHARP)
#ifdef CHRONO_FSI_SPH
#include "chrono_vehicle/terrain/CRMTerrain.h"
#endif
#endif

#include "chrono_thirdparty/rapidjson/document.h"
%}

#ifdef SWIGCSHARP
%import "chrono_swig/interface/core/ChColor.i"
%import "chrono_swig/interface/core/ChColormap.i"
%import "chrono_swig/interface/core/ChGeometry.i"
%import "chrono_swig/interface/core/ChSystem.i"
%import "chrono_swig/interface/core/ChVector2.i"
%import "chrono_swig/interface/core/ChVector3.i"
%import "chrono_swig/interface/core/ChFrame.i"
%import "chrono_swig/interface/core/ChBody.i"
%import "chrono_swig/interface/core/ChNodeXYZ.i"
%import "chrono_swig/interface/core/ChLoadContainer.i"
%import "../../../chrono/assets/ChVisualShapeTriangleMesh.h"
#ifdef CHRONO_FSI_SPH
#define CH_FSI_API
%shared_ptr(chrono::fsi::ChFsiSystem)
%shared_ptr(chrono::fsi::sph::ChFsiSystemSPH)
%shared_ptr(chrono::fsi::sph::ChFsiFluidSystemSPH)
%shared_ptr(chrono::utils::ChBodyGeometry)
%import "chrono_swig/interface/core/ChBodyGeometry.i"
%import "chrono_swig/interface/fsi/ChFsiDefinitions.i"
%import "chrono_swig/interface/fsi/ChFsiSystem.i"
%import "chrono_swig/interface/fsi/ChFsiFluidSystemSPH.i"
%import "chrono_swig/interface/fsi/ChFsiSystemSPH.i"
%import "chrono_swig/interface/fsi/ChFsiDefinitionsSPH.i"
%import "chrono_swig/interface/fsi/ChFsiProblemSPH.i"
#endif
#endif

#ifdef SWIGPYTHON
%import(module = "pychrono.core") "chrono_swig/interface/core/ChColor.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChColormap.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChGeometry.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChSystem.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector2.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector3.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChNodeXYZ.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChLoadContainer.i"
%import(module = "pychrono.core") "../../../chrono/assets/ChVisualShapeTriangleMesh.h"
#endif

%shared_ptr(chrono::vehicle::ChTerrain)
%shared_ptr(chrono::vehicle::FlatTerrain)
%shared_ptr(chrono::vehicle::RigidTerrain::Patch)
%shared_ptr(chrono::vehicle::RigidTerrain)
%shared_ptr(chrono::vehicle::SCMLoader)
%shared_ptr(chrono::vehicle::SCMTerrain)
%shared_ptr(chrono::vehicle::SCMTerrain::SoilParametersCallback)

#if defined(SWIGPYTHON) || defined(SWIGCSHARP)
#ifdef CHRONO_FSI_SPH
%shared_ptr(chrono::vehicle::CRMTerrain)
#endif
#endif

#if defined(SWIGCSHARP) && defined(CHRONO_HAS_OPENCRG)
%shared_ptr(chrono::vehicle::CRGTerrain)
#endif

%template(ChPatchList) std::vector<std::shared_ptr<chrono::vehicle::RigidTerrain::Patch>>;
%template(ChSCMTerrainNodeLevel) std::pair<chrono::ChVector2i, double>;
%template(ChSCMTerrainNodeLevelList) std::vector<std::pair<chrono::ChVector2i, double>>; // To support SCMTerrain::Get/SetModifiedNodes

// Parse the header file to generate wrappers
%include "../../../chrono_vehicle/ChTerrain.h"    
%include "../../../chrono_vehicle/terrain/FlatTerrain.h"
%include "../../../chrono_vehicle/terrain/RigidTerrain.h"

%feature("director") chrono::vehicle::ChTerrain;
%feature("director") SoilParametersCallback;
%include "cpointer.i"
%pointer_functions(int, intp)
%pointer_functions(double, doublep)
%include "../../../chrono_vehicle/terrain/SCMTerrain.h"

#if defined(SWIGPYTHON) || defined(SWIGCSHARP)
#ifdef CHRONO_FSI_SPH
%include "../../../chrono_vehicle/terrain/CRMTerrain.h"
#endif
#endif

// ---------------------------------------------------------------------------
// C# multiple-inheritance workaround for CRMTerrain.
// CRMTerrain inherits from both ChTerrain and ChFsiProblemCartesian, but C#
// (and therefore SWIG/C#) only supports single inheritance.  SWIG picks
// ChTerrain as the base and drops all ChFsiProblemSPH / ChFsiProblemCartesian
// methods.  The %extend block below re-exposes them as delegate methods so
// they are available from C#.  Python handles multiple inheritance natively
// so this is only needed for the C# target.
// ---------------------------------------------------------------------------
#ifdef SWIGCSHARP
#ifdef CHRONO_FSI_SPH

// Force "new" instead of "override" for Initialize() -- SWIG sees it as virtual
// in ChFsiProblemSPH but the C# base class (ChTerrain) has no such method.
%csmethodmodifiers chrono::vehicle::CRMTerrain::Initialize "public new";

%extend chrono::vehicle::CRMTerrain {

    // ---- ChFsiProblemSPH: setup ------------------------------------------
    void SetVerbose(bool verbose) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetVerbose(verbose);
    }
    void SetGravitationalAcceleration(const chrono::ChVector3d& gravity) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetGravitationalAcceleration(gravity);
    }
    void SetStepSizeCFD(double step) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetStepSizeCFD(step);
    }
    void SetStepsizeMBD(double step) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetStepsizeMBD(step);
    }

    // ---- ChFsiProblemSPH: material / SPH parameters ----------------------
    void SetElasticSPH(const chrono::fsi::sph::ChFsiFluidSystemSPH::ElasticMaterialProperties& mat_props) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetElasticSPH(mat_props);
    }
    void SetCfdSPH(const chrono::fsi::sph::ChFsiFluidSystemSPH::FluidProperties& fluid_props) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetCfdSPH(fluid_props);
    }
    void SetSPHParameters(const chrono::fsi::sph::ChFsiFluidSystemSPH::SPHParameters& sph_params) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetSPHParameters(sph_params);
    }
    void SetSplashsurfParameters(const chrono::fsi::sph::ChFsiFluidSystemSPH::SplashsurfParameters& params) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetSplashsurfParameters(params);
    }

    // ---- ChFsiProblemSPH: rigid bodies -----------------------------------
    void AddRigidBody(std::shared_ptr<chrono::ChBody> body,
                      std::shared_ptr<chrono::utils::ChBodyGeometry> geometry,
                      bool check_embedded,
                      bool use_grid_bce = false) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::AddRigidBody(body, geometry, check_embedded, use_grid_bce);
    }
    void AddRigidBodySphere(std::shared_ptr<chrono::ChBody> body,
                            const chrono::ChVector3d& pos,
                            double radius,
                            bool use_grid_bce = false) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::AddRigidBodySphere(body, pos, radius, use_grid_bce);
    }
    void AddRigidBodyBox(std::shared_ptr<chrono::ChBody> body,
                         const chrono::ChFramed& pos,
                         const chrono::ChVector3d& size) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::AddRigidBodyBox(body, pos, size);
    }
    void AddRigidBodyCylinderX(std::shared_ptr<chrono::ChBody> body,
                               const chrono::ChFramed& pos,
                               double radius,
                               double length,
                               bool use_grid_bce = false) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::AddRigidBodyCylinderX(body, pos, radius, length, use_grid_bce);
    }
    void AddRigidBodyMesh(std::shared_ptr<chrono::ChBody> body,
                          const chrono::ChFramed& pos,
                          const std::string& obj_file,
                          const chrono::ChVector3d& interior_point,
                          double scale) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::AddRigidBodyMesh(body, pos, obj_file, interior_point, scale);
    }
    size_t GetNumBCE(std::shared_ptr<chrono::ChBody> body) const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetNumBCE(body);
    }

    // ---- ChFsiProblemSPH: FEA --------------------------------------------
    void UseNodeDirections(chrono::fsi::NodeDirectionsMode mode) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::UseNodeDirections(mode);
    }
    void SetBcePattern1D(chrono::fsi::sph::BcePatternMesh1D pattern, bool remove_center = false) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetBcePattern1D(pattern, remove_center);
    }
    void SetBcePattern2D(chrono::fsi::sph::BcePatternMesh2D pattern, bool remove_center = false) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetBcePattern2D(pattern, remove_center);
    }
    void AddFeaMesh(std::shared_ptr<chrono::fea::ChMesh> mesh, bool check_embedded) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::AddFeaMesh(mesh, check_embedded);
    }

    // ---- ChFsiProblemSPH: lifecycle --------------------------------------
    void Initialize() {
        $self->chrono::fsi::sph::ChFsiProblemSPH::Initialize();
    }
    void DoStepDynamics(double step) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::DoStepDynamics(step);
    }
    void RegisterParticlePropertiesCallback(
            std::shared_ptr<chrono::fsi::sph::ChFsiProblemSPH::ParticlePropertiesCallback> callback) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::RegisterParticlePropertiesCallback(callback);
    }

    // ---- ChFsiProblemSPH: computational domain ---------------------------
    void SetComputationalDomain(const chrono::ChAABB& aabb,
                                chrono::fsi::sph::BoundaryConditions bc_type = {
                                    chrono::fsi::sph::BCType::NONE,
                                    chrono::fsi::sph::BCType::NONE,
                                    chrono::fsi::sph::BCType::NONE}) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetComputationalDomain(aabb, bc_type);
    }

    // ---- ChFsiProblemSPH: accessors --------------------------------------
    std::shared_ptr<chrono::fsi::sph::ChFsiSystemSPH> GetFsiSystemSPH() {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetFsiSystemSPH();
    }
    std::shared_ptr<chrono::fsi::sph::ChFsiFluidSystemSPH> GetFluidSystemSPH() {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetFluidSystemSPH();
    }
    chrono::ChSystem& GetMultibodySystem() {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetMultibodySystem();
    }
    std::shared_ptr<chrono::ChBody> GetGroundBody() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetGroundBody();
    }

    // ---- ChFsiProblemSPH: query ------------------------------------------
    size_t GetNumSPHParticles() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetNumSPHParticles();
    }
    size_t GetNumBoundaryBCEMarkers() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetNumBoundaryBCEMarkers();
    }
    const chrono::ChAABB& GetComputationalDomain() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetComputationalDomain();
    }
    const chrono::fsi::sph::BoundaryConditions& GetBoundaryConditionTypes() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetBoundaryConditionTypes();
    }
    const chrono::ChAABB& GetSPHBoundingBox() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetSPHBoundingBox();
    }
    const chrono::ChVector3d& GetFsiBodyForce(std::shared_ptr<chrono::ChBody> body) const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetFsiBodyForce(body);
    }
    const chrono::ChVector3d& GetFsiBodyTorque(std::shared_ptr<chrono::ChBody> body) const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetFsiBodyTorque(body);
    }
    double GetRtfCFD() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetRtfCFD();
    }
    double GetRtfMBD() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetRtfMBD();
    }
    chrono::fsi::sph::PhysicsProblem GetPhysicsProblem() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetPhysicsProblem();
    }
    std::string GetPhysicsProblemString() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetPhysicsProblemString();
    }
    std::string GetSphIntegrationSchemeString() const {
        return $self->chrono::fsi::sph::ChFsiProblemSPH::GetSphIntegrationSchemeString();
    }

    // ---- ChFsiProblemSPH: output -----------------------------------------
    void SetOutputLevel(chrono::fsi::sph::OutputLevel output_level) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SetOutputLevel(output_level);
    }
    void SaveOutputData(double time, const std::string& sph_dir, const std::string& fsi_dir) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SaveOutputData(time, sph_dir, fsi_dir);
    }
    void SaveInitialMarkers(const std::string& out_dir) const {
        $self->chrono::fsi::sph::ChFsiProblemSPH::SaveInitialMarkers(out_dir);
    }
    void WriteReconstructedSurface(const std::string& dir, const std::string& name, bool quiet = false) {
        $self->chrono::fsi::sph::ChFsiProblemSPH::WriteReconstructedSurface(dir, name, quiet);
    }
    void PrintStats() const {
        $self->chrono::fsi::sph::ChFsiProblemSPH::PrintStats();
    }
    void PrintTimeSteps(const std::string& path) const {
        $self->chrono::fsi::sph::ChFsiProblemSPH::PrintTimeSteps(path);
    }

    // ---- ChFsiProblemCartesian: Construct ---------------------------------
    void Construct(const std::string& sph_file,
                   const std::string& bce_file,
                   const chrono::ChVector3d& pos,
                   bool use_grid_coordinates = true) {
        $self->chrono::fsi::sph::ChFsiProblemCartesian::Construct(sph_file, bce_file, pos, use_grid_coordinates);
    }
    void Construct(const chrono::ChVector3d& box_size,
                   const chrono::ChVector3d& pos,
                   int side_flags) {
        $self->chrono::fsi::sph::ChFsiProblemCartesian::Construct(box_size, pos, side_flags);
    }
    void Construct(const std::string& heightmap_file,
                   double length,
                   double width,
                   const chrono::ChVector2d& height_range,
                   double depth,
                   bool uniform_depth,
                   const chrono::ChVector3d& pos,
                   int side_flags) {
        $self->chrono::fsi::sph::ChFsiProblemCartesian::Construct(
            heightmap_file, length, width, height_range, depth, uniform_depth, pos, side_flags);
    }

    // ---- ChFsiProblemCartesian: AddBoxContainer --------------------------
    size_t AddBoxContainer(const chrono::ChVector3d& box_size,
                           const chrono::ChVector3d& pos,
                           int side_flags) {
        return $self->chrono::fsi::sph::ChFsiProblemCartesian::AddBoxContainer(box_size, pos, side_flags);
    }
}
#endif
#endif

#if defined(SWIGCSHARP) && defined(CHRONO_HAS_OPENCRG)
%include "../../../chrono_vehicle/terrain/CRGTerrain.h"
#endif
