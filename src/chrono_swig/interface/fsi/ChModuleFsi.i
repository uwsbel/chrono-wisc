///////////////////////////////////////////////////
//
//   ChModuleFsi.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the 
//   Python wrapper for Chrono::FSI.
//
///////////////////////////////////////////////////
%module(directors="1") fsi

%feature("autodoc", "1");
%feature("flatnested", "1");

// Enable exception handling to map C++ exceptions to Python
%include "exception.i"
%include "../fea/ChModuleFea.i"
%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}



// Turn on casting for polymorphic objects
%include "../chrono_cast.i"


%inline %{
namespace chrono {
namespace fsi {

// 注意：此处仅做“最简声明”以告诉 SWIG：
//  - 这个类在 chrono::fsi 命名空间
//  - 它含有一些虚函数 / 析构等
//  - 若有复杂构造函数、成员等，你可以选择完全或部分写上
class ChFsiInterface {
public:
    virtual ~ChFsiInterface() {}
    virtual void ExchangeSolidStates() = 0;
    virtual void ExchangeSolidForces() = 0;
};

} // end namespace fsi
} // end namespace chrono
%}


// Include necessary C++ headers
%{
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFluidSystem.h"
//#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/ChFsiSystem.h"
#include "chrono_fsi/ChFsiDefinitions.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceSegmentSet.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fsi;
%}

#define ChApi

%include "std_shared_ptr.i"
%include "std_vector.i"
%include "std_string.i"

%include "chrono/fea/ChContactSurfaceMesh.h"
%include "chrono/fea/ChContactSurfaceSegmentSet.h"
%include "chrono/fea/ChMesh.h"

// Define shared pointers for FSI module classes
%shared_ptr(chrono::fsi::ChFluidSystem)
%shared_ptr(chrono::fsi::ChFsiInterface)
%shared_ptr(chrono::fsi::ChFsiSystem)

// Ignore functions that are not required in Python or complex to wrap
%ignore chrono::fsi::ChFluidSystem::OnDoStepDynamics;
%ignore chrono::fsi::ChFsiInterface::ExchangeSolidStates;
%ignore chrono::fsi::ChFsiInterface::ExchangeSolidForces;

// Templates for STL containers used in FSI
%template(vector_FsiBodyState) std::vector<chrono::fsi::FsiBodyState>;
%template(vector_FsiMeshState) std::vector<chrono::fsi::FsiMeshState>;
%template(vector_FsiBodyForce) std::vector<chrono::fsi::FsiBodyForce>;
%template(vector_FsiMeshForce) std::vector<chrono::fsi::FsiMeshForce>;

// Include FSI-specific classes and their dependencies
%include "chrono_fsi/ChApiFsi.h"
%include "chrono_fsi/ChFluidSystem.h"
//%include "chrono_fsi/ChFsiInterface.h"
%include "chrono_fsi/ChFsiSystem.h"
%include "chrono_fsi/ChFsiDefinitions.h"

// Casting functions to handle polymorphic objects
%DefSharedPtrDynamicCast(chrono::fsi, ChFsiInterface, ChFsiInterfaceGeneric)

// Extend constructors or utility methods for Python
%extend chrono::fsi::ChFsiSystem {
    ChFsiSystem(std::shared_ptr<chrono::ChSystem> sysMBS, std::shared_ptr<chrono::fsi::ChFluidSystem> sysCFD) {
        return new chrono::fsi::ChFsiSystem(*sysMBS, *sysCFD);
    }
}

%extend chrono::fsi::ChFluidSystem {
    void SetVerbose(bool verbose) {
        $self->SetVerbose(verbose);
    }
}

// Support direct access to struct elements
%inline %{
    chrono::fsi::FsiBodyState CreateFsiBodyState(const chrono::ChVector3d& pos, const chrono::ChQuaternion<>& rot) {
        chrono::fsi::FsiBodyState state;
        state.pos = pos;
        state.rot = rot;
        return state;
    }
%}

// Custom Python-side utility methods
%inline %{
    chrono::fsi::ChFsiSystem* CreateFsiSystem(chrono::ChSystem* sysMBS, chrono::fsi::ChFluidSystem* sysCFD) {
        return new chrono::fsi::ChFsiSystem(*sysMBS, *sysCFD);
    }
%}

// Add Python code (optional)
//%pythoncode %{
//# Example: Create a new FSI system
//#from pychrono import fsi
//#fsi_system = fsi.CreateFsiSystem(mbs_system, fluid_system)
//%}