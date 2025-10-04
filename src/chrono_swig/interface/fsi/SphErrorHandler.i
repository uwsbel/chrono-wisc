//////////////////////////////////////////////////
//  
//   SphErrorHandler.i
//
//   SWIG configuration file for SPH error handling functions.
//   This is processed by SWIG to create the Python wrapper
//   for CUDA error handling functions.
//
///////////////////////////////////////////////////

%{
#include "chrono_fsi/sph/utils/SphErrorHandler.h"
using namespace chrono::fsi::sph;
%}

// Include the error handling functions
%include "chrono_fsi/sph/utils/SphErrorHandler.h"

// Make the functions available in Python with simpler names
%inline %{
    // Python-friendly wrapper functions
    bool CheckCudaError() {
        return chrono::fsi::sph::HasCudaError();
    }
    
    std::string GetCudaErrorString() {
        return chrono::fsi::sph::GetCudaErrorMessage();
    }
    
    void ResetCudaError() {
        chrono::fsi::sph::ClearCudaError();
    }
%}
