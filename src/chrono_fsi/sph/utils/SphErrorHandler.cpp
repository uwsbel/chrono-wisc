#include "chrono_fsi/sph/utils/SphErrorHandler.h"
#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"
#include <string>

namespace chrono {
namespace fsi {
namespace sph {

// Global error state variables
bool g_cuda_error_occurred = false;
char g_cuda_error_message[512] = {0};

// Function to check if a CUDA error occurred
bool HasCudaError() {
    return g_cuda_error_occurred;
}

// Function to get the error message
std::string GetCudaErrorMessage() {
    return std::string(g_cuda_error_message);
}

// Function to clear the error state
void ClearCudaError() {
    g_cuda_error_occurred = false;
    g_cuda_error_message[0] = '\0';
}

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
