%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

using namespace chrono::fsi::sph;

%}

// ---------------------------------------------------------------------------
// C# bulk-copy extension: avoids per-particle SWIG P/Invoke overhead.
// Instead of ~200K managed-native transitions per frame, a single P/Invoke
// call copies all particle positions into a pre-allocated float[] buffer.
// ---------------------------------------------------------------------------
#ifdef SWIGCSHARP

%include "arrays_csharp.i"
%apply float OUTPUT[] { float *outBuffer }

%extend chrono::fsi::sph::ChFsiFluidSystemSPH {
    /// Bulk-copy SPH particle positions into a caller-provided float buffer.
    /// Each particle writes 3 consecutive floats (x, y, z) in Chrono frame.
    /// @param outBuffer  Pre-allocated float array of size >= maxCount * 3.
    /// @param maxCount   Maximum number of particles to copy.
    /// @return           Actual number of particles copied.
    int CopyParticlePositionsToArray(float *outBuffer, int maxCount) {
        auto positions = $self->GetParticlePositions();
        int count = (int)positions.size();
        if (count > maxCount) count = maxCount;
        for (int i = 0; i < count; i++) {
            const auto& p = positions[i];
            outBuffer[i * 3 + 0] = (float)p.x();
            outBuffer[i * 3 + 1] = (float)p.y();
            outBuffer[i * 3 + 2] = (float)p.z();
        }
        return count;
    }

    /// Bulk-copy BCE marker positions (boundary + rigid body + flex body)
    /// into a caller-provided float buffer. BCE markers are all markers
    /// beyond the fluid SPH particles in the GetPositions() array.
    /// @param outBuffer  Pre-allocated float array of size >= maxCount * 3.
    /// @param maxCount   Maximum number of BCE markers to copy.
    /// @return           Actual number of BCE markers copied.
    int CopyBCEPositionsToArray(float *outBuffer, int maxCount) {
        auto allPos = $self->GetPositions();
        size_t numFluid = $self->GetNumFluidMarkers();
        size_t total = allPos.size();
        int numBce = (int)(total > numFluid ? total - numFluid : 0);
        if (numBce > maxCount) numBce = maxCount;
        for (int i = 0; i < numBce; i++) {
            const auto& p = allPos[numFluid + i];
            outBuffer[i * 3 + 0] = (float)p.x;
            outBuffer[i * 3 + 1] = (float)p.y;
            outBuffer[i * 3 + 2] = (float)p.z;
        }
        return numBce;
    }
}

%clear float *outBuffer;

#endif

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/sph/ChFsiFluidSystemSPH.h"
