%{
#include "chrono/physics/ChUpdateFlags.h"
using namespace chrono;
%}

// SWIG 4.0 does not handle 'enum class : uint8_t' well.
// Manually declare the enum for the target language.
namespace chrono {

enum class UpdateFlags {
    NONE = 0,
    DYNAMICS = 1,
    JACOBIANS = 2,
    VISUAL_ASSETS = 4,
    UPDATE_ALL = 7,
    UPDATE_ALL_NO_VISUAL = 3
};

}
