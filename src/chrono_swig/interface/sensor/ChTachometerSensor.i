%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/ChTachometerSensor.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChTachometerSensor)
%shared_ptr(chrono::sensor::ChEncoderSensor)

/* Parse the header file to generate wrappers */
%include "../../../chrono_sensor/sensors/ChTachometerSensor.h"

%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChTachometerSensor)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChEncoderSensor)
