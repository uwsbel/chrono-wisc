#ifndef CH_UTILS_TIMING_OUTPUT_H
#define CH_UTILS_TIMING_OUTPUT_H

#include <string>
#include "chrono/core/ChTimer.h"
#include "chrono_fsi/sph/ChFluidSystemSPH.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono/core/ChTimer.h"

namespace chrono {
namespace fsi {

/// Create JSON document and write initial simulation parameters
void OutputParameterJSON(const std::string& json_file_path,
                         ChFluidSystemSPH* sysFSI,
                         double t_end,
                         double step_size,
                         const std::string& viscosity_type,
                         const std::string& boundary_type,
                         int ps_freq,
                         double d0_multiplier,
                         rapidjson::Document& doc);

/// Add timing information to existing JSON document and write to file
void OutputTimingJSON(const std::string& json_file_path,
                      const ChTimer& timer,
                      ChFluidSystemSPH* sysFSI,
                      rapidjson::Document& doc);

}  // namespace fsi
}  // namespace chrono

#endif