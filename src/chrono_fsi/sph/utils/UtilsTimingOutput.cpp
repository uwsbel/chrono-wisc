#include "chrono_fsi/sph/utils/UtilsTimingOutput.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {
/// Create JSON document and write initial simulation parameters
void OutputParameterJSON(const std::string& json_file_path,
                         ChFsiSystem* sysFSI,
                         double t_end,
                         double step_size,
                         const std::string& viscosity_type,
                         const std::string& boundary_type,
                         int ps_freq,
                         double d0_multiplier,
                         rapidjson::Document& doc) {
    doc.SetObject();
    auto& allocator = doc.GetAllocator();
    sph::ChFsiFluidSystemSPH& sysSPH = dynamic_cast<sph::ChFsiFluidSystemSPH&>(sysFSI->GetFluidSystem());
    // Get required parameters
    sph::ChFsiParamsSPH params = sysSPH.GetParams();
    const sph::Counters& counters = sysSPH.GetCounters();
    const sph::CudaDeviceInfo& cuda_info = sysSPH.GetCudaDeviceInfo();

    // Hardware Info
    rapidjson::Value hardwareInfo(rapidjson::kObjectType);
    hardwareInfo.AddMember("gpuName", rapidjson::Value(cuda_info.deviceProp.name, allocator), allocator);
    hardwareInfo.AddMember(
        "computeCapability",
        rapidjson::Value(
            (std::to_string(cuda_info.deviceProp.major) + "." + std::to_string(cuda_info.deviceProp.minor)).c_str(),
            allocator),
        allocator);
    doc.AddMember("hardwareInfo", hardwareInfo, allocator);

    // Simulation Parameters
    rapidjson::Value simParams(rapidjson::kObjectType);
    simParams.AddMember("numNeighborsEst", params.num_neighbors, allocator);
    simParams.AddMember("numFluidMarkers", counters.numFluidMarkers, allocator);
    simParams.AddMember("numBoundaryMarkers", counters.numBoundaryMarkers, allocator);
    simParams.AddMember("numRigidMarkers", counters.numRigidMarkers, allocator);
    simParams.AddMember("numFlexMarkers1D", counters.numFlexMarkers1D, allocator);
    simParams.AddMember("numFlexMarkers2D", counters.numFlexMarkers2D, allocator);
    simParams.AddMember("numAllMarkers", counters.numAllMarkers, allocator);
    simParams.AddMember("numFsiBodies", counters.numFsiBodies, allocator);
    simParams.AddMember("numFsiElements1D", counters.numFsiElements1D, allocator);
    simParams.AddMember("numFsiElements2D", counters.numFsiElements2D, allocator);
    simParams.AddMember("numFsiNodes1D", counters.numFsiNodes1D, allocator);
    simParams.AddMember("numFsiNodes2D", counters.numFsiNodes2D, allocator);
    simParams.AddMember("numActiveParticles", sysSPH.GetNumActiveParticles(), allocator);
    rapidjson::Value cMin(rapidjson::kArrayType);
    cMin.PushBack(params.cMin.x, allocator);
    cMin.PushBack(params.cMin.y, allocator);
    cMin.PushBack(params.cMin.z, allocator);
    simParams.AddMember("cMin", cMin, allocator);

    rapidjson::Value cMax(rapidjson::kArrayType);
    cMax.PushBack(params.cMax.x, allocator);
    cMax.PushBack(params.cMax.y, allocator);
    cMax.PushBack(params.cMax.z, allocator);
    simParams.AddMember("cMax", cMax, allocator);
    doc.AddMember("simulationParameters", simParams, allocator);

    // Simulation Settings
    rapidjson::Value simSettings(rapidjson::kObjectType);
    simSettings.AddMember("tEnd", t_end, allocator);
    simSettings.AddMember("stepSize", step_size, allocator);
    simSettings.AddMember("initialSpacing", params.d0, allocator);
    simSettings.AddMember("kernelRadius", params.h, allocator);
    simSettings.AddMember("numBceLayers", params.num_bce_layers, allocator);
    simSettings.AddMember("densityReinit", params.densityReinit, allocator);
    simSettings.AddMember("viscosityType", rapidjson::Value(viscosity_type.c_str(), allocator), allocator);
    simSettings.AddMember("boundaryType", rapidjson::Value(boundary_type.c_str(), allocator), allocator);
    simSettings.AddMember("proximitySearchFreq", ps_freq, allocator);
    simSettings.AddMember("d0Multiplier", d0_multiplier, allocator);
    doc.AddMember("simulationSettings", simSettings, allocator);

    // Write to file
    std::ofstream json_out(json_file_path);
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    json_out << buffer.GetString();
    json_out.close();
}

///  Add timing information to existing JSON document and write to file
void OutputTimingJSON(const std::string& json_file_path,
                      double timer_step,
                      double timer_CFD,
                      double timer_MBS,
                      double timer_FSI,
                      ChFsiSystem* sysFSI,
                      rapidjson::Document& doc) {
    auto& allocator = doc.GetAllocator();

    // Timing Info
    rapidjson::Value timingInfo(rapidjson::kObjectType);
    timingInfo.AddMember("stepTime", timer_step, allocator);
    timingInfo.AddMember("MBSTime", timer_MBS, allocator);
    timingInfo.AddMember("CFDTime", timer_CFD, allocator);
    timingInfo.AddMember("FSIExchangeTime", timer_FSI, allocator);

    doc.AddMember("timingInfo", timingInfo, allocator);

    // Write updated JSON to file
    std::ofstream json_out(json_file_path);
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    json_out << buffer.GetString();
    json_out.close();
}

}  // namespace fsi
}  // namespace chrono
