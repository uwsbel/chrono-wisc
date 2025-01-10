import json
import os
import logging

# Delete the old log file if it exists
log_file = 'discrepancies.log'
if os.path.exists(log_file):
    os.remove(log_file)

# Set up logging
logging.basicConfig(filename=log_file, level=logging.INFO)


def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)

    # Extracting data from the JSON
    hardware_info = data.get('hardwareInfo', {})
    simulation_parameters = data.get('simulationParameters', {})
    simulation_settings = data.get('simulationSettings', {})

    # Storing data in a dictionary with type enforcement
    result = {
        'gpu_name': str(hardware_info.get('gpuName', '')),
        'compute_capability': str(hardware_info.get('computeCapability', '')),
        'num_fluid_markers': int(simulation_parameters.get('numFluidMarkers', 0)),
        'num_boundary_markers': int(simulation_parameters.get('numBoundaryMarkers', 0)),
        'num_rigid_markers': int(simulation_parameters.get('numRigidMarkers', 0)),
        'num_flex_markers_1d': int(simulation_parameters.get('numFlexMarkers1D', 0)),
        'num_flex_markers_2d': int(simulation_parameters.get('numFlexMarkers2D', 0)),
        'num_active_particles': int(simulation_parameters.get('numActiveParticles', 0)),
        'c_min': [float(x) for x in simulation_parameters.get('cMin', [])],
        'c_max': [float(x) for x in simulation_parameters.get('cMax', [])],
        'end_time': float(simulation_settings.get('tEnd', 0.0)),
        'step_size': float(simulation_settings.get('stepSize', 0.0)),
        'initial_spacing': float(simulation_settings.get('initialSpacing', 0.0)),
        'kernel_radius': float(simulation_settings.get('kernelRadius', 0.0))
    }

    return result


# Compare dictionary items between all dictionaries in list and log discrepancies if any in log file
def ensure_consistency(list_of_results, file_paths, tolerance=1e-3):
    if not list_of_results:
        return

    # Compare each pair of results
    for i in range(len(list_of_results)):
        reference = list_of_results[i]
        reference_file = file_paths[i]

        for j in range(i + 1, len(list_of_results)):
            result = list_of_results[j]
            file_path = file_paths[j]
            discrepancies = []

            for key in reference:
                ref_value = reference[key]
                res_value = result.get(key)

                if isinstance(ref_value, list) and isinstance(res_value, list):
                    # Check lists of doubles with tolerance
                    if len(ref_value) != len(res_value) or not all(abs(r - s) < tolerance for r, s in zip(ref_value, res_value)):
                        discrepancies.append((key, ref_value, res_value))
                elif isinstance(ref_value, float) and isinstance(res_value, float):
                    # Check individual doubles with tolerance
                    if abs(ref_value - res_value) >= tolerance:
                        discrepancies.append((key, ref_value, res_value))
                else:
                    # Check other types (e.g., int) for exact match
                    if ref_value != res_value:
                        discrepancies.append((key, ref_value, res_value))

            if discrepancies:
                # Extract benchmark and demo from file paths
                reference_benchmark = os.path.basename(
                    os.path.dirname(os.path.dirname(reference_file)))
                reference_demo = os.path.basename(
                    os.path.dirname(os.path.dirname(os.path.dirname(reference_file))))
                current_benchmark = os.path.basename(
                    os.path.dirname(os.path.dirname(file_path)))
                current_demo = os.path.basename(
                    os.path.dirname(os.path.dirname(os.path.dirname(file_path))))

                logging.info(
                    f"Discrepancies found between {reference_benchmark}/{reference_demo} and {current_benchmark}/{current_demo}:")
                for key, ref_value, res_value in discrepancies:
                    logging.info(
                        f"  Key: {key}, Reference: {ref_value}, Current: {res_value}")


if __name__ == "__main__":
    demos = ["FSI_Flexible_Cable", "FSI_Baffle_Flow", "FSI_RASSOR_SingleDrum",
             "FSI_RASSOR", "FSI_Viper", "FSI_TRACKED_VEHICLE"]

    for demo in demos:
        list_of_results = []
        file_paths = []
        if demo == "FSI_Baffle_Flow":
            for scale in [1, 2]:
                list_of_results = []
                file_paths = []
                file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK_BASELINE_RTF",
                                         demo, 'CRM_WCSPH', 'rtf_default_default_ps1_d01.2_scale' + str(scale) + '.json')
                result_dict = read_json_file(file_path)
                list_of_results.append(result_dict)
                file_paths.append(file_path)

                file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                         demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2_scale' + str(scale) + '.json')
                result_dict = read_json_file(file_path)
                list_of_results.append(result_dict)
                file_paths.append(file_path)

                file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF",
                                         demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2_scale' + str(scale) + '.json')
                result_dict = read_json_file(file_path)
                list_of_results.append(result_dict)
                file_paths.append(file_path)
                ensure_consistency(list_of_results, file_paths)
        elif demo == "FSI_RigidBCE_Scaling":
            for scale in [1, 10]:
                file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK_BASELINE_RTF",
                                         demo, 'CRM_WCSPH', 'rtf_default_default_ps1_d01.2_scale' + str(scale) + '.json')
                result_dict = read_json_file(file_path)
                list_of_results.append(result_dict)
                file_paths.append(file_path)

                file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                         demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2_scale' + str(scale) + '.json')
                result_dict = read_json_file(file_path)
                list_of_results.append(result_dict)
                file_paths.append(file_path)

                file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF",
                                         demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2_scale' + str(scale) + '.json')
                result_dict = read_json_file(file_path)
                list_of_results.append(result_dict)
                file_paths.append(file_path)
                ensure_consistency(list_of_results, file_paths)
        else:
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK_BASELINE_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_default_default_ps1_d01.2.json')
            result_dict = read_json_file(file_path)
            list_of_results.append(result_dict)
            file_paths.append(file_path)

            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2.json')
            result_dict = read_json_file(file_path)
            list_of_results.append(result_dict)
            file_paths.append(file_path)

            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2.json')
            result_dict = read_json_file(file_path)
            list_of_results.append(result_dict)
            file_paths.append(file_path)
            ensure_consistency(list_of_results, file_paths)
