import json
import os

demo_name_map = {
    "FSI_Flexible_Cable": "Flexible Cable",
    "FSI_Baffle_Flow": "Baffle Flow",
    "FSI_RASSOR_SingleDrum": "RASSOR Single Drum",
    "FSI_RASSOR": "Full RASSOR",
    "FSI_Viper": "Full Viper",
    "FSI_TRACKED_VEHICLE": "Full Tracked Vehicle"
}


def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)

    simulation_parameters = data.get('simulationParameters', {})
    simulation_settings = data.get('simulationSettings', {})
    timingInfo = data.get('timingInfo', {})

    t_end = simulation_settings.get('tEnd', 0.0)
    cfd_time = timingInfo.get('CFDTime', 0.0)
    fsi_exchange_time = timingInfo.get('FSIExchangeTime', 0.0)
    stepTime = timingInfo.get('stepTime', 0.0)
    # rtf = (cfd_time + fsi_exchange_time) / t_end
    rtf = stepTime / t_end

    # Storing data in a dictionary with type enforcement
    result = {
        'Total Number of Particles': int(simulation_parameters.get('numAllMarkers', 0)),
        'Total Number of Active Particles': int(simulation_parameters.get('numActiveParticles', 0)),
        'Step Size': float(simulation_settings.get('stepSize', 0.0)),
        'RTF': round(float(rtf), 1)
    }

    return result


if __name__ == "__main__":
    demos = ["FSI_Flexible_Cable", "FSI_Baffle_Flow", "FSI_RASSOR_SingleDrum",
             "FSI_RASSOR", "FSI_Viper", "FSI_TRACKED_VEHICLE"]

    benchmarks = ["BENCHMARK_BASELINE_RTF", "BENCHMARK1_RTF", "BENCHMARK2_RTF"]
    results = []

    for demo in demos:
        row = [demo_name_map[demo]]
        numPart = 0
        numActivePart = 0
        if demo == "FSI_Baffle_Flow":
            # BASELINE
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK_BASELINE_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_default_default_ps1_d01.2_scale3.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            stepSize = result_dict['Step Size']
            rtf_baseline = result_dict['RTF']

            # BENCHMARK1 1 Freq
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2_scale3.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            rtf_benchmark1_1freq = result_dict['RTF']

            # BENCHMARK1 8 Freq
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps8_d01.2_scale3.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            rtf_benchmark1_8freq = result_dict['RTF']

            # BENCHMARK2 8 Freq
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps8_d01.2_scale3.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            rtf_benchmark2_8freq = result_dict['RTF']

            row.extend([
                numPart,
                numActivePart,
                stepSize,
                rtf_baseline,
                rtf_benchmark1_1freq,
                rtf_benchmark1_8freq,
                rtf_benchmark2_8freq
            ])
            results.append(row)
        else:
            # BASELINE
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK_BASELINE_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_default_default_ps1_d01.2.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            stepSize = result_dict['Step Size']
            rtf_baseline = result_dict['RTF']

            # BENCHMARK1 1 Freq
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            rtf_benchmark1_1freq = result_dict['RTF']

            # BENCHMARK1 8 Freq
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps8_d01.2.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            rtf_benchmark1_8freq = result_dict['RTF']

            # BENCHMARK2 8 Freq
            file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF",
                                     demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps8_d01.2.json')
            result_dict = read_json_file(file_path)

            numPart = max(numPart, result_dict['Total Number of Particles'])
            numActivePart = max(
                numActivePart, result_dict['Total Number of Active Particles'])
            rtf_benchmark2_8freq = result_dict['RTF']

            row.extend([
                numPart,
                numActivePart,
                stepSize,
                rtf_baseline,
                rtf_benchmark1_1freq,
                rtf_benchmark1_8freq,
                rtf_benchmark2_8freq
            ])
            results.append(row)

    # Generate LaTeX table
    print("\\begin{table}[h!]")
    print("\\centering")
    print("\\begin{tabular}{|l|r|r|r|r|r|r|}")
    print("\\hline")
    print("Demo Name & Total Particles & Active Particles & Time Step & Opt. 1 -- Freq. 1 & Opt. 1 -- Freq. 8 & Opt. 2 -- Freq. 8 \\\\ \\hline")
    for result in results:
        result[3] = "{:.1e}".format(float(result[3]))
        print(" & ".join(map(str, result)) + " \\\\ \\hline")
    print("\\end{tabular}")
    print("\\caption{Comparison of RTF values across benchmarks}")
    print("\\end{table}")

    # Generate plain text table
    print("\nPlain Text Table:")
    header = ["Demo Name", "Total Particles", "Active Particles",
              "Time Step", "Baseline RTF", "Opt. 1 -- Freq. 1", "Opt. 1 -- Freq. 8", "Opt. 2 -- Freq. 8"]
    print("{:<20} {:<15} {:<15} {:<15} {:<12} {:<12} {:<12} {:<12}".format(*header))
    print("-" * 100)
    for result in results:
        result[3] = "{:.1e}".format(float(result[3]))
        print("{:<20} {:<15} {:<15} {:<15} {:<12} {:<12} {:<12} {:<12}".format(*result))
