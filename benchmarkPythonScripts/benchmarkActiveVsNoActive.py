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

    rtf = (cfd_time + fsi_exchange_time) / t_end

    # Storing data in a dictionary with type enforcement
    result = {
        'Total Number of Particles': int(simulation_parameters.get('numAllMarkers', 0)),
        'Total Number of Active Particles': int(simulation_parameters.get('numActiveParticles', 0)),
        'Step Size': float(simulation_settings.get('stepSize', 0.0)),
        'RTF': round(float(rtf), 1)
    }

    return result


if __name__ == "__main__":
    demos = ["FSI_RASSOR_SingleDrum"]

    results = []

    row = ["No Active"]

    # BENCHMARK2 8 Freq
    file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF_NOACTIVE",
                             demos[0], 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2.json')
    result_dict = read_json_file(file_path)

    numPart2 = result_dict['Total Number of Particles']
    rtf_benchmark2_1freq = result_dict['RTF']

    row.extend([
        numPart2,
        rtf_benchmark2_1freq
    ])
    results.append(row)

    row = ["Active"]

    # BENCHMARK2 8 Freq - Active
    file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF",
                             demos[0], 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2.json')
    result_dict = read_json_file(file_path)

    numPart1 = result_dict['Total Number of Particles']
    rtf_benchmark2_1freq = result_dict['RTF']

    row.extend([
        numPart1,
        rtf_benchmark2_1freq
    ])
    results.append(row)

    # Generate LaTeX table
    print("\\begin{table}[h!]")
    print("\\centering")
    print("\\begin{tabular}{|r|r|}")
    print("\\hline")
    print(" & Total Particles & RTF \\\\ \\hline")
    for result in results:
        print(" & ".join(map(str, result)) + " \\\\ \\hline")
    print("\\end{tabular}")
    print("\\caption{Comparison of RTF values across benchmarks}")
    print("\\end{table}")

    # Generate plain text table
    print("\nPlain Text Table:")
    header = ["Code", "Total Particles", "RTF"]
    print("{:<20} {:<15} {:<15}".format(*header))
    print("-" * 100)
    for result in results:
        print("{:<20} {:<15} {:<15}".format(*result))
