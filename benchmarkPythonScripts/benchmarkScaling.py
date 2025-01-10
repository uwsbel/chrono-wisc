import json
import os
import matplotlib.pyplot as plt

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
    demo = "FSI_RigidBceScalingTest"

    scales = [1, 10, 100, 1000, 2000, 3000, 4000]
    atomic = []
    shared = []
    for scale in scales:
        # BENCHMARK1
        file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK1_RTF",
                                 demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2_scale' + str(scale) + '.json')
        result_dict = read_json_file(file_path)

        atomic.append(result_dict['RTF'])

        # BENCHMARK1
        file_path = os.path.join(os.path.dirname(__file__), "BENCHMARK2_RTF",
                                 demo, 'CRM_WCSPH', 'rtf_artificial_bilateral_adami_ps1_d01.2_scale' + str(scale) + '.json')
        result_dict = read_json_file(file_path)

        shared.append(result_dict['RTF'])

    # Plotting
    plt.figure(figsize=(6, 6))
    plt.plot(scales, atomic, label='Atomic', marker='o', linestyle='-')
    plt.plot(scales, shared, label='Shared', marker='s', linestyle='--')

    # Formatting
    plt.title('RTF vs Scale', fontsize=14, fontweight='bold')
    plt.xlabel('Scale', fontsize=12)
    plt.ylabel('RTF', fontsize=12)
    plt.legend()
    plt.grid(True)
    plt.xticks(scales)
    plt.yticks(fontsize=10)
    plt.tight_layout()

    # Save as a high-resolution image
    plt.savefig('rtf_vs_scale.png', dpi=300)

    # Show the plot
    plt.show()
