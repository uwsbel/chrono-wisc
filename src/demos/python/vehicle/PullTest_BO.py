from PullTest_sim import Params
from PullTest_sim import sim

def run_simulation(Params):
    total_time_to_reach, sim_failed = sim(Params)
    return total_time_to_reach, sim_failed

if __name__ == "__main__":
    Params = Params()
    particle_spacing = 0.01
    Params.rad = 0.09/particle_spacing
    Params.width = 0.08/particle_spacing
    Params.g_height = 0.025/particle_spacing
    Params.g_width = 0.02/particle_spacing
    Params.g_density = 8
    Params.particle_spacing = particle_spacing
    Params.grouser_type = 0
    Params.fan_theta_deg = 90.0
    total_time_to_reach, sim_failed = run_simulation(Params)
    print(f"Total time to reach: {total_time_to_reach}")
    print(f"Simulation failed: {sim_failed}")