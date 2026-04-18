# Plotting Scripts

This folder is now restricted to the plotting scripts that map to the figures in the paper results section.

Most of these files were copied from `build_new/bin`. The lander sweep analyzer was copied from `src/demos/robot/lander/analysis/plot_lander_crm_sweep.py` because that figure generator lives in source rather than the build tree.

As copied, these scripts still expect a build-tree context and relative paths like `./DEMO_OUTPUT`, `./DEMO_OUTPUT_NREL_LANDER`, and `./paper_plots`.

| Script | Original source | Paper section / figure family | Notes |
| --- | --- | --- | --- |
| `penetrometer_plot_single.py` | `build_new/bin/penetrometer_plot_single.py` | Cone Penetrometer Test | Driver script for the cone-penetration comparison figure. |
| `penetrometer_plotting.py` | `build_new/bin/penetrometer_plotting.py` | Cone Penetrometer Test | Support plotting module used by `penetrometer_plot_single.py`. |
| `penetrometer_folderParser.py` | `build_new/bin/penetrometer_folderParser.py` | Cone Penetrometer Test | Support parser used by `penetrometer_plot_single.py`. |
| `plot_plate_penetration.py` | `build_new/bin/plot_plate_penetration.py` | Normal Bevameter Test | Generates the main bevameter pressure-sinkage comparison plot. |
| `viper_singleWheel_plotting.py` | `build_new/bin/viper_singleWheel_plotting.py` | MGRU3 single-wheel constant-depth results | Generates the constant-depth slip-vs-slope comparison figure. |
| `viper_singleWheel_DepthVar_plotting.py` | `build_new/bin/viper_singleWheel_DepthVar_plotting.py` | MGRU3 depth-sensitivity results | Generates the variable-depth slip-vs-slope figure family, including the separate MCC and `MU_OF_I` panels. |
| `plot_lander_crm_sweep.py` | `src/demos/robot/lander/analysis/plot_lander_crm_sweep.py` | Lunar lander touchdown sensitivity study | Generates `lander_grouped_profiles.png` and `lander_main_effects.png`. |

## Notes

- I removed the cone-drop plotters, actual-slope wheel plotters, bevameter depth/cohesion sensitivity plotter, lander diagnostic quicklooks, and Blender render helper because they do not map directly to the manuscript figures.
- The cone-penetrometer plotting stack is split across three files because `penetrometer_plot_single.py` imports the other two.
