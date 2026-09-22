Wave-Domain Radar Sensor Model {#phys_radar_sensor}
===================================================

\tableofcontents

chrono::sensor::ChPhysRadarSensor models a frequency modulated continuous wave radar in the wave
domain. Rays carry delay, Doppler, amplitude and phase rather than range and reflectivity, and the
sensor forms a range-Doppler-angle map from their coherent sum before detecting on it.

The point of working this way is failure-mode independence. A ray-cast radar inherits a lidar's
error model: it can only miss what is occluded or out of range. A radar formed from a signal misses
a target because its return fell under a detection threshold, reports a ghost because a guardrail
reflected a real vehicle into a mirrored bearing, reports a velocity of -3 m/s for a target closing
at 52 because the waveform folds, and reports a target several cycles late because a track has to
be confirmed. None of that is injected. All of it follows from the signal, which is what makes a
simulated radar useful alongside a simulated lidar and camera rather than a third copy of the same
geometric failure model.

chrono::sensor::ChRadarSensor, the earlier ray-cast radar, is unchanged and still available.

## Pipeline {#phys_radar_pipeline}

Each update runs four stages.

1. **Propagation.** A stratified grid of rays covers the field of view. Each one walks the scene
   under geometric optics and, at every surface it meets, estimates the next event back to the
   antenna through an occlusion test. A visible surface therefore contributes one coherent path per
   bounce depth, carrying total path length, its rate of change, a voltage amplitude and a phase.
   Path length is accumulated in double precision and reduced to a phase before it is rounded,
   because at 300 m a float path length is already coarser than a tenth of a wavelength.

2. **Signal formation** (chrono::sensor::ChFilterPhysRadarSignalForm). A fast-chirp radar forms its
   range-Doppler map by windowing and transforming a dechirped beat signal. For a single path that
   beat signal is a windowed complex exponential, whose transform is known in closed form, so the
   cube is written directly: each path contributes its window's response centred on the range and
   Doppler it implies, on every virtual channel, with the array's phase applied. Thermal noise, the
   transmitter's leakage into the receiver and the converter's quantization floor are added here.

3. **Detection** (chrono::sensor::ChFilterPhysRadarDetect). Beams are formed across the virtual
   array, the phase noise skirt of every strong cell is spread across its range neighbours, and a
   cell averaging or ordered statistic constant false alarm rate detector thresholds the result.
   Peaks are interpolated to sub-bin range, velocity and bearing, and the radar equation is
   inverted on the peak cell to estimate a radar cross section.

4. **Tracking** (chrono::sensor::ChFilterPhysRadarTrack). Detections are grouped, associated with
   existing tracks inside a gate, and updated through a constant-velocity Kalman filter. A track is
   reported only once it has been hit on several of the last few cycles.

## Creating a radar {#phys_radar_create}

~~~{.cpp}
// Start from the shipped 77 GHz front radar, or read an archetype from data/sensor/radar.
ChRadarModelConfig config = MakeDefaultFrontRadarConfig();
config.dsp.suppress_stationary = true;

auto radar = chrono_types::make_shared<ChPhysRadarSensor>(
    parent_body,   // body the radar is mounted on
    14.0f,         // cycles per second
    offset_pose,   // mounting pose
    config);       // radar model
radar->SetName("front radar");
~~~

The constructor appends the signal formation, detection and tracking stages. Pass `false` as a
fifth argument to work with the raw path list instead.

## Configuration {#phys_radar_config}

chrono::sensor::ChRadarModelConfig holds the waveform, the antenna array, the radio-frequency
impairments, the detection chain, the tracker and the ray budget. One engine covers many radar
models: the archetypes under `data/sensor/radar` differ from each other only in this structure.

| File | Model |
|---|---|
| `ars408_like_far.json` | long range front radar, narrow beam, 250 m |
| `ars408_like_near.json` | wide near-field mode, 100 m, fine range resolution |
| `mimo_tdm_like.json` | 3 transmitters by 4 receivers, time division multiplexed |

~~~{.cpp}
auto config = ChRadarModelConfig::ReadJSON(GetChronoDataFile("sensor/radar/ars408_like_far.json"));
std::cout << config.GetDescription() << std::endl;
~~~

`Validate()` rejects an inconsistent model and names the offending field: digitizing for longer
than the sweep lasts, asking for a range the waveform aliases, or declaring several transmitters
with no scheme to separate them.

Derived quantities follow the usual relations and are available as accessors: range resolution
`c / 2B` from the bandwidth actually swept while the converter runs, velocity resolution
`lambda / (2 N T)` from the length of the coherent processing interval, and the folding velocity
`lambda / 4T`. Time division multiplexing multiplies the aperture by the number of transmitters
and divides the folding velocity by the same number, which is where velocity ghosts come from.

## Materials {#phys_radar_materials}

Core Chrono's `ChVisualMaterial` carries no electromagnetic parameters, and radar has no business
adding any. chrono::sensor::ChRadarMaterialRegistry instead keys off the visual material class id
that already reaches the shaders, so a scene declares its radar properties by tagging its visual
materials:

~~~{.cpp}
material->SetClassID(SURFACE_GUARDRAIL);          // on the ChVisualMaterial
radar->GetMaterialRegistry().Assign(SURFACE_GUARDRAIL, "metal");
~~~

Sample surfaces are `metal`, `vehicle_body`, `bumper`, `glass`, `asphalt`, `concrete`,
`pedestrian`, `wood` and `vegetation`. Each splits its response into a coherent specular lobe and
an incoherent diffuse one; that split is what makes fading, clutter and mirror ghosts emerge rather
than be injected. A class id with no entry falls back to a response derived from the material's
metallic and roughness channels, so an untagged scene still behaves sensibly.

## Reading the output {#phys_radar_output}

chrono::sensor::ChFilterPhysRadarAccess hands back one radar frame per cycle: the complex cube, the
beamformed power, the detection power map, the threshold the detector derived from it, the
detections and the tracks. How much of that is copied to the host is a constructor argument, since
the beamformed cube is by far the largest array in a frame.

~~~{.cpp}
auto access = chrono_types::make_shared<ChFilterPhysRadarAccess>(ChRadarFrameContents::MAPS);
radar->PushFilter(access);
...
auto frame = access->GetBuffer();
for (unsigned int i = 0; i < frame->NumDetections; i++) {
    const RadarDetection& d = frame->Detections[i];
    // d.range, d.range_rate, d.azimuth, d.snr_db, d.rcs_dbsm, d.object_id, d.flags
}
for (const RadarObject& object : frame->Objects) {
    // object.id, object.x, object.y, object.vx, object.rcs_dbsm, object.existence_probability
}
~~~

Every detection carries `object_id`, the ground-truth instance that dominated its cell, and
`RADAR_DET_MULTIPATH`, set when that contributor reached the antenna by an indirect path. Together
they are what lets a ghost be told from a target without guessing, which makes automated scoring of
a perception stack against the radar's own errors possible.

The same frame is available from Python as numpy arrays through `GetPowerMapData()`,
`GetThresholdMapData()`, `GetAnglePowerData()` and `GetCubeData()`.

## Looking inside {#phys_radar_visualize}

chrono::sensor::ChFilterPhysRadarVisualize opens a window onto the signal chain: range against
Doppler, the same cells measured against their detection threshold, range against bearing, and a
plan view of the detections and tracks with indirect paths marked. `SetSaveDirectory()` writes the
frames to disk instead, which is how the view is used on a machine with no display.

~~~{.cpp}
radar->PushFilter(chrono_types::make_shared<ChFilterPhysRadarVisualize>(1280, 800, "Radar"));
~~~

## What the model does not do {#phys_radar_limits}

The scattering model is asymptotic: surfaces are assumed large compared with the 3.9 mm
wavelength, and there is no edge diffraction, no creeping wave and no surface wave. Amplitudes come
from the radar equation applied per ray tube with a physical-optics estimate of the specular lobe
width, not from a surface current integral, so lobe structure is approximate where lobe magnitude
is not. Range migration inside one coherent processing interval is not modeled: range is evaluated
once per interval, which stays well inside a range bin at automotive closing speeds. Beams are
formed in azimuth only. Mutual interference between radars is not modeled at all.

## Demos {#phys_radar_demos}

- `demo_SEN_phys_radar.cpp` — a highway arranged so that a mirror ghost, a masked motorcycle, a
  late cut-in and a folded oncoming velocity all appear in one run.
- `demo_SEN_phys_radar.py` — the same scene from Python, reading the maps back as numpy arrays.
