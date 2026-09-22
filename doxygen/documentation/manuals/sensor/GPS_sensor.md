GPS Sensor Model {#GPS_sensor}
=================================

\tableofcontents

The GPS in Chrono computes the latitude, longitude, and altitude of the antenna by mapping the
Cartesian simulation space onto the WGS-84 ellipsoid, relative to a reference location that fixes the
GPS coordinates of the simulation origin. The GPS assumes the Chrono system is Z-Up, X-East, Y-North.

## The reference location is (longitude, latitude, altitude)

`gps_reference` is a `ChVector3d` holding **longitude first**:

~~~{.cpp}
ChVector3d gps_reference(-89.400, 43.070, 260.0);  // Madison, WI: longitude, latitude, altitude
~~~

This is the opposite of the ordering used by NMEA, by most GPS libraries, and by
`chrono::synchrono::GPScoord`. It matters more than it looks: a swapped pair is a valid coordinate
somewhere else on Earth, so nothing downstream reports an error — the simulation simply runs at the
wrong place, which shows up as a wrong magnetic field and a track that will not overlay on a map.
The same ordering applies to the `coords` argument of every function in `ChGPSUtils.h`, and to the
`gps_reference` passed to ChMagnetometerSensor.

## Mounting frame

`offsetPose` is resolved against the parent body's *reference* (REF) frame,
`ChBody::GetFrameRefToAbs()`, the same frame render sensors mount on. For a plain `ChBody` this is
the centroidal frame; for a `ChBodyAuxRef`, which every Chrono::Vehicle chassis is, it is not. The
antenna lever arm is included in both position and velocity.

## Geodesy

Positions are converted through the WGS-84 ellipsoid: local ENU to ECEF to geodetic. The radii of
curvature vary with latitude, so a spherical Earth of mean radius introduces a scale error of about
−0.09 % north and +0.27 % east at mid latitudes, which is 0.9 m and 2.7 m per kilometre travelled.
`ChGPSUtils.h` also exposes the individual steps — `Geodetic2ECEF`, `ECEF2Geodetic`, `ENU2ECEFDelta`
and `ECEF2ENUDelta` — for code that needs them directly.

`EARTH_RADIUS` remains defined for callers that want a single representative radius, but it is not
used by the conversions.

## GPS Creation

~~~{.cpp}
// A receiver-class error model; see "Noise models" below.
auto gps_noise_model = chrono_types::make_shared<ChNoiseGPS>(
    ChNoiseGPS::Preset(ChGPSReceiverClass::SPS));
gps_noise_model->SetNominalState(ChNoiseGPS::PresetFixType(ChGPSReceiverClass::SPS), 0.9, 11);

auto gps = chrono_types::make_shared<ChGPSSensor>(
                parent_body,      // body to which the GPS is attached
                gps_update_rate,  // update rate
                gps_offset_pose,  // offset pose from body
                gps_reference,    // (LONGITUDE, LATITUDE, ALTITUDE) of the simulation origin
                gps_noise_model   // noise model to use for adding GPS noise
);

gps->SetName("GPS");
gps->SetLag(gps_lag);                        // latency before the fix reaches the user
gps->SetCollectionWindow(gps_collection_time);

gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
manager->AddSensor(gps);
~~~

A receiver reports one epoch rather than an average, so the sample is the state at the instant the
collection window closed.

## Noise models

Noise is applied in metres in the local ENU frame, before the conversion to latitude and longitude,
which is where a real receiver's position error enters. All models seed from
ChSensorManager::SetRandomSeed when one is set.

### ChNoiseGPS

Parameterised by a receiver's stated accuracy:

\f[ \text{error} = b_{gm} + b_{slow} + N(0, \sigma_{white}) \f]

`b_gm` is a first-order Gauss-Markov process whose steady-state standard deviation is the receiver's
horizontal (and separately vertical) accuracy and whose correlation time is tens of seconds to hours.
That correlation is the point: the ionospheric, tropospheric, ephemeris and multipath terms that
dominate real GNSS error stay correlated for minutes, and modelling them as white noise per epoch
makes a downstream estimator look far better than it would on real data.

| Field | Meaning |
| --- | --- |
| `horizontal_stdev`, `vertical_stdev` | Standard deviation of the correlated error [m]. Vertical is typically 1.5 to 2 times horizontal. |
| `correlation_time` | Correlation time of that error [s]. |
| `white_stdev` | Receiver tracking noise, uncorrelated between epochs [m]. |
| `slow_bias_stdev`, `slow_bias_correlation_time` | A second, much slower term for long runs. Zero disables it. |

`ChNoiseGPS::Preset` returns representative parameters for `ChGPSReceiverClass::SPS`, `SBAS`,
`RTK_FLOAT` or `RTK_FIXED`. `SetOutageModel(rate, mean_duration)` additionally models loss of fix:
the error processes keep evolving during an outage, so reacquiring does not snap the solution back
towards truth.

The correlated states start at their stationary distribution rather than at zero, so a run does not
begin with an unrepresentatively accurate fix.

### ChNoiseNormal and ChNoiseRandomWalks

`ChNoiseNormal` draws an independent sample per epoch. Real GPS error is not white, so filtering it
gives unrealistically optimistic results; it remains for simple studies and for compatibility.

`ChNoiseRandomWalks` integrates clipped white noise twice with a restoring term, giving a smooth
bounded wander. The shape is qualitatively right, but none of its parameters maps to a receiver
specification and the clipping makes the result non-Gaussian. Both are kept so existing studies
reproduce; prefer `ChNoiseGPS` for new work.

## GPS data access

~~~{.cpp}
ChWriterCSV gps_csv(" ");
UserGPSBufferPtr bufferGPS;
unsigned int gps_last_launch = 0;
while () {
    bufferGPS = gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (bufferGPS->Buffer && bufferGPS->LaunchedCount > gps_last_launch) {
        GPSData gps_data = bufferGPS->Buffer[0];
        gps_csv << std::fixed << std::setprecision(10);
        gps_csv << gps_data.Latitude;   // degrees
        gps_csv << gps_data.Longitude;  // degrees
        gps_csv << gps_data.Altitude;   // metres above the WGS-84 ellipsoid
        gps_csv << gps_data.Time;       // seconds
        gps_csv << gps_data.VelEast;    // m/s
        gps_csv << gps_data.VelNorth;
        gps_csv << gps_data.VelUp;
        gps_csv << gps_data.Speed;      // horizontal ground speed [m/s]
        gps_csv << gps_data.Course;     // degrees clockwise from north, in [0, 360)
        gps_csv << (int)gps_data.Fix;   // GPSFix: NONE, SPS, DGPS, RTK_FLOAT, RTK_FIXED
        gps_csv << gps_data.HDOP;
        gps_csv << gps_data.NumSatellites;
        gps_csv << gps_data.Valid;
        gps_csv << std::endl;
        gps_last_launch = bufferGPS->LaunchedCount;
    }
}

gps_csv.WriteToFile(gps_file);
~~~

`Time` and the buffer's `TimeStamp` both carry the simulation time the fix describes, which is the
instant the collection window closed. With a nonzero lag that precedes the time at which the fix
became visible.

`Fix`, `HDOP`, `NumSatellites` and `Valid` come from the noise model when it is a `ChNoiseGPS`, since
that is what knows about outages, and otherwise from `ChGPSSensor::SetNominalFix`.
