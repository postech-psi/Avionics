# PSI Ground Station

The ground station receives XBee telemetry from the Portenta H7 flight computer.
It shows the rocket's attitude and flight path alongside sensor plots, flight
statistics, and system messages. The same displays can replay recorded flights
without a serial connection.

## Start the app

Run these commands from the repository root:

```bash
cd ground-station
python -m pip install -r requirements.txt
python Groundstation.py
```

Keep the two STL models and `PSI Logo.png` beside the Python script. The 3D
views load these local assets and use OpenGL.

## Connect and record

1. Connect the receiving XBee to the computer over USB.
2. Click **REFRESH**, select its serial port, and click **CONNECT**. The serial
   speed is fixed at 115200 baud.
3. Check the connection status, incoming telemetry, and **REC** indicator.
   Recording starts automatically when the connection opens.
4. Click **DISCONNECT** when finished. This also closes the recording files.

**STOP LOG** closes the current recording while leaving the serial connection
open. **START LOG** opens a new recording. Each session creates two files in
`ground-station/logs/`:

| File | Contents |
|---|---|
| `log_YYYYMMDD_HHMMSS.csv` | Received telemetry, including timestamps and health flags |
| `log_YYYYMMDD_HHMMSS.txt` | Text events and system messages |

The ground station records what arrives over the radio. The flight computer's
SD recording is a separate source and can contain samples missed by the radio
link. Keep both when reviewing a flight.

## Read the display

| Panel | What it shows |
|---|---|
| **3D ATTITUDE** | Rocket orientation from the IMU |
| **3D FLIGHT PATH** | Position history over a grid or a saved satellite map |
| **PLOT CONTROL · REPLAY** | Graph selection, map controls, and log playback |
| **FLIGHT STATS** | Maximum altitude, ascent and descent rates, acceleration, and deployment and ground times |
| **SENSOR HEALTH** | Sensor response, data freshness, reference pressure, and velocity-path status |
| **TELEMETRY / SYSTEM LOG** | Received values and firmware messages |

The available graphs cover acceleration, angular rate, altitude, vertical
velocity, and predicted apogee. Use the checkboxes to choose the graphs and
individual curves. Double-click a graph after manually panning or zooming to
return to its automatic view.

**FOLLOW** fits the camera to the flight path. Moving the camera manually turns
it off; select it again to resume following. **TOP** switches to a north-up
overhead view. The toolbar **RESET** clears the displayed history and statistics
while the connection and recording continue.

Deployment and ground times in **FLIGHT STATS** are measured from the received
LAUNCH transition. If that transition was missed, the panel shows
`no launch ref`. Predicted apogee is the current onboard estimate; the maximum
altitude statistic is the highest received Kalman altitude.

## Sensor status

IMU and barometer status distinguish **FRESH**, **STALE**, and **DEAD**. A sensor
may still respond while its latest measurement is too old for flight control.
GNSS **ALIVE** means the firmware received an accepted position frame; it does
not guarantee a valid navigation fix.

The velocity path shows **USABLE**, **SETTLING**, or **DISABLED**. These describe
the firmware's velocity input, not whether deployment is armed. The
[firmware guide](../flight-computer/README.md#4-텔레메트리) defines the health
bits and timing thresholds.

## Replay a flight

Disconnect the serial port, then click **OPEN LOG** and choose a telemetry file.
Playback starts immediately. Use the pause/play button, select a speed from
1× to 50×, or drag the slider to inspect a particular point in the flight.
Seeking rebuilds the plotted history and flight statistics up to that point.

The reader accepts ground station CSV recordings, flight-computer SD CSV files
such as `flight_000.csv`, and older TXT logs containing telemetry rows. The
current event-only TXT files do not contain frames for replay; open the paired
CSV instead.

For current SD CSV files, sensor status comes from the `note` column. Older
files without encoded health information use a limited estimate based on the
stored values, so their status display is less complete.

## Prepare offline maps

Download maps while internet access is available. From `ground-station/`:

```bash
python fetch_basemap.py site1  # 고흥만 항공센터
python fetch_basemap.py site2  # POSTECH 풍동동
```

For another location, supply a name and coordinates:

```bash
python fetch_basemap.py mysite --lat 34.61 --lon 127.21 --label "예비 발사장"
```

The downloader creates a PNG and matching JSON metadata in `basemaps/`. Keep
each pair together when copying maps to another computer. Restart the ground
station after downloading, then choose the site from **MAP**. The app reads
these files locally during a flight. Without a downloaded map, it shows a grid.

Downloaded maps and `tilecache/` are excluded from Git. Run
`python fetch_basemap.py --help` for coverage and zoom options.

## Common problems

| Symptom | What to check |
|---|---|
| Serial port is missing | USB connection and adapter driver; click **REFRESH** after reconnecting |
| Port cannot be opened | Close other serial monitors using the same port |
| Connected, but no telemetry | XBee pairing, receiver serial settings, and whether both flight-computer cores are running |
| 3D view fails to open | Graphics driver, OpenGL support, and the STL files beside the script |
| Map list is empty | Download a basemap and restart the app |
| Replay finds no telemetry | Open the CSV rather than an event-only TXT file |

The live receiver expects the current dual-core firmware's packet format.
See the [legacy firmware guide](../flight-computer/legacy/README.md) when
working with the archived single-core project.
