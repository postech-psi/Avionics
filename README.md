# PSI Avionics

Flight software and ground station for the POSTECH PSI rocket avionics system.
The flight computer runs on an Arduino Portenta H7. Its M7 core reads the
sensors and controls parachute deployment; the M4 core records flight data to
SD and sends telemetry over XBee.

The ground station brings live telemetry into one view: 3D attitude and flight
path, sensor plots, flight statistics, and sensor status. It records each
connection and replays saved logs for flight review, with offline satellite
maps for use at the launch site.

## Repository layout

```text
flight-computer/
  src/m7/       Flight control and state estimation
  src/m4/       SD logging and XBee telemetry
  src/shared/   Telemetry and inter-core packet definitions
  lib/          Firmware libraries, including the UKF and modified BMP3XX driver
  docs/         IMU configuration
  legacy/       Original single-core firmware
ground-station/
  Groundstation.py         Ground station with map and log replay
  README.md               Ground station setup and operating guide
  fetch_basemap.py         Offline basemap downloader
  logs/                   Recorded flight data
```

## Build the firmware

Install PlatformIO through VS Code or Python, then build both cores from the
repository root:

```bash
python -m pip install platformio
pio run -d flight-computer
```

The project uses ST STM32 19.5.0 and Arduino Mbed 4.5.0. The sensor libraries
are included in the repository. Keep the local BMP3XX driver: the firmware
uses its normal-mode and nonblocking-read functions.

Read the [firmware guide](flight-computer/README.md) before uploading. It covers
wiring, sensor settings, startup calibration, and deployment behavior. Upload
M4 first, then M7:

```bash
pio run -d flight-computer -e portenta_h7_m4 -t upload
pio run -d flight-computer -e portenta_h7_m7 -t upload
```

## Run the ground station

From the repository root:

```bash
cd ground-station
python -m pip install -r requirements.txt
python Groundstation.py
```

Select the XBee serial port and click **CONNECT**. The app uses **115200 baud**
and starts recording automatically. Check the **REC** indicator; recordings
are saved as a telemetry CSV and an event TXT file under `ground-station/logs/`.
**STOP LOG** ends recording, and **START LOG** opens another session.

Use **PLOT CONTROL** to select graphs and **FOLLOW** to keep the flight path
in view. **OPEN LOG** replays a ground station CSV or a flight-computer SD CSV
after disconnecting the serial port. Replay supports pause, speed selection,
and seeking through the flight.

The [ground station guide](ground-station/README.md) explains the displays,
sensor status, recording formats, and replay controls.

The map opens as a grid when no basemap is installed. Download imagery before
going to the launch site:

```bash
python fetch_basemap.py site1  # 고흥만 항공센터
python fetch_basemap.py site2  # POSTECH 풍동동
```

For another location:

```bash
python fetch_basemap.py mysite --lat 34.61 --lon 127.21 --label "예비 발사장"
```

Maps are saved in `basemaps/` and work offline after download. Restart the app
after downloading and select the site from **MAP**. Downloaded maps and the
tile cache are excluded from Git.

## Legacy firmware

The original single-core project is kept in
[`flight-computer/legacy/`](flight-computer/legacy/README.md), with its original
source, libraries, and build configuration. It builds separately from the
current firmware and uses an older telemetry format.
