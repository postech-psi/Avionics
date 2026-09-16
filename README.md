# Avionics Flight Software

Dual-core rocket flight software for the Arduino Portenta H7. M7 runs sensor acquisition, IMU/barometer UKF estimation, flight-state decisions, and the parachute servo. M4 handles SD logging and XBee telemetry. GNSS coordinates and altitude are used for logging and recovery, not flight decisions.

The flight computer is based on **PSI AVINICS 2026 (2), August 8, 2026**, migrated on September 17 with fixes for persistent velocity-path failure and SD-log health replay. See [migration and legacy recovery](docs/MIGRATION-2026.md) and the [firmware hardware/setup guide](flight-computer/README.md). The existing ground station, assets, and flight logs are preserved.

## Project Structure

```
Avionics/
├── flight-computer/     # PlatformIO firmware project
│   ├── src/m7/        # Flight control (100Hz loop, 50Hz UKF)
│   ├── src/m4/        # SD logging (100Hz) and telemetry (25Hz)
│   ├── lib/            # Custom libraries (ukf_ert_rtw)
│   └── platformio.ini  # PlatformIO configuration
└── ground-station/     # Python ground station
    ├── Groundstation.py         # main GUI with live map ground-track tracking
    ├── Groundstation_nomap.py   # fallback GUI (no map) for when basemaps are unavailable
    ├── fetch_basemap.py         # CLI tool to download satellite basemaps into basemaps/
    ├── logs/                    # flight-log archive (committed); new logs also saved here
    └── requirements.txt
```

## Installation

### Flight Computer (PlatformIO)

1. **Install PlatformIO**: 
   - VS Code: Install the "PlatformIO IDE" extension
   - CLI: `pip install platformio`

2. **Open the Firmware Project**:
   ```bash
   cd flight-computer
   ```
   Libraries are vendored in `lib/`. Keep the custom BMP3XX driver: the
   firmware requires its normal-mode and nonblocking-read additions.
   PlatformIO downloads the pinned ST STM32 19.5.0 / Arduino Mbed 4.5.0 runtime.

3. **Build and Upload**:
   ```bash
   pio run                             # Build both M7 and M4
   pio run -e portenta_h7_m4 -t upload   # Upload M4 FIRST
   pio run -e portenta_h7_m7 -t upload   # Upload M7 SECOND
   ```
   Confirm sensor configuration and servo geometry in the firmware guide
   before uploading. Builds and host tests do not validate hardware behavior.

### Ground Station (Python)

1. **Install Python Dependencies**:
   ```bash
   cd ground-station
   pip install -r requirements.txt
   ```

2. **Run Ground Station**:
   ```bash
   python Groundstation.py          # with live map ground-track view
   ```
   The map view falls back to a plain GRID when no basemaps are present, so it
   runs out of the box. To enable satellite basemaps, download them **once** from
   an internet-connected machine (imagery: VWorld / 국토교통부):
   ```bash
   python fetch_basemap.py site1    # 고흥만 항공센터
   python fetch_basemap.py site2    # POSTECH 풍동동
   # custom site: python fetch_basemap.py mysite --lat 34.61 --lon 127.21 --label "예비 발사장"
   ```
   This writes `basemaps/<site>.png` + `.json`, which the ground station reads
   **offline** at runtime (no network needed on the field). Raw tiles are cached
   in `tilecache/` to speed up any re-fetch. Both `basemaps/` and `tilecache/`
   are git-ignored (large, regenerable) — run the fetch above to recreate them.

   If you can't download maps, use the no-map fallback instead:
   ```bash
   python Groundstation_nomap.py
   ```
   Recorded flight logs are written to `ground-station/logs/`.

## Dependencies

### Flight Computer
- **Vendored libraries** (versioned in `flight-computer/lib/`):
  - Adafruit BMP3XX Library
  - Adafruit Unified Sensor
  - Adafruit BusIO
  - Servo

- **Custom libraries** (in `lib/` folder):
  - `ukf_ert_rtw/` - Unscented Kalman Filter implementation

### Ground Station
- numpy
- PyQt5
- pyqtgraph
- numpy-stl
- scipy
- pyserial

## Regression Checks

From the repository root:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
python tests/run_flight_tests.py
pio run -d flight-computer
```

The flight scenarios require a C++17 host compiler (`g++`, `clang++`, or
`cl`). On Windows, run them from an **x64 Native Tools Command Prompt for
VS 2022** so MSVC's include/library paths are configured. Tests fake time
and hardware while compiling the production decision and cadence logic.
