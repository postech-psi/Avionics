# Avionics Flight Software

This project delivers a high-performance, RTOS-based avionics flight computer for the Arduino Portenta H7, designed for autonomous rocket recovery and telemetry. It integrates multi-sensor fusion—combining GNSS, IMU, and BMP3XX barometric data with an Unscented Kalman Filter (UKF)—to provide precise real-time state estimation and robust flight stage detection (Calibration, Prelaunch, Launch, Deploy, Landed). Featuring concurrent threads for decision-making and XBee telemetry transmission, the system ensures reliable parachute deployment based on acceleration, velocity, and orientation thresholds, meeting rigorous standards for stability and mission success.

## Project Structure

```
Avionics/
├── flight-computer/     # PlatformIO firmware project
│   ├── src/            # Source code
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

2. **Install Dependencies**:
   ```bash
   cd flight-computer
   pio lib install  # Automatically installs libraries from platformio.ini
   ```

3. **Build and Upload**:
   ```bash
   pio run -e portenta_h7_m7        # Build
   pio run -e portenta_h7_m7 -t upload  # Upload to board
   ```

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
- **PlatformIO libraries** (auto-installed via `lib_deps` in `platformio.ini`):
  - Adafruit BMP3XX Library
  - Adafruit Unified Sensor
  - Adafruit BusIO
  - Servo (Arduino framework built-in)

- **Custom libraries** (in `lib/` folder):
  - `ukf_ert_rtw/` - Unscented Kalman Filter implementation

### Ground Station
- numpy
- PyQt5
- pyqtgraph
- numpy-stl
- scipy
- pyserial
