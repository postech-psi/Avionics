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
    ├── Groundstation.py
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
   python Groundstation.py
   ```

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

## Development

this is develop code

20240409 박재영
20250142 이태호

2025.12.03