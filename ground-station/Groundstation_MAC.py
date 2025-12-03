import sys, serial, struct, threading, time, datetime, os
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from stl import mesh
from scipy.spatial.transform import Rotation as R
from serial.tools import list_ports

# === Global logging control ===
logging_enabled = False
log_file = None
log_lock = threading.Lock()

STATE_MAP = {
    0xBA: "PRELAUNCH",
    0xBB: "LAUNCH",
    0xBC: "DEPLOY",
    0xBD: "GROUND"
}

def create_log_file():
    global log_file
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"log_{timestamp}.txt"
    log_file = open(filename, 'w', encoding='utf-8')
    print(f"[INFO] Logging started: {filename}")
    return filename

def stop_log_file():
    global log_file
    if log_file:
        log_file.flush()
        log_file.close()
        print("[INFO] Logging stopped.")
        log_file = None

# === AccelGraphViewer: 실시간 가속도 그래프 ===
class AccelGraphViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Accelerometer Graph Viewer")
        self.resize(1000, 600)

        self.x_data = []
        self.ax_data = []
        self.ay_data = []
        self.az_data = []

        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)

        self.plot_ax = self.plot_widget.plot(pen=pg.mkPen('r', width=2), name='Ax')
        self.plot_ay = self.plot_widget.plot(pen=pg.mkPen('g', width=2), name='Ay')
        self.plot_az = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name='Az')

        self.plot_widget.addLegend()
        self.plot_widget.setLabel('left', 'Acceleration (m/s²)')
        self.plot_widget.setLabel('bottom', 'Time (s)')
        self.plot_widget.showGrid(x=True, y=True)

    @QtCore.pyqtSlot(float, float, float, float)
    def update_accel(self, timestamp, ax, ay, az):
        self.x_data.append(timestamp)
        self.ax_data.append(ax)
        self.ay_data.append(ay)
        self.az_data.append(az)

        if len(self.x_data) > 10000:
            self.x_data = self.x_data[-10000:]
            self.ax_data = self.ax_data[-10000:]
            self.ay_data = self.ay_data[-10000:]
            self.az_data = self.az_data[-10000:]

        self.plot_ax.setData(self.x_data, self.ax_data)
        self.plot_ay.setData(self.x_data, self.ay_data)
        self.plot_az.setData(self.x_data, self.az_data)

# === RocketViewer: 자세 시각화 ===
class RocketViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rocket Attitude Viewer")
        self.view = gl.GLViewWidget()
        self.setCentralWidget(self.view)
        self.view.setCameraPosition(distance=50, elevation=20, azimuth=30)
        self.view.setBackgroundColor("k")

        self.add_axes(100)
        self.rocket = self.load_rocket("PSLV_I.V_simplemodel.stl")
        self.view.addItem(self.rocket)

        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.latest_state = "UNKNOWN"
        self.frame_counter = 0
        self.current_fps = 0

        self.overlay_label = QtWidgets.QLabel(self)
        self.overlay_label.setStyleSheet("color: white; font-size: 32px; background-color: rgba(0,0,0,100);")
        self.overlay_label.setAlignment(QtCore.Qt.AlignCenter)
        self.overlay_label.show()

        self.fps_timer = QtCore.QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay_label.move((self.width() - self.overlay_label.width()) // 2, self.height() - self.overlay_label.height() - 10)

    def add_axes(self, length=100):
        axes = [
            (np.array([[0, 0, 0], [1, 0, 0]]), (1, 0, 0, 1)),
            (np.array([[0, 0, 0], [0, 1, 0]]), (0, 1, 0, 1)),
            (np.array([[0, 0, 0], [0, 0, 1]]), (0, 0, 1, 1)),
        ]
        for pos, color in axes:
            line = gl.GLLinePlotItem(pos=pos * length, color=color, width=2)
            self.view.addItem(line)

    def load_rocket(self, path):
        stl = mesh.Mesh.from_file(path)
        verts = stl.vectors.reshape(-1, 3)
        faces = np.arange(len(verts)).reshape(-1, 3)
        verts -= verts.mean(axis=0)
        rocket = gl.GLMeshItem(vertexes=verts, faces=faces, smooth=True,
                               shader='shaded', color=(0.8, 0.8, 0.8, 1))
        rocket.resetTransform()
        rocket.rotate(180, 1, 0, 0)
        return rocket

    @QtCore.pyqtSlot(float, float, float)
    def update_attitude(self, roll, pitch, yaw):
        self.latest_roll = roll
        self.latest_pitch = pitch
        self.latest_yaw = yaw
        self.update_overlay()

        self.rocket.resetTransform()
        self.rocket.rotate(180 + roll, 1, 0, 0)
        self.rocket.rotate(-pitch, 0, 1, 0)
        self.rocket.rotate(-yaw, 0, 0, 1)
        self.frame_counter += 1

    @QtCore.pyqtSlot(str)
    def update_state(self, state_str):
        self.latest_state = state_str
        self.update_overlay()

    def update_fps(self):
        self.current_fps = self.frame_counter
        self.frame_counter = 0
        self.update_overlay()

    def update_overlay(self):
        self.overlay_label.setText(
            f"[STATE]: {self.latest_state}    "
            f"R={self.latest_roll:.0f}°, P={self.latest_pitch:.0f}°, Y={self.latest_yaw:.0f}°    FPS={self.current_fps}"
        )
        self.overlay_label.adjustSize()  # ← 텍스트 크기에 맞춰 오버레이 자동 사이즈 조정
        self.overlay_label.move(
            (self.width() - self.overlay_label.width()) // 2,
            self.height() - self.overlay_label.height() - 10
        )

# === RocketPathViewer: 경로 시각화 ===
class RocketPathViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rocket 3D Path Viewer")
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=700, elevation=45, azimuth=45)
        self.view.setBackgroundColor("k")

        layout = QtWidgets.QVBoxLayout(self.central_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.view)

        self.overlay_label = QtWidgets.QLabel(self)
        self.overlay_label.setStyleSheet("color: white; font-size: 32px;")
        self.overlay_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignBottom)
        self.overlay_label.setText("위도: -  경도: -  고도: - 거리: -\n 기준점 미설정")  # 초기 텍스트 지정
        self.overlay_label.adjustSize()  # 자동 크기 조정
        self.overlay_label.move(
            (self.width() - self.overlay_label.width()) // 2,
            self.height() - self.overlay_label.height() - 10
        )
        self.overlay_label.show()


        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.max_alt = None
        self.path = []

        self.add_axes()
        self.add_floor_grid(200, 5)
        self.init_path_line()
        self.init_position_marker("current_pos.stl")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay_label.move((self.width() - self.overlay_label.width()) // 2, self.height() - self.overlay_label.height() - 10)


    def add_axes(self, length=200):
        axes = [
            (np.array([[0, 0, 0], [1, 0, 0]]), (1, 0, 0, 1)),      # +X (동, 빨강)
            (np.array([[0, 0, 0], [-1, 0, 0]]), (1, 0, 1, 1)),     # -X (서, 보라)
            (np.array([[0, 0, 0], [0, 1, 0]]), (0, 1, 0, 1)),      # +Y (북, 초록)
            (np.array([[0, 0, 0], [0, -1, 0]]), (0, 1, 1, 1)),     # -Y (남, 하늘)
            (np.array([[0, 0, 0], [0, 0, 1]]), (0, 0, 1, 1)),      # +Z (상승, 파랑)

            # 서(-X) → 북(+Y) 방향 흰색 선 추가
            (np.array([[-1, 0, 0], [0, 1, 0]]), (1, 1, 1, 1)),     # 흰색 대각선
        ]
        for pos, color in axes:
            self.view.addItem(gl.GLLinePlotItem(pos=pos * length, color=color, width=2))



    def add_floor_grid(self, size, spacing):
        for x in range(-size, size + 1, spacing):
            pts = np.array([[x, -size, 0], [x, size, 0]], dtype=np.float32)
            self.view.addItem(gl.GLLinePlotItem(pos=pts, color=(1, 1, 1, 0.2), width=1))
        for y in range(-size, size + 1, spacing):
            pts = np.array([[-size, y, 0], [size, y, 0]], dtype=np.float32)
            self.view.addItem(gl.GLLinePlotItem(pos=pts, color=(1, 1, 1, 0.2), width=1))

    def init_path_line(self):
        self.path_line = gl.GLLinePlotItem(pos=np.empty((0, 3), dtype=np.float32),
                                           color=(1, 1, 0, 1), width=2)
        self.view.addItem(self.path_line)

    def init_position_marker(self, stl_path):
        stl = mesh.Mesh.from_file(stl_path)
        verts = stl.vectors.reshape(-1, 3)
        faces = np.arange(len(verts)).reshape(-1, 3)
        verts -= verts.mean(axis=0)
        verts *= 0.1
        self.marker = gl.GLMeshItem(vertexes=verts, faces=faces, smooth=True,
                                    shader=None, color=(1, 1, 0, 1))
        self.view.addItem(self.marker)

    def latlon_to_m(self, lat0, lon0, lat, lon):
        R = 6371000
        dlat = np.radians(lat - lat0)
        dlon = np.radians(lon - lon0)
        dx = R * dlon * np.cos(np.radians(lat0))
        dy = R * dlat
        return dx, dy

    @QtCore.pyqtSlot(float, float, float)
    def update_position(self, lat, lon, alt):
        # === 1. 기준점이 아직 설정되지 않았을 때 ===
        if self.origin_lat is None or self.origin_lon is None:
            if lat == 0.0 and lon == 0.0:
                # 위경도는 없지만 고도는 표시
                self.overlay_label.setText(
                    f"위도: -  경도: -  고도: {alt:.2f}m  거리: {abs(alt):.2f}m\n 기준점 미설정"
                )
                self.overlay_label.adjustSize()
                self.overlay_label.move(
                    (self.width() - self.overlay_label.width()) // 2,
                    self.height() - self.overlay_label.height() - 10
                )
                return

            # 처음으로 유효한 위경도 수신 → 기준점 설정
            self.origin_lat, self.origin_lon, self.origin_alt = lat, lon, alt

        # === 2. 기준점 설정 이후에 (0.0, 0.0)이 오면 무시 ===
        if lat == 0.0 and lon == 0.0:
            return

        # === 3. 정상 업데이트 ===
        dx, dy = self.latlon_to_m(self.origin_lat, self.origin_lon, lat, lon)
        dz = alt - self.origin_alt
        pos = np.array([dx, dy, dz], dtype=np.float32)
        self.path.append(pos)

        if len(self.path) >= 2:
            self.path_line.setData(pos=np.array(self.path, dtype=np.float32))

        self.marker.resetTransform()
        self.marker.translate(*pos)

        if self.max_alt is None or alt > self.max_alt: #최대고도 업데이트/ 기준점 설정 이후 뜸
            self.max_alt = alt
        x_dir = "동" if dx >= 0 else "서"
        y_dir = "북" if dy >= 0 else "남"
        dir_text = f"{x_dir} {abs(dx):.1f}m, {y_dir} {abs(dy):.1f}m"
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        self.overlay_label.setText(
            f"위도: {lat:.6f}  경도: {lon:.6f}  고도: {alt:.2f}m (최대: {self.max_alt:.2f}m)  거리: {distance:.1f}m\n"
            f"{dir_text}"
        )
        self.overlay_label.adjustSize()
        self.overlay_label.move(
            (self.width() - self.overlay_label.width()) // 2,
            self.height() - self.overlay_label.height() - 10
        )


# === SerialReader ===
class SerialReader(threading.Thread):
    def __init__(self, port, baud, attitude_gui, path_gui, accel_gui):
        super().__init__()
        self.ser = serial.Serial(port, baud, timeout=1)
        self.att_gui = attitude_gui
        self.path_gui = path_gui
        self.accel_gui = accel_gui

        # 변경점 1) 새 구조체 크기: <B + 17 floats> = 1 + 17*4 = 69B
        # 끝 바이트(0x0A) 1B 포함해서 총 70B를 읽음 (시작바이트 0x7E는 위에서 이미 1B 따로 읽음)
        self.packet_size = struct.calcsize('<B' + 'f'*21) + 1  # = 70
        self.daemon = True

    def run(self):
        global logging_enabled, log_file
        while True:
            try:
                start = self.ser.read(1)

                if start == b'\x7E':
                    packet = self.ser.read(self.packet_size)
                    if len(packet) == self.packet_size and packet[-1] == 0x0A:

                        # 변경점 2) 언팩 포맷: <B + 17 floats>
                        unpacked = struct.unpack('<B' + 'f'*21, packet[:-1])

                        # 변경점 3) 필드 매핑 (gyro/pos는 언팩만 하고 GUI에선 미사용)
                        state_code = unpacked[0]
                        time_ms    = unpacked[1]
                        lat        = unpacked[2]
                        lon        = unpacked[3]
                        roll       = unpacked[4]
                        pitch      = unpacked[5]
                        yaw        = unpacked[6]
                        ax         = unpacked[7]
                        ay         = unpacked[8]
                        az         = unpacked[9]
                        gx         = unpacked[10]
                        gy         = unpacked[11]
                        gz         = unpacked[12]
                        px         = unpacked[13]
                        py         = unpacked[14]
                        pz         = unpacked[15]
                        alt        = unpacked[16]
                        pressure   = unpacked[17]
                        kalman_alt = unpacked[18] 
                        kalman_velo= unpacked[19]
                        predi_apo  = unpacked[20]
                        r2         = unpacked[21]

                        state_str = STATE_MAP.get(state_code, f"UNKNOWN({state_code:#X})")
                        time_val  = time_ms / 1000.0  # ms -> s

                        # (로그는 참고용으로만 gyro/pos 추가 표기; GUI는 기존 그대로)
                        line = (
                            f"State: {state_str}, Time: {time_val:.2f}s, "
                            f"Lat: {lat:.6f}, Lon: {lon:.6f}, "
                            f"Alt: {alt:.2f}m, "
                            f"P: {pressure:.2f}hPa, "
                            f"Euler: ({roll:.2f}, {pitch:.2f}, {yaw:.2f}) | "
                            f"Gyro: ({gx:.3f}, {gy:.3f}, {gz:.3f}) | "
                            f"Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) | "
                            f"Pos: ({px:.2f}, {py:.2f}, {pz:.2f}) | "
                            f"kalman: (alt :{kalman_alt:.3f}, velo : {kalman_velo:.3f}, apo : {predi_apo:.3f}, r2 : {r2:.3f} )"
                        )
                        print(line)

                        with log_lock:
                            if logging_enabled and log_file:
                                log_file.write(line + '\n')
                                log_file.flush()

                        # === 기존 GUI 업데이트는 그대로 ===
                        QtCore.QMetaObject.invokeMethod(
                            self.att_gui, "update_attitude", QtCore.Qt.QueuedConnection,
                            QtCore.Q_ARG(float, roll), QtCore.Q_ARG(float, pitch), QtCore.Q_ARG(float, yaw))

                        QtCore.QMetaObject.invokeMethod(
                            self.att_gui, "update_state", QtCore.Qt.QueuedConnection,
                            QtCore.Q_ARG(str, state_str))

                        QtCore.QMetaObject.invokeMethod(
                            self.path_gui, "update_position", QtCore.Qt.QueuedConnection,
                            QtCore.Q_ARG(float, lat), QtCore.Q_ARG(float, lon), QtCore.Q_ARG(float, alt))

                        QtCore.QMetaObject.invokeMethod(
                            self.accel_gui, "update_accel", QtCore.Qt.QueuedConnection,
                            QtCore.Q_ARG(float, time_val), QtCore.Q_ARG(float, ax),
                            QtCore.Q_ARG(float, ay), QtCore.Q_ARG(float, az))

                elif start == b'\xAA':
                    line = self.ser.readline().decode(errors='ignore').strip()
                    print(f"{line}")
                    with log_lock:
                        if logging_enabled and log_file:
                            log_file.write(f"{line}\n")
                            log_file.flush()

            except Exception as e:
                print(f"[Serial Error] {e}")


# === Keyboard Listener Thread ===
class KeyboardListener(threading.Thread):
    def run(self):
        global logging_enabled
        while True:
            cmd = input().strip().upper()
            if cmd == 'S':
                with log_lock:
                    if not logging_enabled:
                        create_log_file()
                        logging_enabled = True
            elif cmd == 'E':
                with log_lock:
                    if logging_enabled:
                        logging_enabled = False
                        stop_log_file()

def detect_macos_serial_port():
    """Detect and return the first available macOS serial port."""
    available_ports = list_ports.comports()
    exclude_patterns = ['Bluetooth', 'SPPDev', 'wlan-debug', 'modem', 'tty.Bluetooth']
    
    for port in available_ports:
        device = port.device
        if any(pattern in device for pattern in ['/dev/tty.usbserial', '/dev/tty.usbmodem', '/dev/cu.usbserial', '/dev/cu.usbmodem']):
            if not any(exclude in device for exclude in exclude_patterns):
                return device
    return "COM3" # Fallback for non-Mac or if not found

# === Main ===
if __name__ == "__main__":
    # Essential for macOS to share context
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts)
    
    app = QtWidgets.QApplication(sys.argv)
    viewer1 = RocketViewer()
    viewer2 = RocketPathViewer()
    viewer3 = AccelGraphViewer()
    viewer1.resize(1440, 700)
    viewer2.resize(1440, 1200)
    viewer3.resize(1440, 450)
    viewer1.move(0, 0)
    viewer2.move(1440, 0)
    viewer3.move(0, 750)
    viewer1.show()
    viewer2.show()
    viewer3.show()

    # Detect Mac Port
    target_port = detect_macos_serial_port()
    print(f"[INFO] Attempting to open: {target_port}")
    
    serial_thread = SerialReader(port=target_port, baud=115200, attitude_gui=viewer1, path_gui=viewer2, accel_gui=viewer3)
    serial_thread.start()

    keyboard_thread = KeyboardListener()
    keyboard_thread.daemon = True
    keyboard_thread.start()

    sys.exit(app.exec_())
