import sys, serial, struct, threading, time, datetime, os
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from stl import mesh
from serial.tools import list_ports

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

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

# 비행 단계 칩 색상 (배경, 글자) - 배경이 밝으면 글자를 검게
PHASE_STYLES = {
    "PRELAUNCH": ("#8a8a8a", "#ffffff"),
    "LAUNCH":    ("#f5c451", "#000000"),
    "DEPLOY":    ("#16a34a", "#ffffff"),
    "GROUND":    ("#5b8cff", "#ffffff"),
}
PHASE_STYLE_UNKNOWN = ("#dc2626", "#ffffff")

# === 테마 ===
# 상태색(초록/주황/빨강)과 곡선색은 의미색이라 두 테마 공통으로 둔다.
THEMES = {
    "dark": {
        "bg": "#000000", "panel": "#0a0a0a",
        "border": "#2a2a2a", "border_soft": "#1f1f1f",
        "fg": "#e6e6e6", "muted": "#8a8a8a", "accent": "#f5c451",
        "btn": "#111111", "btn_hover": "#1c1c1c", "handle": "#1a1a1a",
        "gl_bg": "k", "grid": (1, 1, 1, 0.2), "plot_fg": "#b0b0b0",
        # GL 선은 기본이 가산 혼합이라 흰 배경에서는 전부 흰색으로 묻힌다
        "gl_line_mode": "additive",
    },
    "light": {
        "bg": "#eeeeee", "panel": "#ffffff",
        "border": "#c4c4c4", "border_soft": "#dcdcdc",
        "fg": "#1a1a1a", "muted": "#6a6a6a", "accent": "#8a5a00",
        "btn": "#ffffff", "btn_hover": "#e4e4e4", "handle": "#d0d0d0",
        "gl_bg": "w", "grid": (0, 0, 0, 0.25), "plot_fg": "#4a4a4a",
        "gl_line_mode": "translucent",
    },
}

# 라이브 그래프가 보여주는 시간 창 (초) - X는 흐르고 Y는 이 구간에만 맞춰진다
GRAPH_WINDOW_S = 20.0

# === 텔레메트리 로그 컬럼 정의 ===
# 헤더 2줄과 데이터 행이 이 정의 하나를 공유하므로 폭이 어긋날 수 없다.
# 3단 구조: (센서 소스, [(그룹명, [(컬럼명, dict키, 포맷, 폭), ...]), ...])
#   - 소스 경계는 ║, 소스 안쪽 그룹 경계는 │ 로 그어 ALT 세 개(GNSS/BARO/KALMAN)를 구분한다
#   - 체크박스 토글은 그룹 단위, 한 소스의 그룹이 전부 꺼지면 그 소스는 통째로 빠진다
SOURCE_SEP = "║"
GROUP_SEP = "│"

TELEMETRY_COLUMNS = [
    ("FLIGHT", [
        ("FLIGHT", [
            ("STATE", "state", "{:>9}", 9),
            ("T(s)", "time", "{:7.2f}", 7),
        ]),
    ]),
    ("GNSS (°, m)", [
        ("GNSS", [
            ("LAT", "lat", "{:9.5f}", 9),
            ("LON", "lon", "{:10.5f}", 10),
            ("ALT", "gnss_alt", "{:6.1f}", 6),
        ]),
    ]),
    ("IMU (°, g, dps, m)", [
        ("ATTITUDE", [
            ("ROLL", "roll", "{:6.1f}", 6),
            ("PITCH", "pitch", "{:6.1f}", 6),
            ("YAW", "yaw", "{:6.1f}", 6),
        ]),
        ("ACCEL", [
            ("AX", "ax", "{:6.2f}", 6),
            ("AY", "ay", "{:6.2f}", 6),
            ("AZ", "az", "{:6.2f}", 6),
        ]),
        ("GYRO", [
            ("GX", "gx", "{:6.1f}", 6),
            ("GY", "gy", "{:6.1f}", 6),
            ("GZ", "gz", "{:6.1f}", 6),
        ]),
        ("POS", [
            ("PX", "px", "{:6.1f}", 6),
            ("PY", "py", "{:6.1f}", 6),
            ("PZ", "pz", "{:6.1f}", 6),
        ]),
    ]),
    ("BARO (m, hPa)", [
        ("BARO", [
            ("ALT", "alt", "{:6.1f}", 6),
            ("PRESS", "pressure", "{:7.1f}", 7),
        ]),
    ]),
    ("KALMAN (m, m/s)", [
        ("KALMAN", [
            ("ALT", "kf_alt", "{:6.1f}", 6),
            # kalman_velo는 수직속도 - health의 vz_trusted/vz_usable 비트와 같은 값
            ("VZ", "kf_vel", "{:6.1f}", 6),
            ("APOGEE", "apogee", "{:6.1f}", 6),
            ("R²", "r2", "{:6.3f}", 6),
        ]),
    ]),
]

# 체크박스 라벨용 - 정의에서 그대로 뽑아 쓰므로 목록이 어긋나지 않는다
TELEMETRY_GROUPS = [g for _, groups in TELEMETRY_COLUMNS for g, _ in groups]

# "HH:MM:SS " - 프레임 구분은 T(s) 컬럼이 하므로 벽시계는 초 단위로 충분하다
LOG_TS_WIDTH = 9


def active_sources(hidden=frozenset()):
    """숨긴 그룹을 걸러낸 (소스명, [(그룹명, 필드들), ...]) 목록."""
    result = []
    for source, groups in TELEMETRY_COLUMNS:
        visible = [(g, f) for g, f in groups if g not in hidden]
        if visible:
            result.append((source, visible))
    return result


def _group_width(fields):
    """필드 사이 공백 1칸씩 포함한 그룹 한 덩어리의 폭."""
    return sum(w for _, _, _, w in fields) + len(fields) - 1


def build_log_header(hidden=frozenset()):
    """컬럼 헤더 2줄 (센서 소스 / 컬럼명)을 만든다."""
    source_row = " " * LOG_TS_WIDTH
    name_row = " " * LOG_TS_WIDTH
    for i, (source, groups) in enumerate(active_sources(hidden)):
        if i:
            source_row += SOURCE_SEP
            name_row += SOURCE_SEP
        span = sum(_group_width(f) for _, f in groups) + len(groups) - 1
        # 소스명이 span보다 길면 잘라낸다 - center()는 넘치면 그대로 반환해 열이 밀린다
        source_row += source[:span].center(span)
        # 컬럼명은 칸 가운데로 - 데이터는 폭을 꽉 채워 우측정렬되므로
        # 이름을 가운데 두면 숫자 덩어리 한가운데에 얹힌다
        name_row += GROUP_SEP.join(
            " ".join(f"{n:^{w}}" for n, _, _, w in fields) for _, fields in groups
        )
    return f"{source_row}\n{name_row}"


def format_log_row(d, hidden=frozenset()):
    """텔레메트리 dict를 헤더와 같은 폭의 값 행으로 만든다."""
    return SOURCE_SEP.join(
        GROUP_SEP.join(
            " ".join(fmt.format(d[key]) for _, key, fmt, _ in fields)
            for _, fields in groups
        )
        for _, groups in active_sources(hidden)
    )


def log_line_width(hidden=frozenset()):
    """한 줄 전체 길이 - 로그 폰트 자동 맞춤의 기준."""
    return len(build_log_header(hidden).split("\n")[0])


def make_log_font(pixel_size):
    """로그용 고정폭 폰트.

    ! 힌팅을 끄는 것이 핵심 - 켜져 있으면 글자 폭이 정수로 반올림되어
      한 글자당 최대 1px씩 낭비되고(180자면 180px), 그만큼 폰트가 작아진다.
    """
    font = QtGui.QFont("Consolas")
    font.setStyleHint(QtGui.QFont.Monospace)
    font.setPixelSize(pixel_size)
    font.setWeight(QtGui.QFont.Bold)
    font.setHintingPreference(QtGui.QFont.PreferNoHinting)
    return font


APP_TITLE = "PSI Ground Station"
APP_TITLE_PX = 25
LOGO_HEIGHT = 40
LOGO_EXTENSIONS = (".png", ".jpg", ".jpeg", ".bmp", ".webp", ".ico", ".svg")


def find_logo(stem):
    """BASE_DIR에서 로고 파일을 찾는다. 없으면 None (로고 없이 제목만).

    대소문자와 공백/밑줄/하이픈을 무시하므로 "PSI Logo.png", "psi_logo.png",
    "psi-logo.PNG"가 모두 stem="psilogo"에 걸린다.
    """
    def normalize(text):
        return text.lower().replace(" ", "").replace("_", "").replace("-", "")

    for name in sorted(os.listdir(BASE_DIR)):
        base, ext = os.path.splitext(name)
        if normalize(base) == stem and ext.lower() in LOGO_EXTENSIONS:
            return os.path.join(BASE_DIR, name)
    return None


def load_logo(theme_name):
    """제목 높이에 맞춘 로고 픽스맵. 없으면 None.

    psi_logo_light.* 가 있으면 라이트 테마에서 그걸 쓰고, 없으면 psi_logo.* 를
    쓰되 알파가 없는(=배경이 불투명한) 흑백 로고는 반전시킨다. 검은 배경 로고를
    흰 화면에 그대로 얹으면 검은 사각형이 되기 때문.
    """
    path = None
    if theme_name == "light":
        path = find_logo("psilogolight")
    invert = False
    if path is None:
        path = find_logo("psilogo")
        invert = theme_name == "light"
    if path is None:
        return None

    image = QtGui.QImage(path)
    if image.isNull():
        return None
    if invert and not image.hasAlphaChannel():
        image.invertPixels()

    return QtGui.QPixmap.fromImage(image).scaledToHeight(
        LOGO_HEIGHT, QtCore.Qt.SmoothTransformation
    )


def build_stylesheet(theme):
    """테마 토큰으로 앱 전역 스타일시트를 만든다.

    ! 여기에 font-family/font-size를 두지 말 것 - 스타일시트 폰트는
      setFont()를 무시해 로그 폰트 자동 맞춤이 동작하지 않는다.
      기본 폰트는 ensure_pretendard()가 QApplication에 지정한다
    """
    return f"""
        QMainWindow, QWidget {{
            background-color: {theme["bg"]};
            color: {theme["fg"]};
        }}
        QLabel#appTitle {{
            color: {theme["accent"]};
            font-size: {APP_TITLE_PX}px;
            font-weight: 700;
            letter-spacing: 1px;
            padding: 5px;
        }}
        /* 미션 바 위에 얹히므로 배경을 깔면 안 된다 (바 색과 미묘하게 어긋나 상자로 보인다) */
        QLabel#missionClock {{
            background: transparent;
            border: none;
            color: {theme["fg"]};
            font-weight: 700;
            padding: 4px 8px;
        }}
        QLabel#missionMuted {{
            background: transparent;
            border: none;
            color: {theme["muted"]};
            font-weight: 700;
            padding: 4px 8px;
        }}
        QLabel#panelTitle {{
            background-color: {theme["panel"]};
            color: {theme["accent"]};
            font-size: 19px;
            font-weight: 700;
            letter-spacing: 2px;
            padding: 3px;
            border-bottom: 1px solid {theme["border_soft"]};
        }}
        QWidget#panelHeader {{
            background-color: {theme["panel"]};
            border-bottom: 1px solid {theme["border_soft"]};
        }}
        QLabel#panelTitleInHeader {{
            background: transparent;
            border: none;
            color: {theme["accent"]};
            font-size: 19px;
            font-weight: 700;
            letter-spacing: 2px;
        }}
        QLabel#axisUnitLabel {{
            background: transparent;
            border: none;
            color: {theme["muted"]};
            font-size: 17px;
        }}
        QLabel#statsKey {{
            color: {theme["muted"]};
            font-size: 18px;
        }}
        QLabel#statsValue {{
            color: {theme["fg"]};
            font-size: 21px;
            font-weight: 700;
        }}
        QLabel#statsKeyHero {{
            color: {theme["accent"]};
            font-size: 19px;
            font-weight: 700;
            letter-spacing: 1px;
        }}
        QLabel#statsValueHero {{
            color: {theme["accent"]};
            font-size: 30px;
            font-weight: 700;
        }}
        QCheckBox {{
            color: {theme["muted"]};
            font-size: 12px;
            font-weight: 700;
            background: transparent;
        }}
        QCheckBox::indicator {{ width: 12px; height: 12px; }}
        /* 로그바는 30px 헤더라 12px이 맞지만 패널 안에서는 너무 작다 */
        QCheckBox#plotToggle {{ font-size: 15px; }}
        QCheckBox#plotToggle::indicator {{ width: 15px; height: 15px; }}
        QPushButton {{
            background-color: {theme["btn"]};
            border: 1px solid {theme["border"]};
            border-radius: 5px;
            padding: 7px 12px;
        }}
        QPushButton:hover {{
            background-color: {theme["btn_hover"]};
            border-color: {theme["muted"]};
        }}
        QPushButton:pressed {{ background-color: {theme["accent"]}; color: {theme["bg"]}; }}
        QPushButton:disabled {{ color: {theme["muted"]}; }}
        QComboBox {{
            background-color: {theme["panel"]};
            border: 1px solid {theme["border"]};
            color: {theme["fg"]};
            padding: 5px;
        }}
        QComboBox QAbstractItemView {{
            background-color: {theme["panel"]};
            color: {theme["fg"]};
            selection-background-color: {theme["btn_hover"]};
        }}
        /* ! font-size를 두지 말 것 - 스타일시트가 setFont()를 무시해
           로그 폰트 자동 맞춤(fit_log_font)이 동작하지 않는다 */
        QPlainTextEdit {{
            background-color: {theme["panel"]};
            border: 1px solid {theme["border"]};
            color: {theme["fg"]};
            padding: 5px;
        }}
        QSplitter::handle {{ background-color: {theme["handle"]}; }}
    """


def overlay_style(theme, font_size):
    """3D 뷰 하단 오버레이 라벨 스타일 - 테마 전환 때 다시 적용된다."""
    return (
        f"color: {theme['fg']}; font-size: {font_size}px; font-weight: 700; "
        f"background-color: {theme['panel']}; padding: 6px;"
    )


def create_log_file():
    global log_file
    log_dir = os.path.join(BASE_DIR, "logs")   # 로그는 폴더 루트가 아니라 logs/ 하위에 모은다
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(log_dir, f"log_{timestamp}.txt")
    log_file = open(filename, 'w', encoding='utf-8')
    return filename

def stop_log_file():
    global log_file
    if log_file:
        log_file.flush()
        log_file.close()
        log_file = None

# === TimeSeriesGraph: 실시간 시계열 그래프 (모든 그래프 공용) ===
class TimeSeriesGraph(QtWidgets.QWidget):
    """슬라이딩 윈도우 라이브 그래프.

    ! update_data()는 배열에 쓰기만 하고 그리지 않는다. 실제 렌더는 창이 소유한
      단일 타이머가 redraw()를 호출할 때만 일어난다. 25Hz마다 전량 재렌더하면
      그래프 수에 비례해 GUI가 굳는다.
    """
    MAX_POINTS = 10000
    KEEP_ON_COMPACT = MAX_POINTS // 2

    def __init__(self, title_text, y_label, curves):
        """curves: [(이름, 색상hex), ...]"""
        super().__init__()

        # 사전할당 - 매 프레임 배열을 새로 만들지 않기 위함
        self._x = np.zeros(self.MAX_POINTS, dtype=np.float64)
        self._y = np.zeros((len(curves), self.MAX_POINTS), dtype=np.float64)
        self._count = 0
        self._dirty = False

        self.plot_widget = pg.PlotWidget()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 헤더: 제목 + 곡선 on/off 체크박스 (범례 대신)
        header = QtWidgets.QWidget()
        header.setObjectName("panelHeader")
        header.setFixedHeight(30)
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(8, 0, 8, 0)

        title = QtWidgets.QLabel(title_text)
        title.setObjectName("panelTitleInHeader")
        header_layout.addStretch()
        header_layout.addWidget(title)
        header_layout.addSpacing(10)

        # 축 라벨은 축에 크게 붙이지 않고 제목 옆에 작게 표기
        unit_label = QtWidgets.QLabel(f"Y: {y_label}  ·  X: Time (s)")
        unit_label.setObjectName("axisUnitLabel")
        header_layout.addWidget(unit_label)
        header_layout.addSpacing(20)

        self.curves = [
            self.plot_widget.plot(pen=pg.mkPen(color, width=2), name=name)
            for name, color in curves
        ]
        # 곡선이 하나뿐이면 토글이 무의미하므로 체크박스 생략
        # (곡선색 체크박스는 의미색이라 전역 QCheckBox 스타일 대신 인라인으로 덮는다)
        if len(curves) > 1:
            for (name, color), curve in zip(curves, self.curves):
                checkbox = QtWidgets.QCheckBox(name)
                checkbox.setChecked(True)
                checkbox.setStyleSheet(
                    f"QCheckBox {{ color: {color}; font-size: 12px; "
                    f"font-weight: 700; background: transparent; }} "
                    f"QCheckBox::indicator {{ width: 12px; height: 12px; }}"
                )
                checkbox.toggled.connect(curve.setVisible)
                header_layout.addWidget(checkbox)
        header_layout.addStretch()

        layout.addWidget(header)
        layout.addWidget(self.plot_widget)

        # 눈금은 Pretendard 작은 크기 (축 라벨은 헤더로 옮겼으므로 눈금은 보조 역할)
        axis_font = QtGui.QFont("Pretendard", 9)
        axis_font.setWeight(QtGui.QFont.Bold)
        axis_font.setStyleStrategy(QtGui.QFont.PreferAntialias)
        for side in ('left', 'bottom'):
            axis = self.plot_widget.getAxis(side)
            axis.setTickFont(axis_font)
            axis.setStyle(tickTextOffset=5)
            # SI 자동 배율 '(x0.001)'이 라벨에 붙어 단위를 왜곡하므로 비활성화
            axis.enableAutoSIPrefix(False)
        self.plot_widget.showGrid(x=True, y=True)

        # ! 오토레인지를 완전히 끈다. 켜두면 setData/setXRange 한 번마다
        #   pyqtgraph가 모든 곡선의 경계를 다시 훑는 연쇄가 돌아 그래프 하나당
        #   ~20ms가 날아간다 (3개만 켜도 25Hz 예산을 넘김). X/Y 범위는
        #   redraw()에서 numpy로 직접 계산해 넣는 편이 비교도 안 되게 싸다.
        view_box = self.plot_widget.getViewBox()
        view_box.disableAutoRange()
        view_box.setMouseEnabled(x=False, y=False)

    def update_data(self, timestamp, *values):
        """데이터만 적재한다 - 그리기는 redraw()가 담당."""
        if self._count >= self.MAX_POINTS:
            # 오래된 절반을 버리고 앞으로 당긴다 (5000샘플마다 1회)
            keep = self.KEEP_ON_COMPACT
            self._x[:keep] = self._x[-keep:]
            self._y[:, :keep] = self._y[:, -keep:]
            self._count = keep

        index = self._count
        self._x[index] = timestamp
        for row, value in zip(self._y, values):
            row[index] = value
        self._count = index + 1
        self._dirty = True

    def redraw(self):
        """렌더 타이머 전용. 숨겨졌거나 새 데이터가 없으면 아무 것도 하지 않는다."""
        if not self._dirty or not self.isVisible():
            return
        self._dirty = False

        count = self._count
        if count == 0:
            return

        # 창 밖 데이터는 아예 넘기지 않는다. 버퍼에 400초가 쌓여 있어도
        # pyqtgraph가 만지는 건 보이는 20초(=500점)뿐이다.
        right = self._x[count - 1]
        start = int(np.searchsorted(self._x[:count], right - GRAPH_WINDOW_S))
        x = self._x[start:count]

        low = high = None
        for curve, row in zip(self.curves, self._y):
            y = row[start:count]
            curve.setData(x, y)
            if not curve.isVisible() or y.size == 0:
                continue
            y_low, y_high = y.min(), y.max()
            low = y_low if low is None else min(low, y_low)
            high = y_high if high is None else max(high, y_high)

        # 최신 시각을 오른쪽 끝에 고정해 창이 흐르게 한다
        left = right - GRAPH_WINDOW_S
        if right - left < 1e-6:      # 폭 0 범위는 경고를 낸다
            left = right - 1.0
        self.plot_widget.setXRange(left, right, padding=0)

        if low is None:              # 곡선이 전부 꺼져 있으면 Y는 손대지 않는다
            return
        if high - low < 1e-9:        # 값이 상수면 위아래로 조금 벌려준다
            low, high = low - 0.5, high + 0.5
        margin = (high - low) * 0.08
        self.plot_widget.setYRange(low - margin, high + margin, padding=0)

    def clear_data(self):
        self._count = 0
        self._dirty = False
        empty = np.empty(0, dtype=np.float64)
        for curve in self.curves:
            curve.setData(empty, empty)

    def apply_theme(self, theme):
        self.plot_widget.setBackground(theme["panel"])
        pen = pg.mkPen(theme["plot_fg"])
        for side in ('left', 'bottom'):
            axis = self.plot_widget.getAxis(side)
            axis.setPen(pen)
            axis.setTextPen(pen)

# === RocketViewer: 자세 시각화 ===
class RocketViewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        title = QtWidgets.QLabel("3D ATTITUDE")
        title.setObjectName("panelTitle")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(30)
        layout.addWidget(title)

        self.view = gl.GLViewWidget()
        self.view.setMinimumHeight(240)
        layout.addWidget(self.view)
        self.view.setCameraPosition(distance=50, elevation=20, azimuth=30)
        self.view.setBackgroundColor("k")

        self.line_items = []
        self.add_axes(100)
        self.rocket = self.load_rocket(
            os.path.join(BASE_DIR, "PSLV_I.V_simplemodel.stl")
        )
        self.view.addItem(self.rocket)

        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.latest_state = "NO DATA"
        self._dirty = True

        self.overlay_label = QtWidgets.QLabel()
        self.overlay_label.setStyleSheet(overlay_style(THEMES["dark"], 20))
        self.overlay_label.setAlignment(QtCore.Qt.AlignCenter)
        self.overlay_label.setFixedHeight(44)
        layout.addWidget(self.overlay_label)
        self.overlay_label.show()
        self.update_overlay()

    def add_axes(self, length=100):
        axes = [
            (np.array([[0, 0, 0], [1, 0, 0]]), (1, 0, 0, 1)),
            (np.array([[0, 0, 0], [0, 1, 0]]), (0, 1, 0, 1)),
            (np.array([[0, 0, 0], [0, 0, 1]]), (0, 0, 1, 1)),
        ]
        for pos, color in axes:
            line = gl.GLLinePlotItem(pos=pos * length, color=color, width=2)
            self.line_items.append(line)
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
        # 값만 받아두고 GL 재렌더는 redraw()에서 - 25Hz로 돌리면 창이 굳는다
        self.latest_roll = roll
        self.latest_pitch = pitch
        self.latest_yaw = yaw
        self._dirty = True

    @QtCore.pyqtSlot(str)
    def update_state(self, state_str):
        self.latest_state = state_str
        self._dirty = True

    def redraw(self):
        if not self._dirty:
            return
        self._dirty = False

        self.rocket.resetTransform()
        self.rocket.rotate(180 + self.latest_roll, 1, 0, 0)
        self.rocket.rotate(-self.latest_pitch, 0, 1, 0)
        self.rocket.rotate(-self.latest_yaw, 0, 0, 1)
        self.update_overlay()

    def update_overlay(self):
        # 수신율(구 FPS)은 링크 상태 지표라 툴바의 TLM 표시로 옮겼다
        self.overlay_label.setText(
            f"[STATE]: {self.latest_state}    "
            f"R={self.latest_roll:.0f}°, P={self.latest_pitch:.0f}°, "
            f"Y={self.latest_yaw:.0f}°"
        )

    def reset(self):
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.latest_state = "NO DATA"
        self._dirty = True
        self.redraw()

    def apply_theme(self, theme):
        self.overlay_label.setStyleSheet(overlay_style(theme, 20))
        self.view.setBackgroundColor(theme["gl_bg"])
        for item in self.line_items:
            item.setGLOptions(theme["gl_line_mode"])

# === RocketPathViewer: 경로 시각화 ===
class RocketPathViewer(QtWidgets.QWidget):
    MAX_PATH_POINTS = 20000
    KEEP_ON_COMPACT = MAX_PATH_POINTS // 2
    IDLE_OVERLAY = "LAT -   LON -   ALT -   DIST -   ORIGIN NOT SET"

    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        title = QtWidgets.QLabel("3D FLIGHT PATH")
        title.setObjectName("panelTitle")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(30)
        layout.addWidget(title)

        self.view = gl.GLViewWidget()
        self.view.setMinimumHeight(240)
        self.view.setCameraPosition(distance=700, elevation=45, azimuth=45)
        self.view.setBackgroundColor("k")
        layout.addWidget(self.view)

        self.overlay_label = QtWidgets.QLabel()
        self.overlay_label.setStyleSheet(overlay_style(THEMES["dark"], 18))
        self.overlay_label.setAlignment(QtCore.Qt.AlignCenter)
        self.overlay_label.setFixedHeight(58)
        layout.addWidget(self.overlay_label)
        self.overlay_label.show()

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.max_alt = None

        # 경로도 사전할당 - 매 프레임 np.array(list)로 전체를 다시 만들면
        # 길어질수록 비용이 선형으로 커지고 리스트는 무한히 자란다
        self._path = np.zeros((self.MAX_PATH_POINTS, 3), dtype=np.float32)
        self._path_count = 0
        self._overlay_text = self.IDLE_OVERLAY
        self._dirty = True

        self.line_items = []      # 테마별 혼합 모드를 바꿔줘야 하는 GL 선 전체
        self.grid_items = []      # 그중 색까지 바꾸는 바닥 그리드
        self.add_axes()
        self.add_floor_grid(200, 5)
        self.init_path_line()
        self.init_position_marker(os.path.join(BASE_DIR, "current_pos_box.stl"))
        self.overlay_label.setText(self._overlay_text)

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
            line = gl.GLLinePlotItem(pos=pos * length, color=color, width=2)
            self.line_items.append(line)
            self.view.addItem(line)



    def add_floor_grid(self, size, spacing):
        # 흰 배경에서는 흰 그리드가 안 보이므로 테마 전환 때 색을 바꿀 수 있게 모아둔다
        color = THEMES["dark"]["grid"]
        for x in range(-size, size + 1, spacing):
            pts = np.array([[x, -size, 0], [x, size, 0]], dtype=np.float32)
            item = gl.GLLinePlotItem(pos=pts, color=color, width=1)
            self.grid_items.append(item)
            self.line_items.append(item)
            self.view.addItem(item)
        for y in range(-size, size + 1, spacing):
            pts = np.array([[-size, y, 0], [size, y, 0]], dtype=np.float32)
            item = gl.GLLinePlotItem(pos=pts, color=color, width=1)
            self.grid_items.append(item)
            self.line_items.append(item)
            self.view.addItem(item)

    def init_path_line(self):
        self.path_line = gl.GLLinePlotItem(pos=np.empty((0, 3), dtype=np.float32),
                                           color=(1, 1, 0, 1), width=2)
        self.line_items.append(self.path_line)
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
                self._overlay_text = (
                    f"LAT -   LON -   ALT {alt:.2f} m   DIST {abs(alt):.2f} m"
                    "   ORIGIN NOT SET"
                )
                self._dirty = True
                return

            # 처음으로 유효한 위경도 수신 → 기준점 설정
            self.origin_lat, self.origin_lon, self.origin_alt = lat, lon, alt

        # === 2. 기준점 설정 이후에 (0.0, 0.0)이 오면 무시 ===
        if lat == 0.0 and lon == 0.0:
            return

        # === 3. 정상 업데이트 ===
        dx, dy = self.latlon_to_m(self.origin_lat, self.origin_lon, lat, lon)
        dz = alt - self.origin_alt

        if self._path_count >= self.MAX_PATH_POINTS:
            keep = self.KEEP_ON_COMPACT
            self._path[:keep] = self._path[-keep:]
            self._path_count = keep
        self._path[self._path_count] = (dx, dy, dz)
        self._path_count += 1

        if self.max_alt is None or alt > self.max_alt: #최대고도 업데이트/ 기준점 설정 이후 뜸
            self.max_alt = alt
        x_dir = "E" if dx >= 0 else "W"
        y_dir = "N" if dy >= 0 else "S"
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        self._overlay_text = (
            f"LAT {lat:.6f}   LON {lon:.6f}   "
            f"ALT {alt:.2f} m (MAX {self.max_alt:.2f} m)   DIST {distance:.1f} m\n"
            f"{x_dir} {abs(dx):.1f} m, {y_dir} {abs(dy):.1f} m"
        )
        self._dirty = True

    def redraw(self):
        """렌더 타이머 전용 - 그래프와 같은 주기로만 GL을 건드린다."""
        if not self._dirty:
            return
        self._dirty = False

        count = self._path_count
        if count >= 2:
            self.path_line.setData(pos=self._path[:count])
        if count >= 1:
            self.marker.resetTransform()
            self.marker.translate(*self._path[count - 1])
        self.overlay_label.setText(self._overlay_text)

    def reset(self):
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.max_alt = None
        self._path_count = 0
        self._overlay_text = self.IDLE_OVERLAY
        self.path_line.setData(pos=np.empty((0, 3), dtype=np.float32))
        self.marker.resetTransform()
        self._dirty = True
        self.redraw()

    def apply_theme(self, theme):
        self.overlay_label.setStyleSheet(overlay_style(theme, 18))
        self.view.setBackgroundColor(theme["gl_bg"])
        for item in self.line_items:
            item.setGLOptions(theme["gl_line_mode"])
        for item in self.grid_items:
            item.setData(color=theme["grid"])


# === Worker thread -> Qt GUI signals ===
class GroundStationSignals(QtCore.QObject):
    attitude = QtCore.pyqtSignal(float, float, float)
    state = QtCore.pyqtSignal(str)
    position = QtCore.pyqtSignal(float, float, float)
    # 그래프는 telemetry dict 하나로 모두 먹인다 (accel/baro 전용 시그널 폐지)
    telemetry = QtCore.pyqtSignal(dict)
    health = QtCore.pyqtSignal(int)
    line_received = QtCore.pyqtSignal(str)
    connection_changed = QtCore.pyqtSignal(bool, str)


# === FlightStatsPanel: 비행 중 유의미한 값(최대치/이벤트 시각) 표시 ===
class FlightStatsPanel(QtWidgets.QWidget):
    # (키, 라벨, 강조 여부). MAX ALT는 칼만 고도(kf_alt) 기준 - 기압/GNSS보다
    # 융합된 최선 추정값이라 실제 도달 고도로 읽기에 적절하다.
    ROWS = [
        ("max_alt", "MAX ALT", True),
        ("max_vz", "MAX VZ", False),
        ("max_az", "MAX AZ", False),
        ("apogee", "PREDICTED APOGEE", False),
        ("launch_t", "LAUNCH T", False),
        ("deploy_t", "DEPLOY T", False),
        ("ground_t", "GROUND T", False),
    ]

    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 10)

        title = QtWidgets.QLabel("FLIGHT STATS")
        title.setObjectName("panelTitle")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(30)
        layout.addWidget(title)

        self.value_labels = {}
        for key, display_name, hero in self.ROWS:
            row = QtWidgets.QHBoxLayout()
            name_label = QtWidgets.QLabel(display_name)
            name_label.setObjectName("statsKeyHero" if hero else "statsKey")
            value_label = QtWidgets.QLabel("-")
            value_label.setObjectName("statsValueHero" if hero else "statsValue")
            value_label.setAlignment(
                QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
            )
            row.addWidget(name_label)
            row.addStretch()
            row.addWidget(value_label)
            layout.addLayout(row)
            self.value_labels[key] = value_label

        reset_button = QtWidgets.QPushButton("RESET")
        reset_button.clicked.connect(self.reset_stats)
        layout.addWidget(reset_button)

        self.reset_stats()

    def reset_stats(self):
        self.prev_state = None
        self.max_az = None
        self.max_alt = None
        self.max_vz = None
        self.last_apogee = None
        for label in self.value_labels.values():
            label.setText("-")

    def _set(self, key, text):
        self.value_labels[key].setText(text)

    @QtCore.pyqtSlot(dict)
    def update_stats(self, d):
        # 상태 전이 시각 (t = 비행시간)
        state = d["state"]
        if state != self.prev_state:
            if state == "LAUNCH":
                self._set("launch_t", f"{d['time']:.2f} s")
            elif state == "DEPLOY":
                self._set("deploy_t", f"{d['time']:.2f} s")
            elif state == "GROUND":
                self._set("ground_t", f"{d['time']:.2f} s")
            self.prev_state = state

        # 예측 정점은 최대치가 아니라 '지금 예측값'
        # (25Hz로 setText하지 않도록 문자열이 바뀔 때만 갱신)
        apogee_text = f"{d['apogee']:.1f} m"
        if apogee_text != self.last_apogee:
            self.last_apogee = apogee_text
            self._set("apogee", apogee_text)

        # 최대치 갱신 (라벨은 값이 바뀔 때만 setText)
        # accel의 부호 규약이 기체 장착 방향에 달려 있어 크기로 비교한다
        az = abs(d["az"])
        if self.max_az is None or az > self.max_az:
            self.max_az = az
            self._set("max_az", f"{az:.2f} g")

        if self.max_alt is None or d["kf_alt"] > self.max_alt:
            self.max_alt = d["kf_alt"]
            self._set("max_alt", f"{d['kf_alt']:.1f} m")

        if self.max_vz is None or d["kf_vel"] > self.max_vz:
            self.max_vz = d["kf_vel"]
            self._set("max_vz", f"{d['kf_vel']:.1f} m/s")


# === PlotControlPanel: 그래프 표시 선택 + 갱신 주기 ===
class PlotControlPanel(QtWidgets.QWidget):
    # 텔레메트리가 25Hz라 그보다 빨리 그려봐야 같은 데이터를 다시 그릴 뿐이다
    RATES = [("5 Hz", 200), ("10 Hz", 100), ("15 Hz", 67), ("25 Hz", 40)]
    DEFAULT_RATE_INDEX = 3          # 25Hz - XBee 텔레메트리와 동일

    def __init__(self, graphs, on_rate_changed):
        """graphs: [(라벨, TimeSeriesGraph, 기본 표시 여부), ...]"""
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(2)

        title = QtWidgets.QLabel("PLOT CONTROL")
        title.setObjectName("panelTitle")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(30)
        layout.addWidget(title)

        # 한 줄에 하나씩 - 2열로 넣으면 320px 폭에서 긴 이름이 잘린다
        for label, graph, default_on in graphs:
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setObjectName("plotToggle")
            checkbox.setChecked(default_on)
            graph.setVisible(default_on)
            # 숨긴 그래프는 redraw()가 통째로 건너뛰므로 꺼두면 그만큼 가벼워진다
            checkbox.toggled.connect(graph.setVisible)
            layout.addWidget(checkbox)

        rate_row = QtWidgets.QHBoxLayout()
        rate_label = QtWidgets.QLabel("REFRESH RATE")
        rate_label.setObjectName("statsKey")
        self.rate_combo = QtWidgets.QComboBox()
        for name, _ in self.RATES:
            self.rate_combo.addItem(name)
        self.rate_combo.setCurrentIndex(self.DEFAULT_RATE_INDEX)
        self.rate_combo.currentIndexChanged.connect(
            lambda i: on_rate_changed(self.RATES[i][1])
        )
        rate_row.addWidget(rate_label)
        rate_row.addStretch()
        rate_row.addWidget(self.rate_combo)
        layout.addLayout(rate_row)
        layout.addStretch()

    def interval_ms(self):
        return self.RATES[self.rate_combo.currentIndex()][1]


# === Health byte status panel ===
class HealthPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 10)

        title = QtWidgets.QLabel("SENSOR HEALTH")
        title.setObjectName("panelTitle")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(30)
        layout.addWidget(title)

        self.value_labels = {}
        for key, display_name in [
            ("imu", "IMU"),
            ("baro", "BAROMETER"),
            ("gnss", "GNSS"),
            ("baro_ref", "BARO REF"),
            ("vz", "VZ PATH"),
        ]:
            row = QtWidgets.QHBoxLayout()
            name_label = QtWidgets.QLabel(display_name)
            value_label = QtWidgets.QLabel("NO DATA")
            value_label.setAlignment(QtCore.Qt.AlignCenter)
            value_label.setMinimumWidth(100)
            value_label.setMinimumHeight(28)
            row.addWidget(name_label)
            row.addStretch()
            row.addWidget(value_label)
            layout.addLayout(row)
            self.value_labels[key] = value_label

        self._last_status = {}
        self._last_health = 0
        layout.addStretch()
        self.update_health(0)

    def set_status(self, key, text, color):
        # 25Hz로 호출되므로 상태가 실제로 바뀔 때만 스타일 재적용
        if self._last_status.get(key) == (text, color):
            return
        self._last_status[key] = (text, color)
        label = self.value_labels[key]
        label.setText(text)
        label.setStyleSheet(
            f"background-color: {color}; color: white; "
            "font-weight: bold; border-radius: 5px; padding: 5px;"
        )

    def reset_styles(self):
        """테마 전환용 - 캐시를 비우고 마지막 상태로 다시 칠한다."""
        self._last_status.clear()
        self.update_health(self._last_health)

    def reset(self):
        self._last_health = 0
        self.reset_styles()

    @QtCore.pyqtSlot(int)
    def update_health(self, health):
        self._last_health = health
        imu_fresh = bool(health & 0x01)
        baro_fresh = bool(health & 0x02)
        imu_alive = bool(health & 0x04)
        baro_alive = bool(health & 0x08)
        gnss_alive = bool(health & 0x10)
        baro_ref = bool(health & 0x20)
        vz_trusted = bool(health & 0x40)
        vz_usable = bool(health & 0x80)

        if imu_fresh:
            self.set_status("imu", "FRESH", "#16a34a")
        elif imu_alive:
            self.set_status("imu", "STALE", "#d97706")
        else:
            self.set_status("imu", "DEAD", "#dc2626")

        if baro_fresh:
            self.set_status("baro", "FRESH", "#16a34a")
        elif baro_alive:
            self.set_status("baro", "STALE", "#d97706")
        else:
            self.set_status("baro", "DEAD", "#dc2626")

        self.set_status(
            "gnss", "ALIVE" if gnss_alive else "DEAD",
            "#16a34a" if gnss_alive else "#dc2626"
        )
        self.set_status(
            "baro_ref", "VALID" if baro_ref else "INVALID",
            "#16a34a" if baro_ref else "#dc2626"
        )

        if vz_usable:
            self.set_status("vz", "USABLE", "#16a34a")
        elif vz_trusted:
            self.set_status("vz", "SETTLING", "#d97706")
        else:
            self.set_status("vz", "DISABLED", "#dc2626")


# === SerialReader: XBee frame receiver ===
class SerialReader(threading.Thread):
    def __init__(self, port, baud, signals):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.signals = signals
        self.ser = None
        self.running = True
        self._last_flush = 0.0

        # M4 frame: [0x7E][state 1B + float 22 + health 1B][0x0A]
        # 22 floats = time, lat, lon, euler[3], accel[3], gyro[3], pos[3],
        #             altitude, pressure, kalman[4], gnssAlt
        # (telemetry_frame.h의 PackedSensorData 90B와 반드시 일치해야 함)
        self.payload_format = '<B' + 'f'*22 + 'B'
        self.packet_size = struct.calcsize(self.payload_format) + 1  # 91B after start

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def write_log(self, line):
        global logging_enabled, log_file
        with log_lock:
            if logging_enabled and log_file:
                log_file.write(line + '\n')
                # ! 프레임마다 flush하면 이 스레드가 막혀 시리얼 수신이 밀린다.
                #   1초 주기로 낮춘다 - 종료 시 flush는 stop_log_file()이 보장.
                now = time.monotonic()
                if now - self._last_flush >= 1.0:
                    self._last_flush = now
                    log_file.flush()

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.signals.connection_changed.emit(
                True, f"CONNECTED  {self.port} @ {self.baud}"
            )

            while self.running:
                start = self.ser.read(1)

                if start == b'\x7E':
                    packet = self.ser.read(self.packet_size)
                    if len(packet) != self.packet_size or packet[-1] != 0x0A:
                        continue

                    (state_code, time_ms, lat, lon,
                     roll, pitch, yaw,
                     ax, ay, az,
                     gx, gy, gz,
                     px, py, pz,
                     alt, pressure,
                     kalman_alt, kalman_velo, predi_apo, r2,
                     gnss_alt, health) = struct.unpack(
                        self.payload_format, packet[:-1]
                    )

                    state_str = STATE_MAP.get(
                        state_code, f"UNKNOWN({state_code:#X})"
                    )
                    time_val = time_ms / 1000.0

                    line = (
                        f"State: {state_str}, Time: {time_val:.2f}s, "
                        f"Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.2f}m, "
                        f"P: {pressure:.2f}hPa, "
                        f"Euler: ({roll:.2f}, {pitch:.2f}, {yaw:.2f}), "
                        f"Gyro: ({gx:.3f}, {gy:.3f}, {gz:.3f}), "
                        f"Accel: ({ax:.2f}, {ay:.2f}, {az:.2f})g, "
                        f"Pos: ({px:.2f}, {py:.2f}, {pz:.2f}), "
                        f"Kalman: (alt={kalman_alt:.3f}, velo={kalman_velo:.3f}, "
                        f"apo={predi_apo:.3f}, r2={r2:.3f}), "
                        f"GnssAlt: {gnss_alt:.2f}m, "
                        f"Health: 0x{health:02X}"
                    )

                    self.write_log(line)
                    # 25Hz 텔레메트리는 로그 뷰 대신 TELEMETRY 패널에
                    # 종류별로 표시한다 (로그 뷰는 시스템 메시지 전용)
                    self.signals.telemetry.emit({
                        "state": state_str, "time": time_val,
                        "lat": lat, "lon": lon, "gnss_alt": gnss_alt,
                        "roll": roll, "pitch": pitch, "yaw": yaw,
                        "ax": ax, "ay": ay, "az": az,
                        "gx": gx, "gy": gy, "gz": gz,
                        "px": px, "py": py, "pz": pz,
                        "alt": alt, "pressure": pressure,
                        "kf_alt": kalman_alt, "kf_vel": kalman_velo,
                        "apogee": predi_apo, "r2": r2,
                    })
                    self.signals.attitude.emit(roll, pitch, yaw)
                    self.signals.state.emit(state_str)
                    self.signals.position.emit(lat, lon, alt)
                    self.signals.health.emit(health)

                elif start == b'\xAA':
                    line = self.ser.readline().decode(
                        errors='ignore'
                    ).strip()
                    if line:
                        self.write_log(line)
                        self.signals.line_received.emit(line)

        except Exception as error:
            if self.running:
                self.signals.line_received.emit(f"[Serial Error] {error}")
        finally:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.signals.connection_changed.emit(False, "DISCONNECTED")


# === One-window ground station ===
class GroundStationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1600, 1000)
        self.serial_thread = None
        self.signals = GroundStationSignals()
        self.hidden_groups = set()    # 로그에서 숨긴 컬럼 그룹
        self._log_font_px = None
        self.theme_name = "dark"

        # 미션 시계 / 링크 상태
        self.launch_time = None       # LAUNCH로 처음 전이한 프레임의 패킷 시각
        self._phase = None
        self._tplus_text = None
        self._tlm_count = 0
        self._tlm_display = None
        self._connected = False
        self._connection_text = "DISCONNECTED"
        self._log_filename = None
        self._pending_log = []

        self.attitude_view = RocketViewer()
        self.path_view = RocketPathViewer()
        # M7의 accel[3]은 중력이 제거된 선형가속도를 g 단위로 전송한다.
        # Y라벨은 그래프 세로 높이 안에 들어가도록 짧게 (길면 단위가 잘림)
        self.accel_view = TimeSeriesGraph(
            "ACCELERATION", "Accel (g)",
            [("Ax", '#ff4d4d'), ("Ay", '#4ade80'), ("Az", '#5b8cff')]
        )
        # ALT가 3종이므로 한 그래프에 겹쳐 그려 비교한다 (곡선별 토글은 헤더에 자동 생성)
        self.alt_view = TimeSeriesGraph(
            "ALTITUDE", "Alt (m)",
            [("Baro", '#facc15'), ("GNSS", '#4ade80'), ("Kalman", '#5b8cff')]
        )
        self.vz_view = TimeSeriesGraph(
            "VERTICAL VELOCITY", "Vz (m/s)",
            [("Vz", '#f472b6')]
        )
        self.apogee_view = TimeSeriesGraph(
            "PREDICTED APOGEE", "Alt (m)",
            [("Apogee", '#f5c451'), ("Kalman Alt", '#5b8cff')]
        )
        self.graphs = [
            self.accel_view, self.alt_view, self.vz_view, self.apogee_view,
        ]

        self.stats_panel = FlightStatsPanel()
        self.health_panel = HealthPanel()
        self.plot_panel = PlotControlPanel(
            [
                ("ACCELERATION", self.accel_view, True),
                ("ALTITUDE", self.alt_view, True),
                ("VERTICAL VELOCITY", self.vz_view, False),
                ("PREDICTED APOGEE", self.apogee_view, False),
            ],
            self.set_render_interval,
        )

        self.build_ui()
        self.connect_signals()
        self.refresh_ports()
        self.apply_theme()

        # 모든 뷰의 렌더를 이 타이머 하나가 몰아서 처리한다
        self.render_timer = QtCore.QTimer(self)
        self.render_timer.timeout.connect(self.redraw_views)
        self.render_timer.start(self.plot_panel.interval_ms())

        # 로그 뷰는 25Hz로 한 줄씩 넣지 않고 100ms마다 모아서 붙인다
        self.log_timer = QtCore.QTimer(self)
        self.log_timer.timeout.connect(self.flush_log_view)
        self.log_timer.start(100)

        self.clock_timer = QtCore.QTimer(self)
        self.clock_timer.timeout.connect(self.update_clock)
        self.clock_timer.start(1000)
        self.update_clock()

    def build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        toolbar = QtWidgets.QHBoxLayout()

        # 로고는 psi_logo.* 를 BASE_DIR에서 찾아 제목 높이에 맞춘다 (없으면 생략)
        self.logo_label = QtWidgets.QLabel()
        self.logo_label.setAlignment(QtCore.Qt.AlignCenter)
        toolbar.addWidget(self.logo_label)

        app_title = QtWidgets.QLabel(APP_TITLE)
        app_title.setObjectName("appTitle")
        toolbar.addWidget(app_title)

        toolbar.addStretch()

        toolbar.addWidget(QtWidgets.QLabel("PORT"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(110)
        toolbar.addWidget(self.port_combo)

        self.refresh_button = QtWidgets.QPushButton("REFRESH")
        self.connect_button = QtWidgets.QPushButton("CONNECT")
        self.start_log_button = QtWidgets.QPushButton("START LOG")
        self.stop_log_button = QtWidgets.QPushButton("STOP LOG")
        self.stop_log_button.setEnabled(False)
        toolbar.addWidget(self.refresh_button)
        toolbar.addWidget(self.connect_button)
        toolbar.addWidget(self.start_log_button)
        toolbar.addWidget(self.stop_log_button)

        self.log_status_label = QtWidgets.QLabel()
        self.log_status_label.setAlignment(QtCore.Qt.AlignCenter)
        toolbar.addWidget(self.log_status_label)

        self.connection_label = QtWidgets.QLabel("DISCONNECTED")
        self.connection_label.setObjectName("connectionState")
        toolbar.addWidget(self.connection_label)

        self.reset_button = QtWidgets.QPushButton("RESET")
        self.reset_button.setToolTip(
            "Clear graphs, stats, path and log (connection and logging keep running)"
        )
        toolbar.addWidget(self.reset_button)

        self.theme_button = QtWidgets.QPushButton("☀")
        self.theme_button.setToolTip("Toggle light / dark theme")
        self.theme_button.setFixedWidth(42)
        toolbar.addWidget(self.theme_button)
        root.addLayout(toolbar)

        # === 미션 상태 바 ===
        # ! 툴바 한 줄에 다 넣으면 최소 폭이 2300px을 넘어 1920 화면에서 오른쪽
        #   버튼들이 잘려나간다. 상태는 제어와 성격도 다르므로 줄을 나눈다.
        #   패널 헤더와 같은 배경/테두리를 써서 기존 톤에 붙인다.
        mission_bar = QtWidgets.QWidget()
        mission_bar.setObjectName("panelHeader")
        mission_bar.setFixedHeight(40)
        mission_layout = QtWidgets.QHBoxLayout(mission_bar)
        mission_layout.setContentsMargins(12, 0, 12, 0)
        mission_layout.setSpacing(30)
        mission_layout.addStretch()

        # 폭은 '가장 긴 값'을 폰트로 실측해 잡는다. 고정 픽셀로 두면 폰트나 DPI가
        # 바뀔 때 글자가 잘리고, 안 잡으면 값이 바뀔 때마다 줄 전체가 흔들린다.
        def mission_label(object_name, widest, text=""):
            label = QtWidgets.QLabel(text)
            label.setObjectName(object_name)
            font = QtGui.QFont(label.font())
            font.setPixelSize(24)
            label.setFont(font)
            label.setMinimumWidth(QtGui.QFontMetrics(font).horizontalAdvance(widest) + 26)
            label.setAlignment(QtCore.Qt.AlignCenter)
            mission_layout.addWidget(label)
            return label

        self.clock_label = mission_label("missionClock", "KST 00:00:00")
        self.tplus_label = mission_label("missionMuted", "T+ 000:00.0", "T+ --:--")
        self.phase_label = mission_label("missionClock", "PRELAUNCH")
        self.set_phase("NO DATA")
        self.tlm_label = mission_label("missionMuted", "TLM 000 Hz")

        mission_layout.addStretch()
        root.addWidget(mission_bar)

        self.update_log_status(False)

        top_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        top_splitter.addWidget(self.attitude_view)
        top_splitter.addWidget(self.path_view)
        top_splitter.setSizes([800, 800])

        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        # 25Hz x 8줄 = 200줄/s -> 10000줄이면 약 50초 히스토리
        self.log_view.document().setMaximumBlockCount(10000)
        # 섹션 컬럼 정렬을 위해 로그 창만 고정폭 폰트 사용
        # 줄바꿈 없이 한 줄 = 한 프레임 -> 세로 컬럼 정렬 유지
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        # 컬럼 헤더: 로그 뷰와 완전히 같은 위젯/폰트/여백이라 열이 정확히 맞는다
        self.log_header = QtWidgets.QPlainTextEdit()
        self.log_header.setReadOnly(True)
        self.log_header.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.log_header.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.log_header.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.log_header.setPlainText(build_log_header(self.hidden_groups))
        # 로그를 가로 스크롤해도 헤더가 같이 따라가도록 연결
        self.log_view.horizontalScrollBar().valueChanged.connect(
            self.log_header.horizontalScrollBar().setValue
        )
        # 폰트 크기는 창 폭에 맞춰 fit_log_font()가 결정한다
        self.log_view.installEventFilter(self)
        self.fit_log_font()

        # 제목 + 컬럼 그룹 on/off (끄면 줄이 짧아져 폰트가 커진다)
        log_bar = QtWidgets.QWidget()
        log_bar.setObjectName("panelHeader")
        log_bar.setFixedHeight(30)
        bar_layout = QtWidgets.QHBoxLayout(log_bar)
        bar_layout.setContentsMargins(8, 0, 8, 0)
        log_title = QtWidgets.QLabel("TELEMETRY / SYSTEM LOG")
        log_title.setObjectName("panelTitleInHeader")
        bar_layout.addStretch()
        bar_layout.addWidget(log_title)
        bar_layout.addSpacing(16)
        # 토글은 그룹 단위 - 색은 전역 QCheckBox 스타일을 따라 테마와 함께 바뀐다
        for group in TELEMETRY_GROUPS:
            checkbox = QtWidgets.QCheckBox(group)
            checkbox.setChecked(group not in self.hidden_groups)
            checkbox.toggled.connect(
                lambda on, g=group: self.set_group_visible(g, on)
            )
            bar_layout.addWidget(checkbox)
        bar_layout.addStretch()

        log_container = QtWidgets.QWidget()
        log_layout = QtWidgets.QVBoxLayout(log_container)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_layout.setSpacing(0)
        log_layout.addWidget(log_bar)
        log_layout.addWidget(self.log_header)
        log_layout.addWidget(self.log_view)

        # FLIGHT STATS 아래 남는 공간에 PLOT CONTROL을 얹는다
        stats_column = QtWidgets.QWidget()
        stats_column_layout = QtWidgets.QVBoxLayout(stats_column)
        stats_column_layout.setContentsMargins(0, 0, 0, 0)
        stats_column_layout.setSpacing(6)
        stats_column_layout.addWidget(self.stats_panel)
        stats_column_layout.addWidget(self.plot_panel)

        bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        bottom_splitter.addWidget(log_container)
        bottom_splitter.addWidget(stats_column)
        bottom_splitter.addWidget(self.health_panel)
        stats_column.setMaximumWidth(320)
        self.health_panel.setMaximumWidth(300)
        bottom_splitter.setSizes([1250, 280, 270])

        # 켜져 있는 그래프끼리 가로로 나눠 쓴다 (영역 높이는 그대로)
        graph_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        for graph in self.graphs:
            graph_splitter.addWidget(graph)

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        main_splitter.addWidget(top_splitter)
        main_splitter.addWidget(graph_splitter)
        main_splitter.addWidget(bottom_splitter)
        main_splitter.setSizes([500, 200, 350])
        main_splitter.setStretchFactor(0, 4)
        main_splitter.setStretchFactor(1, 2)
        main_splitter.setStretchFactor(2, 4)
        root.addWidget(main_splitter)

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_serial)
        self.start_log_button.clicked.connect(self.start_logging)
        self.stop_log_button.clicked.connect(self.stop_logging)
        self.reset_button.clicked.connect(self.reset_all)
        self.theme_button.clicked.connect(self.toggle_theme)

    # === 테마 ===
    def toggle_theme(self):
        self.theme_name = "light" if self.theme_name == "dark" else "dark"
        self.apply_theme()

    def apply_theme(self):
        theme = THEMES[self.theme_name]
        self.setStyleSheet(build_stylesheet(theme))

        # 로고는 테마마다 다시 읽는다 (라이트에서 반전본을 쓰기 때문)
        logo = load_logo(self.theme_name)
        if logo is None:
            self.logo_label.clear()
            self.logo_label.setVisible(False)
        else:
            self.logo_label.setPixmap(logo)
            self.logo_label.setVisible(True)

        for graph in self.graphs:
            graph.apply_theme(theme)
        self.attitude_view.apply_theme(theme)
        self.path_view.apply_theme(theme)

        self.log_header.setStyleSheet(
            f"color: {theme['accent']}; border: none; "
            f"border-bottom: 1px solid {theme['border']}; "
            f"background-color: {theme['panel']};"
        )

        # 인라인 스타일로 칠해둔 것들은 스타일시트 교체로 갱신되지 않으므로 다시 칠한다
        self._style_connection()
        self.update_log_status(logging_enabled, self._log_filename)
        self.health_panel.reset_styles()
        self._phase = None
        self.set_phase(self.attitude_view.latest_state)
        self._tlm_display = None
        self._style_tlm()

        self.theme_button.setText("☀" if self.theme_name == "dark" else "☾")

    def connect_signals(self):
        self.signals.attitude.connect(self.attitude_view.update_attitude)
        self.signals.state.connect(self.attitude_view.update_state)
        self.signals.position.connect(self.path_view.update_position)
        self.signals.telemetry.connect(self.append_telemetry)
        self.signals.telemetry.connect(self.stats_panel.update_stats)
        self.signals.telemetry.connect(self.update_graphs)
        self.signals.telemetry.connect(self.update_mission)
        self.signals.health.connect(self.health_panel.update_health)
        self.signals.line_received.connect(self.append_log)
        self.signals.connection_changed.connect(
            self.update_connection_state
        )

    # === 렌더 / 미션 시계 ===
    def set_render_interval(self, interval_ms):
        self.render_timer.start(interval_ms)

    def redraw_views(self):
        """모든 뷰의 재렌더를 여기 한 곳에 몰아둔다."""
        for graph in self.graphs:
            graph.redraw()
        self.attitude_view.redraw()
        self.path_view.redraw()

    @QtCore.pyqtSlot(dict)
    def update_graphs(self, d):
        t = d["time"]
        self.accel_view.update_data(t, d["ax"], d["ay"], d["az"])
        self.alt_view.update_data(t, d["alt"], d["gnss_alt"], d["kf_alt"])
        self.vz_view.update_data(t, d["kf_vel"])
        self.apogee_view.update_data(t, d["apogee"], d["kf_alt"])

    @QtCore.pyqtSlot(dict)
    def update_mission(self, d):
        self._tlm_count += 1

        # liftoff = LAUNCH로 처음 전이한 프레임의 패킷 시각
        if d["state"] == "LAUNCH" and self.launch_time is None:
            self.launch_time = d["time"]

        if self.launch_time is None:
            text = "T+ --:--"
        else:
            elapsed = max(0.0, d["time"] - self.launch_time)
            text = f"T+ {int(elapsed) // 60:02d}:{elapsed % 60:04.1f}"

        if text != self._tplus_text:
            self._tplus_text = text
            self.tplus_label.setText(text)
            self.tplus_label.setObjectName(
                "missionMuted" if self.launch_time is None else "missionClock"
            )
            # objectName을 바꿨으면 스타일을 다시 물려야 적용된다
            self.tplus_label.style().polish(self.tplus_label)

        self.set_phase(d["state"])

    def set_phase(self, phase):
        """비행 단계 칩 - HealthPanel.set_status와 같은 '바뀔 때만 칠하기' 방식."""
        if phase == self._phase:
            return
        self._phase = phase
        background, foreground = PHASE_STYLES.get(phase, PHASE_STYLE_UNKNOWN)
        if phase == "NO DATA":
            background, foreground = THEMES[self.theme_name]["muted"], "#ffffff"
        self.phase_label.setText(phase)
        self.phase_label.setStyleSheet(
            f"background-color: {background}; color: {foreground}; "
            "font-weight: bold; border-radius: 5px; padding: 5px;"
        )

    def update_clock(self):
        self.clock_label.setText(datetime.datetime.now().strftime("KST %H:%M:%S"))
        # 1초 동안 도착한 프레임 수 = 실제 텔레메트리 수신율
        self._tlm_rate = self._tlm_count
        self._tlm_count = 0
        self._style_tlm()

    def _style_tlm(self):
        rate = getattr(self, "_tlm_rate", 0)
        if rate == 0:
            color = THEMES[self.theme_name]["muted"]
        elif rate < 20:
            color = "#d97706"       # 25Hz 링크에서 드랍이 나고 있다
        else:
            color = "#4ade80"
        display = (rate, color)
        if display == self._tlm_display:
            return
        self._tlm_display = display
        self.tlm_label.setText(f"TLM {rate} Hz")
        self.tlm_label.setStyleSheet(
            f"background: transparent; border: none; color: {color}; "
            "font-weight: 700; padding: 4px 8px;"
        )

    def reset_all(self):
        """표시만 초기화한다 - 시리얼 연결과 파일 로깅은 계속 유지."""
        for graph in self.graphs:
            graph.clear_data()
        self.attitude_view.reset()
        self.path_view.reset()
        self.stats_panel.reset_stats()
        self.health_panel.reset()

        self._pending_log.clear()
        self.log_view.clear()

        self.launch_time = None
        self._tplus_text = None
        self.tplus_label.setText("T+ --:--")
        self.tplus_label.setObjectName("missionMuted")
        self.tplus_label.style().polish(self.tplus_label)
        self.set_phase("NO DATA")

        self.append_log("[INFO] Display reset.")

    def refresh_ports(self):
        current = self.port_combo.currentText().strip()
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)

        preferred = current or "COM3"
        if preferred not in ports:
            self.port_combo.addItem(preferred)
        self.port_combo.setCurrentText(preferred)

    def toggle_serial(self):
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.stop()
            self.serial_thread = None
            return

        port = self.port_combo.currentText().strip()
        if not port:
            self.append_log("[ERROR] Select a serial port.")
            return

        self.serial_thread = SerialReader(
            port=port,
            baud=115200,
            signals=self.signals,
        )
        self.serial_thread.start()

    @QtCore.pyqtSlot(bool, str)
    def update_connection_state(self, connected, text):
        self._connected = connected
        self._connection_text = text
        self._style_connection()
        self.connect_button.setText("DISCONNECT" if connected else "CONNECT")
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)

        # 버튼 누락으로 비행 데이터를 통째로 잃지 않도록 연결과 로깅을 묶는다.
        # (두 함수 모두 log_lock 아래에서 중복 호출을 방어하므로 그대로 재사용)
        if connected and not logging_enabled:
            self.start_logging()
        elif not connected and logging_enabled:
            self.stop_logging()

    def _style_connection(self):
        self.connection_label.setText(self._connection_text)
        self.connection_label.setStyleSheet(
            "color: #4ade80; font-weight: bold; padding: 5px;"
            if self._connected else
            "color: #f87171; font-weight: bold; padding: 5px;"
        )

    @QtCore.pyqtSlot(str)
    def append_log(self, line):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self._pending_log.append(f"{timestamp} {line}")

    def flush_log_view(self):
        """25Hz로 한 줄씩 append하면 매번 레이아웃이 도므로 모아서 붙인다."""
        if not self._pending_log:
            return
        scrollbar = self.log_view.verticalScrollBar()
        # 사용자가 위로 올려 읽는 중이면 끌어내리지 않는다
        at_bottom = scrollbar.value() >= scrollbar.maximum() - 4
        self.log_view.appendPlainText("\n".join(self._pending_log))
        self._pending_log.clear()
        if at_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def eventFilter(self, obj, event):
        # 로그 뷰 폭이 바뀌면(창 크기·스플리터 이동) 폰트를 다시 맞춘다
        if obj is self.log_view and event.type() == QtCore.QEvent.Resize:
            self.fit_log_font()
        return super().eventFilter(obj, event)

    def set_group_visible(self, group, visible):
        """로그 컬럼 그룹을 켜고 끈다 - 끄면 줄이 짧아져 폰트가 커진다."""
        if visible:
            self.hidden_groups.discard(group)
        else:
            self.hidden_groups.add(group)
        self.log_header.setPlainText(build_log_header(self.hidden_groups))
        self._log_font_px = None      # 폭이 바뀌었으므로 강제 재계산
        self.fit_log_font()

    def fit_log_font(self):
        """로그 한 줄이 잘리거나 넘치지 않는 최대 폰트 크기를 찾아 적용."""
        available = self.log_view.viewport().width() - 5
        line_width = log_line_width(self.hidden_groups)
        if available <= 0 or line_width <= 0:
            return

        sample = "0" * line_width
        low, high, best = 8, 48, 8
        while low <= high:
            mid = (low + high) // 2
            advance = QtGui.QFontMetricsF(
                make_log_font(mid)
            ).horizontalAdvance(sample)
            if advance <= available:
                best = mid
                low = mid + 1
            else:
                high = mid - 1

        # 폰트 적용은 resize를 유발하므로 값이 바뀔 때만 갱신
        if best == self._log_font_px:
            return
        self._log_font_px = best

        font = make_log_font(best)
        self.log_view.setFont(font)
        self.log_header.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        self.log_header.setFixedHeight(metrics.lineSpacing() * 2 + 16)

    @QtCore.pyqtSlot(dict)
    def append_telemetry(self, d):
        """25Hz 텔레메트리 프레임을 헤더와 같은 폭의 값 행으로 출력."""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self._pending_log.append(
            f"{timestamp} {format_log_row(d, self.hidden_groups)}"
        )

    def update_log_status(self, recording, filename=None):
        """툴바의 파일 로깅 상태 표시를 갱신."""
        if recording:
            name = os.path.basename(filename) if filename else ""
            self.log_status_label.setText(f"● REC  {name}")
            self.log_status_label.setStyleSheet(
                "background-color: #dc2626; color: white; font-weight: 700; "
                "border-radius: 5px; padding: 6px 14px;"
            )
        else:
            theme = THEMES[self.theme_name]
            self.log_status_label.setText("○ NOT LOGGING")
            self.log_status_label.setStyleSheet(
                f"background-color: {theme['btn']}; color: {theme['muted']}; "
                f"font-weight: 700; border: 1px solid {theme['border']}; "
                "border-radius: 5px; padding: 6px 14px;"
            )

    def start_logging(self):
        global logging_enabled
        with log_lock:
            if logging_enabled:
                self.append_log("[INFO] Logging is already active.")
                return
            filename = create_log_file()
            logging_enabled = True
        self._log_filename = filename
        self.append_log(f"[INFO] Logging started: {filename}")
        self.update_log_status(True, filename)
        self.start_log_button.setEnabled(False)
        self.stop_log_button.setEnabled(True)

    def stop_logging(self):
        global logging_enabled
        with log_lock:
            if not logging_enabled:
                self.append_log("[INFO] Logging is not active.")
                return
            logging_enabled = False
            stop_log_file()
        self._log_filename = None
        self.append_log("[INFO] Logging stopped.")
        self.update_log_status(False)
        self.start_log_button.setEnabled(True)
        self.stop_log_button.setEnabled(False)

    def closeEvent(self, event):
        global logging_enabled
        if self.serial_thread:
            self.serial_thread.stop()
        with log_lock:
            logging_enabled = False
            stop_log_file()
        event.accept()


def ensure_pretendard():
    """Pretendard를 등록하고 앱 기본 폰트로 지정한다.

    ! 기본 폰트를 스타일시트가 아니라 QApplication에 주는 이유: 스타일시트로
      폰트를 지정하면 setFont()가 무시되어 로그 폰트 자동 맞춤이 죽는다.
    """
    if "Pretendard" not in QtGui.QFontDatabase().families():
        user_fonts = os.path.join(
            os.environ.get("LOCALAPPDATA", ""), "Microsoft", "Windows", "Fonts"
        )
        if os.path.isdir(user_fonts):
            for name in os.listdir(user_fonts):
                if name.lower().startswith("pretendard"):
                    QtGui.QFontDatabase.addApplicationFont(
                        os.path.join(user_fonts, name)
                    )

    app = QtWidgets.QApplication.instance()
    if app is not None:
        base_font = QtGui.QFont("Pretendard")
        base_font.setPixelSize(21)
        base_font.setWeight(QtGui.QFont.Bold)
        app.setFont(base_font)


# === Main ===
if __name__ == "__main__":
    # ! vsync를 끈다 - 반드시 QApplication 생성 전에.
    #   3D 뷰(GLViewWidget)의 기본 swapInterval=1이 창 전체 합성을 60Hz vsync에
    #   묶어버려서, 위젯 하나 갱신할 때마다 ~17ms씩 대기가 걸린다. 그래프를
    #   2개 넘게 켜면 이 대기가 쌓여 이벤트 루프가 막힌다 (4개에서 평균 187ms).
    #   끄면 같은 조건에서 15ms. pyqtgraph나 파이썬의 한계가 아니라 이 설정 문제.
    surface_format = QtGui.QSurfaceFormat.defaultFormat()
    surface_format.setSwapInterval(0)
    QtGui.QSurfaceFormat.setDefaultFormat(surface_format)

    app = QtWidgets.QApplication(sys.argv)
    ensure_pretendard()
    window = GroundStationWindow()
    window.showMaximized()
    sys.exit(app.exec_())
#20260806_1648 업데이트
