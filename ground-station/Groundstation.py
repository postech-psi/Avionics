# =============================================================================
# PSI Ground Station - 25Hz XBee 텔레메트리 수신 / 시각화 / 로깅
#   SerialReader (스레드) -> GroundStationSignals -> 각 뷰 (GUI 스레드)
#   모든 렌더는 창이 소유한 단일 타이머(RENDER_INTERVAL_MS)가 몰아서 처리한다.
# =============================================================================
import sys, serial, struct, threading, time, datetime, os, json, math, re, csv
import numpy                    as np
import pyqtgraph                as pg
import pyqtgraph.opengl         as gl
from   PyQt5                    import QtWidgets, QtCore, QtGui
from   OpenGL                   import GL
from   stl                      import mesh
from   serial.tools             import list_ports

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#                               Global logging control
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
logging_enabled = False
log_file        = None
csv_file        = None
log_lock        = threading.Lock()

# logging_enabled : 로깅 on/off. 툴바 버튼이 뒤집는다
# log_file        : 이벤트/시스템 메시지 (.txt)
# csv_file        : 텔레메트리 프레임 (.csv)
# log_lock        : 수신 스레드와 GUI 스레드가 같은 핸들을 만지므로 필요

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

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#                       Tunable parameters - 화면 성능/가독성
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
GRAPH_WINDOW_S     = 20.0
MAX_VISIBLE_GRAPHS = 4
RENDER_INTERVAL_MS = 40

# GRAPH_WINDOW_S     : 라이브 그래프의 시간 창 [s]. X는 흐르고 Y는 이 구간에만 맞춘다
# MAX_VISIBLE_GRAPHS : 가로로 나란히 둘 수 있는 그래프 수. 넘기면 칸 폭이 최소치
#                      밑으로 눌려 축과 헤더가 겹친다 (PlotControlPanel이 강제한다)
# RENDER_INTERVAL_MS : 렌더 주기 [ms]. 텔레메트리가 25Hz(40ms)라 그보다 빨리
#                      그려봐야 같은 데이터를 다시 그릴 뿐이다. 렉이 심하면 이 값만
#                      키우면 된다 (100 = 10Hz)

# 위성사진 위에 경로/축을 겹쳐 그리기 위한 GL 설정.
# ! pyqtgraph 프리셋으로는 둘 다 만족할 수 없다.
#     'additive'   - 깊이 테스트 꺼짐(항상 보임) 이지만 밝은 사진 위에서 색이 씻긴다
#     'translucent'- 색은 맞지만 깊이 테스트가 켜져, 기압 노이즈로 경로가 사진
#                    평면(z=-0.5) 아래로 내려가면 통째로 가려진다 (실측 58% 소실)
#   깊이 테스트만 끄고 정상 알파 합성을 쓰면 색도 살고 가려지지도 않는다.
OVERLAY_GL_OPTIONS = {
    GL.GL_DEPTH_TEST: False,
    GL.GL_BLEND: True,
    GL.GL_ALPHA_TEST: False,
    GL.GL_CULL_FACE: False,
    "glBlendFunc": (GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA),
}

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


BASEMAP_DIR = os.path.join(BASE_DIR, "basemaps")


def discover_basemaps():
    """basemaps/의 지점 목록을 읽는다 (메타데이터만, 이미지는 안 읽음).

    ! 영상 한 장이 수백 MB라 전부 올려두면 안 된다. 목록만 먼저 만들고
      실제 픽셀은 사용자가 고른 지점만 load_basemap_pixels()로 읽는다.
    """
    if not os.path.isdir(BASEMAP_DIR):
        return []

    sites = []
    for name in sorted(os.listdir(BASEMAP_DIR)):
        stem, ext = os.path.splitext(name)
        if ext.lower() != ".json":
            continue
        image_path = os.path.join(BASEMAP_DIR, stem + ".png")
        if not os.path.isfile(image_path):
            continue
        try:
            with open(os.path.join(BASEMAP_DIR, name), encoding="utf-8") as handle:
                meta = json.load(handle)
            meta["_image_path"] = image_path
            meta.setdefault("name", stem)
            meta.setdefault("label", stem)
            sites.append(meta)
        except (OSError, ValueError) as error:
            print(f"[basemap] {name} 읽기 실패: {error}")
    return sites


def load_basemap_pixels(meta):
    """선택된 지점의 영상을 RGBA 배열로 읽는다. 실패하면 None."""
    image = QtGui.QImage(meta["_image_path"])
    if image.isNull():
        print(f"[basemap] {meta['_image_path']} 디코딩 실패")
        return None
    image = image.convertToFormat(QtGui.QImage.Format_RGBA8888)
    pointer = image.constBits()
    pointer.setsize(image.byteCount())
    rows = np.frombuffer(pointer, np.ubyte).reshape(
        image.height(), image.width(), 4
    )
    # ! GLImageItem은 배열을 (x, y, RGBA)로 읽는다. QImage는 (행=y, 열=x)이므로
    #   그대로 넘기면 대각선 기준으로 전치된 영상이 깔린다. 정사각형에 가까운
    #   모자이크에서는 눈에 잘 안 띄지만 좌표가 통째로 틀어진다.
    return np.ascontiguousarray(rows.transpose(1, 0, 2))


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
        /* FLIGHT STATS / SENSOR HEALTH 는 같은 키-값 문법을 공유한다 */
        QLabel#statsKey {{
            color: {theme["fg"]};
            font-size: 19px;
            font-weight: 600;
        }}
        QLabel#statsValue {{
            color: {theme["fg"]};
            font-size: 21px;
            font-weight: 700;
        }}
        QLabel#seekTime {{
            color: {theme["muted"]};
            background: transparent;
        }}
        QSlider::groove:horizontal {{
            height: 6px;
            background: {theme["btn"]};
            border: 1px solid {theme["border"]};
            border-radius: 3px;
        }}
        QSlider::sub-page:horizontal {{
            background: {theme["accent"]};
            border: 1px solid {theme["accent"]};
            border-radius: 3px;
        }}
        QSlider::handle:horizontal {{
            width: 12px;
            margin: -6px 0;
            background: {theme["fg"]};
            border: 1px solid {theme["border"]};
            border-radius: 3px;
        }}
        QSlider::handle:horizontal:hover {{ background: {theme["accent"]}; }}
        QSlider:disabled::sub-page:horizontal {{
            background: {theme["border"]}; border-color: {theme["border"]};
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
        QCheckBox#plotToggle {{ color: {theme["fg"]}; font-size: 15px; }}
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
        QPushButton:checked {{
            background-color: {theme["accent"]};
            border-color: {theme["accent"]};
            color: {theme["bg"]};
            font-weight: 700;
        }}
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


# === 로그 포맷 ===
# 텔레메트리는 CSV, 이벤트(0xAA 텍스트/시스템 메시지)는 .txt 로 나눠 적는다.
#   - 예전 단일 .txt는 라벨을 매 줄 반복해 294 B/줄이었고, 값도 반올림해 적어
#     원본 float 정밀도를 잃었다. CSV는 더 작으면서 더 정확하다.
#   - 실측: 17,258줄 로그에서 이벤트는 8줄뿐이라 둘을 섞을 이유가 없다.
# (컬럼명, dict키, 포맷) - 정밀도는 float32가 실제로 담을 수 있는 만큼만.
CSV_FIELDS = [
    ("wall", None, None),                 # 지상국 벽시계 (프레임 도착 시각)
    ("t", "time", "{:.3f}"),
    ("state", "state", "{}"),
    ("lat", "lat", "{:.7f}"),
    ("lon", "lon", "{:.7f}"),
    ("gnss_alt", "gnss_alt", "{:.2f}"),
    ("roll", "roll", "{:.3f}"),
    ("pitch", "pitch", "{:.3f}"),
    ("yaw", "yaw", "{:.3f}"),
    ("ax", "ax", "{:.4f}"),
    ("ay", "ay", "{:.4f}"),
    ("az", "az", "{:.4f}"),
    ("gx", "gx", "{:.3f}"),
    ("gy", "gy", "{:.3f}"),
    ("gz", "gz", "{:.3f}"),
    ("px", "px", "{:.3f}"),
    ("py", "py", "{:.3f}"),
    ("pz", "pz", "{:.3f}"),
    ("baro_alt", "alt", "{:.3f}"),
    ("pressure", "pressure", "{:.3f}"),
    ("kf_alt", "kf_alt", "{:.3f}"),
    ("kf_vz", "kf_vel", "{:.3f}"),
    ("apogee", "apogee", "{:.3f}"),
    ("r2", "r2", "{:.5f}"),
    ("health", "health", "{}"),
]
CSV_HEADER = ",".join(name for name, _, _ in CSV_FIELDS)


def format_csv_row(d, wall):
    cells = [wall]
    for _, key, fmt in CSV_FIELDS[1:]:
        value = d.get(key)
        cells.append("" if value is None else fmt.format(value))
    return ",".join(cells)


def create_log_file():
    """log_<시각>.csv (텔레메트리) 와 log_<시각>.txt (이벤트) 를 함께 연다."""
    global log_file, csv_file
    log_dir = os.path.join(BASE_DIR, "logs")   # 로그는 폴더 루트가 아니라 logs/ 하위에 모은다
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(log_dir, f"log_{timestamp}")
    csv_file = open(base + ".csv", 'w', encoding='utf-8', newline='')
    csv_file.write(CSV_HEADER + "\n")
    log_file = open(base + ".txt", 'w', encoding='utf-8')
    return base + ".csv"


def stop_log_file():
    global log_file, csv_file
    for handle in (log_file, csv_file):
        if handle:
            handle.flush()
            handle.close()
    log_file = None
    csv_file = None

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
        header_layout.addSpacing(12)

        # 사용자가 확대/이동하면 자동 추적이 멈추므로 그 상태를 알려준다
        self.manual_hint = QtWidgets.QLabel("MANUAL · 더블클릭=복귀")
        self.manual_hint.setStyleSheet(
            "color: #d97706; font-size: 12px; font-weight: 700; background: transparent;"
        )
        self.manual_hint.setVisible(False)
        header_layout.addWidget(self.manual_hint)
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
        # 마우스는 켠다 (실측 +2ms, 25Hz 예산 40ms에 비하면 무시할 수준).
        # 대신 사용자가 범위를 만지면 _auto_range를 내려 redraw가 덮어쓰지 않게 한다.
        view_box.setMouseEnabled(x=True, y=True)
        self._auto_range = True
        view_box.sigRangeChangedManually.connect(self._on_manual_range)
        self.plot_widget.viewport().installEventFilter(self)

    def _on_manual_range(self, *_):
        if self._auto_range:
            self._auto_range = False
            self.manual_hint.setVisible(True)

    def set_auto_range(self, on):
        self._auto_range = on
        self.manual_hint.setVisible(not on)
        self._dirty = True
        if on:
            self.redraw()

    def eventFilter(self, obj, event):
        # 더블클릭으로 자동 추적 복귀 - pyqtgraph 기본 우클릭 메뉴보다 찾기 쉽다
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.set_auto_range(True)
            return True
        return super().eventFilter(obj, event)

    def update_data(self, timestamp, *values):
        """데이터만 적재한다 - 그리기는 redraw()가 담당.

        timestamp : 패킷 시각 [s]
        values    : 곡선 수와 같은 개수의 값. 생성자 curves 순서와 1:1로 짝
        저장 구조 : _x [MAX_POINTS], _y [곡선 수, MAX_POINTS] 사전할당 링 아님 -
                    가득 차면 오래된 절반을 버리고 앞으로 당긴다
        """
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
        # pyqtgraph가 만지는 건 화면에 보이는 구간뿐이다.
        right = self._x[count - 1]
        if self._auto_range:
            view_left, view_right = right - GRAPH_WINDOW_S, right
        else:
            # 사용자가 맞춰둔 범위를 그대로 두고 그 구간만 넘긴다
            view_left, view_right = self.plot_widget.getViewBox().viewRange()[0]
        start = int(np.searchsorted(self._x[:count], view_left))
        stop = int(np.searchsorted(self._x[:count], view_right, side="right"))
        stop = max(stop, start + 1)
        x = self._x[start:stop]

        low = high = None
        for curve, row in zip(self.curves, self._y):
            y = row[start:stop]
            curve.setData(x, y)
            if not curve.isVisible() or y.size == 0:
                continue
            y_low, y_high = y.min(), y.max()
            low = y_low if low is None else min(low, y_low)
            high = y_high if high is None else max(high, y_high)

        if not self._auto_range:     # 수동 모드에서는 범위를 건드리지 않는다
            return

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

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#                     AttitudeGauge - 소형 인공수평의 (계기판)
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
class AttitudeGauge(QtWidgets.QWidget):
    """3D ATTITUDE 뷰 좌상단에 얹는 항공계기식 자세계.

    입력 (deg)   : roll  - 기수축 둘레 회전. 눈금판을 반대로 돌린다
                   pitch - 기수 들림. 수평선을 위아래로 민다
    표시         : 하늘/땅 이분할 + 고정 기체 심볼 + 롤 눈금 + TILT 숫자
    ! 3D 씬 안에 링/사다리를 그리면 카메라를 돌릴 때마다 읽는 법이 달라진다.
      계기는 화면에 고정된 2D여야 각도가 항상 같은 자리에서 읽힌다.
    """
    SIZE            = 96        # 지름 [px] - 3D 뷰를 가리지 않는 선
    PITCH_PX_PER_DEG = 0.9      # 수평선 이동량 [px/deg]. 크면 조금만 기울여도 판이 넘어간다
    ROLL_TICKS      = (-60, -30, 0, 30, 60)   # 롤 눈금 [deg]
    SKY             = QtGui.QColor("#2b6cb0")
    GROUND          = QtGui.QColor("#8b5a2b")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(self.SIZE, self.SIZE)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.roll  = 0.0
        self.pitch = 0.0
        self._rim  = QtGui.QColor(THEMES["dark"]["border"])
        self._fg   = QtGui.QColor(THEMES["dark"]["fg"])
        self._accent = QtGui.QColor(THEMES["dark"]["accent"])

    def set_attitude(self, roll, pitch):
        if (roll, pitch) != (self.roll, self.pitch):
            self.roll, self.pitch = roll, pitch
            self.update()

    def apply_theme(self, theme):
        self._rim    = QtGui.QColor(theme["border"])
        self._fg     = QtGui.QColor(theme["fg"])
        self._accent = QtGui.QColor(theme["accent"])
        self.update()

    def tilt(self):
        """발사대(연직) 기준 이탈각 [deg]. 요는 연직축 회전이라 들어가지 않는다."""
        cos_tilt = math.cos(math.radians(self.roll)) * math.cos(math.radians(self.pitch))
        return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))

    def paintEvent(self, _event):
        radius = self.SIZE / 2 - 6
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.translate(self.SIZE / 2, self.SIZE / 2)

        # --- 눈금판: 원 안쪽만 그린다 (clip) ---
        face = QtGui.QPainterPath()
        face.addEllipse(QtCore.QPointF(0, 0), radius, radius)
        painter.save()
        painter.setClipPath(face)
        painter.rotate(-self.roll)
        offset = self.pitch * self.PITCH_PX_PER_DEG
        span = radius * 2
        painter.fillRect(QtCore.QRectF(-span, -span + offset, span * 2, span),
                         self.SKY)
        painter.fillRect(QtCore.QRectF(-span, offset, span * 2, span), self.GROUND)
        painter.setPen(QtGui.QPen(QtGui.QColor("#f0f0f0"), 1.5))
        painter.drawLine(QtCore.QPointF(-span, offset), QtCore.QPointF(span, offset))

        # 피치 눈금 - 10° 간격의 짧은 가로선
        painter.setPen(QtGui.QPen(QtGui.QColor(240, 240, 240, 170), 1))
        for deg in (-20, -10, 10, 20):
            y = offset - deg * self.PITCH_PX_PER_DEG
            half = radius * (0.30 if deg % 20 else 0.18)
            painter.drawLine(QtCore.QPointF(-half, y), QtCore.QPointF(half, y))
        painter.restore()

        # --- 롤 눈금 (계기 테두리, 화면 고정) ---
        painter.setPen(QtGui.QPen(self._fg, 1))
        for deg in self.ROLL_TICKS:
            painter.save()
            painter.rotate(deg)
            length = 6 if deg == 0 else 4
            painter.drawLine(QtCore.QPointF(0, -radius),
                             QtCore.QPointF(0, -radius + length))
            painter.restore()

        # --- 고정 기체 심볼: 이 표식이 눈금판 어디에 놓이는지가 곧 자세다 ---
        painter.setPen(QtGui.QPen(self._accent, 2))
        painter.drawLine(QtCore.QPointF(-radius * 0.5, 0), QtCore.QPointF(-radius * 0.15, 0))
        painter.drawLine(QtCore.QPointF(radius * 0.15, 0), QtCore.QPointF(radius * 0.5, 0))
        painter.drawPoint(QtCore.QPointF(0, 0))

        # --- 테두리 + TILT 숫자 ---
        painter.setPen(QtGui.QPen(self._rim, 2))
        painter.drawEllipse(QtCore.QPointF(0, 0), radius, radius)
        font = QtGui.QFont("Pretendard", 8, QtGui.QFont.Bold)
        painter.setFont(font)
        painter.setPen(self._fg)
        painter.drawText(QtCore.QRectF(-radius, radius - 12, radius * 2, 14),
                         QtCore.Qt.AlignCenter, f"{self.tilt():.0f}°")


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

        # 계기는 GL 뷰의 자식이라 뷰 좌상단에 붙어 다닌다 (레이아웃 밖 = 겹쳐 그리기)
        self.gauge = AttitudeGauge(self.view)
        self.gauge.move(8, 8)
        self.gauge.raise_()

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

        self.gauge.set_attitude(self.latest_roll, self.latest_pitch)
        self.update_overlay()

    def update_overlay(self):
        # 수신율(구 FPS)은 링크 상태 지표라 툴바의 TLM 표시로 옮겼다.
        # 기울기 각도는 좌상단 계기(AttitudeGauge)가 숫자로 같이 보여준다
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
        self.gauge.apply_theme(theme)
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

        # 제목만 가운데. MAP/FOLLOW/TOP 조작부는 PLOT CONTROL 박스로 옮겨간다
        # (여기서 만들어두기만 하고 배치는 PlotControlPanel이 한다)
        title = QtWidgets.QLabel("3D FLIGHT PATH")
        title.setObjectName("panelTitle")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(30)
        layout.addWidget(title)

        self.basemap_combo = QtWidgets.QComboBox()
        self.basemap_combo.setMinimumWidth(160)

        # 켜두면 카메라가 비행 범위를 따라간다. 사용자가 뷰를 만지면 자동으로 꺼진다.
        self.follow_check = QtWidgets.QCheckBox("FOLLOW")
        self.follow_check.setObjectName("plotToggle")
        self.follow_check.setChecked(True)
        self.follow_check.setToolTip(
            "비행 범위에 맞춰 카메라를 자동으로 맞춥니다.\n"
            "뷰를 직접 조작하면 해제됩니다."
        )

        # 기울어진 3D 시점에서는 지면이 눌려 보여 위성사진을 지도처럼 읽기 어렵다
        self.top_button = QtWidgets.QPushButton("TOP")
        self.top_button.setCheckable(True)
        # 폭은 실측으로 - 고정 픽셀로 두면 폰트/DPI가 바뀔 때 글자가 잘린다
        self.top_button.setFixedWidth(
            QtGui.QFontMetrics(self.top_button.font()).horizontalAdvance("TOP") + 30
        )
        self.top_button.setToolTip(
            "북쪽이 위인 정사 시점으로 전환합니다.\n"
            "위성사진과 위치를 대조할 때 쓰세요. FOLLOW와 같이 켤 수 있습니다."
        )

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
        self._theme_line_mode = THEMES["dark"]["gl_line_mode"]
        self._follow_state = None
        self._view_angle = (35, 45)        # (elevation, azimuth) - TOP 버튼이 바꾼다
        self.view.installEventFilter(self)
        self.follow_check.toggled.connect(self._on_follow_toggled)
        self.top_button.toggled.connect(self.toggle_top_view)
        self.add_axes()
        self.add_floor_grid(200, 5)
        self.init_path_line()
        self.init_position_marker(os.path.join(BASE_DIR, "current_pos_box.stl"))
        self.init_basemap()
        self.overlay_label.setText(self._overlay_text)

    def add_axes(self, length=200):
        # ! 축은 배경 참조선이다. 예전엔 궤적과 같은 굵기/채도라 서로 싸웠다.
        #   가늘고 반투명하게 낮춰 궤적이 주인공이 되게 한다.
        axes = [
            (np.array([[0, 0, 0], [1, 0, 0]]), (1, 0.25, 0.25, 0.55), "E"),   # +X 동
            (np.array([[0, 0, 0], [-1, 0, 0]]), (1, 0.35, 1, 0.55), "W"),     # -X 서
            (np.array([[0, 0, 0], [0, 1, 0]]), (0.3, 1, 0.4, 0.55), "N"),     # +Y 북
            (np.array([[0, 0, 0], [0, -1, 0]]), (0.3, 0.9, 1, 0.55), "S"),    # -Y 남
            (np.array([[0, 0, 0], [0, 0, 1]]), (0.45, 0.6, 1, 0.55), None),   # +Z 상승
        ]
        self.compass_labels = []
        for pos, color, name in axes:
            line = gl.GLLinePlotItem(pos=pos * length, color=color, width=1)
            self.line_items.append(line)
            self.view.addItem(line)
            if name is None:
                continue
            # 방위 글자를 축 끝에 붙인다 - 뷰를 돌려도 어디가 북쪽인지 바로 읽힌다
            tip = (pos[1] * (length * 1.06)).astype(np.float32)
            label = gl.GLTextItem(
                pos=tip, text=name,
                color=QtGui.QColor.fromRgbF(*color[:3], 0.95),
                font=QtGui.QFont("Pretendard", 13, QtGui.QFont.Bold),
            )
            self.compass_labels.append(label)
            self.view.addItem(label)



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

    # 궤적 색 - 오래된 구간은 흐리게, 최근은 밝게 (혜성 꼬리처럼 진행 방향이 읽힌다)
    TRAIL_OLD = (0.98, 0.62, 0.20)      # 짙은 호박색
    TRAIL_NEW = (1.00, 0.98, 0.90)      # 거의 흰색
    TRAIL_MIN_ALPHA = 0.18

    def init_path_line(self):
        # 지면 투영선은 제거했다 - 궤적과 색이 같아 얇은 유령선처럼 읽혔고,
        # 수평 위치는 TOP 시점과 오버레이의 동/북 거리로 더 정확히 읽힌다
        self.path_line = gl.GLLinePlotItem(pos=np.empty((0, 3), dtype=np.float32),
                                           width=2.5, antialias=True)
        self.line_items.append(self.path_line)
        self.view.addItem(self.path_line)

        # 색 램프는 한 번만 만들어두고 필요한 만큼 뒤에서 잘라 쓴다
        ramp = np.linspace(0.0, 1.0, self.MAX_PATH_POINTS, dtype=np.float32)[:, None]
        old = np.array(self.TRAIL_OLD, dtype=np.float32)
        new = np.array(self.TRAIL_NEW, dtype=np.float32)
        self._trail_colors = np.empty((self.MAX_PATH_POINTS, 4), dtype=np.float32)
        self._trail_colors[:, :3] = old + (new - old) * ramp
        self._trail_colors[:, 3] = self.TRAIL_MIN_ALPHA + (1.0 - self.TRAIL_MIN_ALPHA) * ramp[:, 0]

    def init_position_marker(self, stl_path):
        stl = mesh.Mesh.from_file(stl_path)
        verts = stl.vectors.reshape(-1, 3)
        faces = np.arange(len(verts)).reshape(-1, 3)
        verts -= verts.mean(axis=0)
        verts *= 0.1
        self.marker = gl.GLMeshItem(vertexes=verts, faces=faces, smooth=True,
                                    shader=None, color=(1, 1, 0, 1))
        self.view.addItem(self.marker)

    def init_basemap(self):
        """받아둔 지점 목록으로 MAP 콤보를 채운다. 영상은 선택 시 읽는다."""
        self.basemap_item = None
        self.basemap_meta = None
        self.basemap_sites = discover_basemaps()

        self.basemap_combo.addItem("GRID", None)
        for meta in self.basemap_sites:
            self.basemap_combo.addItem(meta["label"], meta["name"])
            index = self.basemap_combo.count() - 1
            self.basemap_combo.setItemData(
                index,
                f"{meta['name']} · {meta['center_lat']:.5f}, {meta['center_lon']:.5f}"
                f" · z{meta['zoom']} · {meta['meters_per_pixel']:.2f} m/px",
                QtCore.Qt.ToolTipRole,
            )
        self.basemap_combo.currentIndexChanged.connect(self._on_basemap_changed)

        if not self.basemap_sites:
            self.basemap_combo.setEnabled(False)
            self.basemap_combo.setToolTip(
                "basemaps/ 가 비어 있습니다. fetch_basemap.py 로 먼저 받으세요."
            )
        else:
            self.basemap_combo.setCurrentIndex(1)      # 첫 지점을 기본으로

    def _on_basemap_changed(self, index):
        self.set_basemap(self.basemap_combo.itemData(index))

    def _on_follow_toggled(self, enabled):
        if not enabled:
            return
        self._follow_state = None         # 다시 켜면 즉시 한 번 맞춘다
        if self._path_count:
            self.follow_camera()
        else:
            self.fit_camera()

    def set_basemap(self, name):
        """지점을 바꾼다. name이 None이면 위성사진을 끄고 격자로 돌아간다.

        ! 이전 영상은 반드시 먼저 버린다. 지점 하나가 수백 MB라
          겹쳐 들고 있으면 메모리가 배로 뛴다.
        """
        if self.basemap_item is not None:
            self.view.removeItem(self.basemap_item)
            self.basemap_item = None
            self.basemap_meta = None

        meta = next((m for m in self.basemap_sites if m["name"] == name), None)
        if meta is not None:
            pixels = load_basemap_pixels(meta)
            if pixels is not None:
                self.basemap_meta = meta
                # ! opaque + 낮은 depthValue 라야 축/경로보다 먼저 그려진다.
                #   기본값(translucent)이면 위성사진이 경로를 덮어버린다.
                self.basemap_item = gl.GLImageItem(
                    pixels, smooth=True, glOptions='opaque'
                )
                self.basemap_item.setDepthValue(-10)
                self.view.addItem(self.basemap_item)
                self._place_basemap()

        # 위성사진 위에 격자까지 겹치면 지저분하다
        showing = self.basemap_item is not None
        for item in self.grid_items:
            item.setVisible(not showing)
        # 밝은 배경에서는 가산 혼합이 4방위 색을 씻어내므로 같이 바꿔준다
        self._refresh_line_mode()
        self.fit_camera()

    def _refresh_line_mode(self):
        # 위성사진이 깔려 있으면 깊이 테스트를 끈 오버레이 모드로 (위 상수 주석 참고)
        mode = OVERLAY_GL_OPTIONS if self.basemap_item else self._theme_line_mode
        for item in self.line_items:
            item.setGLOptions(mode)
        # 현재 위치 마커도 같이 - 안 그러면 마커만 사진에 파묻힌다
        self.marker.setGLOptions(mode if self.basemap_item else "opaque")

    # 비행 시작 직후 경로가 한 점일 때 무한히 확대되지 않도록 하는 하한
    MIN_FOLLOW_SPAN = 200.0
    FOLLOW_MARGIN = 1.5

    def _distance_for_span(self, span):
        """주어진 폭이 화면에 담기는 카메라 거리.

        ! 이 패널은 가로세로비가 3:1이 넘는 납작한 띠다. pyqtgraph의 fov는
          세로 기준이라 거리를 고정해두면 세로가 먼저 잘려서, 가장자리가
          안 보이고 그냥 깔린 사진처럼 보인다.
        """
        width = max(self.view.width(), 1)
        height = max(self.view.height(), 1)
        aspect = width / height
        half_fov = math.radians(self.view.opts["fov"] / 2)
        # 세로로 담고, 세로로 긴 패널이면 가로 쪽이 더 빡빡하므로 그쪽에 맞춘다
        return span / (2 * math.tan(half_fov)) * max(1.0, 1.0 / aspect)

    def _content_bounds(self):
        """지금 담아야 할 범위 (중심, 폭). 경로가 없으면 발사장 주변 최소 화각."""
        if self._path_count == 0:
            return (np.zeros(3, dtype=np.float32),
                    self.MIN_FOLLOW_SPAN * self.FOLLOW_MARGIN)
        points = self._path[:self._path_count]
        low = points.min(axis=0)
        high = points.max(axis=0)
        center = (low + high) / 2.0
        span = max(float((high - low).max()), self.MIN_FOLLOW_SPAN) * self.FOLLOW_MARGIN
        return center, span

    def toggle_top_view(self, top):
        """북쪽 위 정사 시점 <-> 비스듬한 3D 시점.

        ! elevation 90은 up 벡터가 퇴화해 화면이 뒤집히므로 89.9를 쓴다.
          azimuth 270이라야 북(+Y)이 화면 위로 온다.
        """
        self._view_angle = (89.9, 270) if top else (35, 45)
        center, span = self._content_bounds()
        elevation, azimuth = self._view_angle
        self.view.setCameraPosition(
            pos=QtGui.QVector3D(*(float(v) for v in center)),
            distance=self._distance_for_span(span),
            elevation=elevation, azimuth=azimuth,
        )
        self._follow_state = None       # FOLLOW가 켜져 있으면 다음 프레임에 다시 맞춘다

    def fit_camera(self):
        """비행 데이터가 없을 때의 초기 화각 - 발사장 주변만 당겨서 잡는다.

        첫 수신 때 화각이 튀지 않도록 follow_camera()의 최소 화각과 같은 값을 쓴다.
        TOP이 켜져 있으면 그 각도를 유지한다.
        """
        elevation, azimuth = self._view_angle
        self.view.setCameraPosition(
            pos=QtGui.QVector3D(0.0, 0.0, 0.0),
            distance=self._distance_for_span(self.MIN_FOLLOW_SPAN * self.FOLLOW_MARGIN),
            elevation=elevation, azimuth=azimuth,
        )
        self._follow_state = None

    def follow_camera(self):
        """경로 전체가 들어오도록 카메라를 따라 움직인다.

        발사 지점과 현재 위치를 함께 담되 여백은 좁게 - 비행이 커지는 만큼만
        서서히 빠진다. 각도는 건드리지 않으므로 TOP과 같이 켜둘 수 있다.
        """
        if not self.follow_check.isChecked() or self._path_count == 0:
            return

        center, span = self._content_bounds()
        distance = self._distance_for_span(span)

        # 매 프레임 조금씩 흔들리면 눈이 피로하므로 변화가 클 때만 반영한다
        previous = self._follow_state
        if previous is not None:
            old_center, old_distance = previous
            moved = float(np.abs(center - old_center).max())
            if moved < span * 0.02 and abs(distance - old_distance) < old_distance * 0.02:
                return
        self._follow_state = (center.copy(), distance)

        self.view.setCameraPosition(
            pos=QtGui.QVector3D(float(center[0]), float(center[1]), float(center[2])),
            distance=distance,
        )

    def eventFilter(self, obj, event):
        # 뷰를 직접 조작하면 추종을 멈춘다 - 안 그러면 카메라와 싸우게 된다
        if obj is self.view and event.type() in (
            QtCore.QEvent.MouseButtonPress, QtCore.QEvent.Wheel
        ):
            if self.follow_check.isChecked():
                self.follow_check.setChecked(False)
        return super().eventFilter(obj, event)

    def site_offset(self, meta):
        """원점에서 본 지점 중심까지의 거리(m)와, 원점이 그 영상 안에 드는지."""
        dx, dy = self.latlon_to_m(
            self.origin_lat, self.origin_lon,
            meta["center_lat"], meta["center_lon"]
        )
        scale = meta["meters_per_pixel"]
        inside = (abs(dx) <= meta["width_px"] * scale / 2 and
                  abs(dy) <= meta["height_px"] * scale / 2)
        return math.hypot(dx, dy), inside

    def _autoselect_site(self):
        """첫 GNSS 수신 위치를 담고 있는 지점으로 자동 전환한다.

        발사 지점을 미리 정확히 몰라도 된다 - 영상은 위경도로 georeference
        되어 있어서, 원점이 정해지면 그만큼 밀어 맞추면 그만이다. 여러 지점을
        받아뒀다면 실제로 켜진 자리를 덮는 것을 골라준다.
        """
        if not self.basemap_sites or self.origin_lat is None:
            return

        current = self.basemap_meta
        if current is not None and self.site_offset(current)[1]:
            return                            # 지금 것이 이미 원점을 덮는다

        covering = [(self.site_offset(m)[0], m)
                    for m in self.basemap_sites if self.site_offset(m)[1]]
        if not covering:
            print("[basemap] 경고: 현재 위치를 덮는 위성영상이 없습니다. "
                  "fetch_basemap.py 로 이 좌표를 받으세요.")
            return

        _, best = min(covering, key=lambda pair: pair[0])
        index = self.basemap_combo.findData(best["name"])
        if index >= 0:
            # setCurrentIndex가 set_basemap을 부르고, 거기서 _place_basemap까지 간다
            self.basemap_combo.setCurrentIndex(index)

    def _place_basemap(self):
        """영상을 원점 기준으로 정렬한다.

        캐시는 발사장 좌표를 중심으로 받아뒀고, 화면 원점은 첫 GNSS 수신 위치다.
        둘이 다를 수 있으므로 원점이 잡히면 그 차이만큼 밀어준다.
        """
        if self.basemap_item is None:
            return
        meta = self.basemap_meta
        scale = meta["meters_per_pixel"]

        shift_x = shift_y = 0.0
        if self.origin_lat is not None:
            # 원점에서 본 캐시 중심의 위치
            shift_x, shift_y = self.latlon_to_m(
                self.origin_lat, self.origin_lon,
                meta["center_lat"], meta["center_lon"]
            )

        self.basemap_item.resetTransform()
        # 이미지 y는 아래로 증가, 지도 y(북)는 위로 증가하므로 y를 뒤집는다
        self.basemap_item.scale(scale, -scale, 1)
        self.basemap_item.translate(
            meta["offset_x_m"] + shift_x,
            meta["offset_y_m"] + shift_y,
            -0.5,                       # 경로/축보다 살짝 아래
        )

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
            # 실제 발사 위치를 담고 있는 지점으로 갈아타고 영상을 정렬한다
            self._autoselect_site()
            self._place_basemap()

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
            f"ALT {alt:.2f} m   DIST {distance:.1f} m\n"
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
            track = self._path[:count]
            self.path_line.setData(pos=track, color=self._trail_colors[-count:])
        if count >= 1:
            self.marker.resetTransform()
            self.marker.translate(*self._path[count - 1])
        self.overlay_label.setText(self._overlay_text)
        self.follow_camera()

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
        self._place_basemap()
        self.fit_camera()
        self.redraw()

    def apply_theme(self, theme):
        self.overlay_label.setStyleSheet(overlay_style(theme, 18))
        self.view.setBackgroundColor(theme["gl_bg"])

        # ! 위성사진은 밝아서 가산 혼합이면 4방위 색이 전부 흰색으로 씻겨나간다.
        #   배경이 깔려 있으면 테마와 무관하게 translucent로 합성해 색을 지킨다.
        self._theme_line_mode = theme["gl_line_mode"]
        self._refresh_line_mode()

        if self.basemap_item is not None:
            return                      # 위성사진이 깔려 있으면 격자는 계속 꺼둔다
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
    replay_progress = QtCore.pyqtSignal(int, int)
    connection_changed = QtCore.pyqtSignal(bool, str)


# === FlightStatsPanel: 비행 중 유의미한 값(최대치/이벤트 시각) 표시 ===
class FlightStatsPanel(QtWidgets.QWidget):
    # (키, 라벨, 강조 여부). MAX ALT는 칼만 고도(kf_alt) 기준 - 기압/GNSS보다
    # 융합된 최선 추정값이라 실제 도달 고도로 읽기에 적절하다.
    ROWS = [
        ("max_alt", "MAX ALT", True),
        # VZ는 부호를 살려 두 방향을 따로 잡는다 - 하강률(↓)이 낙하산 성능이라
        # 상승 최대치 하나로 뭉뚱그리면 정작 필요한 값이 사라진다
        ("max_vz_up", "MAX VZ ↑", False),
        ("max_vz_down", "MAX VZ ↓", False),
        ("max_az", "MAX AZ", False),
        # |a|는 축 부호/장착 방향과 무관한 합성 가속도라 AZ와 나란히 본다
        ("max_a", "MAX |a|", False),
        # PREDICTED APOGEE는 최대치도 이벤트도 아닌 '지금 예측값'이라
        # 전용 그래프(PREDICTED APOGEE)에서만 본다.
        # LAUNCH T는 자기 자신 기준이라 늘 T+0.00 - 표시할 정보가 없다.
        ("deploy_t", "DEPLOY T", False),
        ("ground_t", "GROUND T", False),
    ]
    # 이벤트 시각은 패킷 절대시각이 아니라 발사(LAUNCH) 기준 경과시간으로 읽는다.
    # DEPLOY T가 곧 "발사부터 사출까지 걸린 시간"이 되어 그대로 비행 성능이 된다.

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
        self.launch_time = None
        self.max_az = None
        self.max_a = None
        self.max_alt = None
        self.max_vz_up = None
        self.max_vz_down = None
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
                # 표시하지는 않지만 DEPLOY/GROUND의 기준점이라 반드시 잡아둔다
                self.launch_time = d["time"]
            elif state in ("DEPLOY", "GROUND"):
                key = "deploy_t" if state == "DEPLOY" else "ground_t"
                if self.launch_time is None:
                    # 발사 전이를 못 받았으면 기준이 없다 - 절대시각을 쓰면 오독한다
                    self._set(key, "no launch ref")
                else:
                    self._set(key, f"T+{d['time'] - self.launch_time:.2f} s")
            self.prev_state = state

        # 최대치 갱신 (라벨은 값이 바뀔 때만 setText)
        # accel의 부호 규약이 기체 장착 방향에 달려 있어 크기로 비교한다
        az = abs(d["az"])
        if self.max_az is None or az > self.max_az:
            self.max_az = az
            self._set("max_az", f"{az:.2f} g")

        a_mag = math.sqrt(d["ax"] ** 2 + d["ay"] ** 2 + d["az"] ** 2)
        if self.max_a is None or a_mag > self.max_a:
            self.max_a = a_mag
            self._set("max_a", f"{a_mag:.2f} g")

        if self.max_alt is None or d["kf_alt"] > self.max_alt:
            self.max_alt = d["kf_alt"]
            self._set("max_alt", f"{d['kf_alt']:.1f} m")

        # 상승/하강을 따로 - 하강은 음수라 최솟값이 곧 최대 하강률이다.
        # 표시할 때만 부호를 떼어 두 줄의 자릿수를 맞춘다.
        vz = d["kf_vel"]
        if self.max_vz_up is None or vz > self.max_vz_up:
            self.max_vz_up = vz
            self._set("max_vz_up", f"{vz:.1f} m/s")

        if self.max_vz_down is None or vz < self.max_vz_down:
            self.max_vz_down = vz
            self._set("max_vz_down", f"{abs(vz):.1f} m/s")


# === PlotControlPanel: 그래프 표시 선택 + 지도 조작 + 리플레이 ===
class PlotControlPanel(QtWidgets.QWidget):
    SPEEDS = [("x1", 1.0), ("x2", 2.0), ("x5", 5.0), ("x10", 10.0), ("x50", 50.0)]

    def __init__(self, graphs, path_view):
        """graphs: [(라벨, TimeSeriesGraph, 기본 표시 여부), ...]
        path_view: 3D FLIGHT PATH 뷰 - 그쪽 조작부(MAP/FOLLOW/TOP)를 여기로 가져온다

        FLIGHT STATS + SENSOR HEALTH 두 칸 너비를 쓰는 가로 띠라서, 세로로
        쌓지 않고 한 줄에 늘어놓는다. 그만큼 그래프 영역이 넓어진다.
        """
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(4)

        title = QtWidgets.QLabel("PLOT CONTROL  ·  REPLAY")
        title.setObjectName("panelTitle")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(30)
        layout.addWidget(title)

        # --- 그래프 토글 (한 줄) ---
        plot_row = QtWidgets.QHBoxLayout()
        plot_row.setSpacing(10)
        self._graph_boxes = []
        for label, graph, default_on in graphs:
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setObjectName("plotToggle")
            checkbox.setChecked(default_on)
            graph.setVisible(default_on)
            # 숨긴 그래프는 redraw()가 통째로 건너뛰므로 꺼두면 그만큼 가벼워진다
            checkbox.toggled.connect(graph.setVisible)
            checkbox.toggled.connect(self._enforce_graph_limit)
            plot_row.addWidget(checkbox)
            self._graph_boxes.append(checkbox)
        self._enforce_graph_limit()
        plot_row.addStretch()
        layout.addLayout(plot_row)

        # --- 3D FLIGHT PATH 조작부 (그쪽 헤더에서 옮겨옴) ---
        # (토글 제한 로직은 _enforce_graph_limit 참조)
        map_row = QtWidgets.QHBoxLayout()
        map_row.setSpacing(10)
        map_label = QtWidgets.QLabel("MAP")
        map_label.setObjectName("statsKey")
        map_row.addWidget(map_label)
        map_row.addWidget(path_view.basemap_combo)
        map_row.addSpacing(8)
        map_row.addWidget(path_view.follow_check)
        map_row.addWidget(path_view.top_button)
        map_row.addStretch()
        layout.addLayout(map_row)

        # --- 리플레이 (한 줄) ---
        replay_row = QtWidgets.QHBoxLayout()
        replay_row.setSpacing(8)
        self.open_button = QtWidgets.QPushButton("OPEN LOG")
        # PAUSE/STOP을 하나로 - 정지는 슬라이더를 처음으로 끌면 되므로 버튼이 따로 필요없다
        self.play_button = QtWidgets.QPushButton("❚❚")
        self.play_button.setCheckable(True)
        self.play_button.setFixedWidth(52)
        self.play_button.setToolTip("재생 / 일시정지")
        self.play_button.setEnabled(False)
        replay_row.addWidget(self.open_button)
        replay_row.addWidget(self.play_button)

        speed_label = QtWidgets.QLabel("SPEED")
        speed_label.setObjectName("statsKey")
        self.speed_combo = QtWidgets.QComboBox()
        for name, _ in self.SPEEDS:
            self.speed_combo.addItem(name)
        self.speed_combo.setFixedWidth(84)
        replay_row.addWidget(speed_label)
        replay_row.addWidget(self.speed_combo)
        replay_row.addStretch()
        layout.addLayout(replay_row)

        # 진행 슬라이더 - 끌어서 원하는 시점으로 이동
        seek_row = QtWidgets.QHBoxLayout()
        seek_row.setSpacing(8)
        self.seek_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.seek_slider.setRange(0, 0)
        self.seek_slider.setEnabled(False)
        self.seek_slider.setToolTip("끌어서 원하는 시점으로 이동")
        seek_row.addWidget(self.seek_slider, 1)
        # ! 고정폭 폰트 + 고정 자리수. 비례 폰트로 두면 숫자가 바뀔 때마다
        #   라벨 폭이 출렁여 슬라이더 길이까지 같이 흔들린다.
        self.progress_label = QtWidgets.QLabel()
        self.progress_label.setObjectName("seekTime")
        font = QtGui.QFont("Consolas")
        font.setStyleHint(QtGui.QFont.Monospace)
        font.setPixelSize(15)
        font.setBold(True)
        self.progress_label.setFont(font)
        self.progress_label.setFixedWidth(
            QtGui.QFontMetrics(font).horizontalAdvance("0000.0 / 0000.0 s") + 12
        )
        self.progress_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.set_progress(None, None)
        seek_row.addWidget(self.progress_label)
        layout.addLayout(seek_row)

    def set_progress(self, elapsed, total):
        if elapsed is None:
            self.progress_label.setText("   -   /    -   ")
        else:
            self.progress_label.setText(f"{elapsed:6.1f} / {total:6.1f} s")

    def speed(self):
        return self.SPEEDS[self.speed_combo.currentIndex()][1]

    def _enforce_graph_limit(self):
        """동시에 켤 수 있는 그래프를 MAX_VISIBLE_GRAPHS개로 묶는다.

        ! 가로 스플리터에 그래프를 5개 이상 밀어넣으면 각 칸이 최소폭 밑으로
          눌려 축과 헤더가 겹친다. 한도에 닿으면 꺼진 체크박스를 비활성화해서
          애초에 켜지지 않게 한다 (켠 뒤 되돌리면 깜빡여서 더 나쁘다).
        """
        checked = sum(box.isChecked() for box in self._graph_boxes)
        at_limit = checked >= MAX_VISIBLE_GRAPHS
        for box in self._graph_boxes:
            box.setEnabled(box.isChecked() or not at_limit)
            box.setToolTip(
                f"그래프는 최대 {MAX_VISIBLE_GRAPHS}개까지 동시에 표시합니다."
                if at_limit and not box.isChecked() else ""
            )


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
            # FLIGHT STATS와 같은 키 스타일을 써서 두 패널의 글자를 통일한다
            name_label = QtWidgets.QLabel(display_name)
            name_label.setObjectName("statsKey")
            value_label = QtWidgets.QLabel("NO DATA")
            value_label.setAlignment(QtCore.Qt.AlignCenter)
            value_label.setMinimumWidth(112)
            value_label.setMinimumHeight(30)
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
            f"background-color: {color}; color: white; font-size: 18px; "
            "font-weight: 700; border-radius: 5px; padding: 4px;"
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
        self.port        = port
        self.baud        = baud
        self.signals     = signals
        self.ser         = None
        self.running     = True
        self._last_flush = 0.0

        self.payload_format = '<B' + 'f'*22 + 'B'
        self.packet_size    = struct.calcsize(self.payload_format) + 1

        # M4 frame        : [0x7E][state 1B + float 22 + health 1B][0x0A]
        # 22 floats       : time, lat, lon, euler[3], accel[3], gyro[3], pos[3],
        #                   altitude, pressure, kalman[4], gnssAlt
        # packet_size     : 시작바이트를 뺀 91B (종료바이트 0x0A 포함)
        # ! telemetry_frame.h 의 PackedSensorData 90B와 반드시 일치해야 한다

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def _maybe_flush(self):
        # ! 프레임마다 flush하면 이 스레드가 막혀 시리얼 수신이 밀린다.
        #   1초 주기로 낮춘다 - 종료 시 flush는 stop_log_file()이 보장.
        now = time.monotonic()
        if now - self._last_flush >= 1.0:
            self._last_flush = now
            if log_file:
                log_file.flush()
            if csv_file:
                csv_file.flush()

    def write_log(self, line):
        """이벤트/시스템 메시지 -> .txt"""
        with log_lock:
            if logging_enabled and log_file:
                log_file.write(line + '\n')
                self._maybe_flush()

    def write_telemetry(self, d):
        """텔레메트리 프레임 -> .csv"""
        with log_lock:
            if logging_enabled and csv_file:
                wall = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                csv_file.write(format_csv_row(d, wall) + '\n')
                self._maybe_flush()

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

                    frame = {
                        "state": state_str, "time": time_val,
                        "lat": lat, "lon": lon, "gnss_alt": gnss_alt,
                        "roll": roll, "pitch": pitch, "yaw": yaw,
                        "ax": ax, "ay": ay, "az": az,
                        "gx": gx, "gy": gy, "gz": gz,
                        "px": px, "py": py, "pz": pz,
                        "alt": alt, "pressure": pressure,
                        "kf_alt": kalman_alt, "kf_vel": kalman_velo,
                        "apogee": predi_apo, "r2": r2,
                        "health": health,
                    }
                    self.write_telemetry(frame)
                    self.signals.telemetry.emit(frame)
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


# === LogReplayer: 저장된 로그를 다시 흘려보낸다 ===
# 하드웨어 없이 GUI를 점검하고, 비행 후 궤적을 되짚고, 발사 전 리허설을 한다.
# SerialReader와 같은 시그널을 쏘므로 화면 쪽은 실기와 구분하지 않는다.
LEGACY_LINE = re.compile(
    r"State:\s*(?P<state>\S+),\s*Time:\s*(?P<time>[-\d.]+)s,\s*"
    r"Lat:\s*(?P<lat>[-\d.]+),\s*Lon:\s*(?P<lon>[-\d.]+),\s*Alt:\s*(?P<alt>[-\d.na]+)m,\s*"
    r"P:\s*(?P<pressure>[-\d.na]+)hPa,\s*"
    r"Euler:\s*\((?P<roll>[-\d.na]+),\s*(?P<pitch>[-\d.na]+),\s*(?P<yaw>[-\d.na]+)\),\s*"
    r"Gyro:\s*\((?P<gx>[-\d.na]+),\s*(?P<gy>[-\d.na]+),\s*(?P<gz>[-\d.na]+)\),\s*"
    r"Accel:\s*\((?P<ax>[-\d.na]+),\s*(?P<ay>[-\d.na]+),\s*(?P<az>[-\d.na]+)\)g,\s*"
    r"Pos:\s*\((?P<px>[-\d.na]+),\s*(?P<py>[-\d.na]+),\s*(?P<pz>[-\d.na]+)\),\s*"
    r"Kalman:\s*\(alt=(?P<kf_alt>[-\d.na]+),\s*velo=(?P<kf_vel>[-\d.na]+),\s*"
    r"apo=(?P<apogee>[-\d.na]+),\s*r2=(?P<r2>[-\d.na]+)\),\s*"
    r"GnssAlt:\s*(?P<gnss_alt>[-\d.na]+)m,\s*Health:\s*0x(?P<health>[0-9A-Fa-f]+)"
)
FLOAT_KEYS = ("time", "lat", "lon", "gnss_alt", "roll", "pitch", "yaw",
              "ax", "ay", "az", "gx", "gy", "gz", "px", "py", "pz",
              "alt", "pressure", "kf_alt", "kf_vel", "apogee", "r2")


def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return float("nan")           # 로그에 'nan'이 그대로 적힌 경우가 있다


# 기체 SD카드가 직접 뱉는 raw CSV(time_ms 기준) 열 이름 → 프레임 키.
# 지상국 CSV와 이름이 다를 뿐 내용은 같아서, 열 이름만 갈아끼우면 그대로 재생된다.
RAW_CSV_MAP = {
    "lat": "lat", "lon": "lon",
    "euler_x": "roll", "euler_y": "pitch", "euler_z": "yaw",
    "accel_x": "ax", "accel_y": "ay", "accel_z": "az",
    "gyro_x": "gx", "gyro_y": "gy", "gyro_z": "gz",
    "pos_x": "px", "pos_y": "py", "pos_z": "pz",
    "altitude": "alt", "pressure": "pressure",
    "kf_alt": "kf_alt", "kf_vel": "kf_vel",
    "apogee": "apogee", "r2": "r2", "gnss_alt": "gnss_alt",
}


def _raw_health(frame, note):
    """Decode M4 healthNote tokens; infer values only for older unannotated logs."""
    # A dead sensor retains its previous values. Explicit notes, rather than
    # coordinates/pressure, determine health for the current firmware's CSV.
    clear_bits = {
        "imu_dead": 0x05,       # IMU fresh + alive
        "imu_stale": 0x01,
        "baro_dead": 0x0A,      # baro fresh + alive
        "baro_stale": 0x02,
        "gnss_dead": 0x10,
        "no_baro_ref": 0x20,
        "vz_off": 0xC0,         # trusted + usable
        "vz_settle": 0x80,      # temporarily unusable; still trusted
    }
    tokens = {token.strip() for token in note.split("|")}
    if "ok" in tokens or tokens.intersection(clear_bits):
        health = 0xFF
        for token in tokens:
            health &= ~clear_bits.get(token, 0)
        return health

    # Legacy files without encoded health keep the previous best-effort view.
    health = 0x01 | 0x02 | 0x04 | 0x08          # imu/baro fresh + alive
    if frame.get("lat") or frame.get("lon"):
        health |= 0x10                          # gnss_alive
    if frame.get("pressure") == frame.get("pressure"):
        health |= 0x20                          # baro_ref
    return health


def read_log_frames(path):
    """로그 파일에서 텔레메트리 프레임을 뽑는다. 지상국 CSV·기체 raw CSV·옛 .txt 지원.

    옛 형식도 읽는 이유: 이미 쌓인 실기 로그가 CSV 전환 전 것들이라
    그것들을 못 읽으면 리플레이의 의미가 절반으로 준다.
    """
    frames = []
    with open(path, encoding="utf-8", errors="ignore") as handle:
        first = handle.readline()
        if first.startswith("wall,"):
            columns = first.rstrip("\n").split(",")
            for row in csv.DictReader(handle, fieldnames=columns):
                frame = {}
                for name, key, _ in CSV_FIELDS:
                    if key is None:
                        continue
                    raw = row.get(name)
                    if key == "state":
                        frame[key] = raw or "UNKNOWN"
                    elif key == "health":
                        frame[key] = int(raw) if raw else 0
                    else:
                        frame[key] = _to_float(raw)
                if frame.get("time") == frame.get("time"):     # NaN 아님
                    frames.append(frame)
        elif first.startswith("time_ms,"):
            columns = first.rstrip("\n").split(",")
            for row in csv.DictReader(handle, fieldnames=columns):
                frame = {key: float("nan") for key in FLOAT_KEYS}
                frame["time"] = _to_float(row.get("time_ms")) / 1000.0
                # 기체는 소문자로 적는다. 화면 쪽 PHASE_STYLES는 대문자 기준.
                frame["state"] = (row.get("state") or "UNKNOWN").strip().upper()
                for name, key in RAW_CSV_MAP.items():
                    if name in row:
                        frame[key] = _to_float(row.get(name))
                frame["health"] = _raw_health(frame, (row.get("note") or "").strip())
                if frame["time"] == frame["time"]:             # NaN 아님
                    frames.append(frame)
        else:
            handle.seek(0)
            for line in handle:
                match = LEGACY_LINE.search(line)
                if not match:
                    continue
                group = match.groupdict()
                frame = {key: _to_float(group[key]) for key in FLOAT_KEYS}
                frame["state"] = group["state"]
                frame["health"] = int(group["health"], 16)
                frames.append(frame)
    return frames


class LogReplayer(threading.Thread):
    def __init__(self, frames, signals, speed=1.0):
        super().__init__(daemon=True)
        self.frames = frames
        self.signals = signals
        self.speed = speed
        self.running = True
        self.paused = False
        self.index = 0
        self.previous = None

    def stop(self):
        self.running = False

    def run(self):
        # ! connection_changed는 쏘지 않는다. 그걸 쏘면 자동 로깅이 걸려서
        #   재생 중인 옛 로그를 새 파일로 다시 기록해버린다.
        try:
            while self.running and self.index < len(self.frames):
                if self.paused:
                    time.sleep(0.05)
                    continue

                frame = self.frames[self.index]
                # 패킷 시각 간격만큼 기다려 실제 속도로 재생한다
                if self.previous is not None:
                    delay = (frame["time"] - self.previous) / max(self.speed, 0.01)
                    if 0 < delay < 5.0:
                        time.sleep(delay)
                self.previous = frame["time"]

                self.signals.telemetry.emit(frame)
                self.signals.attitude.emit(
                    frame["roll"], frame["pitch"], frame["yaw"]
                )
                self.signals.state.emit(frame["state"])
                self.signals.position.emit(
                    frame["lat"], frame["lon"], frame["alt"]
                )
                self.signals.health.emit(int(frame["health"]))
                self.signals.replay_progress.emit(self.index + 1, len(self.frames))
                self.index += 1
        finally:
            self.signals.replay_progress.emit(len(self.frames), len(self.frames))


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
        self._tplus_state = None
        self._tlm_count = 0        # 1초 창에 쌓이는 프레임 수
        self._tlm_rate  = 0        # 직전 1초의 수신율 [Hz] - 링크 상태 색의 근거
        self._tlm_display = None
        self._connected = False
        self._connection_text = "DISCONNECTED"
        self._log_filename = None
        self._pending_log = []

        self.attitude_view = RocketViewer()
        self.path_view = RocketPathViewer()
        # M7의 accel[3]은 중력이 제거된 선형가속도를 g 단위로 전송한다.
        # Y라벨은 그래프 세로 높이 안에 들어가도록 짧게 (길면 단위가 잘림)
        # |a|(합성 가속도)는 장착 방향과 무관한 스칼라라 축 부호를 몰라도 읽힌다
        self.accel_view = TimeSeriesGraph(
            "ACCELERATION", "Accel (g)",
            [("Ax", '#ff4d4d'), ("Ay", '#4ade80'), ("Az", '#5b8cff'),
             ("|a|", '#f5c451')]
        )
        self.gyro_view = TimeSeriesGraph(
            "ANGULAR RATE", "Gyro (deg/s)",
            [("Gx", '#ff4d4d'), ("Gy", '#4ade80'), ("Gz", '#5b8cff')]
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
            self.accel_view, self.gyro_view, self.alt_view,
            self.vz_view, self.apogee_view,
        ]

        self.stats_panel = FlightStatsPanel()
        self.health_panel = HealthPanel()
        self.replay_thread = None
        self.replay_frames = []
        self._seek_resume = False
        self.plot_panel = PlotControlPanel(
            # 가로 띠라 폭이 빠듯하다 - 전체 제목은 그래프 헤더에 있으니 여기선 축약
            [
                ("ACCEL", self.accel_view, True),
                ("GYRO", self.gyro_view, False),
                ("ALTITUDE", self.alt_view, True),
                ("VZ", self.vz_view, False),
                ("APOGEE", self.apogee_view, False),
            ],
            self.path_view,
        )

        self.build_ui()
        self.connect_signals()
        self.refresh_ports()
        self.apply_theme()

        # 모든 뷰의 렌더를 이 타이머 하나가 몰아서 처리한다
        self.render_timer = QtCore.QTimer(self)
        self.render_timer.timeout.connect(self.redraw_views)
        self.render_timer.start(RENDER_INTERVAL_MS)

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

        # 오른쪽 열 구성
        #   FLIGHT STATS | SENSOR HEALTH
        #   PLOT CONTROL · REPLAY          <- 두 칸 너비로 확장
        # 컨트롤을 가로 띠로 눕혀 세로를 아끼면 그만큼 그래프가 커진다.
        panels_row = QtWidgets.QHBoxLayout()
        panels_row.setContentsMargins(0, 0, 0, 0)
        panels_row.setSpacing(6)
        panels_row.addWidget(self.stats_panel, 1)
        panels_row.addWidget(self.health_panel, 1)

        right_column = QtWidgets.QWidget()
        right_column_layout = QtWidgets.QVBoxLayout(right_column)
        right_column_layout.setContentsMargins(0, 0, 0, 0)
        right_column_layout.setSpacing(6)
        right_column_layout.addLayout(panels_row, 1)
        right_column_layout.addWidget(self.plot_panel)

        bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        bottom_splitter.addWidget(log_container)
        bottom_splitter.addWidget(right_column)
        right_column.setMaximumWidth(640)
        bottom_splitter.setSizes([1160, 620])

        # 켜져 있는 그래프끼리 가로로 나눠 쓴다 (영역 높이는 그대로)
        graph_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        for graph in self.graphs:
            graph_splitter.addWidget(graph)

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        main_splitter.addWidget(top_splitter)
        main_splitter.addWidget(graph_splitter)
        main_splitter.addWidget(bottom_splitter)
        # 컨트롤을 가로 띠로 눕혀 아낀 세로를 그래프에 준다
        main_splitter.setSizes([440, 380, 380])
        main_splitter.setStretchFactor(0, 3)
        main_splitter.setStretchFactor(1, 4)
        main_splitter.setStretchFactor(2, 3)
        root.addWidget(main_splitter)

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_serial)
        self.start_log_button.clicked.connect(self.start_logging)
        self.stop_log_button.clicked.connect(self.stop_logging)
        self.reset_button.clicked.connect(self.reset_all)
        self.theme_button.clicked.connect(self.toggle_theme)
        self.plot_panel.open_button.clicked.connect(self.open_replay)
        self.plot_panel.play_button.toggled.connect(self.set_replay_paused)
        self.plot_panel.speed_combo.currentIndexChanged.connect(
            lambda _: self._apply_replay_speed()
        )
        self.plot_panel.seek_slider.sliderPressed.connect(self._seek_begin)
        self.plot_panel.seek_slider.sliderReleased.connect(self._seek_end)

    # === 로그 리플레이 ===
    def open_replay(self):
        if self.serial_thread and self.serial_thread.is_alive():
            self.append_log("[REPLAY] 시리얼 연결 중에는 재생할 수 없습니다.")
            return
        self.stop_replay()

        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open flight log", os.path.join(BASE_DIR, "logs"),
            "Flight logs (*.csv *.txt);;All files (*)"
        )
        if not path:
            return

        try:
            frames = read_log_frames(path)
        except OSError as error:
            self.append_log(f"[REPLAY] 읽기 실패: {error}")
            return
        if not frames:
            self.append_log(f"[REPLAY] 텔레메트리를 찾지 못했습니다: {os.path.basename(path)}")
            return

        self.reset_all()
        self.append_log(
            f"[REPLAY] {os.path.basename(path)} · {len(frames)} frames · "
            f"{frames[-1]['time'] - frames[0]['time']:.1f}s"
        )
        self.replay_frames = frames
        self.replay_thread = LogReplayer(
            frames, self.signals, self.plot_panel.speed()
        )
        self.replay_thread.start()
        self.plot_panel.play_button.setEnabled(True)
        self.plot_panel.play_button.setChecked(False)
        self.plot_panel.play_button.setText("❚❚")
        self.plot_panel.seek_slider.setRange(0, len(frames) - 1)
        self.plot_panel.seek_slider.setValue(0)
        self.plot_panel.seek_slider.setEnabled(True)
        self.connect_button.setEnabled(False)
        self._connection_text = f"REPLAY  {os.path.basename(path)}"
        self.connection_label.setText(self._connection_text)
        self.connection_label.setStyleSheet(
            "color: #22d3ee; font-weight: bold; padding: 5px;"
        )

    def _apply_replay_speed(self):
        if self.replay_thread and self.replay_thread.is_alive():
            self.replay_thread.speed = self.plot_panel.speed()

    def set_replay_paused(self, paused):
        if self.replay_thread and self.replay_thread.is_alive():
            self.replay_thread.paused = paused
        self.plot_panel.play_button.setText("▶" if paused else "❚❚")

    def _seek_begin(self):
        """슬라이더를 잡는 동안은 재생을 멈춘다 - 안 그러면 손잡이와 싸운다."""
        self._seek_resume = not self.plot_panel.play_button.isChecked()
        if self.replay_thread:
            self.replay_thread.paused = True

    def _seek_end(self):
        self.seek_replay(self.plot_panel.seek_slider.value())
        if self.replay_thread and self._seek_resume:
            self.replay_thread.paused = False

    def seek_replay(self, index):
        """지정 프레임으로 이동. 그때까지의 궤적을 다시 쌓아 화면 상태를 맞춘다."""
        if not (self.replay_thread and self.replay_frames):
            return
        index = max(0, min(index, len(self.replay_frames) - 1))

        # ! 시그널로 흘리면 TELEMETRY 로그에 수천 줄이 쌓여 수 초가 걸린다.
        #   화면 상태 복원에 필요한 위젯만 직접 먹인다 (로그 뷰는 건너뜀).
        self.reset_all()
        for frame in self.replay_frames[:index]:
            self.path_view.update_position(frame["lat"], frame["lon"], frame["alt"])
            self.update_graphs(frame)
            self.stats_panel.update_stats(frame)
            self.update_mission(frame)
        last = self.replay_frames[index]
        self.signals.attitude.emit(last["roll"], last["pitch"], last["yaw"])
        self.signals.state.emit(last["state"])
        self.signals.health.emit(int(last["health"]))
        self.replay_thread.index = index
        self.replay_thread.previous = last["time"]
        self.redraw_views()

    def stop_replay(self):
        was_running = self.replay_thread is not None
        if self.replay_thread:
            self.replay_thread.stop()
            self.replay_thread = None
        self.plot_panel.play_button.setEnabled(False)
        self.plot_panel.play_button.setChecked(False)
        self.plot_panel.play_button.setText("❚❚")
        self.plot_panel.seek_slider.setEnabled(False)
        self.connect_button.setEnabled(True)
        if was_running and not self._connected:
            self._connection_text = "DISCONNECTED"
            self._style_connection()

    @QtCore.pyqtSlot(int, int)
    def update_replay_progress(self, index, total):
        slider = self.plot_panel.seek_slider
        if not slider.isSliderDown():
            slider.blockSignals(True)
            slider.setValue(index)
            slider.blockSignals(False)
        frames = self.replay_frames
        if frames:
            span = frames[-1]["time"] - frames[0]["time"]
            now = frames[min(index, len(frames) - 1)]["time"] - frames[0]["time"]
            self.plot_panel.set_progress(now, span)
        if index >= total:
            self.stop_replay()

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
        self._tplus_state = None
        self._style_tplus()

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
        self.signals.replay_progress.connect(self.update_replay_progress)
        self.signals.line_received.connect(self.append_log)
        self.signals.connection_changed.connect(
            self.update_connection_state
        )

    # === 렌더 / 미션 시계 ===
    def redraw_views(self):
        """모든 뷰의 재렌더를 여기 한 곳에 몰아둔다."""
        for graph in self.graphs:
            graph.redraw()
        self.attitude_view.redraw()
        self.path_view.redraw()

    @QtCore.pyqtSlot(dict)
    def update_graphs(self, d):
        t = d["time"]
        a_mag = math.sqrt(d["ax"] ** 2 + d["ay"] ** 2 + d["az"] ** 2)
        self.accel_view.update_data(t, d["ax"], d["ay"], d["az"], a_mag)
        self.gyro_view.update_data(t, d["gx"], d["gy"], d["gz"])
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
        self._style_tplus()
        self.set_phase(d["state"])

    def _style_tplus(self):
        """T+ 색으로 '지금 살아 있는 시계인지'를 보여준다.

        발사 전은 흐리게, 돌아가는 중은 액센트, 데이터가 끊기면 빨강.
        숫자만 봐서는 멈춘 건지 알 수 없으므로 색이 그 역할을 한다.
        """
        if self.launch_time is None:
            state = "idle"
        elif self._tlm_rate == 0:
            state = "stale"
        else:
            state = "live"
        if state == self._tplus_state:
            return
        self._tplus_state = state
        color = {"idle": THEMES[self.theme_name]["muted"],
                 "live": THEMES[self.theme_name]["accent"],
                 "stale": "#f87171"}[state]
        self.tplus_label.setStyleSheet(
            f"background: transparent; border: none; color: {color}; "
            "font-weight: 700; padding: 4px 8px;"
        )

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
        self._style_tplus()

    def _style_tlm(self):
        rate = self._tlm_rate
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
        self._tplus_state = None
        self._style_tplus()
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
        if self.replay_thread:
            self.replay_thread.stop()
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
