"""Aircraft SD write/sync heartbeat, separate from the PC REC indicator."""
from dataclasses import dataclass
import re
import time

PATTERN = re.compile(r"\[SD\] (CHECKING|READY|RECORDING|STALE|DEGRADED|ERROR) "
                     r"n=(\d+) bytes=(\d+) age=(\d+) drop=(\d+)/(\d+) file=(-?\d+)(?:\s|$)")
COLORS = {"CHECKING": "#64748b", "READY": "#d97706", "RECORDING": "#16a34a",
          "STALE": "#d97706", "DEGRADED": "#d97706", "ERROR": "#dc2626",
          "NO DATA": "#64748b", "LINK LOST": "#d97706", "DISCONNECTED": "#64748b"}


@dataclass(frozen=True)
class Status:
    state: str
    samples: int
    byte_count: int
    age_ms: int
    sample_drops: int
    event_drops: int
    session: int

    @classmethod
    def parse(cls, text):
        match = PATTERN.search(text)
        if not match:
            return None
        state, *values = match.groups()
        values = [int(value) for value in values]
        samples, byte_count, age, drops, events, session = values
        # A file-open message, or an impossible heartbeat, must not light green.
        if state == "RECORDING":
            if not samples or byte_count < samples * 97 + 8 or session < 0:
                state = "ERROR"
            elif age > 1500:
                state = "STALE"
            elif drops or events:
                state = "DEGRADED"
        return cls(state, *values)


class Monitor:
    def __init__(self):
        self.status = None
        self.received_at = None
        self.disconnected = False

    def receive(self, text, now=None):
        status = Status.parse(text)
        if status is None:
            return False
        self.status = status
        self.received_at = time.monotonic() if now is None else now
        self.disconnected = False
        return True

    def display(self, now=None, replay=False):
        if self.disconnected:
            return "DISCONNECTED"
        if self.status is None:
            return "NO DATA"
        now = time.monotonic() if now is None else now
        if not replay and now - self.received_at > 3.5:
            return "LINK LOST"
        return self.status.state

    def tooltip(self):
        if self.status is None:
            return "기체 SD 기록 상태 보고를 기다리는 중입니다. PC REC와 별개입니다."
        s = self.status
        filename = f"flight_{s.session:03d}.bin" if s.session >= 0 else "세션 없음"
        return (f"기체 SD: {filename}\n"
                f"쓰기+sync 성공 센서 {s.samples:,}개 / {s.byte_count:,} bytes\n"
                f"마지막 센서 sync 경과 {s.age_ms}ms / 센서 유실 {s.sample_drops} / 이벤트 유실 {s.event_drops}\n"
                "READY: 부팅 쓰기/읽기 검사 통과, 센서 기록 대기\n"
                "RECORDING: 최근 센서 레코드 쓰기+sync 성공\n"
                "STALE: 최근 센서 sync 없음 / LINK LOST: 상태 보고 수신 없음\n"
                "파일시스템을 통한 확인이며 SD 내부 비휘발성 보존을 독립 측정한 것은 아닙니다.")
