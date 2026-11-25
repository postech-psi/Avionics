"""PSILOG v1 reader/converter for RENEWAL 20261122; standard library only.

The 90-byte sensor payload and CRC record format match the tested BIN logger.
No output is written when opening a BIN for replay. Conversion uses exclusive
creation so an existing CSV/TXT is never overwritten.
"""
from __future__ import annotations

import argparse
import binascii
import csv
from dataclasses import dataclass
import math
import mmap
from pathlib import Path
import struct
import sys

FILE_HEADER = b"PSILOG\x01\x00"
RECORD_SYNC = b"\xa5\x5a"
RECORD_HEADER = struct.Struct("<2sBH")
RECORD_CRC = struct.Struct("<H")
SAMPLE = struct.Struct("<B22fB")
DATA_RECORD, EVENT_RECORD = 1, 2
MAX_EVENT_BYTES = 150
STATES = {0xBA: "PRELAUNCH", 0xBB: "LAUNCH", 0xBC: "DEPLOY", 0xBD: "GROUND"}
SAMPLE_KEYS = (
    "time", "lat", "lon", "roll", "pitch", "yaw", "ax", "ay", "az",
    "gx", "gy", "gz", "px", "py", "pz", "alt", "pressure", "kf_alt",
    "kf_vel", "apogee", "r2", "gnss_alt",
)
CSV_COLUMNS = (
    "time_ms", "state", "lat", "lon", "euler_x", "euler_y", "euler_z",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
    "pos_x", "pos_y", "pos_z", "altitude", "pressure", "kf_alt", "kf_vel",
    "apogee", "r2", "gnss_alt", "note",
)
PRECISIONS = (0, 7, 7, *(3,) * 13, 2, 3, 3, 3, 4, 1)


@dataclass
class DecodeStats:
    samples: int = 0
    events: int = 0
    skipped_bytes: int = 0
    invalid_times: int = 0


def records(data, stats):
    """Yield CRC-valid (kind, payload) and resynchronize after damaged bytes."""
    if data[:len(FILE_HEADER)] != FILE_HEADER:
        raise ValueError("Unsupported or incomplete PSILOG v1 BIN header")
    offset = len(FILE_HEADER)
    while offset < len(data):
        if offset + RECORD_HEADER.size + RECORD_CRC.size <= len(data):
            sync, kind, length = RECORD_HEADER.unpack_from(data, offset)
            start = offset + RECORD_HEADER.size
            end = start + length
            after = end + RECORD_CRC.size
            valid_length = ((kind == DATA_RECORD and length == SAMPLE.size)
                            or (kind == EVENT_RECORD and 0 < length <= MAX_EVENT_BYTES))
            if sync == RECORD_SYNC and valid_length and after <= len(data):
                expected = RECORD_CRC.unpack_from(data, end)[0]
                actual = binascii.crc_hqx(data[offset + 2:end], 0xFFFF)
                if expected == actual:
                    if kind == DATA_RECORD:
                        stats.samples += 1
                    else:
                        stats.events += 1
                    yield kind, data[start:end]
                    offset = after
                    continue
        after = data.find(RECORD_SYNC, offset + 1)
        if after == -1:
            after = len(data)
        stats.skipped_bytes += after - offset
        offset = after


def health_note(health):
    notes = []
    if not health & 0x04:
        notes.append("imu_dead")
    elif not health & 0x01:
        notes.append("imu_stale")
    if not health & 0x08:
        notes.append("baro_dead")
    elif not health & 0x02:
        notes.append("baro_stale")
    if not health & 0x10:
        notes.append("gnss_dead")
    if not health & 0x20:
        notes.append("no_baro_ref")
    if not health & 0x40:
        notes.append("vz_off")
    elif not health & 0x80:
        notes.append("vz_settle")
    return "|".join(notes) if notes else "ok"


def csv_row(payload):
    state, *values, health = SAMPLE.unpack(payload)
    fields = [format(value, f".{digits}f") for value, digits in zip(values, PRECISIONS)]
    fields.insert(1, STATES.get(state, "UNKNOWN").lower())
    fields.append(health_note(health))
    return fields


def sample_frame(payload):
    state, *values, health = SAMPLE.unpack(payload)
    frame = dict(zip(SAMPLE_KEYS, values))
    frame["time"] /= 1000.0
    frame["state"] = STATES.get(state, "UNKNOWN")
    frame["health"] = health
    return frame


def read_bin_frames(path):
    """Return replay frames, including text events in stored record order.

    Events are assigned to the next stored sensor frame (trailing events to
    the last frame). Their printed M7/M4 timestamps are retained; boot and
    mission clocks are never incorrectly sorted into one numeric time axis.
    """
    frames, pending = [], []
    stats = DecodeStats()
    with Path(path).open("rb") as source:
        if source.read(len(FILE_HEADER)) != FILE_HEADER:
            raise ValueError("Unsupported or incomplete PSILOG v1 BIN header")
        with mmap.mmap(source.fileno(), 0, access=mmap.ACCESS_READ) as data:
            for kind, payload in records(data, stats):
                if kind == EVENT_RECORD:
                    pending.extend(payload.decode("utf-8", errors="replace").splitlines())
                    continue
                frame = sample_frame(payload)
                if not math.isfinite(frame["time"]) or frame["time"] < 0:
                    stats.invalid_times += 1
                    continue
                if pending:
                    frame["_events_before"] = pending
                    pending = []
                frames.append(frame)
    if frames:
        if pending:
            frames[-1]["_events_after"] = pending
        summary = (f"[BIN] samples={stats.samples} events={stats.events} "
                   f"skipped_bytes={stats.skipped_bytes} invalid_times={stats.invalid_times}")
        frames[0].setdefault("_events_before", []).insert(0, summary)
    return frames


def convert(path):
    """Create flight_NNN.csv and flight_NNN.txt; never overwrite existing files."""
    path = Path(path)
    csv_path, txt_path = path.with_suffix(".csv"), path.with_suffix(".txt")
    created = []
    stats = DecodeStats()
    try:
        with path.open("rb") as source:
            if source.read(len(FILE_HEADER)) != FILE_HEADER:
                raise ValueError("Unsupported or incomplete PSILOG v1 BIN header")
            with mmap.mmap(source.fileno(), 0, access=mmap.ACCESS_READ) as data:
                with csv_path.open("x", encoding="utf-8", newline="") as telemetry_file:
                    created.append(csv_path)
                    with txt_path.open("xb") as event_file:
                        created.append(txt_path)
                        telemetry = csv.writer(telemetry_file, lineterminator="\n")
                        telemetry.writerow(CSV_COLUMNS)
                        for kind, payload in records(data, stats):
                            if kind == DATA_RECORD:
                                telemetry.writerow(csv_row(payload))
                            else:
                                event_file.write(payload)
    except BaseException:
        for output in created:
            output.unlink()
        raise
    return stats


def main(argv=None):
    arguments = argparse.ArgumentParser(description=__doc__)
    arguments.add_argument("bin_file", type=Path)
    options = arguments.parse_args(argv)
    try:
        stats = convert(options.bin_file)
    except (OSError, ValueError) as error:
        arguments.exit(1, f"Conversion failed: {error}\n")
    print(f"{options.bin_file.with_suffix('.csv')} ({stats.samples} rows)")
    print(f"{options.bin_file.with_suffix('.txt')} ({stats.events} events)")
    if stats.skipped_bytes:
        print(f"Warning: skipped {stats.skipped_bytes} damaged/incomplete bytes.", file=sys.stderr)


if __name__ == "__main__":
    main()
