# Portenta H7 QSPI recording

QSPI is an optional second PSILOG writer on M4. It has its own queue and runs
alongside SD. It can record with no SD card installed. A hard fault that
stops M4 or removes board power stops both writers. The default build remains
SD-only. Install `portenta_h7_m4_qspi` instead of `portenta_h7_m4` to enable QSPI
support. The usual `portenta_h7_m7` build is shared by both.

## 첫 설정과 복구

The M4 logger accepts the Arduino 16 MiB layout with MBR partition 4 at
7–14 MiB and an existing Mbed LittleFileSystem volume. The check rejects
out-of-range and overlapping entries. This region can already contain PLC or
other user data. The logger checks for an Avionics ownership marker before it
creates a session. Both firmware targets leave partition tables and formats
alone. [Arduino partition example](https://github.com/arduino/ArduinoCore-mbed/blob/4.5.0/libraries/STM32H747_System/examples/QSPIFormat/QSPIFormat.ino)

On a spare or secured bench board, use `portenta_h7_qspi_service` in the M7
slot, cold-boot into that image, and connect the Portenta USB port. The service
image runs without the flight application or M4. With pyserial from the ground
station requirements, run from the repository root.

```powershell
python ground-station/qspi_service.py COM7 info
python ground-station/qspi_service.py COM7 claim
python ground-station/qspi_service.py COM7 list
python ground-station/qspi_service.py COM7 get 0
```

Run `claim` once, only after identifying the dedicated empty volume. It creates
a small marker and refuses a populated volume. If `info` reports an unrecognized
layout or an unavailable filesystem, confirm the hardware and provision a
compatible empty user partition separately. Arduino's general formatter can
erase other partitions. Restore the regular M7 application after service.
`get` downloads the original BIN without replacing a local file. Open it with
**OPEN LOG** or [convert it to CSV and TXT](../../ground-station/README.md#replay-a-flight).

## 기록 시작과 종료

Connect the XBee ground station. **START QSPI** becomes available when the
board reports `IDLE`. Click it before the data you want begins. `ARMED` means
it created the file, and `RECORDING` means it synced sensor records. Check the
sample count in the QSPI status tooltip. **STOP QSPI** drains the queue and
closes the current file. A radio disconnect leaves the existing recording
running. There is no automatic start when SD fails, which avoids filling
flash without approval. Commands need a working ground-to-air XBee path and
rely on the configured radio network for access control.

QSPI stops six minutes after a session opens, or 30 seconds after it sees a
`LANDED` transition in that session, whichever comes first. **STOP QSPI**
closes it sooner. When the status returns to `IDLE`, **START QSPI** opens a new
numbered BIN if space remains. A later session started while already landed
has its own six-minute window. Closed files are preserved.

The 5 MiB per-file limit is a second guard against bursts and malformed data.
It closes the file and returns to `IDLE`. `FULL` means the volume or file
numbers are exhausted. `ERROR` means the writer stopped on an I/O failure.
`STALE` means no recent sensor sync; `DEGRADED` means records were dropped.
The stop reason is reported in status and, on clean close, in the BIN footer.

## 벤치 확인

With the board secured and actuators disconnected, check a start/stop cycle
and restart into a new file without a board reset. Repeat with the SD card
absent and confirm QSPI sample
counts rise. Compare reset and controlled power-cut results on QSPI and SD.
LittleFS commits file data at sync/close and is designed for power-loss
recovery. The writer aims to sync every 200 ms, but backlog and write latency
make actual data loss board-specific. [Mbed LittleFS](https://github.com/ARMmbed/mbed-os/blob/master/storage/filesystem/littlefs/README.md)
