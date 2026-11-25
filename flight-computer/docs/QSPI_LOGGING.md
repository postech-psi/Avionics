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

A session caps at 5 MiB, roughly nine minutes at 100 Hz before text events.
`LIMIT`, `FULL`, or `ERROR` ends QSPI recording for that boot. Sessions are
never overwritten or automatically deleted. `DEGRADED` reports local queue
drops and `STALE` means no recent sensor sync. Both media can miss frames
lost upstream in the M7→M4 stream.

## 벤치 확인

With the board secured and actuators disconnected, check a start/stop cycle
and export the file. Repeat with the SD card absent and confirm QSPI sample
counts rise. Compare reset and controlled power-cut results on QSPI and SD.
LittleFS commits file data at sync/close and is designed for power-loss
recovery. The writer aims to sync every 200 ms, but backlog and write latency
make actual data loss board-specific. [Mbed LittleFS](https://github.com/ARMmbed/mbed-os/blob/master/storage/filesystem/littlefs/README.md)
