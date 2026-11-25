# 2026 year-end launch integration review

This branch combines the variable-interval UKF and diagnostic GRU from
`flight-computer.zip` with the tail-fin control math and PSILOG v1 from
`Avionics-main-RENEWAL.zip`. M7 still owns flight decisions, M4 owns aircraft
storage and XBee, and the dashboard still records its own CSV/TXT from the radio.
The actual units and field mapping are in the [firmware guide](../README.md).

`[GRU]` events pair `h` (GRU) with `quad` (the quadratic predictor from `main`)
at the same window endpoint. The quadratic fit updates at each accepted baro
sample with altitude from the new UKF. The prediction is `nan` when the fit
has no usable maximum. Both are diagnostics. CSV telemetry keeps `apogee` as
GRU and `r2` as reserved. Decode the event TXT to compare both predictions.

## Fin wiring and coordinate assumptions

| Connector | Schematic | Portenta J2 | STM32 | Arduino |
| --- | --- | ---: | --- | --- |
| H7 | SG90_E / PWM_1 | 59 | PA8 | D6 |
| H8 | SG90_W / PWM_2 | 61 | PC6 | D5 |
| H9 | SG90_S / PWM_3 | 63 | PC7 | D4 |
| H10 | SG90_N / PWM_4 | 65 | PG7 | D3 |
| H11 | Para | 67 | PJ11 | D2 |

Use the inner J2 numbers on the carrier schematic and the
[Arduino pinout](https://docs.arduino.cc/resources/pinouts/ABX00042-full-pinout.pdf).
The old `{6,5,4,3}` is connector order E/W/S/N. The mixer slots run around the
body and pair 0° with 180° and 90° with 270°. The electrical order pairs E/S and
W/N under that mixer, which is inconsistent if the labels describe the real
fin positions. The branch separates the connector table from model slots and
leaves all four slot assignments unset.

The mixer expects NED and model-body +X toward the nose. The mount quaternion
maps model body to IMU body. The code composes it with the IMU Euler rotation
and the ENU-to-NED rotation. The gyro is rotated by the inverse mount and
converted from degrees/s to radians/s. RENEWAL assumed sensor +Z toward the
nose. The retained parachute backup interprets upright as sensor +Z pointing
down (`cos(roll) × cos(pitch) ≈ −1`). Test both with the same installed IMU and
verify the vertical target's heading before confirming a mount. The build rejects
a configured fin mount with the model nose inconsistent with that recovery
assumption. A different sensor convention needs a shared review of both paths.

D3–D6 are detached in the default build. Enabling outputs requires a unit
mount quaternion, unique verified connectors, neutral pulses, direction and
travel calibration in [tailfin_config.h](../src/m7/tailfin_config.h). The new
integration has one velocity-gated mode. It requires `LAUNCH`, a usable and
positive vertical velocity, fresh sensors and valid timing/pressure. Recovery
logic and the D2 assignment remain in `runDecision` and `setup`. Verify real
neutral, direction, travel and no mechanical binding independently of the
command. The Mbed Servo library truncates offsets when attaching with custom
1000/2000 µs limits. This integration uses default attach and applies the
configured clamp before writing each pulse. A control loop stall can leave the
Servo timer repeating the previous pulse until the loop runs again. Neutral on
bad inputs is a software response and does not cover a stalled core or lost power.

## Failures and evidence

| Area | What the code can show | What remains to verify |
| --- | --- | --- |
| UKF and recovery | Timing tests cover jitter, sensor gaps and re-anchoring. The variable interval changes the velocity seen by the existing deployment logic. | I²C completion is a sample-time proxy. Compare `[BMP]` intervals, velocity and state transitions against hardware captures. |
| GRU | Predictions run in slices after the decision and telemetry enqueue. A synthetic window at 600 m and zero vertical speed produced about 543 m. | Treat predictions as diagnostic. Compare to a physics baseline and independent flight logs. The 6 ms entry guard is not a hard slice deadline. Check `cpu_us`, `age_us` and `deadline skips` on the board. |
| Tail fins | Host checks cover the mounting rotation, gyro signs, neutral and stale timing. | Verify the installed axes, physical fin order and pulse widths. A PWM command is not position feedback. |
| SD and IPC | The heartbeat reports recent successful sample sync and local queue drops. BIN verifies each stored record with CRC16. | IPC rejects and M7 queue loss happen before the SD queue. A green badge does not prove a complete log. Separate data and event queues can change their relative order in the file. |

## Battery disconnect, replay and conversion

BIN improves recovery from partial records in a readable file. Loss of FAT
metadata or the file header can still make the file inaccessible. The writer
tries to sync batches about every 200 ms, but unsynced records, FAT file metadata and the card's internal writes can be lost or damaged at power
cut. The interval is not a guaranteed maximum amount of lost data. See the
[FatFs sync guidance](https://elm-chan.org/fsw/ff/doc/sync.html).

Keep the original BIN. Use **OPEN LOG** to play it directly, or run the
[converter](../../ground-station/README.md#replay-a-flight) later. It exports
samples as CSV and events as TXT and does not rewrite the BIN. The reverse trip
cannot recreate original record order, CRCs, or values rounded in CSV.

## QSPI implementation plan

1. **Inventory the board read-only.** The installed PlatformIO M4 framework
   includes `QSPIFBlockDevice`. Verify the actual JEDEC geometry and partition
   table on a spare Portenta. Identify an available user region and confirm
   it does not overlap Wi-Fi, OTA, provisioning, or existing user data. Keep
   QSPI disabled if the expected region is absent. No automatic reformat or
   repartition. [Arduino flash and partitions guide](https://docs.arduino.cc/tutorials/portenta-h7/reading-writing-flash-memory/)
2. **Prototype one owner on M4.** Start with a bounded append file on a
   dedicated user partition and a separate QSPI worker/queue so slow erase,
   program or sync operations do not back up SD or XBee. Use the same PSILOG
   record encoding. Size a session from measured capacity. At 100 Hz, 10 min
   uses about 5.82 MB for sensors plus events and storage overhead. Stop when
   full and report it. Leave older sessions in place until exported.
3. **Add retrieval and independent status.** Export the original BIN over a
   read-only service path while recording is stopped. Report per-medium
   committed sample counts, failures and drops. Add a source sequence number
   and firmware/schema metadata in a versioned format if exact gaps and
   matching between replicas are required. Keep the reader compatible with v1.
4. **Verify on a spare board before enabling it on the vehicle.** Start with
   known test records and inspect decoded counts, gaps and CRC failures. Compare
   SD-only and dual-write timing, then test without SD, with QSPI full, and with
   controlled power cuts while writing. Measure how much data remains readable
   on each medium after reboot. Both media share the board power supply and can
   lose their uncommitted tail at the same time.
