# PSI AVINICS 2026 migration and legacy recovery

Imported September 17, 2026 from `Downloads/PSI AVINICS 2026 (2)`, the August 8 package. All 104 non-cache ZIP files matched the extracted source during review. ZIP SHA-256:

```text
663b5909335d70c219fa704ed16ece98a706df973d9201a6b70bd81b16f17eb1
```

Only the firmware project was imported into `flight-computer/`. The existing ground station, logo/model assets, basemap utility, requirements, and recorded flight logs were checkpointed and retained. Build caches and personal editor/assistant settings were excluded. Vendored libraries were preserved, including the modified BMP3XX driver and UKF source/header pair.

## Legacy versions

The original single-core firmware is preserved directly in
[`flight-computer/legacy/`](../flight-computer/legacy/README.md), including its
original source, libraries, and PlatformIO configuration. It builds as a
separate project and is excluded from the current firmware build. Git history
and the recovery tags are also retained.

| Local tag | Contents |
|---|---|
| `legacy-single-core-2026-09-17` | Original committed repository at `6df6d0e`, including the old single-core firmware and old ground station |
| `legacy-pre-migration-2026-09-17` | Complete working-state checkpoint at `bf886d3`: old single-core firmware plus the user's newer ground station, assets, and logs |

The original committed version was verified on GitHub's main branch before migration and remains accessible at [commit 6df6d0e](https://github.com/postech-psi/Avionics/tree/6df6d0e4ed5a65a6c21af55266a050eb3e845649). Local tags do not automatically appear on GitHub; they must be pushed when publishing the migration.

Inspect an old file without changing the current checkout:

```bash
git show legacy-single-core-2026-09-17:flight-computer/src/main.cpp
```

Export the full checkpoint without switching branches:

```bash
git archive --format=zip --output=../Avionics-legacy-2026-09-17.zip legacy-pre-migration-2026-09-17
```

The full checkpoint combines old firmware with the newer 92-byte ground station, so it is a recovery snapshot, not a matched old flight/receiver release. Use the original committed tag to inspect the original pairing. A standalone Git bundle containing the pre-migration history and checkpoint was also saved outside the repository during migration.

## Fixes applied after import

### Persistent velocity-path failure

The original candidate indefinitely blocked velocity votes on a 40 ms barometer stream while excluding those intervals from cadence statistics. It continued selecting velocity as the primary and resetting tilt votes, leaving only the fourteen-second timer. Persistent invalid velocity could do the same.

After the five-second deployment lockout, `runDecision()` now times continuous failure of `vzUsableNow()`. At 1000 ms it permanently clears `vzTrusted`, resets velocity votes, and enables the existing tilt backup if the IMU is alive and fresh. A usable interval resets the timer. Existing immediate fault latches, the five-second lockout, normal primary priority, and fourteen-second force timer remain in effect. One second is twice the existing 500 ms settling period, allowing a short gap to recover before permanently yielding the primary.

This timer is evaluated by the flight loop. It does not protect against a stopped processor, hung sensor bus, or other failure that prevents that loop from running. Confirm the chosen timeout and deployment timing on the actual airframe before flight use.

### SD-log health replay

`_raw_health()` now decodes M4's `healthNote()` output: `imu_dead`, `imu_stale`, `baro_dead`, `baro_stale`, `gnss_dead`, `no_baro_ref`, `vz_off`, and `vz_settle`. Retained sensor values cannot override these notes. `vz_settle` retains trust while clearing usability. An alive GNSS receiver without a position fix remains alive when the encoded flags say so. Older unannotated logs retain their previous best-effort interpretation; ground-station CSV health bytes are read unchanged.

## Build and regression checks

ST STM32 **19.5.0** and Arduino Mbed **4.5.0** are pinned in `platformio.ini`. Both M4 and M7 must be built. When hardware upload is requested, follow the firmware guide's **M4 first, M7 second** sequence and verify saved IMU/GNSS configuration and servo geometry.

From the repository root:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
python tests/run_flight_tests.py
pio run -d flight-computer
```

Use a C++17 host compiler for flight tests; for MSVC, open the x64 Native Tools Command Prompt first. The test runner extracts the working production decision/cadence code into a temporary translation unit, stubbing hardware and time only. Python replay tests execute the real replay functions without loading Qt/OpenGL.

Regression coverage includes healthy primary priority, tilt backup, lockout, force timer, 40 ms cadence after arming, persistent NaN velocity, short barometer gaps, brief invalid velocity, separate interruptions, and a fault that remains latched after sensor recovery. Replay tests cover explicit fault tokens, full CSV loading, GNSS without a fix, and legacy formats. Both defects were reproduced as failing tests before the fixes were applied.

No hardware was flashed during migration. Host checks do not replace sensor/servo/SD/radio tests or flight qualification.
