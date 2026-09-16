# Legacy single-core firmware

This directory preserves the original firmware from commit `6df6d0e4ed5a65a6c21af55266a050eb3e845649` (`legacy-single-core-2026-09-17`). Its source, vendored libraries, and PlatformIO configuration are unchanged from that commit.

The current dual-core firmware lives one directory above. This legacy project has its own `src/`, `lib/`, and `platformio.ini`; it is outside the current project's source/library directories and is not included in a normal current-firmware build.

Build the legacy project separately from the repository root:

```bash
pio run -d flight-computer/legacy -e portenta_h7_m7
```

The legacy configuration retains its original unpinned platform dependency. Its older telemetry format does not match the current ground station. The original ground station is available in the same Git commit/tag. See [migration and recovery](../../docs/MIGRATION-2026.md) for the preserved versions.
