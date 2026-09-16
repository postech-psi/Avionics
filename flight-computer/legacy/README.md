# Legacy firmware

This is the original single-core Portenta H7 firmware from
[commit 6df6d0e](https://github.com/postech-psi/Avionics/tree/6df6d0e4ed5a65a6c21af55266a050eb3e845649).
The source, libraries, and PlatformIO configuration are kept as they were.

Build it from the repository root:

```bash
pio run -d flight-computer/legacy -e portenta_h7_m7
```

This is a separate PlatformIO project. Building `flight-computer/` does not
include the legacy sources or libraries. Its platform dependency remains
unpinned, as in the original configuration.

The current ground station expects a newer telemetry packet. To use the old
firmware with its original receiver, take the ground-station code from the
same commit or the `legacy-single-core-2026-09-17` tag.

The `legacy-pre-migration-2026-09-17` tag preserves the working tree before
the firmware replacement, including newer ground-station files and flight
logs. It is an archive of that working state; the firmware and receiver in
that tag use different packet formats.
