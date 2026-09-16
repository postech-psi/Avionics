"""Compile real firmware decision logic on a host; fake only time/hardware.

Requires C++17: g++/clang++, or MSVC (Developer Command Prompt on Windows).
The extracted functions come from the working firmware, never a test copy.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / 'flight-computer'


def between(source, start, end):
    a = source.index(start)
    return source[a:source.index(end, a)]


def harness():
    source = (FIRMWARE / 'src/m7/main_m7.cpp').read_text(encoding='utf-8')
    prefix = r'''
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <chrono>
#include "src/parsers.h"
using namespace std::chrono_literals;
uint32_t fakeMs = 1000;
uint32_t millis() { return fakeMs; }
constexpr float DEG_TO_RAD = 0.017453292519943295f;
struct FakeServo { int angle=-1; void write(int value) { angle=value; } } Parachute;
struct FakeTimeout {
    void detach() {}
    template<class T> void attach(void(*)(), T) {}
} servoReopenGuard;
void servoReopenIsr() {}
void logMsg(const char*) {}
void logEvent(const char*) {}
struct FakeBaro {
    double pressure=101300;
    bool ready=true;
    bool readNonBlocking() { return ready; }
} Barometer;
'''
    parts = [prefix]
    for start, end in [
        ('struct SensorData {', '//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//'),
        ('const int SERVO_OPEN_DEG', '//### CLOSE fail-open guard'),
        ('enum class FlightState', '//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//'),
        ('const uint32_t LOOP_PERIOD_MS', 'bool     rpcOK'),
        ('static inline bool imuFreshNow', 'SensorData          d{};'),
        ('bool parseBaro(', '// ! CALL ONLY ON A FRESH BARO SAMPLE'),
        ('struct VoteWin {', '//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//'),
        ('void runDecision(', '//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//'),
    ]:
        parts.append(between(source, start, end))
    parts.append('void cadence(SensorData& d) {\n' + between(
        source, '    static bool cadenceArmDone', '    //### RPC link report') + '\n}\n')
    parts.append((ROOT / 'tests/flight_scenarios.cpp').read_text(encoding='utf-8'))
    return '\n'.join(parts)


def main():
    compiler = os.environ.get('CXX') or shutil.which('g++') or shutil.which('clang++') or shutil.which('cl')
    if not compiler:
        raise SystemExit('C++17 compiler required; set CXX or run from a compiler developer shell.')
    with tempfile.TemporaryDirectory(prefix='avionics-tests-') as folder:
        folder = Path(folder)
        cpp = folder / 'flight.cpp'
        exe = folder / ('flight.exe' if os.name == 'nt' else 'flight')
        cpp.write_text(harness(), encoding='utf-8')
        if Path(compiler).stem.lower() == 'cl':
            command = [compiler, '/nologo', '/EHsc', '/std:c++17', '/permissive-', '/utf-8',
                       '/D_CRT_SECURE_NO_WARNINGS', '/I' + str(FIRMWARE), str(cpp), '/Fe:' + str(exe)]
        else:
            command = [compiler, '-std=c++17', '-I', str(FIRMWARE), str(cpp), '-o', str(exe)]
        subprocess.run(command, cwd=folder, check=True)
        failures = []
        for mode in ('normal', 'tilt', 'timer', 'slow40', 'nan', 'short_gap', 'brief_nan',
                     'separate_gaps', 'recovery_latched', 'primary_priority'):
            result = subprocess.run([str(exe), mode], check=False)
            if result.returncode:
                failures.append(mode)
                print(f'FAIL {mode}: scenario assertion {result.returncode}', flush=True)
        if failures:
            raise SystemExit('Failed scenarios: ' + ', '.join(failures))
        print('All 10 flight scenarios passed.')


if __name__ == '__main__':
    main()
