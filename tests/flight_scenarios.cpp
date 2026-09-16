// Appended to production decision/cadence code by run_flight_tests.py.
int main(int argc, char** argv) {
    const char* mode = argc > 1 ? argv[1] : "normal";
    const auto is = [&](const char* name) { return strcmp(mode, name) == 0; };
    SensorData s;
    baroRefValid = baroAlive = imuAlive = true;
    // Prove normal cadence before injecting an in-flight fault.
    for (; fakeMs < 3400; fakeMs += 10) {
        s.time = float(fakeMs); imuLastMs = fakeMs;
        Barometer.ready = fakeMs % 20 == 0;
        parseBaro(s); cadence(s);
    }
    assert(vzCadenceOK);
    s.accel[2] = 3;
    for (int i = 0; i < 12; ++i) {
        s.time = float(fakeMs); imuLastMs = baroLastMs = fakeMs;
        runDecision(s); fakeMs += 10;
    }
    assert(flightState == FlightState::LAUNCH);
    uint32_t launch = fakeMs - 10;
    s.accel[2] = 0;
    s.euler[0] = 180;
    s.kalman[1] = -10;
    if (is("tilt")) { vzTrusted = false; s.euler[0] = 0; }
    if (is("timer")) { imuAlive = baroAlive = false; }
    if (is("slow40") || is("nan") || is("recovery_latched")) s.euler[0] = 0;
    // With a healthy primary, nose-down alone must not bypass its priority.
    if (is("primary_priority")) { s.euler[0] = 0; s.kalman[1] = 0; }
    uint32_t deployed = 0;
    for (; fakeMs - launch < 14500; fakeMs += 10) {
        uint32_t elapsed = fakeMs - launch;
        s.time = float(fakeMs); imuLastMs = fakeMs;
        bool degraded = is("slow40") || (is("recovery_latched") && elapsed < 6500);
        Barometer.ready = fakeMs % (degraded ? 40 : 20) == 0;
        // One missed/late sample triggers a 500ms blank, not permanent fallback.
        if (is("short_gap") && elapsed >= 4800 && elapsed < 4900) Barometer.ready = false;
        if (is("nan")) s.kalman[1] = NAN;
        if (is("brief_nan")) s.kalman[1] = elapsed >= 4700 && elapsed < 5300 ? NAN : -10;
        // Two separated interruptions must not accumulate into a permanent fault.
        if (is("separate_gaps")) {
            s.kalman[1] = (elapsed >= 5000 && elapsed < 5600) ||
                         (elapsed >= 5800 && elapsed < 6400) ? NAN : 0;
        }
        // Delay tilt until after good cadence returns: the fault must stay latched.
        if (is("recovery_latched")) s.euler[0] = elapsed < 7000 ? 180 : 0;
        parseBaro(s); runDecision(s); cadence(s);
        if (flightState == FlightState::DEPLOY) { deployed = elapsed; break; }
    }
    if (!deployed || deployed < 5000 || Parachute.angle != SERVO_OPEN_DEG) return 1;
    if (is("normal") && !(deployed < 5200 && vzTrusted)) return 2;
    if (is("tilt") && !(deployed < 5300)) return 3;
    if ((is("slow40") || is("nan")) && !(deployed >= 6000 && deployed <= 6200 && !vzTrusted)) return 4;
    if (is("recovery_latched") && !(deployed >= 7000 && deployed < 7300 && !vzTrusted)) return 5;
    if ((is("short_gap") || is("brief_nan")) && !(deployed < 5800 && vzTrusted)) return 6;
    if ((is("timer") || is("primary_priority") || is("separate_gaps")) && deployed != 14010) return 7;
    if (is("separate_gaps") && !vzTrusted) return 8;
    printf("PASS %s: deployment at %ums, vzTrusted=%d\n", mode, deployed, vzTrusted);
    return 0;
}
