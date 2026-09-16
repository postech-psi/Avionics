//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~ main_m7.cpp - Portenta H7 (STM32H747) dual-core avionics : M7 ~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// All real-time flight control, single-thread superloop at a fixed 100Hz:
//   GNSS/IMU/Baro -> UKF -> decision FSM -> parachute servo, then the full
//   frame to M4 over RPC. XBEE(Serial1) and SD belong to M4 - never touched here.
//
// EVERYTHING about this file is documented in README section 6:
//   6.1 loop structure   6.3 sensor validity   6.4 freshness grades
//   6.5 UKF at 50Hz      6.6 vote windows      6.7 state transitions
//   6.9 parameter table  6.11 lines you must not delete
// Comments below are limited to "// !" markers on those must-not-delete lines.
// Build & upload (M4 FIRST, then M7) : README 1-2.
//
// [File map] - the [N] tags match the section banners in this file
//   [1] Data Structure      [2] Sensor Mapping     [3] Flight State
//   [4] Parameters          [5] State & Freshness  [6] IPC Transmit
//   [7] Logging             [8] Parsing            [9] Sensor Health
//  [10] VoteWin            [11] Decision          [12] Startup Checks
//  [13] Sensor Check       [14] Setup             [15] Main Loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include "mbed.h"
#include "rtos.h"
#include <chrono>
#include <RPC.h>
#include "ukf.h"
#include "rtwtypes.h"
#include "../parsers.h"
#include "../shared/telemetry_frame.h"
#include "../shared/ipc_protocol.h"

using namespace mbed;
using namespace rtos;
using namespace std::chrono;
kf Kalmanfilter;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~ [1] Data Structure (M7 local) ~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
struct SensorData {
    uint8_t state = 0xBA;      // 0xBA prelaunch 0xBB launch 0xBC deploy 0xBD landed
    float time = 0.0;          // mission ms
    float lat = 0.0;
    float lon = 0.0;
    float euler[3] = {0, 0, 0};  // roll pitch yaw, deg
    float accel[3] = {0, 0, 0};  // linear accel, gravity-free, GLOBAL frame (soa3), g
    float gyro[3]  = {0, 0, 0};  // dps
    float pos[3]   = {0, 0, 0};  // integrated distance, global, m (drifts - log only)
    float altitude = 0.0;        // above pad reference, m
    float pressure = 0.0;        // hPa
    float kalman[4] = {0, 0, 0, 0};  // kf_alt, kf_vel, apogee, r2
    float gnssAlt = 0.0f;            // GNSS hMSL, m - log/recovery only,
                                     // NEVER used by any flight decision
    uint8_t health = 0;              // TLM_* flags for THIS frame
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~ [2] Sensor Mapping (XBEE excluded - owned by M4) ~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
Adafruit_BMP3XX Barometer;                    // BMP390, I2C Wire2, 0x76
Servo           Parachute;                    // deploy servo on D2
UART            GNSS(PG_14, PG_9, NC, NC);    // NEO-M9N, UBX-POSLLH, 38400
UART            IMU(PJ_8, PJ_9, NC, NC);      // EBIMU-9DOFV6, 230400

//### Servo geometry (this airframe): 0 deg releases the chute, 120 deg locks ###
const int SERVO_OPEN_DEG  = 0;
const int SERVO_CLOSE_DEG = 120;

//### CLOSE fail-open guard: if the loop dies mid jam-retry, an ISR reopens ###
// ! armed when the retry cycle CLOSES the servo, detached on normal reopen.
//   ISR-safe: after the first write, Servo::write is bounds math + one int32
//   store consumed by the lib's own Ticker (verified in lib/Servo-master).
static mbed::Timeout servoReopenGuard;
static void servoReopenIsr() { Parachute.write(SERVO_OPEN_DEG); }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~ [3] Flight State ~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
enum class FlightState { STARTUP, PRELAUNCH, LAUNCH, DEPLOY, LANDED };
FlightState flightState = FlightState::STARTUP;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~ [4] Parameters (README 6.9) ~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//### Loop & flight thresholds ###
const uint32_t LOOP_PERIOD_MS       = 10;
const float   launchThreshold       = 2;
unsigned long force_deploy_time     = 14000;
const float   fallingspeedThreshold = -7;
const float   TILT_COS_LIMIT        = 0.70711f;
const float   launchAltThreshold    = 10.0f;

//### Vote windows: fire when >= need of the last N votes are true ###
// need = 60% of N (2026-08-04). vz is 7/12 = 58.3% - 12*0.6 = 7.2 is not an
// integer and 7 lands far closer to 60% than 8 (66.7%).
const uint16_t VOTE_ACCEL_N  = 20,   VOTE_ACCEL_NEED  = 12;    // 60%,   ~120ms
const uint16_t VOTE_ALT_N    = 40,   VOTE_ALT_NEED    = 24;    // 60%,   ~240ms
const uint16_t VOTE_VZ_N     = 12,   VOTE_VZ_NEED     = 7;     // 58.3%,  ~70ms
const uint16_t VOTE_TILT_N   = 25,   VOTE_TILT_NEED   = 15;    // 60%,   ~150ms
const uint16_t VOTE_STATIC_N = 2500, VOTE_STATIC_NEED = 1500;  // 60%,    ~15s
const uint32_t DEPLOY_LOCKOUT_MS    = 5000;   // launch+5s: ALL deploy paths locked

//### Static landing (the only landing path) ###
const float    STATIC_GYRO_DPS      = 5.0f;
const float    STATIC_ACCEL_G       = 0.2f;
const float    STATIC_ALT_BAND_M    = 2.0f;

//### Barometer plausibility ###
const float    BARO_MAX_RATE_MPS = 680.0f;    // ~Mach 2 = 13.6m per 20ms sample
const uint32_t BARO_RATE_GAP_MS  = 200;
const uint32_t BARO_STEP_MAX_MS  = 35;        // 10ms polling x 50Hz -> 10/20/30 normal

//### Sensor health timeouts (death/recovery reporting, cal skip) ###
const uint32_t IMU_TIMEOUT_MS    = 500;
const uint32_t BARO_TIMEOUT_MS   = 500;
const uint32_t GNSS_TIMEOUT_MS   = 1000;   // 25Hz UBX stream: 25 missing = dead
const uint32_t HEALTH_LOG_MIN_MS = 2000;
const float    GNSS_HACC_MAX_M   = 100.0f;
const uint32_t BARO_BOOT_SETTLE_MS         = 3000;
const uint32_t BARO_BOOT_RETRY_WINDOW_MS   = 10000;
const uint32_t BARO_BOOT_RETRY_INTERVAL_MS = 1000;
// GNSS_HACC_MAX : POSLLH has no fix flag - a no-fix frame reads lat=lon=0 with
//                 hAcc ~4.29e6 m, so coordinates are taken only when the
//                 reported horizontal accuracy is recovery-grade (measured
//                 module config 2026-08-04). Log/recovery only, gates nothing.

//### Control freshness - SEPARATE from health, gates votes and the UKF ###
const uint32_t IMU_CTRL_FRESH_MS  = 30;
const uint32_t BARO_CTRL_FRESH_MS = 60;
const uint32_t VZ_BARO_GAP_MS     = 200;
const uint32_t VZ_SETTLE_MS       = 500;
// After the deploy lockout, sustained unusability must release the tilt backup.
// Two settle periods allow an isolated short gap to recover without a latch.
const uint32_t VZ_UNUSABLE_TIMEOUT_MS = 1000;
const float    VZ_PLAUSIBLE_MAX   = 200.0f; // |vz| beyond this = filter divergence
const uint16_t CADENCE_ARM_N       = 100;   // clean intervals to arm the vz path
const uint16_t CADENCE_BLOCK_N     = 50;    // rolling re-check block size
const float    CADENCE_MEAN_LO_MS  = 17.5f; // accepted mean band (= rolling-4
const float    CADENCE_MEAN_HI_MS  = 22.5f; //   sum 70..90). Bench: real BMP390
                                            //   ODR measured 19.42ms (+3% fast,
                                            //   oscillator tolerance) - a tight
                                            //   19.5..20.5 band killed vz on a
                                            //   HEALTHY sensor. +-12.5% scale
                                            //   error on vz is harmless; the
                                            //   guard targets 25%+ (25/30ms).
const uint32_t CADENCE_DEADLINE_MS = 10000; // arming give-up (mission ms)
const uint32_t CADENCE_FAST_SUM_LO = 70;    // last-4 clean intervals: sum 70~90
const uint32_t CADENCE_FAST_SUM_HI = 90;    //  = mean 17.5~22.5ms, checked per sample

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~ [5] State Variables & Freshness ~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
uint32_t Setup_Time    = 0;
uint32_t Setup_Time_us = 0;
float    basePressure  = 1013.0f;
float    altLaunchAvg  = 0.0f;   // mean of the 2 latest REAL baro samples
bool     baroRefValid  = false;  // ground reference passed the quality gate
bool     vzTrusted     = true;   // kalman-vz deploy path enable (one-way latch)
bool     vzCadenceOK   = false;  // measured baro interval matches the UKF's 20ms
uint32_t vzBlockUntil  = 0;      // short-gap blanking deadline
bool     altVotePendingReset = false;  // pre-launch baro gap -> wipe altitude votes
bool     vzVotePendingReset  = false;  // bad cadence -> wipe vz votes (runDecision)

uint32_t baroIvalSum   = 0;      // accepted-sample interval accounting,
uint32_t baroIvalCnt   = 0;      // reported once at PRELAUNCH

bool     imuAlive      = false;
bool     baroAlive     = false;
bool     gnssAlive     = false;
uint32_t imuLastMs     = 0;
uint32_t baroLastMs    = 0;
uint32_t gnssLastMs    = 0;
bool     rpcOK         = false;

//### Single source of truth for "fresh enough to act on" ###
static inline bool imuFreshNow(uint32_t now) {
    return imuLastMs && (now - imuLastMs) <= IMU_CTRL_FRESH_MS;
}
static inline bool baroFreshNow(uint32_t now) {
    return baroLastMs && (now - baroLastMs) <= BARO_CTRL_FRESH_MS;
}

// Every precondition of the vz deploy vote, in one place, so the flight logic
// and the logged health bit can never disagree (README 6.7).
static inline bool vzUsableNow(uint32_t now, float vz) {
    return vzTrusted && vzCadenceOK &&
           imuFreshNow(now) && baroFreshNow(now) &&
           std::isfinite(vz) && fabsf(vz) < VZ_PLAUSIBLE_MAX &&
           (int32_t)(now - vzBlockUntil) >= 0;
}

SensorData          d{};

static uint8_t sensorFlags() {
    uint32_t now = millis();
    uint8_t f = 0;
    if (imuFreshNow(now))  f |= TLM_IMU_FRESH;
    if (baroFreshNow(now)) f |= TLM_BARO_FRESH;
    if (imuAlive)          f |= TLM_IMU_ALIVE;
    if (baroAlive)         f |= TLM_BARO_ALIVE;
    if (gnssAlive)         f |= TLM_GNSS_ALIVE;
    if (baroRefValid)      f |= TLM_BARO_REF;
    if (vzTrusted)         f |= TLM_VZ_TRUSTED;
    if (vzUsableNow(now, d.kalman[1])) f |= TLM_VZ_USABLE;
    return f;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~ [6] IPC Transmit (ring -> BelowNormal TX thread -> RPC) ~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// ! the flight loop NEVER calls the blocking RPC.write. ipcSend only pushes a
//   finished wire frame into a SPSC ring (microseconds, always returns); the
//   txThread does the blocking write. A wedged M4 therefore costs telemetry
//   frames (counted, reported) - never flight control. Same producer-priority
//   pattern as M4's hardware-proven SD writer (README 6.1 / 6.10).
const uint32_t RPC_WRITE_WARN_US = 2000;
const uint16_t TX_PAYLOAD_MAX    = 128;   // real payloads: data 90, logs <= 95
static volatile bool     g_rpcWarn   = false;
static volatile uint32_t g_rpcWarnUs = 0;
static volatile uint32_t g_rpcDrops  = 0;

struct TxRec { uint8_t len; uint8_t b[ipc::IPC_HEADER + TX_PAYLOAD_MAX + ipc::IPC_TRAILER]; };
static mbed::CircularBuffer<TxRec, 32> txRing;
static rtos::Thread txThread(osPriorityBelowNormal, 4096, nullptr, "rpctx");

static void ipcSend(uint8_t type, const uint8_t* payload, uint16_t len) {
    if (!rpcOK) return;
    if (len > TX_PAYLOAD_MAX) len = TX_PAYLOAD_MAX;
    TxRec r;
    r.b[0] = ipc::IPC_START;
    r.b[1] = type;
    r.b[2] = (uint8_t)(len & 0xFF);
    r.b[3] = (uint8_t)(len >> 8);
    if (len) memcpy(&r.b[ipc::IPC_HEADER], payload, len);
    r.b[ipc::IPC_HEADER + len] = ipc::checksum(r.b, len);
    r.len = (uint8_t)(ipc::IPC_HEADER + len + ipc::IPC_TRAILER);
    if (txRing.full()) { g_rpcDrops++; return; }   // ! drop and count, never block
    txRing.push(r);
}

static void txThreadFn() {
    TxRec r;
    while (true) {
        while (txRing.pop(r)) {
            uint32_t t0 = micros();
            RPC.write(r.b, r.len);
            uint32_t dt = micros() - t0;
            if (dt > RPC_WRITE_WARN_US) { g_rpcWarnUs = dt; g_rpcWarn = true; }
            if (r.b[1] != ipc::TYPE_DATA) {
                // USB mirror of log text lives HERE, off the flight loop
                Serial.write(&r.b[ipc::IPC_HEADER],
                             (size_t)r.len - ipc::IPC_HEADER - ipc::IPC_TRAILER);
                Serial.write((const uint8_t*)"\r\n", 2);
            }
        }
        ThisThread::sleep_for(1ms);
    }
}

static void ipcSendData(const SensorData& s) {
    // copy into the PACKED wire struct instead of sharing one struct: the wire
    // layout starts with a 1-byte state, so every float in it is misaligned -
    // using it directly in the flight loop would force unaligned/bytewise FP
    // access on every read. One ~90B copy per 10ms is the cheaper trade.
    PackedSensorData p;
    p.state = s.state; p.time = s.time; p.lat = s.lat; p.lon = s.lon;
    for (int i = 0; i < 3; ++i) {
        p.euler[i] = s.euler[i]; p.accel[i] = s.accel[i];
        p.gyro[i]  = s.gyro[i];  p.pos[i]   = s.pos[i];
    }
    p.altitude = s.altitude; p.pressure = s.pressure;
    for (int i = 0; i < 4; ++i) p.kalman[i] = s.kalman[i];
    p.gnssAlt = s.gnssAlt;
    p.health = s.health;
    ipcSend(ipc::TYPE_DATA, (const uint8_t*)&p, sizeof(p));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~ [7] Logging ~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Tag protocol ([INIT]/[CAL]/[ZERO]/[TEST]/[STATE]/[BARO]/[SYS]/[WARN]/[ERROR])
// is defined in README 6.9.
// default = USB + XBEE + SD; pass ipc::TYPE_LOG_SD for bulk SD-only output.
// Every line is prefixed "[ms] " on the M7 MISSION clock - the same axis as the
// CSV time_ms column, so txt events and CSV rows line up with no conversion.
// During setup the prefix is the boot clock (Setup_Time still 0); the switch is
// marked by "[ZERO] mission clock start" (README 5).
// ! no Serial here: the USB mirror happens in txThread when the frame is sent,
//   so the flight loop performs ZERO I/O of any kind (RPC or USB)
static void logMsg(const char* s, uint8_t type = ipc::TYPE_LOG) {
    char b[112];
    int n = snprintf(b, sizeof(b), "[%lu] %s",
                     (unsigned long)(millis() - Setup_Time), s);
    if (n < 0) return;
    if ((size_t)n >= sizeof(b)) n = (int)sizeof(b) - 1;   // truncated, still sent
    ipcSend(type, (const uint8_t*)b, (uint16_t)n);
}

//### Deferred event log - state changes NOW, text AFTER runDecision ###
// ! logMsg rides on a BLOCKING RPC write. A warning produced while parsing or
//   health-checking must therefore never be SENT before the flight decision of
//   the same loop - only queued here and flushed after runDecision. Worst
//   same-loop burst is 4 events (gap warn + baro recovered + imu/gnss lost);
//   depth 6 leaves margin, overflow silently drops (README 6.7).
static char    g_evtQ[6][80];
static uint8_t g_evtCnt = 0;
static uint32_t g_evtDrops = 0;
static void logEvent(const char* s) {
    if (g_evtCnt < 6) {
        strncpy(g_evtQ[g_evtCnt], s, sizeof(g_evtQ[0]) - 1);
        g_evtQ[g_evtCnt][sizeof(g_evtQ[0]) - 1] = '\0';
        g_evtCnt++;
    } else g_evtDrops++;
}
static void flushEvents() {
    for (uint8_t i = 0; i < g_evtCnt; ++i) logMsg(g_evtQ[i]);
    g_evtCnt = 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~ [8] Parsing (byte loops here, line parsing lives in parsers.h) ~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void parseGNSS(SensorData &d) {
    static parsers::UbxPosllh ubx;
    while (GNSS.available()) {
        if (ubx.feed((uint8_t)GNSS.read())) {
            gnssLastMs = millis();   // checksum-valid frame = receiver alive
            if (ubx.hAccM <= GNSS_HACC_MAX_M &&
                fabsf(ubx.lat) <= 90.0f && fabsf(ubx.lon) <= 180.0f) {
                d.lat     = ubx.lat;   // ! hAcc gate stands in for the missing
                d.lon     = ubx.lon;   //   fix flag (README 6.3)
                d.gnssAlt = ubx.hMSLm; // log/recovery only, never control
            }
        }
    }
}

//### Physical range check, shared by parseIMU and sensor_check (README 6.3) ###
// accel is 30g, NOT the 16g of ssa:4 - soa3 output is rotated+gravity-removed
// and is not re-clipped, so a real crash reaches ~28.7g on one global axis.
static bool imuSampleSane(const float v[12]) {
    return fabsf(v[0]) <= 180.5f  && fabsf(v[1]) <= 90.5f   &&
           fabsf(v[2]) <= 180.5f  &&
           fabsf(v[3]) <= 2200.0f && fabsf(v[4]) <= 2200.0f &&
           fabsf(v[5]) <= 2200.0f &&
           fabsf(v[6]) <= 30.0f   && fabsf(v[7]) <= 30.0f   &&
           fabsf(v[8]) <= 30.0f;
}

void parseIMU(SensorData &d) {
    static char   imuBuf[128];
    static size_t imuPos = 0;
    while (IMU.available()) {
        char c = IMU.read();
        if (c == '\r') continue;
        if (c == '\n') {
            imuBuf[imuPos] = '\0';
            imuPos = 0;
            float vals[12];
            if (parsers::parseImuLine(imuBuf, vals)) {
                if (imuSampleSane(vals)) {
                    imuLastMs = millis();
                    d.euler[0] = vals[0];  d.euler[1] = vals[1];  d.euler[2] = vals[2];
                    d.gyro[0]  = vals[3];  d.gyro[1]  = vals[4];  d.gyro[2]  = vals[5];
                    d.accel[0] = vals[6];  d.accel[1] = vals[7];  d.accel[2] = vals[8];
                    d.pos[0]   = vals[9];  d.pos[1]   = vals[10]; d.pos[2]   = vals[11];
                }
            }
            continue;
        }
        if (imuPos < sizeof(imuBuf) - 1) imuBuf[imuPos++] = c;
        else imuPos = 0;
    }
}

// Returns true on a FRESH sample - the caller uses that to step the UKF at 50Hz.
bool parseBaro(SensorData &d) {
    if (Barometer.readNonBlocking()) {
        float p = (float)Barometer.pressure / 100.0f;
        if (!std::isfinite(p) || p < 300.0f || p > 1250.0f) return false;
        float altNow = parsers::pressureToAltitude(p, basePressure);
        if (!std::isfinite(altNow)) return false;
        uint32_t nowMs = millis();

        //### Rate check against the last ACCEPTED sample (README 6.3) ###
        static float    prevValidAlt = 0.0f;
        static uint32_t prevValidMs  = 0;
        if (prevValidMs) {
            uint32_t dt = nowMs - prevValidMs;
            if (dt <= BARO_RATE_GAP_MS &&
                fabsf(altNow - prevValidAlt) > BARO_MAX_RATE_MPS * (float)dt * 0.001f)
                return false;

            //### Gap judged HERE, from the interval - not later from an age ###
            // ! runDecision can only ask "how old is the last sample", and by
            //   the time it runs baroLastMs has already been refreshed below,
            //   so a gap that ENDS is invisible to it. This sample is fresh but
            //   the stream before it was not, and parseKalman is about to fold
            //   the whole gap into one 20ms step -> real velocity transient.
            //   Covers the cases runDecision structurally cannot: the M7 loop
            //   itself stalling, and gaps during PRELAUNCH (README 6.7).
            if (dt > BARO_STEP_MAX_MS) {
                vzBlockUntil = nowMs + VZ_SETTLE_MS;      // any state
                if (flightState == FlightState::STARTUP ||
                    flightState == FlightState::PRELAUNCH)
                    altVotePendingReset = true;
                // ! pre-launch gap invalidates the altitude vote history too -
                //   reacquisition transients must re-earn all 50 votes and
                //   never ride on pre-gap votes (consumed in runDecision)
                if (vzTrusted && flightState == FlightState::LAUNCH &&
                    dt > VZ_BARO_GAP_MS) {
                    vzTrusted = false;
                    char m[72];
                    snprintf(m, sizeof(m), "[WARN] vz deploy path disabled (baro gap %lums)",
                             (unsigned long)dt);
                    logEvent(m);   // ! queued - no RPC before this loop's decision
                }
            } else {
                baroIvalSum += dt; baroIvalCnt++;   // clean intervals only
                //### Fast cadence guard: rolling mean of the last 4 intervals ###
                // ! replaces the consecutive-slow counter, which a 20/30ms
                //   ALTERNATING degradation bypassed forever (every 20 reset
                //   it) while still scaling UKF velocity by 1.25x. A 4-sample
                //   sum outside 70..90 (mean 17.5~22.5ms) blanks vz on the
                //   spot and voids its votes - one baro sample feeds ~3 loop
                //   votes, so any guard slower than ~4 samples loses the race
                //   against the 10/12 window. Legit phase jitter pairs each 30
                //   with a 10, keeping the sum at ~80; a false trip only costs
                //   a self-recovering 500ms blank.
                static uint32_t dtRing[4] = {20, 20, 20, 20};
                static uint8_t  dtIdx = 0, dtCnt = 0;
                dtRing[dtIdx] = dt;
                dtIdx = (uint8_t)((dtIdx + 1) & 3);
                if (dtCnt < 4) dtCnt++;
                if (dtCnt == 4) {
                    uint32_t sum = dtRing[0] + dtRing[1] + dtRing[2] + dtRing[3];
                    if (sum < CADENCE_FAST_SUM_LO || sum > CADENCE_FAST_SUM_HI) {
                        vzBlockUntil = nowMs + VZ_SETTLE_MS;
                        vzVotePendingReset = true;
                    }
                }
            }
        }
        prevValidAlt = altNow;
        prevValidMs  = nowMs;

        baroLastMs = nowMs;
        d.pressure = p;
        d.altitude = altNow;

        // ! averaged HERE (at the sample source), not in the loop - phase-free
        static float prevAlt  = 0.0f;
        static bool  havePrev = false;
        altLaunchAvg = havePrev ? (prevAlt + d.altitude) * 0.5f : d.altitude;
        prevAlt  = d.altitude;
        havePrev = true;
        return true;
    }
    return false;
}

// ! CALL ONLY ON A FRESH BARO SAMPLE (=50Hz). The generated UKF hardcodes
//   dt=0.02s and the baro produces 50 new values/s (README 6.5).
void parseKalman(SensorData &d) {
    float aLin = imuFreshNow(millis()) ? d.accel[2] : 0.0f;   // ! stale -> 0, never frozen
    Kalmanfilter.rtU.accel_z_in       = aLin * 9.81f + 9.81f; // ! +g = specific force
    Kalmanfilter.rtU.baro_altitude_in = d.altitude;
    Kalmanfilter.rtU.current_time_in  = (micros() - Setup_Time_us) / 1000000.0;
    Kalmanfilter.step();
    d.kalman[0] = Kalmanfilter.rtY.kf_altitude_out;
    d.kalman[1] = Kalmanfilter.rtY.kf_velocity_out;
    d.kalman[2] = Kalmanfilter.rtY.apogee_out;
    d.kalman[3] = Kalmanfilter.rtY.r2_out;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~ [9] Runtime Sensor Health (death/recovery, every loop) ~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Flags always follow the truth; only the LOGGING is rate-limited.
static void updateSensorHealth() {
    uint32_t now = millis();
    bool imuFresh  = imuLastMs  && (now - imuLastMs)  < IMU_TIMEOUT_MS;
    bool baroFresh = baroLastMs && (now - baroLastMs) < BARO_TIMEOUT_MS;
    bool gnssFresh = gnssLastMs && (now - gnssLastMs) < GNSS_TIMEOUT_MS;

    static uint32_t lastImuLog = 0, lastBaroLog = 0, lastGnssLog = 0;
    if (imuFresh != imuAlive) {
        imuAlive = imuFresh;
        if (now - lastImuLog >= HEALTH_LOG_MIN_MS) {
            lastImuLog = now;
            logEvent(imuFresh ? "[INIT] IMU recovered" : "[WARN] IMU lost (data timeout)");
        }
    }
    if (baroFresh != baroAlive) {
        baroAlive = baroFresh;
        if (now - lastBaroLog >= HEALTH_LOG_MIN_MS) {
            lastBaroLog = now;
            logEvent(baroFresh ? "[INIT] Baro recovered" : "[WARN] Baro lost (data timeout)");
        }
    }
    if (gnssFresh != gnssAlive) {
        gnssAlive = gnssFresh;
        if (now - lastGnssLog >= HEALTH_LOG_MIN_MS) {
            lastGnssLog = now;
            logEvent(gnssFresh ? "[INIT] GNSS recovered" : "[WARN] GNSS lost (data timeout)");
        }
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~ [10] VoteWin - shared vote window for state transitions ~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// update() must be called exactly once per loop.
struct VoteWin {
    uint8_t* bits; uint16_t n, need, idx, count;
    VoteWin(uint8_t* storage, uint16_t n_, uint16_t need_)
        : bits(storage), n(n_), need(need_), idx(0), count(0) {}
    bool update(bool v) {
        uint16_t byteI = idx >> 3; uint8_t mask = (uint8_t)(1u << (idx & 7));
        bool old = bits[byteI] & mask;
        if (old != v) {
            if (v) { bits[byteI] |= mask;  count++; }
            else   { bits[byteI] &= (uint8_t)~mask; count--; }
        }
        if (++idx >= n) idx = 0;
        return count >= need;
    }
    void reset() { memset(bits, 0, (size_t)((n + 7) >> 3)); idx = 0; count = 0; }
};

static uint8_t vbAccel [(VOTE_ACCEL_N  + 7) / 8];
static uint8_t vbAlt   [(VOTE_ALT_N    + 7) / 8];
static uint8_t vbVz    [(VOTE_VZ_N     + 7) / 8];
static uint8_t vbTilt  [(VOTE_TILT_N   + 7) / 8];
static uint8_t vbStatic[(VOTE_STATIC_N + 7) / 8];
static VoteWin vwAccel (vbAccel,  VOTE_ACCEL_N,  VOTE_ACCEL_NEED);
static VoteWin vwAlt   (vbAlt,    VOTE_ALT_N,    VOTE_ALT_NEED);
static VoteWin vwVz    (vbVz,     VOTE_VZ_N,     VOTE_VZ_NEED);
static VoteWin vwTilt  (vbTilt,   VOTE_TILT_N,   VOTE_TILT_NEED);
static VoteWin vwStatic(vbStatic, VOTE_STATIC_N, VOTE_STATIC_NEED);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~ [11] Decision (called every loop - non-blocking FSM) ~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Full transition table with rationale : README 6.7
void runDecision(SensorData& s) {
    static uint32_t Launch_Time = 0;
    static int      deployPhase = 0;    // 0 open, 1 hold+landing check, 2 closed
    static uint32_t deployTimer = 0;
    static bool     vzUnavailableTracking = false;
    static uint32_t vzUnavailableSince = 0;

    uint32_t now     = millis();
    float    z_accel = s.accel[2];
    float    alt     = s.altitude;
    float    vz      = s.kalman[1];  // 1D UKF vertical velocity, global Z (+up)
    float    euler_x = s.euler[0];
    float    euler_y = s.euler[1];
    bool imuCtrlFresh  = imuFreshNow(now);    // ! votes use these, NOT xxxAlive
    bool baroCtrlFresh = baroFreshNow(now);

    // ! the servo command must precede the log: logMsg goes out over RPC, which
    //   is a BLOCKING write, and the chute is the safety action (README 6.7)
    auto enterDeploy = [&](const char* why) {
        Parachute.write(SERVO_OPEN_DEG);
        deployTimer = now;
        deployPhase = 1;
        flightState = FlightState::DEPLOY;
        logMsg(why);
    };

    switch (flightState) {
        default: {  // STARTUP / PRELAUNCH
            if (altVotePendingReset) {
                altVotePendingReset = false;
                vwAlt.reset();   // ! set by parseBaro on a pre-launch baro gap
            }
            //### Launch: accel OR altitude, each with its configured vote ###
            bool byAccel = vwAccel.update(imuCtrlFresh && fabsf(z_accel) > launchThreshold);
            bool byAlt   = vwAlt.update(baroRefValid && baroCtrlFresh &&
                                        altLaunchAvg > launchAltThreshold);

            if (byAccel or byAlt) {
                // Launch_Time is the DETECTION instant - the vote fill time
                // (~120ms accel / ~240ms altitude) is deliberately NOT
                // back-compensated, so every downstream timer simply starts
                // that much later (README 6.7).
                Launch_Time = (uint32_t)s.time;
                flightState = FlightState::LAUNCH;
                logMsg(byAccel ? "[STATE] LAUNCH (accel)" : "[STATE] LAUNCH (altitude)");
            }
            break;
        }

        case FlightState::LAUNCH: {
            //### Baro gap: short = blank the vz votes, long = latch the path off ###
            if (!baroCtrlFresh) vzBlockUntil = now + VZ_SETTLE_MS;
            if (vzTrusted && (baroLastMs == 0 || now - baroLastMs > VZ_BARO_GAP_MS)) {
                vzTrusted = false;
                logEvent("[WARN] vz deploy path disabled (baro gap in flight)");
                // ! queued - the deploy checks below must not wait on RPC
            }
            //### IMU death in flight: same one-way latch. LAUNCH-only on purpose:
            //    a pad connector blip must not burn the flight's primary path ###
            if (vzTrusted && !imuAlive) {
                vzTrusted = false;
                logEvent("[WARN] vz deploy path disabled (IMU dead in flight)");
            }
            bool vzUsable = vzUsableNow(now, vz);   // ! same helper as sensorFlags
            bool deployArmed = (s.time - Launch_Time) >= DEPLOY_LOCKOUT_MS;
            // A 40ms stream can blank vz forever without ever reaching the
            // cadence counter (only <=35ms intervals count). Persistent invalid
            // velocity has the same effect. Bound either condition, then latch
            // the primary off so recovery cannot steal authority back from tilt.
            if (deployArmed && vzTrusted && !vzUsable) {
                if (!vzUnavailableTracking) {
                    vzUnavailableTracking = true;
                    vzUnavailableSince = now;
                } else if ((uint32_t)(now - vzUnavailableSince) >= VZ_UNUSABLE_TIMEOUT_MS) {
                    vzTrusted = false;
                    vwVz.reset();
                    logEvent("[WARN] vz deploy path disabled (unusable for 1000ms)");
                }
            } else {
                vzUnavailableTracking = false;
            }
            bool vzPrimaryAvailable = baroRefValid && baroAlive && vzTrusted;
            if (vzVotePendingReset) {
                vzVotePendingReset = false;
                vwVz.reset();   // ! votes gathered on a bad cadence are void
            }

            //### Deploy ladder: vz primary -> tilt (IMU alive) -> 14s force.
            //    Exactly one voting path per loop; the idle ones hold zero
            //    votes. ALL paths stay locked for the first 5s of flight -
            //    burn-phase transients may not open the chute (README 6.7) ###
            if (vzPrimaryAvailable) {
                vwTilt.reset();
                //### Deploy 1: primary falling speed (global Z velocity) ###
                if (vwVz.update(deployArmed && vzUsable && vz < fallingspeedThreshold)) {
                    enterDeploy("[STATE] DEPLOY (vz<-7m/s) - servo active");
                    break;
                }
            } else if (imuAlive) {
                vwVz.reset();
                //### Deploy 2: tilt backup = direction cosine vs vertical, locked 5s ###
                if (vwTilt.update(deployArmed && imuCtrlFresh &&
                                  cosf(euler_x * DEG_TO_RAD) * cosf(euler_y * DEG_TO_RAD)
                                      > TILT_COS_LIMIT)) {
                    enterDeploy("[STATE] DEPLOY (tilt backup) - servo active");
                    break;
                }
            } else {
                vwVz.reset();     // IMU dead: no sensor path may deploy - timer only
                vwTilt.reset();
            }
            //### Deploy 3: force timer. deployArmed is redundant at 14s > 5s;
            //    it guards a future force_deploy_time set below the lockout ###
            if (deployArmed && s.time - Launch_Time > force_deploy_time) {
                enterDeploy("[STATE] DEPLOY (force 14s) - servo active");
            }
            break;
        }

        case FlightState::DEPLOY: {
            //### Static landing watch - refreshed every loop, consumed in phase 1 ###
            static float staticRefAlt = 0;
            float g2 = s.gyro[0]*s.gyro[0] + s.gyro[1]*s.gyro[1] + s.gyro[2]*s.gyro[2];
            float a2 = s.accel[0]*s.accel[0] + s.accel[1]*s.accel[1] + s.accel[2]*s.accel[2];
            bool moving = imuAlive &&      // ! imuAlive on purpose: stale = quiet
                (g2 > STATIC_GYRO_DPS * STATIC_GYRO_DPS ||
                 a2 > STATIC_ACCEL_G  * STATIC_ACCEL_G);
            // ! vector norm, not per-axis: [0.15,0.15,0.15]g is 0.26g of real
            //   motion but passed three separate 0.2g tests
            if (fabsf(alt - staticRefAlt) > STATIC_ALT_BAND_M) {
                staticRefAlt = alt;
                vwStatic.reset();     // ! hard reset, not a vote (README 6.7)
            }
            bool staticLanded = vwStatic.update(!moving);

            //### Parachute cycle: open -> 1000ms -> check -> close 500ms -> open ###
            // phase 0 is only the jam-retry re-open; the first open happens in
            // enterDeploy, before the state even changes
            switch (deployPhase) {
                case 0:
                    Parachute.write(SERVO_OPEN_DEG);
                    servoReopenGuard.detach();   // normal reopen happened in time
                    deployTimer = now;
                    deployPhase = 1;
                    break;
                case 1:
                    // checked only while OPEN -> LANDED always ends open
                    if (now - deployTimer >= 1000) {
                        if (staticLanded && !moving) {
                            // ! !moving is mandatory: the 2500-slot ring keeps
                            //   up to 500 unwritten slots after 2000 quiet votes,
                            //   so history alone can say "landed" for ~5s while
                            //   the rocket is ALREADY moving again
                            flightState = FlightState::LANDED;
                            logMsg("[STATE] LANDED (static 15s of last 25s)");
                        } else {
                            Parachute.write(SERVO_CLOSE_DEG);
                            servoReopenGuard.attach(&servoReopenIsr, 600ms);
                            // ! fail-open guard: if the loop dies right after
                            //   this CLOSE, the ISR reopens at 600ms anyway
                            deployTimer = now;
                            deployPhase = 2;
                        }
                    }
                    break;
                case 2:
                    if (now - deployTimer >= 500) deployPhase = 0;
                    break;
            }
            break;
        }

        case FlightState::LANDED:
            break;
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~ [12] Startup Checks / Zeroing ~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void servo_selftest() {
    logMsg("[TEST] Servo sweep: lock120-open0-lock120");
    Parachute.write(SERVO_CLOSE_DEG); delay(1000);
    Parachute.write(SERVO_OPEN_DEG);  delay(1000);
    Parachute.write(SERVO_CLOSE_DEG); delay(1000);
    logMsg("[TEST] Servo sweep OK");
}

void imu_preflight_zero() {
    logMsg("[CAL] IMU gyro zero (5s) - KEEP STILL");
    delay(1000);
    IMU.print("<cg>");
    delay(3000);
    logMsg("[CAL] IMU gyro zero done");

    IMU.print("<posz>");
    delay(1000);
    logMsg("[ZERO] IMU pos/vel reset");

    delay(500);
    logMsg("[CAL] IMU magnetometer (15s) - ROTATE ALL DIRECTIONS");
    IMU.print("<cmf>");
    delay(15000);
    IMU.print(">");
    // scan the reply for <ok>/<er> only - dumping raw bytes would pollute the
    // logs with stream lines and non-printables
    char respBuf[128];
    size_t rp = 0;
    uint32_t start = millis();
    while (millis() - start < 1000) {
        if (IMU.available()) {
            char c = IMU.read();
            if (rp < sizeof(respBuf) - 1) respBuf[rp++] = c;
        }
    }
    respBuf[rp] = '\0';
    const char* resp = strstr(respBuf, "<ok>") ? "ok"
                     : strstr(respBuf, "<er>") ? "er" : "none";
    char msg[64];
    snprintf(msg, sizeof(msg), "[CAL] IMU magnetometer done (resp: %s)", resp);
    logMsg(msg);
}

// Sets "pad reference pressure = altitude zero". The chip's own correction
// coefficients are factory-burned and applied per sample by the driver.
void baro_set_reference() {
    Barometer.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    Barometer.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    Barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bool modeOK = Barometer.setupNormalMode(BMP3_ODR_50_HZ);
    if (!modeOK) logMsg("[WARN] Baro normal mode setup failed");
    logMsg("[ZERO] Baro ground reference (6s)...");
    delay(1000);

    //### 6s of samples; every one goes to SD only (~300 lines) ###
    float    pressureSum   = 0;
    uint32_t pressureCount = 0;
    uint32_t rejectCount   = 0;
    float    pMin = 0, pMax = 0;
    char     buffer[96];
    uint32_t start = millis();
    while (millis() - start < 6000) {
        if (Barometer.readNonBlocking()) {
            float p  = (float)Barometer.pressure / 100.0f;
            float t  = (float)Barometer.temperature;
            bool  ok = (p > 300.0f && p < 1250.0f);
            if (ok) {
                if (pressureCount == 0) { pMin = pMax = p; }
                else if (p < pMin)      { pMin = p; }
                else if (p > pMax)      { pMax = p; }
                pressureSum += p;
                pressureCount++;
            } else {
                rejectCount++;
            }
            snprintf(buffer, sizeof(buffer), "[BARO] n=%lu t=%lums p=%.2fhPa T=%.2fC %s",
                     (unsigned long)(pressureCount + rejectCount),
                     (unsigned long)(millis() - start),
                     (double)p, (double)t, ok ? "OK" : "REJECT");
            logMsg(buffer, ipc::TYPE_LOG_SD);   // SD only - keeps ~300 lines off the XBEE
        }
        delay(20);
    }
    if (pressureCount > 0) {
        basePressure = pressureSum / (float)pressureCount;
        snprintf(buffer, sizeof(buffer), "[ZERO] samples=%lu reject=%lu min=%.2f max=%.2f spread=%.2f hPa",
                 (unsigned long)pressureCount, (unsigned long)rejectCount,
                 (double)pMin, (double)pMax, (double)(pMax - pMin));
        logMsg(buffer);
        snprintf(buffer, sizeof(buffer), "[ZERO] Ground reference set: %.2f hPa", basePressure);
        logMsg(buffer);
    } else {
        snprintf(buffer, sizeof(buffer), "[WARN] Baro zero failed: no valid readings (reject=%lu)",
                 (unsigned long)rejectCount);
        logMsg(buffer);
    }

    //### One quality gate arms BOTH baro paths ###
    // A sensor that fails on reject/spread is not one whose DERIVATIVE (vz)
    // should trigger the parachute, so the altitude path and the vz path share
    // the verdict. Accel launch + tilt + 14s are untouched (README 6.7).
    bool refQualityOK = modeOK && pressureCount >= 200 && rejectCount <= 20 &&
                        (pMax - pMin) <= 0.5f;
    if (refQualityOK) {
        baroRefValid = true;
    } else {
        vzTrusted = false;
        logMsg("[WARN] vz deploy path disabled (baro quality)");
        if (pressureCount > 0) {
            snprintf(buffer, sizeof(buffer),
                     "[WARN] Baro reference quality fail (mode=%s n=%lu rej=%lu spread=%.2f)",
                     modeOK ? "ok" : "fail", (unsigned long)pressureCount,
                     (unsigned long)rejectCount, (double)(pMax - pMin));
            logMsg(buffer);
        }
        logMsg("[WARN] Altitude launch path disabled (no ground reference)");
    }
}

static bool baro_begin_retry(uint32_t maxMs, uint32_t intervalMs) {
    uint32_t start = millis();
    uint8_t tries = 0;

    while ((uint32_t)(millis() - start) <= maxMs) {
        tries++;
        if (Barometer.begin_I2C(0x76, &Wire2)) {
            if (tries > 1) {
                char m[80];
                snprintf(m, sizeof(m),
                         "[INIT] Baro BMP390 recovered during boot retry (try=%u)",
                         (unsigned)tries);
                logMsg(m);
            }
            return true;
        }

        uint32_t elapsed = millis() - start;
        if (elapsed >= maxMs || intervalMs > maxMs - elapsed) break;
        delay(intervalMs);
    }
    return false;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~ [13] Sensor Check (does every sensor actually respond?) ~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Call AFTER Wire2/IMU/GNSS begin(), BEFORE any cal/zeroing work.
// Servo has no electrical feedback; M4/SD are covered by RPC.begin + [SYS].
void sensor_check() {
    logMsg("[INIT] Sensor check (baro retry up to 10s, UART 2s)...");

    baroAlive = baro_begin_retry(BARO_BOOT_RETRY_WINDOW_MS,
                                 BARO_BOOT_RETRY_INTERVAL_MS);

    static char ib[128];  size_t ip = 0;
    parsers::UbxPosllh gub;
    uint32_t imuLines = 0, imuValid = 0, gnssFrames = 0;
    bool     gnssFix  = false;
    // gnssFrames   : checksum-valid UBX-POSLLH frames (expect ~50 in 2s at 25Hz)
    // gnssFix      : any frame with recovery-grade hAcc (indoors fix=no is normal)

    uint32_t start = millis();
    while (millis() - start < 2000) {
        while (IMU.available()) {
            char c = IMU.read();
            if (c == '\r') continue;
            if (c == '\n') {
                ib[ip] = '\0'; ip = 0;
                if (ib[0] == '*') {
                    imuLines++;
                    float v[12];
                    // ! same bar as runtime parseIMU (format AND physical range)
                    if (parsers::parseImuLine(ib, v) && imuSampleSane(v)) imuValid++;
                }
                continue;
            }
            if (ip < sizeof(ib) - 1) ib[ip++] = c; else ip = 0;
        }
        while (GNSS.available()) {
            if (gub.feed((uint8_t)GNSS.read())) {
                gnssFrames++;
                if (gub.hAccM <= GNSS_HACC_MAX_M) gnssFix = true;
            }
        }
    }

    char m[96];
    logMsg(baroAlive ? "[INIT] Baro BMP390 OK (I2C 0x76)"
                     : "[WARN] Baro BMP390 no response - check I2C wiring/power");

    imuAlive = (imuValid > 0);       // wrong format is as unusable as silence
    gnssAlive = (gnssFrames > 0);
    if (imuValid > 0) {
        snprintf(m, sizeof(m), "[INIT] IMU OK (%lu/%lu lines valid, %lu Hz)",
                 (unsigned long)imuValid, (unsigned long)imuLines, (unsigned long)(imuValid / 2));
    } else if (imuLines > 0) {
        snprintf(m, sizeof(m), "[WARN] IMU responding but format invalid (%lu lines, need 12 fields)",
                 (unsigned long)imuLines);
    } else {
        snprintf(m, sizeof(m), "[WARN] IMU no response - check power/TX wiring");
    }
    logMsg(m);

    if (gnssFrames > 0) {
        snprintf(m, sizeof(m), "[INIT] GNSS OK (%lu ubx frames, ~%luHz, fix=%s)",
                 (unsigned long)gnssFrames, (unsigned long)(gnssFrames / 2),
                 gnssFix ? "yes" : "no");
    } else {
        snprintf(m, sizeof(m), "[WARN] GNSS no response - check power/wiring/UART1 cfg (38400 UBX)");
    }
    logMsg(m);

    snprintf(m, sizeof(m), "[INIT] Sensor summary: baro=%s imu=%s gnss=%s servo=n/a",
             baroAlive ? "OK" : "NO",
             imuValid > 0 ? "OK" : (imuLines > 0 ? "BAD" : "NO"),
             gnssFrames > 0 ? "OK" : "NO");
    logMsg(m);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~ [14] Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void setup() {
    Parachute.attach(2);
    Serial.begin(115200);
    //### RPC failure is the one event no log can carry (logMsg rides on RPC,
    //    and without M4 there is no XBEE and no SD). Signal it on the board and
    //    STOP - flying with neither logging nor telemetry is not allowed.
    //    Portenta RGB LED is active LOW; digitalWrite configures the pin lazily.
    rpcOK = RPC.begin();
    if (!rpcOK) {
        Serial.println("[ERROR] RPC.begin() failed - M4 미부팅/통신실패");
        while (true) {
            digitalWrite(LEDR, LOW);
            delay(200);
            digitalWrite(LEDR, HIGH);
            delay(800);
        }
    }
    // ! the transmitter must exist before the first logMsg (HELLO_GROUND);
    //   without it nothing ever leaves the board -> same fatal treatment
    if (txThread.start(mbed::callback(txThreadFn)) != osOK) {
        Serial.println("[ERROR] RPC tx thread start failed");
        while (true) {
            digitalWrite(LEDR, LOW);
            delay(200);
            digitalWrite(LEDR, HIGH);
            delay(800);
        }
    }
    GNSS.begin(38400);         // NEO-M9N UART1 (UBX-only, measured 2026-08-04)
    IMU.begin(230400);         // <sb6>, persisted in the IMU's own NVM
    Wire2.begin();
    delay(BARO_BOOT_SETTLE_MS); // give M4 time to boot and BMP390/I2C time to settle
    logMsg("[INIT] HELLO_GROUND");

    sensor_check();

    Kalmanfilter.initialize();
    flightState = FlightState::STARTUP;

    //### Boot checks & zeroing - skipped for dead sensors ###
    servo_selftest();     delay(1000);
    if (imuAlive) { imu_preflight_zero(); delay(1000); }
    else          logMsg("[WARN] IMU cal/zero skipped (no response)");
    if (baroAlive) { baro_set_reference(); delay(1000); }
    else {
        logMsg("[WARN] Baro ground reference skipped (no response) - altitude launch path disabled");
        vzTrusted = false;
    }

    // The 14s timer only starts AFTER LAUNCH, so it cannot back up a failed
    // launch DETECTION - with both paths gone the servo never fires (README 6.8).
    if (!imuAlive && !baroRefValid)
        logMsg("[ERROR] NO LAUNCH DETECTION PATH - accel and altitude both unavailable");

    flightState = FlightState::PRELAUNCH;

    // ! re-stamp: sensor_check ran ~30s ago, so without this the first loop
    //   would declare every sensor dead
    uint32_t tH = millis();
    if (imuAlive)  imuLastMs  = tH;
    if (baroAlive) baroLastMs = tH;
    if (gnssAlive) gnssLastMs = tH;

    Setup_Time    = millis();
    Setup_Time_us = micros();
    // log prefixes switch from boot ms to mission ms exactly here
    char mc[64];
    snprintf(mc, sizeof(mc), "[ZERO] mission clock start (boot +%lums)",
             (unsigned long)Setup_Time);
    logMsg(mc);
    logMsg("[STATE] PRELAUNCH");
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~ [15] Main Loop (100Hz) ~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void loop() {
    uint32_t loopT0 = micros();   // worst-case work-time instrumentation
    parseGNSS(d);
    parseIMU(d);
    d.time = millis() - Setup_Time;

    if (parseBaro(d)) parseKalman(d);   // ! UKF at 50Hz, everything else 100Hz

    updateSensorHealth();
    runDecision(d);

    switch (flightState) {
        case FlightState::LAUNCH: d.state = 0xBB; break;
        case FlightState::DEPLOY: d.state = 0xBC; break;
        case FlightState::LANDED: d.state = 0xBD; break;
        default:                  d.state = 0xBA; break;
    }

    d.health = sensorFlags();
    ipcSendData(d);
    flushEvents();
    // ! the ONLY exit for deferred warnings - always after runDecision, and
    //   after the data frame so M4's record order stays data-then-text

    //### Baro cadence: arm once, then keep watching (post-decision zone) ###
    // ! the UKF hardcodes dt=0.02s, so a systematically different sensor rate
    //   scales every velocity estimate. vzCadenceOK starts false: vz stays
    //   disarmed until the first CADENCE_ARM_N clean intervals prove the rate,
    //   then every CADENCE_BLOCK_N intervals re-check it (a mid-flight slowdown
    //   raises the mean; phase jitter 10/30ms does not move it). Any failure is
    //   permanent for the flight - vzTrusted drops too, so the CSV note reads
    //   vz_off, not the transient vz_settle. Outliers are excluded upstream
    //   (single dropped samples are the per-sample gap check's job).
    static bool cadenceArmDone = false;
    if (!cadenceArmDone) {
        if (baroIvalCnt >= CADENCE_ARM_N) {
            cadenceArmDone = true;
            float mean = (float)baroIvalSum / (float)baroIvalCnt;
            baroIvalSum = 0; baroIvalCnt = 0;
            vzCadenceOK = (mean >= CADENCE_MEAN_LO_MS && mean <= CADENCE_MEAN_HI_MS);
            if (!vzCadenceOK) vzTrusted = false;
            char m[96];
            snprintf(m, sizeof(m), "[%s] Baro cadence: n=%u mean=%.2fms (UKF assumes 20.00) %s",
                     vzCadenceOK ? "ZERO" : "WARN", (unsigned)CADENCE_ARM_N, (double)mean,
                     vzCadenceOK ? "OK" : "-> vz deploy path disabled");
            logMsg(m);
        } else if (d.time > (float)CADENCE_DEADLINE_MS) {
            // never collected enough clean intervals - fail closed, but SAY so
            // (a boot-dead baro stays quiet: "Baro lost" already told the story)
            cadenceArmDone = true;
            if (baroAlive && vzTrusted) {
                vzTrusted = false;
                char m[96];
                snprintf(m, sizeof(m),
                         "[WARN] Baro cadence: insufficient clean samples (n=%lu) - vz deploy path disabled",
                         (unsigned long)baroIvalCnt);
                logMsg(m);
            }
        }
    } else if (baroIvalCnt >= CADENCE_BLOCK_N) {
        float mean = (float)baroIvalSum / (float)baroIvalCnt;
        baroIvalSum = 0; baroIvalCnt = 0;
        // ! reset is unconditional - inside the vzCadenceOK branch the counters
        //   would never reset again after a trip
        if (vzCadenceOK && (mean < CADENCE_MEAN_LO_MS || mean > CADENCE_MEAN_HI_MS)) {
            vzCadenceOK = false;
            vzTrusted   = false;
            vwVz.reset();   // scaled votes gathered before the trip are void
            logMsg("[WARN] vz deploy path disabled (baro cadence drift)");
        }
    }

    //### RPC link report (slow writes seen by txThread + ring/event drops) ###
    static uint32_t lastRpcLog = 0, dropsReported = 0, evtReported = 0;
    uint32_t dropsNow = g_rpcDrops;
    if ((g_rpcWarn || dropsNow != dropsReported || g_evtDrops != evtReported) &&
        millis() - lastRpcLog >= 5000) {
        lastRpcLog = millis();
        char m[96];
        snprintf(m, sizeof(m), "[WARN] RPC link: slow=%luus dropped=%lu evdrop=%lu",
                 (unsigned long)g_rpcWarnUs, (unsigned long)dropsNow,
                 (unsigned long)g_evtDrops);
        g_rpcWarn = false;
        dropsReported = dropsNow;
        evtReported = g_evtDrops;
        logMsg(m);
    }

    //### Loop timing report: worst work-time + skipped deadlines ###
    static uint32_t loopMaxUs = 0, dlSkips = 0, lastLoopLog = 0, skipsReported = 0;
    uint32_t work = micros() - loopT0;
    if (work > loopMaxUs) loopMaxUs = work;
    if ((loopMaxUs > 5000 || dlSkips != skipsReported) && millis() - lastLoopLog >= 10000) {
        lastLoopLog = millis();
        char m[80];
        snprintf(m, sizeof(m), "[WARN] loop max %luus, deadline skips=%lu",
                 (unsigned long)loopMaxUs, (unsigned long)dlSkips);
        loopMaxUs = 0;              // rolling max per report window
        skipsReported = dlSkips;
        logMsg(m);
    }

    //### Fixed 100Hz by absolute deadline; missed deadlines are skipped, ###
    //### never caught up, so the 10ms grid phase holds.                  ###
    static auto next_deadline = Kernel::Clock::now();
    next_deadline += milliseconds(LOOP_PERIOD_MS);
    const auto tick_now = Kernel::Clock::now();
    while (next_deadline < tick_now) {
        next_deadline += milliseconds(LOOP_PERIOD_MS);
        dlSkips++;
    }
    ThisThread::sleep_until(next_deadline);
}
