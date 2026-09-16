//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~ main_m4.cpp - Portenta H7 M4 core: telemetry gateway + SD logger ~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// RPC receive -> (1) 100Hz CSV to SD  (2) decimated XBEE downlink  (3) text logs.
// Serial1(XBEE) and the SD card belong to M4 exclusively.
//
// How it works, the priority rule that makes logging lossless, the IPC checksum,
// SD durability limits, upload order : README 5, 6.1, 6.10
//
// [File map]  [1] XBEE Downsampling   [2] SD Logging (CSV/note)  [3] IPC Receive
//             [4] SD Writer Thread    [5] Setup                  [6] Main Loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#include <Arduino.h>
#include <RPC.h>
#include <string.h>
#include "../shared/telemetry_frame.h"
#include "../shared/ipc_protocol.h"
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "mbed.h"
#include <cstdio>
// mbed.h also provides the POSIX retarget declarations (fsync etc.)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~ [1] XBEE Telemetry Downsampling (user set) ~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// 1=100Hz, 2=50Hz, 4=25Hz, 5=20Hz. SD always records the full 100Hz.
static const uint8_t TELEM_DECIMATE = 4;
static uint8_t       g_telemCount   = 0;

static uint8_t  g_frame[FRAME_LEN];
static bool     g_haveFrame = false;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~ [2] SD Logging (full 100Hz frames to the card) ~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
static SDMMCBlockDevice        sd_bd;
static mbed::FATFileSystem     sd_fs("fs");
static FILE*                   sdFile = nullptr;
static volatile bool           sdOK   = false;
struct SDRec { uint8_t b[FRAME_LEN]; };
static mbed::CircularBuffer<SDRec, 512> sdRing;   // ~44KB = ~5s at 100Hz

static const char CSV_HEADER[] =
    "time_ms,state,lat,lon,euler_x,euler_y,euler_z,accel_x,accel_y,accel_z,"
    "gyro_x,gyro_y,gyro_z,pos_x,pos_y,pos_z,altitude,pressure,kf_alt,kf_vel,apogee,r2,"
    "gnss_alt,note\n";

static const char* stateName(uint8_t s) {
    switch (s) {
        case 0xBA: return "prelaunch";
        case 0xBB: return "launch";
        case 0xBC: return "deploy";
        case 0xBD: return "ground";
        default:   return "unknown";
    }
}

//### health byte -> CSV note column (values documented in README 5) ###
static void healthNote(char* out, size_t cap, uint8_t h) {
    if (h == TLM_HEALTH_ALL) { snprintf(out, cap, "ok"); return; }
    const char* tok[8];
    uint8_t cnt = 0;
    if      (!(h & TLM_IMU_ALIVE))  tok[cnt++] = "imu_dead";
    else if (!(h & TLM_IMU_FRESH))  tok[cnt++] = "imu_stale";
    if      (!(h & TLM_BARO_ALIVE)) tok[cnt++] = "baro_dead";
    else if (!(h & TLM_BARO_FRESH)) tok[cnt++] = "baro_stale";
    if      (!(h & TLM_GNSS_ALIVE)) tok[cnt++] = "gnss_dead";
    if      (!(h & TLM_BARO_REF))   tok[cnt++] = "no_baro_ref";
    if      (!(h & TLM_VZ_TRUSTED)) tok[cnt++] = "vz_off";
    else if (!(h & TLM_VZ_USABLE))  tok[cnt++] = "vz_settle";
    size_t n = 0;
    for (uint8_t i = 0; i < cnt && n + 1 < cap; ++i) {
        int w = snprintf(out + n, cap - n, i ? "|%s" : "%s", tok[i]);
        if (w < 0 || (size_t)w >= cap - n) break;
        n += (size_t)w;
    }
    if (n == 0) snprintf(out, cap, "ok");
}

static int csvLine(char* out, size_t cap, const PackedSensorData& d) {
    char note[56];
    healthNote(note, sizeof(note), d.health);
    return snprintf(out, cap,
        "%.0f,%s,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.4f,%.1f,%s\n",
        (double)d.time, stateName(d.state), (double)d.lat, (double)d.lon,
        (double)d.euler[0], (double)d.euler[1], (double)d.euler[2],
        (double)d.accel[0], (double)d.accel[1], (double)d.accel[2],
        (double)d.gyro[0],  (double)d.gyro[1],  (double)d.gyro[2],
        (double)d.pos[0],   (double)d.pos[1],   (double)d.pos[2],
        (double)d.altitude, (double)d.pressure,
        (double)d.kalman[0], (double)d.kalman[1], (double)d.kalman[2], (double)d.kalman[3],
        (double)d.gnssAlt, note);
}

static volatile uint32_t       sdWritten = 0;
static volatile uint32_t       sdDrops   = 0;
static volatile uint32_t       fmtDrops  = 0;   // csvLine truncated/failed (huge floats)

//### SD text log (M7 boot-check/state lines mirrored to SD alongside XBEE) ###
static FILE*                   logFile  = nullptr;
static volatile bool           logOK    = false;
struct LogRec { uint8_t len; char line[150]; };
static mbed::CircularBuffer<LogRec, 48> logRing;   // fits the [BARO] burst
static volatile uint32_t       logDrops = 0;
static volatile uint32_t       ipcBadFrames = 0;
static volatile bool           g_sdFailReport = false;
static volatile bool           g_landedReport = false;
static volatile bool           g_ipcBadReport = false;
static bool                    g_ipcBadSeen   = false;

// M7 text arrives already stamped with the M7 mission clock (same axis as the
// CSV time_ms column) and is written as-is. M4's OWN lines get an "[M4 <ms>] "
// prefix so the two clocks can never be confused in one file (README 5).
static void logPush(const uint8_t* text, uint16_t len, bool m4Origin = false) {
    if (!logOK) return;
    LogRec lr;
    int n = 0;
    if (m4Origin) {
        n = snprintf(lr.line, sizeof(lr.line), "[M4 %lu] ", (unsigned long)millis());
        if (n < 0 || (size_t)n >= sizeof(lr.line)) return;
    }
    size_t maxTxt = sizeof(lr.line) - (size_t)n - 1;
    size_t cl = (len > maxTxt) ? (uint16_t)maxTxt : len;
    memcpy(lr.line + n, text, cl);
    lr.line[n + cl] = '\n';
    lr.len = (uint8_t)(n + cl + 1);
    if (logRing.full()) logDrops++; else logRing.push(lr);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~ [3] IPC Receive Path ~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
static void onData(const uint8_t* payload, uint16_t len) {
    if (len != sizeof(PackedSensorData)) return;

    static uint8_t lastState = 0;
    if (payload[0] == 0xBD && lastState != 0xBD) g_landedReport = true;
    lastState = payload[0];

    SDRec r;
    r.b[0] = FRAME_START;
    memcpy(&r.b[1], payload, sizeof(PackedSensorData));
    r.b[FRAME_LEN - 1] = FRAME_END;

    if (sdOK) {
        if (sdRing.full()) sdDrops++; else sdRing.push(r);
    }
    if (++g_telemCount >= TELEM_DECIMATE) {
        g_telemCount = 0;
        memcpy(g_frame, r.b, FRAME_LEN);
        g_haveFrame = true;
    }
}

// ! main(loop) context ONLY - from the writer thread this interleaves with the
//   telemetry frame bytes and corrupts the XBEE stream
static void xbeeLog(const char* s) {
    Serial1.write(XBEE_LOG_PREFIX);
    Serial1.write((const uint8_t*)s, strlen(s));
    Serial1.write((const uint8_t*)"\r\n", 2);
}

static void dispatch(uint8_t type, const uint8_t* payload, uint16_t len) {
    switch (type) {
        case ipc::TYPE_DATA:   onData(payload, len); break;
        case ipc::TYPE_LOG:                        // text -> XBEE + SD
            Serial1.write(XBEE_LOG_PREFIX);
            Serial1.write(payload, len);
            Serial1.write((const uint8_t*)"\r\n", 2);
            logPush(payload, len);
            break;
        case ipc::TYPE_LOG_SD: logPush(payload, len); break;
        default: break;
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~ [4] SD Writer Thread: BelowNormal (never blocks RPC/XBEE) ~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// ! BelowNormal is what makes 100Hz logging lossless - do not raise it
static rtos::Thread sdWriter(osPriorityBelowNormal, 8192, nullptr, "sdwriter");
static void sdWriterFn() {
    static char batch[8192];
    const size_t MAX_LINE = 320;
    size_t   bl = 0;
    size_t   pend = 0;
    uint32_t lastFlush = 0;
    uint8_t  failCnt = 0;
    while (true) {
        if (!sdOK && !logOK) { delay(100); continue; }

        SDRec r;
        // pop only when a full line still fits -> a full batch never loses a popped record
        while (bl + MAX_LINE <= sizeof(batch) && sdRing.pop(r)) {
            PackedSensorData d;
            memcpy(&d, r.b + 1, sizeof(d));
            int n = csvLine(batch + bl, sizeof(batch) - bl, d);
            if (n > 0 && (size_t)n < sizeof(batch) - bl) { bl += (size_t)n; pend++; }
            else fmtDrops++;   // popped record whose line did not fit - counted, never silent
        }
        uint32_t ms = millis();
        if ((uint32_t)(ms - lastFlush) >= 200 && (bl > 0 || !logRing.empty())) {
            lastFlush = ms;
            bool ok = true;
            if (sdOK && bl > 0) {
                if (fwrite(batch, 1, bl, sdFile) == bl &&
                    fflush(sdFile) == 0 && fsync(fileno(sdFile)) == 0) {
                    sdWritten += pend;
                    bl = 0; pend = 0;
                } else ok = false;
            }
            if (logOK && !logRing.empty()) {
                LogRec lr;
                bool wrote = false;
                while (logRing.pop(lr)) {
                    if (fwrite(lr.line, 1, lr.len, logFile) != lr.len) { ok = false; break; }
                    wrote = true;
                }
                if (wrote && (fflush(logFile) != 0 || fsync(fileno(logFile)) != 0)) ok = false;
            }
            if (ok) failCnt = 0;
            else if (++failCnt >= 3) {
                //### Card removed/bad: stop SD to protect what is already written ###
                sdOK = false; logOK = false;
                if (sdFile)  { fclose(sdFile);  sdFile  = nullptr; }
                if (logFile) { fclose(logFile); logFile = nullptr; }
                g_sdFailReport = true;   // ! flag only - see xbeeLog note
            }
        }
        delay(1);
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~ [5] Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void setup() {
    Serial1.begin(115200);
    // 0xAA prefix is mandatory - the ground station discards unprefixed text
    if (!RPC.begin()) xbeeLog("[ERROR] M4 RPC.begin() failed");

    //### SD mount + session rotation (existing files are never overwritten) ###
    int sessionIdx = -1;
    if (sd_fs.mount(&sd_bd) == 0) {
        char csvPath[28], txtPath[28];
        for (int i = 0; i < 1000; ++i) {
            snprintf(csvPath, sizeof(csvPath), "/fs/flight_%03d.csv", i);
            snprintf(txtPath, sizeof(txtPath), "/fs/flight_%03d.txt", i);
            char binPath[28];
            snprintf(binPath, sizeof(binPath), "/fs/flight_%03d.bin", i);
            FILE* p;
            if ((p = fopen(csvPath, "rb"))) { fclose(p); continue; }
            if ((p = fopen(binPath, "rb"))) { fclose(p); continue; }
            if ((p = fopen(txtPath, "rb"))) { fclose(p); continue; }
            sessionIdx = i;
            break;
        }
        if (sessionIdx >= 0) {
            sdFile  = fopen(csvPath, "wb");
            logFile = fopen(txtPath, "wb");
            sdOK  = (sdFile  != nullptr);
            logOK = (logFile != nullptr);
            if (sdOK) fputs(CSV_HEADER, sdFile);
            if (sdOK || logOK) sdWriter.start(mbed::callback(sdWriterFn));
        }
    }
    char sdMsg[72];
    if (sessionIdx >= 0)
        snprintf(sdMsg, sizeof(sdMsg), "[SYS] SD data=%s log=%s file=flight_%03d (100Hz)",
                 sdOK ? "ON" : "OFF", logOK ? "ON" : "OFF", sessionIdx);
    else
        snprintf(sdMsg, sizeof(sdMsg), "[ERROR] SD disabled (mount fail or 1000 sessions full)");
    xbeeLog(sdMsg);
    logPush((const uint8_t*)sdMsg, strlen(sdMsg), true);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~ [6] Main Loop (1ms polling) ~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void loop() {
    //### IPC state machine: 0=START 1=TYPE 2=LEN_L 3=LEN_H 4=payload 5=CHECK ###
    static int      st   = 0;
    static uint8_t  type = 0, ck = 0;
    static uint16_t need = 0, got = 0;
    static uint8_t  payload[ipc::IPC_MAX_PAYLOAD];

    while (RPC.available()) {
        uint8_t b = (uint8_t)RPC.read();
        switch (st) {
            case 0: if (b == ipc::IPC_START) st = 1; break;
            case 1: type = b; ck  = b; st = 2; break;
            case 2: need = b; ck ^= b; st = 3; break;
            case 3:
                need |= (uint16_t)b << 8; ck ^= b;
                got = 0;
                if (need > sizeof(payload)) st = 0;      // abnormal length -> resync
                else st = (need == 0) ? 5 : 4;
                break;
            case 4:
                payload[got++] = b; ck ^= b;
                if (got >= need) st = 5;
                break;
            default:
                // ! verify before anything is believed - a length-only check
                //   accepts frames spliced from two others (README 6.10)
                if (b == ck) dispatch(type, payload, need);
                else {
                    ipcBadFrames++;
                    if (!g_ipcBadSeen) { g_ipcBadSeen = true; g_ipcBadReport = true; }
                }
                st = 0;
                break;
        }
    }

    if (g_haveFrame) {
        g_haveFrame = false;
        Serial1.write(g_frame, FRAME_LEN);
    }

    //### One-shot reports (flags set elsewhere, all sent from this context) ###
    if (g_sdFailReport) {
        g_sdFailReport = false;
        xbeeLog("[ERROR] SD write failed - logging stopped");   // SD is dead: XBEE only
    }
    if (g_ipcBadReport) {
        g_ipcBadReport = false;
        static const char ipcBadMsg[] = "[WARN] IPC frame rejected (checksum) - inter-core byte loss";
        xbeeLog(ipcBadMsg);
        logPush((const uint8_t*)ipcBadMsg, sizeof(ipcBadMsg) - 1, true);
    }
    if (g_landedReport) {
        g_landedReport = false;
        char m[128];
        snprintf(m, sizeof(m),
                 "[SYS] flight summary: written=%lu sd_drops=%lu log_drops=%lu ipc_bad=%lu fmt_drops=%lu",
                 (unsigned long)sdWritten, (unsigned long)sdDrops,
                 (unsigned long)logDrops, (unsigned long)ipcBadFrames,
                 (unsigned long)fmtDrops);
        xbeeLog(m);
        logPush((const uint8_t*)m, strlen(m), true);
    }

    delay(1);   // yield to sdWriter; 1ms polling is ample for 100Hz
}
