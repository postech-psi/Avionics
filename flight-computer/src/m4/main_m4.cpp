//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~ main_m4.cpp - Portenta H7 M4 core: telemetry gateway + SD logger ~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// RPC receive -> 100Hz binary SD log (samples and events) + decimated XBee.
// Serial1(XBEE) and the SD card belong to M4 exclusively.
//
// Binary log format and recovery limits: README.md.
//
// [File map]  [1] XBEE Downsampling   [2] SD Logging            [3] IPC Receive
//             [4] SD Writer Thread    [5] Setup                  [6] Main Loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#include <Arduino.h>
#include <RPC.h>
#include <string.h>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include "../shared/telemetry_frame.h"
#include "../shared/ipc_protocol.h"
#include "sd_log_format_20261122.h"
#include "sd_status_20261122.h"
#ifdef AVIONICS_QSPI_LOG
#include "qspi_log.h"
#endif
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "filesystem/File.h"
#include "mbed.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~ [1] XBEE Telemetry Downsampling (user set) ~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// 1=100Hz, 2=50Hz, 4=25Hz, 5=20Hz. SD always records the full 100Hz.
static const uint8_t TELEM_DECIMATE = 4;
static uint8_t       g_telemCount   = 0;

static uint8_t  g_frame[FRAME_LEN];
static bool     g_haveFrame = false;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~ [2] SD Logging (100Hz samples and events) ~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
static SDMMCBlockDevice sd_bd;
static mbed::FATFileSystem sd_fs("fs");
static mbed::File sdFile;
static std::atomic<bool> sdOK{false};
static std::atomic<bool> sdInitDone{false};
static mbed::CircularBuffer<PackedSensorData, 512> sdRing;
static std::atomic<uint32_t> sdWritten{0};
static std::atomic<uint32_t> sdBytes{0}, sdLastSampleSyncMs{0};
static int sdSessionIdx=-1;
static const char* sdSetupPhase="mount";
static uint32_t sdDrops = 0;

struct LogRec { uint8_t len; char line[150]; };
static mbed::CircularBuffer<LogRec, 48> logRing;
static uint32_t logDrops = 0;
static uint32_t ipcBadFrames = 0;
static std::atomic<bool> g_sdFailReport{false};
static bool g_landedReport = false;
static bool g_ipcBadReport = false;
static bool g_ipcBadSeen = false;

// M7 text arrives already stamped with the M7 mission clock (same axis as the
// sample timestamps) and is stored as-is. M4's own lines get an "[M4 <ms>] " prefix.
static void logPush(const uint8_t* text, uint16_t len, bool m4Origin = false) {
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
#ifdef AVIONICS_QSPI_LOG
    qspi::push(sdlog::eventRecord,lr.line,lr.len);
#endif
    if (sdOK.load()) {
        if (logRing.full()) logDrops++; else logRing.push(lr);
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~ [3] IPC Receive Path ~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
static void onData(const uint8_t* payload, uint16_t len) {
    if (len != sizeof(PackedSensorData)) return;
#ifdef AVIONICS_QSPI_LOG
    qspi::push(sdlog::dataRecord,payload,len);
#endif

    static uint8_t lastState = 0;
    if (payload[0] == 0xBD && lastState != 0xBD) g_landedReport = true;
    lastState = payload[0];

    if (sdOK.load()) {
        if (sdRing.full()) {
            sdDrops++;
        } else {
            PackedSensorData sample;
            memcpy(&sample, payload, sizeof(sample));
            sdRing.push(sample);
        }
    }
    if (++g_telemCount >= TELEM_DECIMATE) {
        g_telemCount = 0;
        g_frame[0] = FRAME_START;
        memcpy(g_frame + 1, payload, len);
        g_frame[FRAME_LEN - 1] = FRAME_END;
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
//~ [4] SD Writer Thread: BelowNormal ~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Keep SD I/O and checksums off the RPC/XBee thread.
static rtos::Thread sdWriter(osPriorityBelowNormal, 8192, nullptr, "sdwriter");

static bool writeAndSync(const uint8_t* bytes, size_t size) {
    return sdstatus::writeAndSync(sdFile,bytes,size,[](){ delay(10); });
}

// Called only from loop/setup, so status text cannot interleave XBee frames.
static void reportSdStatus(bool force=false) {
    static uint32_t lastReport=0;
    static sdstatus::State previous=sdstatus::State::Checking;
    const uint32_t now=millis();
    const uint32_t samples=sdWritten.load(std::memory_order_acquire);
    const uint32_t lastSync=sdLastSampleSyncMs.load(std::memory_order_relaxed);
    const bool initialized = sdInitDone.load();
    const auto state = initialized ? sdstatus::evaluate(sdOK.load(),samples,now,lastSync,sdDrops,logDrops)
                                  : sdstatus::State::Checking;
    if (!force && state==previous && uint32_t(now-lastReport)<1000) return;
    previous=state; lastReport=now;
    char message[144];
    snprintf(message,sizeof(message),
        "[SD] %s n=%lu bytes=%lu age=%lu drop=%lu/%lu file=%d",
        sdstatus::name(state),(unsigned long)samples,
        (unsigned long)sdBytes.load(std::memory_order_relaxed),
        (unsigned long)(samples ? uint32_t(now-lastSync) : 0),
        (unsigned long)sdDrops,(unsigned long)logDrops,initialized ? sdSessionIdx : -1);
    xbeeLog(message);
    logPush((const uint8_t*)message,strlen(message),true);
}

static int openSession();

static void sdWriterFn() {
    sdSessionIdx = openSession();
    sdOK.store(sdSessionIdx >= 0);
    sdInitDone.store(true);
    if (!sdOK.load()) return;
    static uint8_t batch[8192];
    uint32_t lastWrite = millis();
    while (sdOK.load()) {
        const uint32_t now = millis();
        if (now - lastWrite < 200 || (sdRing.empty() && logRing.empty())) {
            delay(1);
            continue;
        }
        lastWrite = now;

        size_t used = 0;
        uint32_t samples = 0;
        PackedSensorData sample;
        LogRec event;
        for (;;) {
            const size_t previous = used;
            if (sizeof(batch) - used >= sizeof(sample) + sdlog::recordOverhead
                && sdRing.pop(sample)) {
                used += sdlog::encode(batch + used, sdlog::dataRecord, &sample, sizeof(sample));
                samples++;
            }
            if (sizeof(batch) - used >= sizeof(event.line) + sdlog::recordOverhead
                && logRing.pop(event)) {
                used += sdlog::encode(batch + used, sdlog::eventRecord, event.line, event.len);
            }
            if (used == previous) break;
        }
        if (used > 0 && !writeAndSync(batch, used)) {
            sdOK.store(false);
            g_sdFailReport.store(true);  // XBee notices are sent from loop().
            sdFile.close();
            return;
        }
        sdBytes.fetch_add((uint32_t)used,std::memory_order_relaxed);
        if (samples) {
            sdLastSampleSyncMs.store(millis(),std::memory_order_relaxed);
            sdWritten.fetch_add(samples,std::memory_order_release);
        }
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~ [5] Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
static int openSession() {
    sdSetupPhase="mount";
    if (sd_fs.mount(&sd_bd) != 0) return -1;

    // Reserve old CSV/TXT session numbers too, so the matching PC outputs
    // never collide with names already assigned on this card.
    constexpr const char* extensions[] = {"csv", "txt", "bin"};
    char path[24];
    for (int i = 0; i < 1000; ++i) {
        bool used = false;
        for (const char* ext : extensions) {
            snprintf(path, sizeof(path), "flight_%03d.%s", i, ext);
            struct stat info;
            const int result = sd_fs.stat(path, &info);
            if (result == 0) {
                used = true;
                break;
            }
            if (result != -ENOENT) return -1;
        }
        if (used) continue;

        snprintf(path, sizeof(path), "flight_%03d.bin", i);
        // FatFs can open an existing file with O_CREAT. Use append mode and
        // check the file after opening so the header never overwrites data.
        sdSetupPhase="open";
        if (sdFile.open(&sd_fs, path, O_RDWR | O_CREAT | O_APPEND) != 0) return -1;
        const off_t size = sdFile.size();
        if (size != 0) {
            sdFile.close();
            if (size < 0) return -1;
            continue;
        }
        // A valid BIN event is the probe; no temporary file or format operation.
        uint8_t probe[96];
        memcpy(probe,sdlog::fileHeader,sizeof(sdlog::fileHeader));
        static const char text[]="[SD] boot write/read probe\n";
        const size_t probeSize=sizeof(sdlog::fileHeader) + sdlog::encode(
            probe+sizeof(sdlog::fileHeader),sdlog::eventRecord,text,sizeof(text)-1);
        sdSetupPhase="probe_write";
        if (!writeAndSync(probe,probeSize)) { sdFile.close(); return -1; }
        sdSetupPhase="probe_readback";
        if (!sdstatus::verifyProbe(sdFile,probe,probeSize)) { sdFile.close(); return -1; }
        sdBytes.store((uint32_t)probeSize);
        sdSetupPhase="none";
        return i;
    }
    return -1;
}

void setup() {
    Serial1.begin(115200);
    // 0xAA prefix is mandatory - the ground station discards unprefixed text
    if (!RPC.begin()) xbeeLog("[ERROR] M4 RPC.begin() failed");

    // Mount independently so slow SD startup cannot block RPC, radio or QSPI.
    if (sdWriter.start(mbed::callback(sdWriterFn)) != osOK) {
        sdSetupPhase = "writer_thread";
        sdInitDone.store(true);
    }
#ifdef AVIONICS_QSPI_LOG
    qspi::begin();
#endif
    reportSdStatus(true);
}

static void reportStorage() {
    static bool sdAnnounced = false;
    if (!sdAnnounced && sdInitDone.load()) {
        sdAnnounced = true;
        char message[112];
        if (sdSessionIdx >= 0)
            snprintf(message,sizeof(message),"[SYS] SD BIN flight_%03d.bin; boot probe OK",sdSessionIdx);
        else
            snprintf(message,sizeof(message),"[ERROR] SD disabled (phase=%s)",sdSetupPhase);
        xbeeLog(message);
        logPush((const uint8_t*)message,strlen(message),true);
    }
    reportSdStatus();
#ifdef AVIONICS_QSPI_LOG
    static qspi::RadioCommands commands;
    // Keep serial input work bounded even with a noisy radio link.
    for (unsigned i = 0; i < 32 && Serial1.available(); ++i) {
        const int in = Serial1.read();
        if (in < 0) break;
        const auto command = commands.feed(uint8_t(in));
        if (command != qspi::Command::None) qspi::request(command);
    }
    char status[104];
    if (qspi::report(status,sizeof(status),millis())) xbeeLog(status);
#endif
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
    if (g_sdFailReport.exchange(false)) {
        static const char message[] = "[ERROR] SD write failed - SD logging stopped";
        xbeeLog(message);
        logPush((const uint8_t*)message,sizeof(message)-1,true);
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
                 "[SYS] flight summary: written=%lu sd_drops=%lu log_drops=%lu ipc_bad=%lu",
                 (unsigned long)sdWritten.load(std::memory_order_relaxed),
                 (unsigned long)sdDrops, (unsigned long)logDrops, (unsigned long)ipcBadFrames);
        xbeeLog(m);
        logPush((const uint8_t*)m, strlen(m), true);
    }

    reportStorage();
    delay(1);   // yield to the storage workers
}
