//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~ parsers.h - pure parsing logic, Arduino-free (host-testable) ~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Line-level parsing, separated from the UART byte loops so it builds and runs
// unchanged on a PC. Validity rules and why each check exists : README 6.3
#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cctype>

namespace parsers {

//### NMEA checksum verification ($....*HH) ###
inline bool nmeaChecksumOK(const char* s) {
    if (!s || s[0] != '$') return false;
    uint8_t cs = 0;
    const char* p = s + 1;
    while (*p && *p != '*') cs ^= static_cast<uint8_t>(*p++);
    if (*p != '*') return false;
    auto hex = [](char c) -> int {
        if ('0' <= c && c <= '9') return c - '0';
        if ('A' <= c && c <= 'F') return 10 + (c - 'A');
        if ('a' <= c && c <= 'f') return 10 + (c - 'a');
        return -1;
    };
    if (!p[1] || !p[2]) return false;
    if (p[3] != '\0') return false;   // nothing is allowed after *HH
    int hi = hex(p[1]), lo = hex(p[2]);
    if (hi < 0 || lo < 0) return false;
    return cs == static_cast<uint8_t>((hi << 4) | lo);
}

//### Split on ',' (line is modified in place) ###
inline int splitComma(char* line, char** fields, int maxF) {
    for (char* q = line; *q; ++q) if (*q == '\r' || *q == '\n') *q = '\0';
    int n = 0;
    char* p = line;
    if (*p == '$') ++p;
    fields[n++] = p;
    while (*p && n < maxF) {
        if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
        else if (*p == '*') { *p = '\0'; break; }
        ++p;
    }
    return n;
}

//### ddmm.mmmm -> decimal degrees ###
inline bool nmeaToDeg(const char* v, const char* hemi, bool isLat, float& out) {
    if (!v || !*v || !hemi || !*hemi) return false;

    char h = static_cast<char>(toupper(static_cast<unsigned char>(*hemi)));
    if (isLat) { if (h != 'N' && h != 'S') return false; }   // ! not just S/W
    else       { if (h != 'E' && h != 'W') return false; }

    char* end = nullptr;
    double raw = strtod(v, &end);
    if (end == v || *end != '\0' || !std::isfinite(raw) || raw < 0) return false;

    int degPart = static_cast<int>(floor(raw / 100.0));
    double minPart = raw - degPart * 100.0;
    if (minPart >= 60.0) return false;   // ddmm.mmmm: minutes must be 00-59
    double deg = degPart + (minPart / 60.0);
    if (h == 'S' || h == 'W') deg = -deg;
    if (isLat) { if (deg < -90.0  || deg > 90.0)  return false; }
    else       { if (deg < -180.0 || deg > 180.0) return false; }
    out = static_cast<float>(deg);
    return true;
}

//### Parse one NMEA line: lat/lon from RMC/GGA ###
inline bool parseNmeaLine(const char* nmea, float& lat, float& lon,
                          bool& gotLat, bool& gotLon) {
    gotLat = gotLon = false;
    if (!nmea || nmea[0] != '$') return false;
    if (!nmeaChecksumOK(nmea)) return false;

    char line[128];
    strncpy(line, nmea, sizeof(line));
    line[sizeof(line) - 1] = '\0';

    char* f[24];
    int nf = splitComma(line, f, 24);
    if (nf < 6) return false;

    //### RMC : field 2 = status, 'A' = valid fix ###
    if (!strcmp(f[0], "GNRMC") || !strcmp(f[0], "GPRMC")) {
        if (nf >= 7 && f[2] && f[2][0] == 'A') {
            float la, lo;
            if (nmeaToDeg(f[3], f[4], true,  la)) { lat = la; gotLat = true; }
            if (nmeaToDeg(f[5], f[6], false, lo)) { lon = lo; gotLon = true; }
        }
        return true;
    }
    //### GGA : field 6 = fix quality, '0' = no fix ###
    if (!strcmp(f[0], "GNGGA") || !strcmp(f[0], "GPGGA")) {
        if (nf >= 7 && f[6] && f[6][0] != '\0' && f[6][0] != '0') {
            float la, lo;
            if (nmeaToDeg(f[2], f[3], true,  la)) { lat = la; gotLat = true; }
            if (nmeaToDeg(f[4], f[5], false, lo)) { lon = lo; gotLon = true; }
        }
        return true;
    }
    return true;   // other sentences are valid but carry no update
}

//### Parse one IMU line: "*v0,v1,...,v11" -> vals[12] ###
inline bool parseImuLine(const char* line, float vals[12]) {
    if (!line || line[0] != '*') return false;
    int commaCount = 0;
    for (const char* p = line; *p; ++p) if (*p == ',') ++commaCount;
    if (commaCount != 11) return false;

    int cnt = 0;
    char* s   = const_cast<char*>(line) + 1;
    char* end = s;
    while (cnt < 12) {
        while (*s == ' ' || *s == '\t') ++s;
        float value = strtof(s, &end);
        if (s == end || !std::isfinite(value)) { cnt = -1; break; }   // ! NaN kills conditions
        vals[cnt] = value;
        ++cnt;
        if (*end == ',') { s = end + 1; }
        else if (*end == '\0') { if (cnt != 12) cnt = -1; break; }
        else { cnt = -1; break; }
    }
    return cnt == 12;
}

//### Pressure -> altitude ###
inline float pressureToAltitude(float pressure_hPa, float basePressure_hPa) {
    return 44330.0f * (1.0f - powf(pressure_hPa / basePressure_hPa, 0.1903f));
}

//### UBX-NAV-POSLLH byte-stream parser (NEO-M9N UART1: 38400 8N1, 25Hz) ###
// The module is configured UBX-only on UART1 (no NMEA - measured 2026-08-04):
//   [B5 62][01 02][1C 00][payload 28B][CK_A CK_B]
// Fletcher-8 checksum runs over class..payload. Length-based framing, so a
// 0xB5 inside the payload is harmless. POSLLH carries NO fix-validity flag -
// a no-fix frame reads lat=lon=0 with hAcc ~4,294,967m, so callers MUST gate
// coordinate updates on hAcc (README 6.3). The NMEA functions above are no
// longer used by the firmware (kept: zero flash cost as unused inline, host
// tests still cover them, and the module's USB port still speaks NMEA).
struct UbxPosllh {
    float lat   = 0.0f;     // deg
    float lon   = 0.0f;     // deg
    float hMSLm = 0.0f;     // height above mean sea level, m
    float hAccM = 1.0e9f;   // horizontal accuracy, m (huge until first decode)

    // Feed one byte; returns true exactly when a checksum-valid POSLLH frame
    // was completed and lat/lon/hAccM were refreshed.
    bool feed(uint8_t b) {
        switch (st) {
            case 0: st = (b == 0xB5) ? 1 : 0; break;
            case 1: st = (b == 0x62) ? 2 : ((b == 0xB5) ? 1 : 0); break;
            case 2: ckA = ckB = 0; ck(b); st = (b == 0x01) ? 3 : 0; break;
            case 3: ck(b); st = (b == 0x02) ? 4 : 0; break;
            case 4: ck(b); st = (b == 28)   ? 5 : 0; break;
            case 5: ck(b); idx = 0; st = (b == 0) ? 6 : 0; break;
            case 6: ck(b); pay[idx++] = b; if (idx >= 28) st = 7; break;
            case 7: st = (b == ckA) ? 8 : 0; break;
            default:  // 8: CK_B
                st = 0;
                if (b != ckB) return false;
                lon   = (float)(i4(pay + 4) * 1e-7);
                lat   = (float)(i4(pay + 8) * 1e-7);
                hMSLm = (float)(i4(pay + 16) / 1000.0);
                hAccM = (float)(u4(pay + 20) / 1000.0);
                return true;
        }
        return false;
    }

private:
    uint8_t st = 0, idx = 0, ckA = 0, ckB = 0;
    uint8_t pay[28] = {};
    void ck(uint8_t b) { ckA = (uint8_t)(ckA + b); ckB = (uint8_t)(ckB + ckA); }
    static uint32_t u4(const uint8_t* p) {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }
    static int32_t i4(const uint8_t* p) {
        uint32_t v = u4(p);
        int32_t r;
        memcpy(&r, &v, 4);
        return r;
    }
};

}  // namespace parsers
