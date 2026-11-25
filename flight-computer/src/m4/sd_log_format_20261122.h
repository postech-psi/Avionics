#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace sdlog {

constexpr uint8_t fileHeader[] = {'P', 'S', 'I', 'L', 'O', 'G', 1, 0};
constexpr uint8_t dataRecord = 1;
constexpr uint8_t eventRecord = 2;
constexpr size_t recordOverhead = 7;

// CRC-16/CCITT-FALSE over the type, little-endian length and payload.
inline uint16_t crc16(const uint8_t* data, size_t size) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            const bool topBit = (crc & 0x8000) != 0;
            crc <<= 1;
            if (topBit) crc ^= 0x1021;
        }
    }
    return crc;
}

// The caller reserves length + recordOverhead bytes in its batch.
inline size_t encode(uint8_t* out, uint8_t type, const void* payload, uint16_t length) {
    out[0] = 0xA5;
    out[1] = 0x5A;
    out[2] = type;
    out[3] = static_cast<uint8_t>(length);
    out[4] = static_cast<uint8_t>(length >> 8);
    memcpy(out + 5, payload, length);
    const uint16_t crc = crc16(out + 2, 3 + length);
    out[5 + length] = static_cast<uint8_t>(crc);
    out[6 + length] = static_cast<uint8_t>(crc >> 8);
    return length + recordOverhead;
}

}  // namespace sdlog
