#pragma once

#include <cstdint>

namespace qspi {
constexpr uint32_t MiB = 1024 * 1024;
constexpr uint32_t flashBytes = 16 * MiB;
constexpr uint32_t userStart = 7 * MiB;
constexpr uint32_t userEnd = 14 * MiB;
constexpr uint32_t sessionLimit = 5 * MiB;

inline uint32_t loadLe32(const uint8_t* p) {
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}

// Mbed's MBR stores LBA in erase-sized units on this NOR (minimum 512 B).
inline bool approvedLayout(const uint8_t (&mbr)[512], uint32_t eraseBytes, uint64_t size) {
    if (size != flashBytes || !eraseBytes || mbr[510] != 0x55 || mbr[511] != 0xaa) return false;
    const uint32_t sector = eraseBytes < 512 ? 512 : eraseBytes;
    for (int i = 0; i < 4; ++i) {
        const auto* entry = mbr + 446 + 16*i;
        if (i < 3 && entry[4] == 0) continue;
        const uint64_t start = uint64_t(loadLe32(entry+8))*sector;
        const uint64_t length = uint64_t(loadLe32(entry+12))*sector;
        if ((entry[0] != 0 && entry[0] != 0x80) || !length || start < sector ||
            start >= size || length > size-start) return false;
        if (i == 3) {
            if (entry[4] != 0x0b || start != userStart || start+length != userEnd) return false;
        } else if (entry[4] == 0x05 || entry[4] == 0x0f || entry[4] == 0xee ||
                   (start < userEnd && start+length > userStart)) {
            return false;
        }
    }
    return true;
}
} // namespace qspi
