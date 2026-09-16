//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~ ipc_protocol.h - M7 <-> M4 inter-core framing (over the RPC stream) ~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//   wire : [IPC_START][TYPE][LEN_L][LEN_H][payload(LEN bytes)][CHECK]
// Inter-core only - M4 re-wraps everything into the XBEE formats before it
// leaves the board. Why CHECK exists (frame splicing) : README 6.10
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace ipc {
    static const uint8_t IPC_START   = 0xA5;
    static const uint8_t TYPE_DATA   = 'D';   // payload = PackedSensorData
    static const uint8_t TYPE_LOG    = 'L';   // payload = text -> XBEE + SD
    static const uint8_t TYPE_LOG_SD = 'S';   // payload = text -> SD only
    static const size_t  IPC_HEADER  = 4;
    static const size_t  IPC_TRAILER = 1;
    static const size_t  IPC_MAX_PAYLOAD = 250;

    // XOR of every byte after IPC_START. Shared so sender/receiver cannot drift.
    inline uint8_t checksum(const uint8_t* frame, size_t len) {
        uint8_t c = 0;
        for (size_t i = 1; i < IPC_HEADER + len; ++i) c ^= frame[i];
        return c;
    }
}
