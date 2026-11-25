#include "../src/m4/ipc_receiver.h"
#include "../src/m4/qspi_window.h"
#include "../src/m4/sd_status_20261122.h"

#include <cassert>
#include <initializer_list>
#include <vector>

int main() {
    ipc::Receiver receiver(2);
    std::vector<uint8_t> accepted;
    std::vector<ipc::Reject> rejected;
    std::vector<uint8_t> rawNoise;
    const auto good = [&](uint8_t, const uint8_t* bytes, size_t len) {
        accepted.insert(accepted.end(),bytes,bytes+len);
    };
    const auto raw = [&](ipc::Reject reason, uint32_t offset, const uint8_t* bytes, size_t len) {
        rejected.push_back(reason);
        if (offset == 0) rawNoise.assign(bytes,bytes+len);
    };
    auto feed = [&](std::initializer_list<uint8_t> bytes, uint32_t now) {
        for (auto byte : bytes) receiver.feed(byte,now,good,raw);
    };
    const uint8_t goodCheck = 'D' ^ 2 ^ 7 ^ 8;
    // The start byte of a good frame is consumed as the previous frame's bad checksum.
    feed({0xfe,0,0xa5,'D',2,0,1,2,0xa5,'D',2,0,7,8,goodCheck},100);
    assert((rawNoise == std::vector<uint8_t>{0xfe,0}));
    assert((accepted == std::vector<uint8_t>{7,8}));
    assert((rejected == std::vector<ipc::Reject>{ipc::Reject::Noise,ipc::Reject::Checksum}));
    // An incomplete log frame can enclose a complete valid frame at the tail.
    feed({0xa5,'L',200,0,32,0xa5,'D',2,0,7,8,goodCheck},200);
    receiver.flushIdle(230,good,raw);
    assert(rejected.back()==ipc::Reject::Partial && accepted.size()==4);

    using qspi::StopReason;
    assert(qspi::timedStop(100+qspi::maxSessionMs-1,100,false,0)==StopReason::None);
    assert(qspi::timedStop(100+qspi::maxSessionMs,100,false,0)==StopReason::Timeout);
    assert(qspi::timedStop(150+qspi::landingTailMs,100,true,150)==StopReason::Landing);
    assert(qspi::timedStop(150+qspi::landingTailMs,150,false,150)==StopReason::None);
    assert(qspi::timedStop(0xfffffff0u+qspi::maxSessionMs,0xfffffff0u,false,0)==StopReason::Timeout);

    struct File {
        int attempts = 0;
        std::vector<uint8_t> bytes;
        int write(const uint8_t* data, size_t length) {
            const size_t count = length > 2 ? 2 : length;
            bytes.insert(bytes.end(),data,data+count);
            return count;
        }
        int sync() { return ++attempts < 3 ? -EIO : 0; }
    } file;
    const uint8_t batch[]{1,2,3,4,5};
    assert(sdstatus::writeAndSync(file,batch,sizeof(batch),[](){}) == 0);
    assert(file.bytes == std::vector<uint8_t>(batch,batch+sizeof(batch)) && file.attempts==3);
}
