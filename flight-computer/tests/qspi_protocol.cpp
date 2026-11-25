#include "../src/m4/qspi_log.h"
#include "../src/shared/qspi_partition.h"

#include <cassert>
#include <cstdio>

namespace {
void putLe32(uint8_t* p, uint32_t n) {
    for (unsigned i=0; i<4; ++i) p[i] = n >> (8*i);
}
}

int main() {
    constexpr uint32_t erase = 4096;
    uint8_t mbr[512]{};
    mbr[510] = 0x55;
    mbr[511] = 0xaa;
    uint8_t* user = mbr+446+3*16;
    user[4] = 0x0b;
    putLe32(user+8,qspi::userStart/erase);
    putLe32(user+12,(qspi::userEnd-qspi::userStart)/erase);
    assert(qspi::approvedLayout(mbr,erase,qspi::flashBytes));
    assert(!qspi::approvedLayout(mbr,erase,8*qspi::MiB));
    assert(!qspi::approvedLayout(mbr,0,qspi::flashBytes));
    putLe32(user+12,0xffffffffu);
    assert(!qspi::approvedLayout(mbr,erase,qspi::flashBytes));
    putLe32(user+12,(qspi::userEnd-qspi::userStart)/erase);
    uint8_t* other = mbr+446;
    other[4] = 0x0b;
    putLe32(other+8,1);
    putLe32(other+12,qspi::userStart/erase);
    assert(!qspi::approvedLayout(mbr,erase,qspi::flashBytes));
    putLe32(other+12,qspi::userStart/erase-1);
    assert(qspi::approvedLayout(mbr,erase,qspi::flashBytes));

    qspi::RadioCommands parser;
    const auto feed = [&](const char* text) {
        qspi::Command result = qspi::Command::None;
        for (; *text; ++text) {
            const auto cmd = parser.feed(*text);
            if (cmd != qspi::Command::None) result = cmd;
        }
        return result;
    };
    assert(feed("PSIQ START") == qspi::Command::None);
    assert(feed("garbage\n") == qspi::Command::None);
    assert(feed("noise\nPSIQ START\n") == qspi::Command::Start);
    assert(feed("PSIQ STOP\n") == qspi::Command::Stop);
    assert(feed("PSIQ STOP extra\n") == qspi::Command::None);
    std::puts("QSPI layout bounds, overlap rejection and command framing passed");
}
