#pragma once

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include "QSPIFBlockDevice.h"
#include "blockdevice/SlicingBlockDevice.h"
#include "littlefs/LittleFileSystem.h"
#include "filesystem/File.h"
#include "qspi_partition.h"

namespace qspi {
constexpr char ownerFile[] = "avionics.owner";
constexpr char ownerTag[] = "PSI PSILOG QSPI 1\n";

inline void sessionName(char (&name)[24], unsigned id) {
    snprintf(name,sizeof(name),"qspi_%03u.bin",id);
}

class Volume {
public:
    bool begin() {
        if (flash_.init() != 0) return false;
        uint8_t mbr[512];
        bytes = flash_.size();
        eraseBytes = flash_.get_erase_size();
        const auto readSize = flash_.get_read_size();
        layoutOK = readSize && sizeof(mbr)%readSize == 0 && flash_.read(mbr,0,sizeof(mbr)) == 0 &&
            approvedLayout(mbr,eraseBytes,bytes) && flash_.is_valid_erase(userStart,userEnd-userStart);
        if (flash_.deinit() != 0 || !layoutOK) return false;
        mounted = fs.mount(&partition_) == 0;
        return mounted;
    }

    bool claimed() {
        mbed::File file;
        if (!mounted || file.open(&fs,ownerFile,O_RDONLY) != 0) return false;
        char content[sizeof(ownerTag)];
        const auto n = file.read(content,sizeof(content));
        const int closed = file.close();
        return closed == 0 && n == sizeof(ownerTag)-1 && memcmp(content,ownerTag,n) == 0;
    }

    mbed::LittleFileSystem fs{"qspi"};
    uint64_t bytes = 0;
    uint32_t eraseBytes = 0;
    bool layoutOK = false;
    bool mounted = false;

private:
    QSPIFBlockDevice flash_;
    mbed::SlicingBlockDevice partition_{&flash_, userStart, userEnd};
};
} // namespace qspi
