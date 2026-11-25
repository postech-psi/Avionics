#include <Arduino.h>
#include "filesystem/Dir.h"
#include "../shared/qspi_volume.h"

namespace {
qspi::Volume volume;

bool getSession(const char* digits, unsigned& id) {
    if (strlen(digits) != 3) return false;
    id = 0;
    for (unsigned i = 0; i < 3; ++i) {
        if (digits[i] < '0' || digits[i] > '9') return false;
        id = id*10 + digits[i]-'0';
    }
    return true;
}

void listSessions() {
    mbed::Dir dir;
    if (dir.open(&volume.fs,"/") != 0) { Serial.println("ERR directory"); return; }
    dirent entry;
    ssize_t result;
    while ((result=dir.read(&entry)) > 0) {
        const char* name = entry.d_name;
        if (strlen(name)==12 && strncmp(name,"qspi_",5)==0 && strcmp(name+8,".bin")==0)
            Serial.println(name);
    }
    dir.close();
    Serial.println(result < 0 ? "ERR directory" : "OK");
}

void claimEmptyVolume() {
    if (volume.claimed()) { Serial.println("OK already claimed"); return; }
    mbed::Dir dir;
    if (dir.open(&volume.fs,"/") != 0) { Serial.println("ERR directory"); return; }
    dirent entry;
    ssize_t result;
    while ((result=dir.read(&entry)) > 0) {
        if (strcmp(entry.d_name,".") != 0 && strcmp(entry.d_name,"..") != 0) break;
    }
    dir.close();
    if (result != 0) { Serial.println("ERR volume must be empty"); return; }
    mbed::File file;
    if (file.open(&volume.fs,qspi::ownerFile,O_WRONLY | O_CREAT | O_EXCL) != 0) {
        Serial.println("ERR claim open"); return;
    }
    const bool written = file.write(qspi::ownerTag,sizeof(qspi::ownerTag)-1) == sizeof(qspi::ownerTag)-1;
    const bool synced = written && file.sync() == 0;
    const int closed = file.close();
    Serial.println(synced && closed == 0 && volume.claimed() ? "OK claimed" : "ERR claim write");
}

void exportFile(unsigned id) {
    if (!volume.claimed()) { Serial.println("ERR unclaimed"); return; }
    char name[24];
    qspi::sessionName(name,id);
    mbed::File file;
    if (file.open(&volume.fs,name,O_RDONLY) != 0) { Serial.println("ERR file"); return; }
    const auto size = file.size();
    if (size <= 0 || uint64_t(size) > qspi::sessionLimit) { file.close(); Serial.println("ERR size"); return; }
    Serial.print("BIN ");
    Serial.println((unsigned long)size);
    uint8_t buffer[512];
    size_t remaining = static_cast<size_t>(size);
    while (remaining > 0) {
        const auto count = file.read(buffer,remaining > sizeof(buffer) ? sizeof(buffer) : remaining);
        if (count <= 0 || Serial.write(buffer,count) != size_t(count)) break;
        remaining -= count;
    }
    const int closed = file.close();
    if (remaining == 0 && closed == 0) Serial.println("OK");
    // A partial transfer deliberately omits the footer.
}

void handle(const char* command) {
    if (strcmp(command,"INFO") == 0) {
        Serial.print("FLASH "); Serial.println((unsigned long)volume.bytes);
        Serial.print("ERASE "); Serial.println((unsigned long)volume.eraseBytes);
        Serial.print("LAYOUT "); Serial.println(volume.layoutOK ? "OK" : "UNRECOGNIZED");
        Serial.print("LITTLEFS "); Serial.println(volume.mounted ? "OK" : "UNAVAILABLE");
        Serial.print("CLAIMED "); Serial.println(volume.claimed() ? "YES" : "NO");
        Serial.println("OK");
        return;
    }
    if (!volume.mounted) { Serial.println("ERR mount"); return; }
    if (strcmp(command,"LIST") == 0) listSessions();
    else if (strcmp(command,"CLAIM") == 0) claimEmptyVolume();
    else if (strncmp(command,"GET ",4) == 0) {
        unsigned id;
        if (getSession(command+4,id)) exportFile(id);
        else Serial.println("ERR session number");
    } else Serial.println("ERR command");
}
} // namespace

void setup() {
    Serial.begin(115200);
    volume.begin();
}

void loop() {
    static char line[16];
    static size_t length = 0;
    while (Serial.available()) {
        const int b = Serial.read();
        if (b < 0) break;
        if (b == '\n') {
            if (length < sizeof(line)) {
                line[length] = '\0';
                handle(line);
            } else Serial.println("ERR command");
            length = 0;
        } else if (b < 32 || b > 126) {
            length = sizeof(line);
        } else if (length < sizeof(line)) {
            line[length++] = char(b);
        }
    }
    delay(1);
}
