#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace qspi {
enum class Command { None, Start, Stop };

class RadioCommands {
public:
    Command feed(uint8_t b) {
        if (b == '\n') {
            const auto starts = matches("PSIQ START");
            const auto stops = matches("PSIQ STOP");
            size_ = 0;
            return starts ? Command::Start : stops ? Command::Stop : Command::None;
        }
        if (size_ < sizeof(line_)) line_[size_] = char(b);
        if (size_ <= sizeof(line_)) ++size_;  // mark any overlong line as invalid
        return Command::None;
    }

private:
    bool matches(const char* command) const {
        return size_ == strlen(command) && memcmp(line_,command,size_) == 0;
    }
    char line_[10];
    size_t size_ = 0;
};

void begin();
void request(Command command);
void push(uint8_t type, const void* payload, size_t length);
// Returns true only when a status line is due.
bool report(char* out, size_t size, uint32_t now);
} // namespace qspi
