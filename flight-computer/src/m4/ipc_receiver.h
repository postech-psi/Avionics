#pragma once

#include "../shared/ipc_protocol.h"
#include <cstring>

namespace ipc {
enum class Reject : uint8_t { Noise=1, Length, Type, Checksum, Partial };

// Keep the bytes the M4 actually saw. Rejecting a candidate never makes its
// payload trusted. Scan its suffix so an invalid length/checksum cannot hide
// the start of a sound frame already received behind it.
class Receiver {
public:
    template<class Good, class Raw>
    void feed(uint8_t byte, uint32_t now, Good good, Raw raw) {
        if (used_ && frame_[0] != IPC_START && byte == IPC_START) {
            raw(Reject::Noise,offset_,frame_,used_);
            consume(used_);
        }
        if (!used_) offset_ = total_;
        frame_[used_++] = byte;
        ++total_;
        last_ = now;
        process(good,raw);
    }

    template<class Good, class Raw>
    void flushIdle(uint32_t now, Good good, Raw raw) {
        if (uint32_t(now-last_) < 30) return;
        while (used_) {
            const bool partial = frame_[0] == IPC_START;
            raw(partial ? Reject::Partial : Reject::Noise,offset_,frame_,used_);
            auto* next = partial ? static_cast<uint8_t*>(memchr(frame_+1,IPC_START,used_-1)) : nullptr;
            consume(next ? size_t(next-frame_) : used_);
            process(good,raw);
        }
    }

private:
    void consume(size_t count) {
        used_ -= count;
        offset_ += count;
        memmove(frame_,frame_+count,used_);
    }

    template<class Good, class Raw>
    void process(Good good, Raw raw) {
        while (used_) {
            if (frame_[0] != IPC_START) {
                auto* next = static_cast<uint8_t*>(memchr(frame_,IPC_START,used_));
                if (!next && used_ < 64) return;
                const size_t count = next ? size_t(next-frame_) : used_;
                raw(Reject::Noise,offset_,frame_,count);
                consume(count);
                continue;
            }
            if (used_ < IPC_HEADER) return;
            const uint8_t type = frame_[1];
            const size_t len = size_t(frame_[2]) | size_t(frame_[3]) << 8;
            const size_t length = IPC_HEADER+len+IPC_TRAILER;
            Reject reason;
            size_t rejected;
            if (len > IPC_MAX_PAYLOAD || !len || (type == TYPE_DATA && len != dataSize_)) {
                reason = Reject::Length;
                rejected = IPC_HEADER;
            } else if (type != TYPE_DATA && type != TYPE_LOG && type != TYPE_LOG_SD) {
                reason = Reject::Type;
                rejected = IPC_HEADER;
            } else {
                if (used_ < length) return;
                if (checksum(frame_,len) == frame_[length-1]) {
                    good(type,frame_+IPC_HEADER,len);
                    consume(length);
                    continue;
                }
                reason = Reject::Checksum;
                rejected = length;
            }
            raw(reason,offset_,frame_,rejected);
            auto* next = static_cast<uint8_t*>(memchr(frame_+1,IPC_START,rejected-1));
            consume(next ? size_t(next-frame_) : rejected);
        }
    }

public:
    explicit Receiver(size_t dataSize) : dataSize_(dataSize) {}

private:
    const size_t dataSize_;
    uint8_t frame_[maxFrameBytes];
    size_t used_ = 0;
    uint32_t offset_ = 0, total_ = 0, last_ = 0;
};
} // namespace ipc
