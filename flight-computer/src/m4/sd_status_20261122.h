#pragma once
#include <cstddef>
#include <cstdio>
#include <cstdint>

namespace sdstatus {
enum class State { Checking, Ready, Recording, Stale, Degraded, Error };
constexpr uint32_t sampleFreshMs=1500;
inline const char* name(State state) {
    switch(state) {
        case State::Checking: return "CHECKING";
        case State::Ready: return "READY";
        case State::Recording: return "RECORDING";
        case State::Stale: return "STALE";
        case State::Degraded: return "DEGRADED";
        default: return "ERROR";
    }
}
inline State evaluate(bool open, uint32_t samples, uint32_t now, uint32_t lastSampleSync,
                      uint32_t sampleDrops, uint32_t eventDrops) {
    if (!open) return State::Error;
    if (samples==0) return State::Ready;
    if (uint32_t(now-lastSampleSync)>sampleFreshMs) return State::Stale;
    if (sampleDrops || eventDrops) return State::Degraded;
    return State::Recording;
}

// Shared by startup probe and normal logging. A sync retry never rewrites data.
template<class File, class Pause>
bool writeAndSync(File& file, const uint8_t* bytes, size_t size, Pause pause) {
    while (size) {
        const auto count=file.write(bytes,size);
        if (count<=0 || size_t(count)>size) return false;
        bytes+=count; size-=size_t(count);
    }
    for (int attempt=0;attempt<3;++attempt) {
        if (file.sync()==0) return true;
        pause();
    }
    return false;
}
template<class File>
bool verifyProbe(File& file,const uint8_t* expected,size_t size) {
    if (file.seek(0,SEEK_SET)!=0) return false;
    uint8_t received[96];
    if(size>sizeof(received)) return false;
    size_t used=0;
    while(used<size) {
        const auto count=file.read(received+used,size-used);
        if(count<=0 || size_t(count)>size-used) return false;
        used+=size_t(count);
    }
    for(size_t i=0;i<size;++i) if(received[i]!=expected[i]) return false;
    return file.seek(0,SEEK_END)==static_cast<decltype(file.seek(0,SEEK_END))>(size);
}
} // namespace sdstatus
