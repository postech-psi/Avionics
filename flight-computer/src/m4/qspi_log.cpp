#ifdef AVIONICS_QSPI_LOG
#include <Arduino.h>
#include <atomic>
#include <cerrno>
#include "mbed.h"
#include "qspi_log.h"
#include "qspi_window.h"
#include "sd_status_20261122.h"
#include "sd_log_format_20261122.h"
#include "../shared/qspi_volume.h"

namespace qspi {
namespace {
enum class State { Checking, Unavailable, Unclaimed, Idle, Starting, Recording, Stopping, Full, Error };
constexpr const char* stateNames[] = {"CHECKING","UNAVAILABLE","UNCLAIMED","IDLE","STARTING",
    "RECORDING","STOPPING","FULL","ERROR"};
constexpr size_t stopReserve = 96;
mbed::CircularBuffer<sdlog::Record, 192> queue;
rtos::Thread worker(osPriorityBelowNormal, 8192, nullptr, "qspi");
std::atomic<State> state{State::Checking};
std::atomic<bool> requested{false}, landed{false};
std::atomic<uint32_t> startedAt{0}, landedAt{0};
std::atomic<StopReason> stopReason{StopReason::None};
constexpr const char* stopNames[] = {"none","manual","landing","timeout","limit"};
std::atomic<uint32_t> samples{0}, bytes{0}, drops{0}, lastSampleMs{0};
std::atomic<int> session{-1};

int newFile(Volume& volume, mbed::File& file) {
    for (unsigned i = 0; i < 1000; ++i) {
        char name[24];
        sessionName(name,i);
        const int result = file.open(&volume.fs,name,O_WRONLY | O_CREAT | O_EXCL);
        if (result == 0) return i;
        if (result != -EEXIST) return result;
    }
    return -ENOSPC;
}

int writeBatch(mbed::File& file, const uint8_t* data, size_t size) {
    return sdstatus::writeAndSync(file,data,size,[](){ delay(10); });
}

void requestStop(StopReason reason) {
    if (requested.load()) {
        stopReason.store(reason);
        requested.store(false);
    }
}

void workerLoop() {
    static Volume volume;
    if (!volume.begin()) { state.store(State::Unavailable); return; }
    if (!volume.claimed()) { state.store(State::Unclaimed); return; }
    state.store(State::Idle);
    while (true) {
        if (!requested.load()) { delay(10); continue; }
        stopReason.store(StopReason::None);
        state.store(State::Starting);
        mbed::File file;
        const int id = newFile(volume,file);
        if (id < 0) { state.store(id == -ENOSPC ? State::Full : State::Error); return; }
        const int header = writeBatch(file,sdlog::fileHeader,sizeof(sdlog::fileHeader));
        if (header != 0) { file.close(); state.store(header == -ENOSPC ? State::Full : State::Error); return; }
        session.store(id);
        samples.store(0);
        lastSampleMs.store(0);
        bytes.store(sizeof(sdlog::fileHeader));
        drops.store(0);
        landed.store(false);
        startedAt.store(millis());
        state.store(State::Recording);
        uint32_t lastFlush = millis();
        static uint8_t batch[4096];
        State terminal = State::Idle;
        while (true) {
            const bool stopping = !requested.load();
            if (stopping) state.store(State::Stopping);
            if (queue.empty() && stopping) break;
            const uint32_t now = millis();
            if (queue.empty() || (!stopping && now-lastFlush < 200)) { delay(5); continue; }
            lastFlush = now;
            const uint32_t committed = bytes.load();
            size_t used = 0;
            uint32_t count = 0;
            sdlog::Record record;
            while (sizeof(batch)-used >= sdlog::maxPayload+sdlog::recordOverhead && queue.pop(record)) {
                if (committed+used+record.len+sdlog::recordOverhead > sessionLimit-stopReserve) {
                    state.store(State::Stopping);
                    stopReason.store(StopReason::Limit);
                    requested.store(false);
                    ++drops;
                    while (queue.pop(record)) ++drops;
                    break;
                }
                used += sdlog::encode(batch+used,record.kind,record.payload,record.len);
                count += record.kind == sdlog::dataRecord;
            }
            const int result = used ? writeBatch(file,batch,used) : 0;
            if (result != 0) { terminal = result == -ENOSPC ? State::Full : State::Error; break; }
            bytes.store(committed+used);
            if (count) {
                lastSampleMs.store(millis());
                samples.fetch_add(count);
            }
            if (stopReason.load() == StopReason::Limit) break;
        }
        if (terminal == State::Idle) {
            char footer[72];
            const int n = snprintf(footer,sizeof(footer),"[QSPI] stop=%s n=%lu drop=%lu\n",
                stopNames[unsigned(stopReason.load())],(unsigned long)samples.load(),(unsigned long)drops.load());
            if (n > 0 && size_t(n) < sizeof(footer)) {
                const auto used = sdlog::encode(batch,sdlog::eventRecord,footer,n);
                const int result = writeBatch(file,batch,used);
                if (result == 0) bytes.fetch_add(used);
                else terminal = result == -ENOSPC ? State::Full : State::Error;
            }
        }
        const int closed = file.close();
        if (closed != 0 && terminal == State::Idle) terminal = closed == -ENOSPC ? State::Full : State::Error;
        state.store(terminal);
        // Idle permits another explicit START. Full/error preserve existing files.
        if (terminal != State::Idle) return;
    }
}
} // namespace

void begin() {
    if (worker.start(mbed::callback(workerLoop)) != osOK) state.store(State::Error);
}

void request(Command command) {
    const auto current = state.load();
    if (command == Command::Start && (current == State::Idle || current == State::Checking)) requested.store(true);
    if (command == Command::Stop) requestStop(StopReason::Manual);
}

void noteLanding(uint32_t now) {
    if (state.load() == State::Recording && !landed.load()) {
        landedAt.store(now);
        landed.store(true);
    }
}

void tick(uint32_t now) {
    if (state.load() != State::Recording) return;
    const auto reason = timedStop(now,startedAt.load(),landed.load(),landedAt.load());
    if (reason != StopReason::None) requestStop(reason);
}

void push(uint8_t type, const void* payload, size_t length) {
    if (state.load() != State::Recording || !requested.load() || !length || length > sdlog::maxPayload) return;
    if (queue.full()) { ++drops; return; }
    sdlog::Record record{};
    record.kind = type;
    record.len = length;
    memcpy(record.payload,payload,length);
    queue.push(record);
}

bool report(char* out, size_t size, uint32_t now) {
    static const char* previous = nullptr;
    static uint32_t last = 0;
    const State current = state.load();
    const auto committed = samples.load();
    const auto dropped = drops.load();
    const char* name = stateNames[unsigned(current)];
    if (current == State::Recording) {
        if (committed && int32_t(now-lastSampleMs.load()) > 1500) name = "STALE";
        else if (dropped) name = "DEGRADED";
        else if (!committed) name = "ARMED";
    }
    if (name == previous && now-last < 1000) return false;
    previous = name;
    last = now;
    snprintf(out,size,"[QSPI] %s n=%lu bytes=%lu drop=%lu file=%d stop=%s",name,
        (unsigned long)committed,(unsigned long)bytes.load(),(unsigned long)dropped,session.load(),
        stopNames[unsigned(stopReason.load())]);
    return true;
}
} // namespace qspi
#endif
