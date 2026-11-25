#ifdef AVIONICS_QSPI_LOG
#include <Arduino.h>
#include <atomic>
#include <cerrno>
#include "mbed.h"
#include "qspi_log.h"
#include "sd_log_format_20261122.h"
#include "../shared/qspi_volume.h"

namespace qspi {
namespace {
enum class State { Checking, Unavailable, Unclaimed, Idle, Starting, Recording, Stopping, Limit, Full, Error };
constexpr const char* stateNames[] = {"CHECKING","UNAVAILABLE","UNCLAIMED","IDLE","STARTING",
    "RECORDING","STOPPING","LIMIT","FULL","ERROR"};
constexpr size_t maxPayload = 150;
struct Record { uint8_t type; uint8_t len; uint8_t payload[maxPayload]; };
mbed::CircularBuffer<Record, 192> queue;
rtos::Thread worker(osPriorityBelowNormal, 8192, nullptr, "qspi");
std::atomic<State> state{State::Checking};
std::atomic<bool> requested{false};
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
    const auto written = file.write(data,size);
    if (written < 0) return static_cast<int>(written);
    if (size_t(written) != size) return -EIO;
    return file.sync();
}

void workerLoop() {
    static Volume volume;
    if (!volume.begin()) { state.store(State::Unavailable); return; }
    if (!volume.claimed()) { state.store(State::Unclaimed); return; }
    state.store(State::Idle);
    while (true) {
        if (!requested.load()) { delay(10); continue; }
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
            Record record;
            while (sizeof(batch)-used >= maxPayload+sdlog::recordOverhead && queue.pop(record)) {
                if (committed+used+record.len+sdlog::recordOverhead > sessionLimit) {
                    terminal = State::Limit;
                    break;
                }
                used += sdlog::encode(batch+used,record.type,record.payload,record.len);
                count += record.type == sdlog::dataRecord;
            }
            const int result = used ? writeBatch(file,batch,used) : 0;
            if (result != 0) { terminal = result == -ENOSPC ? State::Full : State::Error; break; }
            bytes.store(committed+used);
            if (count) {
                lastSampleMs.store(millis());
                samples.fetch_add(count);
            }
            if (terminal != State::Idle) break;
        }
        const int closed = file.close();
        if (closed != 0 && terminal == State::Idle) terminal = closed == -ENOSPC ? State::Full : State::Error;
        state.store(terminal);
        // After a limit, full volume or I/O failure, keep the finished session
        // and wait for bench service. A normal STOP can start another file.
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
    if (command == Command::Stop) requested.store(false);
}

void push(uint8_t type, const void* payload, size_t length) {
    if (state.load() != State::Recording || !requested.load() || !length || length > maxPayload) return;
    if (queue.full()) { ++drops; return; }
    Record record{};
    record.type = type;
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
    snprintf(out,size,"[QSPI] %s n=%lu bytes=%lu drop=%lu file=%d",name,
        (unsigned long)committed,(unsigned long)bytes.load(),(unsigned long)dropped,session.load());
    return true;
}
} // namespace qspi
#endif
