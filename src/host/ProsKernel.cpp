/* ---------------------------------------------------------------------------
 * ProsKernel.cpp  --  desktop stand-in for the PROS kernel pieces the
 * pipeline actually calls: clock, delay, mutex, tasks, controller text,
 * and a brain LCD that reports "not present".
 *
 * Linked only by the host build. The V5 image gets these from libpros.
 * clang++ on the MSVC target does not pass /subsystem to lld-link, so the
 * pragma below gives the simulation executable a console subsystem.
 * ------------------------------------------------------------------------- */
#if defined(_MSC_VER)
#pragma comment(linker, "/SUBSYSTEM:CONSOLE")
#endif

#include "api.h"
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>

namespace pros {
namespace c {
extern "C" std::uint32_t millis(void) {
    static const auto start = std::chrono::steady_clock::now();
    return (std::uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
}
extern "C" void delay(const std::uint32_t milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
extern "C" std::int32_t controller_print(controller_id_e_t, std::uint8_t line, std::uint8_t col,
                                        const char* fmt, ...) {
    std::printf("[ctrl %u:%u] ", (unsigned)line, (unsigned)col);
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
    std::printf("\n");
    return 1;
}
}  // namespace c

mutex_t Mutex::lazy_init() {
    mutex_t current = mutex.load();
    if (current != nullptr) return current;
    auto* created = new std::mutex();
    mutex_t expected = nullptr;
    if (!mutex.compare_exchange_strong(expected, created)) {
        delete created;
        return expected;
    }
    return created;
}

bool Mutex::take() {
    static_cast<std::mutex*>(lazy_init())->lock();
    return true;
}

bool Mutex::give() {
    auto* held = static_cast<std::mutex*>(mutex.load());
    if (held != nullptr) held->unlock();
    return true;
}

Mutex::~Mutex() {
    delete static_cast<std::mutex*>(mutex.exchange(nullptr));
}

Task::Task(task_fn_t function, void* parameters, std::uint32_t, std::uint16_t, const char*) {
    std::thread([function, parameters]() { function(parameters); }).detach();
}

Controller::Controller(controller_id_e_t id) : _id(id) {}
}  // namespace pros

extern "C" void initialize(void);

int main() {
    initialize();
    for (;;) std::this_thread::sleep_for(std::chrono::hours(24));
}
