#pragma once
// =============================================================================
// apc_perf_timer.h — High-resolution performance profiling utilities
// =============================================================================
//
// Provides lightweight profiling for the game loop subsystems:
//
//   - PerfTimer: RAII scoped timer for measuring elapsed time
//   - PerfSection: named timing section with accumulation and averaging
//   - PerfReport: collection of named sections with per-frame and rolling stats
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays)
//   - Uses <chrono> for high-resolution timing
//   - C++17
//
// =============================================================================

#include <chrono>
#include <cstdint>
#include <cstdio>

namespace apc {

// =============================================================================
// Constants
// =============================================================================
static constexpr uint32_t MAX_PERF_SECTIONS = 16;
static constexpr uint32_t PERF_ROLLING_WINDOW = 60; // frames for rolling average

// =============================================================================
// PerfTimer — RAII scoped timer
// =============================================================================
struct PerfTimer {
    double start_ns = 0.0;
    double elapsed_ns = 0.0;
    uint8_t running = 0;

    void begin() {
        start_ns = now_ns();
        running = 1;
    }

    double end() {
        if (running) {
            elapsed_ns = now_ns() - start_ns;
            running = 0;
        }
        return elapsed_ns;
    }

    // Convert nanoseconds to milliseconds
    double elapsed_ms() const {
        return elapsed_ns * 0.000001;
    }

    // Convert nanoseconds to microseconds
    double elapsed_us() const {
        return elapsed_ns * 0.001;
    }

    static double now_ns() {
        using Clock = std::chrono::high_resolution_clock;
        auto t = Clock::now().time_since_epoch();
        return static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t).count()
        );
    }
};

// =============================================================================
// PerfSection — Named timing accumulator
// =============================================================================
struct PerfSection {
    const char* name       = "";
    double      total_ns   = 0.0;    // Accumulated time this frame
    double      peak_ns    = 0.0;    // Worst-case single measurement
    uint32_t    call_count = 0u;     // Number of measurements this frame

    // Rolling average state
    double      rolling_sum = 0.0;   // Sum of last N frame totals
    uint32_t    rolling_count = 0u;  // How many frames in rolling window
    double      avg_ns       = 0.0;  // Rolling average in nanoseconds

    void reset_frame() {
        total_ns   = 0.0;
        peak_ns    = 0.0;
        call_count = 0u;
    }

    void record(double nanoseconds) {
        total_ns += nanoseconds;
        if (nanoseconds > peak_ns) peak_ns = nanoseconds;
        ++call_count;
    }

    void update_rolling() {
        rolling_sum += total_ns;
        if (rolling_count >= PERF_ROLLING_WINDOW) {
            rolling_sum -= avg_ns * PERF_ROLLING_WINDOW;
        } else {
            ++rolling_count;
        }
        avg_ns = rolling_sum / static_cast<double>(rolling_count);
    }

    double avg_ms() const  { return avg_ns * 0.000001; }
    double peak_ms() const { return peak_ns * 0.000001; }
    double total_ms() const { return total_ns * 0.000001; }
};

// =============================================================================
// PerfReport — Collection of named timing sections
// =============================================================================
struct PerfReport {
    PerfSection sections[MAX_PERF_SECTIONS];
    uint32_t    section_count = 0u;
    uint32_t    frame_number  = 0u;
    uint8_t     enabled       = 0u;

    // --- Find or create a section by name ---
    PerfSection* get_section(const char* name) {
        for (uint32_t i = 0u; i < section_count; ++i) {
            // Simple string comparison (deterministic, no strcmp)
            uint32_t j = 0u;
            while (sections[i].name[j] && name[j] &&
                   sections[i].name[j] == name[j]) { ++j; }
            if (sections[i].name[j] == '\0' && name[j] == '\0') {
                return &sections[i];
            }
        }
        // Not found — create new
        if (section_count < MAX_PERF_SECTIONS) {
            PerfSection& s = sections[section_count];
            s.name = name;
            s.reset_frame();
            ++section_count;
            return &s;
        }
        return nullptr;
    }

    // --- Begin a new frame ---
    void begin_frame() {
        for (uint32_t i = 0u; i < section_count; ++i) {
            sections[i].reset_frame();
        }
        ++frame_number;
    }

    // --- End frame: compute rolling averages ---
    void end_frame() {
        for (uint32_t i = 0u; i < section_count; ++i) {
            sections[i].update_rolling();
        }
    }

    // --- Print report every N frames ---
    void print_report(uint32_t interval = 60u) const {
        if (!enabled) return;
        if (frame_number % interval != 0u) return;

        std::fprintf(stdout, "\n--- Perf Report [frame %u] ---\n", frame_number);
        for (uint32_t i = 0u; i < section_count; ++i) {
            const PerfSection& s = sections[i];
            std::fprintf(stdout,
                "  %-24s  total=%7.3fms  peak=%7.3fms  avg=%7.3fms  calls=%u\n",
                s.name,
                s.total_ms(),
                s.peak_ms(),
                s.avg_ms(),
                s.call_count);
        }
        std::fprintf(stdout, "--------------------------------\n");
    }

    void reset() {
        section_count = 0u;
        frame_number = 0u;
        for (uint32_t i = 0u; i < MAX_PERF_SECTIONS; ++i) {
            sections[i] = PerfSection();
        }
    }
};

// =============================================================================
// ScopedTimer — RAII helper that records into a PerfSection
// =============================================================================
struct ScopedTimer {
    PerfSection* section;
    PerfTimer    timer;

    ScopedTimer(PerfSection* sec) : section(sec) {
        if (section) timer.begin();
    }

    ~ScopedTimer() {
        if (section) {
            double elapsed = timer.end();
            section->record(elapsed);
        }
    }
};

} // namespace apc
