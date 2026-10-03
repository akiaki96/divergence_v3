#include "app/wall_edge_log.hpp"
#include "app/logger.hpp"
#include "device/device_instance.hpp"
#include "device/uart.hpp"

namespace wall_edge_log {

namespace {
constexpr const char* COLUMNS[] = {"side", "x", "boundary", "offset", "shift", "velocity"};
constexpr uint32_t COLUMN_COUNT = sizeof(COLUMNS) / sizeof(COLUMNS[0]);

const char* g_last_dir = nullptr;
const char* g_last_file = nullptr;
} // namespace

void dump(const char* dir, const char* file) {
    g_last_dir = dir;
    g_last_file = file;
    uint32_t n = wallEdge.eventCount();
    bin_table::begin(dir, file, false, n * COLUMN_COUNT * sizeof(float), COLUMNS, COLUMN_COUNT);
    for (uint32_t i = 0; i < n; ++i) {
        const WallEdge::Event& e = wallEdge.event(i);
        float row[COLUMN_COUNT] = {static_cast<float>(e.side), e.x, e.boundary, e.x - e.boundary, e.shift,
                                   e.velocity};
        uart_write(reinterpret_cast<const uint8_t*>(row), sizeof(row));
    }
    bin_table::end();
}

void dumpLast() {
    if (g_last_dir != nullptr) dump(g_last_dir, g_last_file);
}

} // namespace wall_edge_log
