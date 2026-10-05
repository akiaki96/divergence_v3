#include "app/maze_menu.hpp"
#include "app/maze_store.hpp"
#include "common/debug.hpp"
#include "device/device_instance.hpp"
#include "device/flash_bank.hpp"

void maze_show_onenter() {
    for (uint8_t b = 0; b < flash_bank::COUNT; ++b) {
        const maze_store::Record* r = maze_store::recordIn(b);
        unsigned long used = maze_store::usedSlots(b);
        if (r == nullptr) {
            LOG("bank %c: %lu/%lu slots, no valid record\r\n", 'A' + b, used,
                static_cast<unsigned long>(maze_store::SLOT_COUNT));
        } else {
            LOG("bank %c: %lu/%lu slots, last sequence %lu, goal (%u,%u), %s\r\n", 'A' + b, used,
                static_cast<unsigned long>(maze_store::SLOT_COUNT), static_cast<unsigned long>(r->sequence),
                r->goal_x, r->goal_y, (r->flags & maze_store::FLAG_COMPLETE) ? "complete" : "partial");
        }
    }
    uint8_t bank = 0;
    const maze_store::Record* r = maze_store::latest(&bank);
    if (r == nullptr) {
        LOG("no saved maze\r\n");
        return;
    }
    LOG("latest: bank %c, sequence %lu\r\n", 'A' + bank, static_cast<unsigned long>(r->sequence));
    maze_store::print(*r);
}

void maze_clear_onenter() {
    motorDriver.setBreak();
    ledBar16.set(0xFFFF);   // 消去中（2面で2〜4s，CPUが止まる）
    bool ok = maze_store::clear();
    ledBar16.set(0x0000);
    LOG("maze clear: %s\r\n", ok ? "ok" : "erase failed");
}
