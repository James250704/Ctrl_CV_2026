#pragma once
#include <cstdint>
#include <cstdlib>

// Mock watchdog — watchdog_reboot calls exit() to simulate reboot
// Tests can check for clean exit instead of actual hardware reset
inline bool mock_watchdog_reboot_called = false;
inline uint32_t mock_watchdog_reboot_pc = 0;
inline uint32_t mock_watchdog_reboot_sp = 0;
inline uint32_t mock_watchdog_reboot_delay = 0;

inline void watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms) {
    mock_watchdog_reboot_called = true;
    mock_watchdog_reboot_pc = pc;
    mock_watchdog_reboot_sp = sp;
    mock_watchdog_reboot_delay = delay_ms;
    // Don't actually exit — tests will check the flag
}

inline void watchdog_enable(uint32_t, bool) {}
inline void watchdog_disable() {}
inline void watchdog_update() {}
