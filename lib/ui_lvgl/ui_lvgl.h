// Minimal LVGL UI wrapper for ESP32-S3.
//
// M2.3+: extended with LED indicators per channel, M3.4 64-pattern
// scrollable dropdown with search, M4.5 sweep+compression tabs, M5.7
// DSL editor modal, M7 waveform canvas. See implementation_plan.md §6
// (Agent E).

#pragma once

// This header is S3/display-only. WROOM (headless) builds must NOT include
// it — main.cpp gates the include behind SIGGEN_HAS_DISPLAY. The guard
// below is belt-and-suspenders so an accidental include in a headless TU
// emits a clear diagnostic rather than chains of LVGL macro errors.
#if !defined(SIGGEN_HAS_DISPLAY)
#  error "ui_lvgl.h included in a build without SIGGEN_HAS_DISPLAY"
#endif

#include <lvgl.h>
#include <stdint.h>

#include "CkpGenerator.h"
#include "PatternRef.h"

typedef void (*ui_on_rpm_cb)(uint32_t rpm);
typedef void (*ui_on_pattern_cb)(uint8_t pattern_index);
typedef void (*ui_on_run_cb)(bool running);
typedef void (*ui_on_invert_cb)(bool inverted);

// Initialize LVGL and create UI page. Provide callbacks for actions.
// Returns false if display/touch/LVGL cannot be initialized.
bool ui_init(ui_on_rpm_cb on_rpm, ui_on_pattern_cb on_pattern,
             ui_on_run_cb on_run, ui_on_invert_cb on_invert);

bool ui_is_ready();

// Backend -> UI synchronization (thread-safe; applied on next ui_task_handler())
void ui_update_rpm(uint32_t rpm);
void ui_update_pattern(uint8_t pattern_index);
void ui_update_running(bool running);
void ui_update_inverted(bool inverted);
void ui_show_error(const char* msg);

// M2.3: live channel-state LEDs. invert_mask: bit0=crank, bit1=cam1, bit2=cam2.
// channel_mask: bits set indicate the active pattern actually uses that
// channel (others are greyed out).
void ui_update_channels(uint8_t channel_mask, uint8_t invert_mask);

// Pump LVGL (call often from loop)
void ui_task_handler();
