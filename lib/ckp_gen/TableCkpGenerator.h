// TableCkpGenerator — native byte-table backend (M1.1 + M2.1).
//
// Parent reference: implementation_plan.md §3.2 (IGenerator contract),
// §6 (Agent A hard rules), and milestones M1.1 (single-channel) + M2.1
// (three-channel widening). This class plays back a PatternRef byte
// table from .rodata, one slot per timer alarm, using the ESP-IDF
// GPTimer driver (1 MHz tick) and a dedic_gpio bundle for fast GPIO
// writes.
//
// Bundle width (set in begin()):
//   - cam1 == -1            → width 1 (crank only)
//   - cam1 ≥ 0, cam2 == -1  → width 2 (crank + cam1)
//   - cam1 ≥ 0, cam2 ≥ 0    → width 3 (crank + cam1 + cam2)
// The active bits in each byte are bit0=crank, bit1=cam1, bit2=cam2 —
// matching the Ardu-Stim byte-packing convention. Bit3 (knock) is
// reserved and never wired to a pin in this backend.
//
// Per-channel slot-alignment guarantee (M2.1 exit criterion):
//   Every cam edge lands on the exact crank slot specified by the
//   source byte (within one tick of the gptimer). All three channels
//   are written by a single dedic_gpio_bundle_write() call in the same
//   ISR statement, so there is zero inter-channel skew — the bundle is
//   atomic at the GPIO matrix level.
//
// Why GPTimer (not the Arduino esp32-hal-timer wrapper):
//   - Direct ESP-IDF API (driver/gptimer.h) gives us a deterministic ISR
//     callback signature with IRAM_ATTR, and a single-call setAlarmAction
//     for the sweep fast path.
//
// Timer formula equivalence vs Ardu-Stim (References/ardustim.ino:408):
//   AVR:  OCR1A = 8_000_000 / (rpm_scaler * rpm)
//         where rpm_scaler = slot_count / 120, and the AVR timer prescaler
//         yields OCR1A counts in units of (1 / 8 MHz) seconds == 0.125 us.
//   ESP32 at 1 MHz tick — we want microseconds-per-slot directly, with RPM
//   interpreted as a UNIVERSAL CRANK RPM across both 360 and 720 spans:
//             period_us = 60_000_000 * table_degrees / (360 * rpm * slot_count)
//   For a 360-degree table this reduces to 60_000_000 / (rpm * slot_count)
//   (one revolution = one engine cycle). For a 720-degree table the span is
//   two crank revolutions, so the numerator carries the 720/360 = 2 factor:
//   the full table now takes 2× wall-clock per crank-rev → the cam (½ crank)
//   runs at half rate, which is physically correct. The previous unscaled
//   formula 60_000_000/(rpm*slot_count) was WRONG for 720 tables (it doubled
//   the crank rate, e.g. 800 RPM read as 1600 on CKP+CMP patterns); the
//   table_degrees scaling fixes that. table_degrees is stored in
//   _table_degrees (set in apply() from PatternRef.degrees).
//
// ISR hard contract (per §6 Agent A — ≤ 5 statements of real work):
//   1) byte = table[edge] XOR invert_mask        (load + XOR fused)
//   2) dedic_gpio_bundle_write(bundle, mask, byte)
//   3) advance+wrap branch: forward path increments and wraps to 0 (and
//      publishes cycleDuration/cycleStart on wrap); reverse path decrements
//      and wraps to slot_count-1. Counted as one selection-statement.
//   4) commit advanced counter back to _edge_counter
//   5) return false
//
// Memory layout:
//   - _table:        const uint8_t* — points into .rodata. Read in ISR.
//   - _slot_count:   uint16_t       — wrap limit. Read in ISR.
//   - _bundle_width: uint8_t        — 1/2/3, sets _bundle_mask = (1<<w)-1.
//   - _bundle_mask:  uint8_t        — pre-computed (1u << width) - 1.
//                    Used as both the bundle write mask AND the byte
//                    output mask, so bits beyond the active channels
//                    are dropped automatically.
//   - _invert_mask:  volatile uint8_t — set from setInverted(), XOR'd in ISR.
//                    bit0=crank, bit1=cam1, bit2=cam2, bit3=knock(rsvd).
//   - _reverse:      volatile bool — when true, ISR decrements edge counter.
//                    Free feature per §8 risk #4. UI hooks land in M2.3.
//   - _edge_counter: volatile uint16_t — naturally aligned, single-word
//                    atomic on Xtensa, read lock-free via getEdgeCounter().
//   - _cycle_start_us, _cycle_duration_us: volatile uint32_t — published
//                    on wrap, consumed by Agent C's compression task in M4.2.

#pragma once

#include <stdint.h>

#include "CkpGenerator.h"

// ESP-IDF gptimer / dedic_gpio types are typedefs of anonymous structs in the
// SDK, so they cannot be forward-declared. Pull the driver headers in here.
#include "driver/gptimer.h"
#include "driver/dedic_gpio.h"

class TableCkpGenerator : public IGenerator {
public:
  TableCkpGenerator();
  ~TableCkpGenerator() override;

  // IGenerator
  bool      begin(int pin_crank, int pin_cam1 = -1, int pin_cam2 = -1) override;
  bool      apply(const PatternRef& ref, uint32_t rpm) override;
  bool      setRpm(uint32_t rpm) override;
  void      setInverted(uint8_t channel_mask) override;
  uint8_t   getInverted() const override;
  bool      start() override;
  bool      stop() override;
  uint16_t  getEdgeCounter() const override;
  GenError  lastError() const override { return _last_error; }
  bool      isReady() const override { return _initialized && _table != nullptr && _slot_count > 0; }

  // Cycle accessors (Agent C / M4.2 consumer). Naturally aligned 32-bit
  // reads on Xtensa are atomic against the ISR writer — no critical
  // section required. Overrides the IGenerator default (which returns 0
  // so the legacy adapter cleanly disables compression).
  uint32_t  getCycleStartUs() const override;
  uint32_t  getCycleDurationUs() const override;

  // Reverse rotation (§8 risk #4). When true, the ISR walks the table
  // backwards (wrapping at 0 → slot_count-1). UI hookup lands in M2.3.
  // Safe to flip at runtime — single-byte volatile store, no ISR re-arm
  // needed.
  void      setReverse(bool reverse);
  bool      getReverse() const;

private:
  // ISR callback registered with gptimer. MUST stay in IRAM and respect
  // the ≤ 5-statement rule.
  static bool IRAM_ATTR onAlarm(gptimer_handle_t timer,
                                const gptimer_alarm_event_data_t* edata,
                                void* user_ctx);

  // Recompute the alarm period from rpm + slot_count and push it to the
  // gptimer. Used by both apply() and setRpm().
  bool      reprogramAlarm(uint32_t rpm);

  // --- Configuration ---
  int       _pin_crank;
  int       _pin_cam1;
  int       _pin_cam2;

  // Bundle layout: 1, 2, or 3 channels (crank, +cam1, +cam2).
  uint8_t   _bundle_width;
  uint8_t   _bundle_mask;        // (1u << _bundle_width) - 1

  // --- Active pattern ---
  // _table is published from apply(); ISR reads it. The ISR is briefly
  // stopped during reprogramAlarm() before _table is swapped so the
  // pointer/slot_count pair stays consistent for ISR readers.
  const uint8_t* _table;
  uint16_t       _slot_count;
  // Degree span of the active table (360 or 720). Set in apply() from
  // PatternRef.degrees (clamped to 360 for any non-720 value) and read by
  // reprogramAlarm() so the period scales by crank-degrees — keeping RPM a
  // universal CRANK RPM across 360 and 720 patterns. See header timing note.
  uint16_t       _table_degrees;

  // --- Runtime state ---
  volatile uint16_t _edge_counter;
  volatile uint8_t  _invert_mask;
  volatile bool     _reverse;
  volatile uint32_t _cycle_start_us;
  volatile uint32_t _cycle_duration_us;

  // --- Cached RPM (for setRpm reentry validation) ---
  uint32_t _last_rpm;

  // --- ESP-IDF handles (opaque in the header) ---
  gptimer_handle_t           _timer;
  dedic_gpio_bundle_handle_t _bundle;

  bool _running;
  bool _initialized;

  mutable GenError _last_error = GenError::OK;

  uint8_t* _playback_buffer = nullptr;
  static constexpr size_t kPlaybackBufferBytes = 24 * 1024;  // tune via banner in .cpp
};
