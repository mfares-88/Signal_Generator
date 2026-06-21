// Signal generation API for ESP32 CKP / CAM outputs.
//
// M0.1 (interface freeze): IGenerator widened per implementation_plan.md §3.2
// to accept a PatternRef + RPM, a fast setRpm() path, a per-channel invert
// mask, and an atomic edge-counter accessor.
//
// This header now declares only the SHARED generator contract: the GenError
// enum, the GapPosition enum, the SignalConfig POD, and the abstract
// IGenerator interface. The concrete backend is TableCkpGenerator
// (TableCkpGenerator.h), the native byte-table generator used in the real
// build (SIGGEN_BACKEND_TABLE).

#pragma once

#include <Arduino.h>

#include "PatternRef.h"

enum class GenError : uint8_t {
  OK = 0,
  NOT_INITIALIZED,
  NO_TABLE,
  BAD_SLOT_COUNT,
  BAD_RPM,
  TIMER_FAIL,
  GPIO_FAIL,
  BUFFER_OVERFLOW
};

// Position of the missing-tooth gap within a period
enum GapPosition {
  GAP_AT_END,
  GAP_AT_START
};

// Configuration for the generated crank signal (shared "Symmetric/Missing"
// POD). Retained as shared infrastructure: read by gCfg (the global base
// config in main.cpp) and re-exported transitively through ctrl_msg.h and
// the DSL host-test POD block.
struct SignalConfig {
  uint32_t   rpm;        // 100..6000
  uint16_t   nTeeth;     // total teeth if no gaps (e.g., 60, 36)
  uint8_t    pMiss;      // number of gap periods per revolution
  uint8_t    nMiss;      // number of missing teeth per period
  GapPosition gapPos;    // START or END of period
  bool        gapLvl;    // gap level (false=LOW, true=HIGH)
};

// IGenerator — manager <-> backend contract (implementation_plan.md §3.2).
//
// Invariants:
//   - getEdgeCounter() returns a single naturally-aligned uint16_t and is
//     read without a critical section (Xtensa aligned 16-bit reads are
//     atomic against the ISR writer of the same word).
//   - setRpm() never reallocates or rebuilds buffers — it is the fast path
//     used by the sweep task at priority 2.
//   - apply() may rebuild buffers; callers must not invoke from ISR context.
//   - channel_mask bit positions: bit0=crank, bit1=cam1, bit2=cam2,
//     bit3=knock(reserved). The mask XORs the published byte before
//     bundle write — i.e. a set bit means "invert this channel".
struct IGenerator {
  virtual bool begin(int pin_crank, int pin_cam1 = -1, int pin_cam2 = -1) = 0;
  virtual bool apply(const PatternRef& ref, uint32_t rpm) = 0;
  virtual bool setRpm(uint32_t rpm) = 0;              // fast path — used in sweep
  virtual void setInverted(uint8_t channel_mask) = 0; // per-channel XOR
  virtual uint8_t getInverted() const = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual uint16_t getEdgeCounter() const = 0;        // atomic read for waveform cursor

  virtual GenError lastError() const { return GenError::OK; }
  virtual bool isReady() const { return false; }

  // Cycle-timing accessors (M4.2, Agent C consumer). Published by the byte-
  // table backend on every wrap of the active pattern — the compression
  // task uses (now - cycleStart) / cycleDuration to derive the current
  // crank angle. Naturally aligned 32-bit reads on Xtensa are atomic
  // against the ISR writer; no critical section required.
  //
  // Default implementations return 0 so a backend that does not track cycle
  // timing silently disables the compression effect — the
  // calculateCompressionModifier() consumer treats cycleDuration == 0 as
  // "no signal", matching References/ardustim.ino:379.
  virtual uint32_t getCycleStartUs() const { return 0; }
  virtual uint32_t getCycleDurationUs() const { return 0; }

  virtual ~IGenerator() = default;
};
