// Dsl.h — public API for the runtime pattern DSL compiler.
//
// Parent spec: implementation_plan.md §3.5 (Agent D ownership) and
// integration_report.md §7 ("Pattern DSL — Closing Out the Original TODO").
//
// The DSL compiles a small grammar (per integration_report.md §7.1) into a
// canonical, validated PatternRef whose .table is byte-packed exactly like
// Ardu-Stim's wheel_defs.h arrays (bit0=crank, bit1=cam1, bit2=cam2, bit3=knock).
//
// Pipeline (subsequent milestones — this header only declares the public face):
//
//   source (const char*) ──Lexer──> Token stream
//                                 ──Parser──> WheelDef AST (M5.2)
//                                 ──Compiler──> byte table + metadata (M5.3)
//                                 ──Validator──> 12 rules in §7.5 (M5.4)
//                                 ──> DslResult { ok, pattern, error, error_offset }
//
// Memory rules (frozen — §6 "Agent D" hard rules in the plan):
//   - Compiled table ≤ 4096 bytes per pattern (§7.5 rule #9).
//   - .table is allocated with heap_caps_malloc(len, MALLOC_CAP_SPIRAM).
//   - dslFree() releases it. Builtin PatternRefs (from .rodata) MUST NOT be
//     passed to dslFree(); only DslResult.pattern from dslCompile* belongs here.
//
// Status: M5.1 dispatch declares this surface only; bodies live in
// Lexer.cpp (now) and Parser.cpp / Compiler.cpp / Validator.cpp (M5.2+).

#pragma once

#include <stdint.h>

#include "PatternRef.h"

// Host unit-test builds (e.g. native gcc with no Arduino-ESP32 toolchain)
// don't have <Arduino.h> and cannot include the full CkpGenerator.h. The
// DSL surface only needs the SignalConfig POD + GapPosition enum from
// that header, so we provide a stand-alone definition in that mode. The
// firmware build path (Arduino-ESP32) takes the normal include and the
// types resolve to the same in-memory layout.
#if defined(DSL_HOST_TEST) || !defined(ARDUINO)
#  ifndef DSL_HAS_SIGNAL_CONFIG
#    define DSL_HAS_SIGNAL_CONFIG 1
enum GapPosition {
  GAP_AT_END,
  GAP_AT_START
};
struct SignalConfig {
  uint32_t    rpm;
  uint16_t    nTeeth;
  uint8_t     pMiss;
  uint8_t     nMiss;
  GapPosition gapPos;
  bool        gapLvl;
};
#  endif
#else
#  include "CkpGenerator.h"  // provides SignalConfig (legacy UI custom modal input)
#endif

struct DslResult {
  bool       ok;             // true if pattern is valid and populated
  PatternRef pattern;        // valid only if ok; .table malloc'd in PSRAM
  char       error[96];      // human-readable diagnostic (zero-terminated)
  uint16_t   error_offset;   // index into source where compilation failed
};

// Compile a DSL source string into a PatternRef.
// On success, the caller owns the returned PatternRef.table and must call
// dslFree() to release it. On failure, .ok == false and .error/.error_offset
// describe the diagnostic; .pattern.table is guaranteed nullptr.
DslResult dslCompile(const char* source);

// Release a PatternRef.table previously returned by dslCompile* via
// heap_caps_free. Safe on a default-constructed/zeroed PatternRef. After
// the call, ref.table == nullptr and ref.slot_count == 0.
void dslFree(PatternRef& ref);
