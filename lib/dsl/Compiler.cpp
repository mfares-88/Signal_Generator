// Compiler.cpp — wires Lexer → Parser → Validator → byte-table compiler.
//
// Parent spec:
//   - implementation_plan.md §M5.3 (Compiler) and §M5.5 (SignalConfig adapter)
//   - integration_report.md §7.2 (compilation algorithm)
//   - integration_report.md §7.5 (validation rules, enforced by Validator)
//
// Memory model:
//   - The compiled .table is allocated in PSRAM via heap_caps_malloc with
//     MALLOC_CAP_SPIRAM (falls back to internal heap on hosts without
//     PSRAM, e.g. native unit tests).
//   - dslFree() releases the table via heap_caps_free.
//   - The intermediate per-wheel buffers live on the regular heap and are
//     freed before the function returns.
//
// Algorithm summary (§7.2 expanded):
//   1. Build per-wheel slot vector (uint8_t per slot, 1 = pin asserted).
//      - Symmetric: total_teeth × duty_den slots; each tooth = duty_num
//        slots "on" followed by (duty_den - duty_num) slots "off".
//      - Missing: same as symmetric for present teeth (per run-list order),
//        with the 'm' runs emitting `count × duty_den` slots of "off".
//      - Angular: alternating H/L, one slot per degree; each entry is the
//        duration in degrees of the next half-cycle. Starts HIGH.
//   2. Apply CCW (reverse) before placement.
//   3. Cam-doubling: if any wheel is cam (CCW, 720°), expand every CW
//      (360°) wheel's slot vector by repetition ×2 so they all span the
//      same angular period. We expand to the largest "domain" (720° if
//      any cam present, otherwise 360°).
//   4. LCM merge: L = lcm of all expanded slot counts. Expand each wheel
//      to length L by integer repetition. OR-combine into a uint8_t[L]
//      byte buffer at bit (pin-1).
//   5. Canonicalize: rotate so slot 0 = first rising edge of lowest active
//      pin. (Dedups phase-shifted duplicates.)
//   6. Enforce L ≤ 4096 (§7.5 rule #9). Return DslResult.
//
// Host-build shim: on native test builds (no IDF), heap_caps_malloc is
// not available — we provide a tiny stub that defers to malloc/free.

#include "Compiler.h"
#include "Lexer.h"
#include "Parser.h"
#include "Validator.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <vector>

// Conditionally include esp_heap_caps.h. On native host builds (gcc/clang
// without IDF) fall back to malloc/free so the unit tests can run.
#if defined(__has_include)
#  if __has_include("esp_heap_caps.h")
#    include "esp_heap_caps.h"
#    define DSL_HAS_HEAP_CAPS 1
#  endif
#endif

#ifndef DSL_HAS_HEAP_CAPS
#  define MALLOC_CAP_SPIRAM 0
static inline void* heap_caps_malloc(size_t n, int /*caps*/) { return malloc(n); }
static inline void  heap_caps_free(void* p)                  { free(p); }
#endif

namespace {

// ── Diagnostic helpers ─────────────────────────────────────────────────────

DslResult makeError(const char* msg, uint16_t off) {
  DslResult r{};
  r.ok = false;
  r.pattern = PatternRef{};   // zero-initialized
  size_t n = strlen(msg);
  if (n >= sizeof(r.error)) n = sizeof(r.error) - 1;
  memcpy(r.error, msg, n);
  r.error[n] = '\0';
  r.error_offset = off;
  return r;
}

// ── Per-wheel slot expansion ──────────────────────────────────────────────
//
// Each output element is 0 or 1 — whether the wheel's pin is asserted in
// that slot. We later shift into bit (pin-1) when OR-merging.

bool expandWheel(const WheelDef& w, std::vector<uint8_t>& out) {
  out.clear();
  if (w.kind == WheelKind::Symmetric) {
    // total_teeth × duty_den slots; each tooth: duty_num HIGH then
    // (duty_den - duty_num) LOW.
    const uint32_t total = uint32_t(w.total_teeth) * uint32_t(w.duty_den);
    out.reserve(total);
    for (uint16_t i = 0; i < w.total_teeth; ++i) {
      for (int16_t k = 0; k < w.duty_num; ++k) out.push_back(1);
      for (int16_t k = w.duty_num; k < w.duty_den; ++k) out.push_back(0);
    }
  } else if (w.kind == WheelKind::Missing) {
    // Walk the run-list. 't' entries emit present-tooth slots; 'm' entries
    // emit gap slots (level 0).
    const uint32_t total = uint32_t(w.total_teeth) * uint32_t(w.duty_den);
    out.reserve(total);
    for (auto& r : w.runs) {
      if (r.suffix == 't') {
        for (uint16_t i = 0; i < r.value; ++i) {
          for (int16_t k = 0; k < w.duty_num; ++k) out.push_back(1);
          for (int16_t k = w.duty_num; k < w.duty_den; ++k) out.push_back(0);
        }
      } else {  // 'm' — missing
        const uint32_t gap_slots = uint32_t(r.value) * uint32_t(w.duty_den);
        for (uint32_t i = 0; i < gap_slots; ++i) out.push_back(0);
      }
    }
    if (out.size() != total) return false;  // Validator should catch
  } else {
    // Angular: one slot per degree, alternating HIGH/LOW starting HIGH.
    // (Per §7.1 BNF comment: "alternating H,L durations in degrees".)
    bool level = true;
    for (auto& a : w.angular) {
      for (uint16_t d = 0; d < a.degrees; ++d) out.push_back(level ? 1 : 0);
      level = !level;
    }
  }
  return true;
}

// ── Math helpers ───────────────────────────────────────────────────────────

uint32_t gcd_u32(uint32_t a, uint32_t b) {
  while (b != 0) { uint32_t r = a % b; a = b; b = r; }
  return a;
}

uint32_t lcm_u32(uint32_t a, uint32_t b) {
  if (a == 0 || b == 0) return 0;
  return (a / gcd_u32(a, b)) * b;
}

// ── Canonicalization ───────────────────────────────────────────────────────
//
// Rotate `buf` so slot 0 is the first rising edge of the lowest-numbered
// active pin (= lowest set bit in channel_mask). A "rising edge" is a slot
// whose bit is set when the previous slot's bit was clear (modulo wrap).
//
// Modifies buf in place.
void canonicalizeBuffer(std::vector<uint8_t>& buf, uint8_t channel_mask) {
  if (buf.empty() || channel_mask == 0) return;
  // Find lowest active pin.
  uint8_t lowest_bit = 0;
  for (int b = 0; b < 8; ++b) {
    if (channel_mask & (1u << b)) { lowest_bit = uint8_t(b); break; }
  }
  const uint8_t mask = uint8_t(1u << lowest_bit);
  const size_t n = buf.size();
  size_t rotate_to = 0;
  for (size_t i = 0; i < n; ++i) {
    const size_t prev = (i + n - 1) % n;
    if ((buf[i] & mask) && !(buf[prev] & mask)) {
      rotate_to = i;
      break;
    }
  }
  if (rotate_to == 0) return;
  // In-place rotation: copy out the new sequence. (n is small — 4096 max
  // per §7.5 #9 — so allocate a scratch.)
  std::vector<uint8_t> tmp(n);
  for (size_t i = 0; i < n; ++i) tmp[i] = buf[(i + rotate_to) % n];
  buf.swap(tmp);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

DslResult dslCompileAst(const ProgramAst* ast) {
  if (!ast || ast->wheels.empty()) {
    return makeError("compile: empty AST", 0);
  }
  const auto& wheels = ast->wheels;

  // Detect cam presence (any wheel with rotation 'c').
  bool any_cam = false;
  for (auto& w : wheels) if (w.rotation == Rotation::CCW) any_cam = true;
  const uint16_t total_degrees = any_cam ? 720u : 360u;

  // Step 1+2: expand per-wheel slot vectors, applying CCW reversal.
  // Step 3 (cam-doubling): if total_degrees == 720 and wheel is CW (360°),
  //        repeat its expanded vector twice.
  std::vector<std::vector<uint8_t>> per_wheel(wheels.size());
  std::vector<uint32_t> per_wheel_len(wheels.size(), 0);

  for (size_t i = 0; i < wheels.size(); ++i) {
    const WheelDef& w = wheels[i];
    std::vector<uint8_t>& vec = per_wheel[i];
    if (!expandWheel(w, vec)) {
      return makeError("compile: wheel expansion failed", w.src_offset);
    }
    // Apply rotation: CCW means reverse the slot order. The 'c' marker
    // *also* indicates a 720° wheel in the grammar — that's the cam-
    // doubling axis, handled separately. Per §7.1 BNF comment, the
    // C/c letters tag rotation/period; reversal is the intuitive
    // CCW semantics. Spec invariant: for an angular cam wheel `c`,
    // reversing should still leave the alternating H/L starting HIGH.
    // We treat 'c' = CCW reverse strictly; tests pin this behavior.
    if (w.rotation == Rotation::CCW) {
      // For 720° cam wheels, the natural pattern already spans 720°, no
      // doubling. We DO still reverse the slot order to honor rotation.
      // Most cam patterns in the worked examples have symmetric mod-2
      // duty so the reversed and forward forms differ only in phase,
      // which canonicalization absorbs.
      for (size_t lo = 0, hi = vec.size() ? vec.size() - 1 : 0;
           lo < hi; ++lo, --hi) {
        uint8_t t = vec[lo]; vec[lo] = vec[hi]; vec[hi] = t;
      }
    }

    // Cam-doubling for CW wheels in a 720° group.
    if (total_degrees == 720 && w.rotation == Rotation::CW) {
      const size_t base = vec.size();
      vec.resize(base * 2);
      for (size_t k = 0; k < base; ++k) vec[base + k] = vec[k];
    }

    per_wheel_len[i] = uint32_t(vec.size());
    if (per_wheel_len[i] == 0) {
      return makeError("compile: wheel expanded to zero slots", w.src_offset);
    }
    if (per_wheel_len[i] > 4096) {
      return makeError("compile: per-wheel slot count exceeds 4096", w.src_offset);
    }
  }

  // Step 4: LCM merge.
  uint32_t L = per_wheel_len[0];
  for (size_t i = 1; i < per_wheel_len.size(); ++i) {
    L = lcm_u32(L, per_wheel_len[i]);
    if (L == 0 || L > 4096) {
      return makeError("compile: LCM exceeds 4096-byte limit (rule #9)",
                       wheels[i].src_offset);
    }
  }

  // Allocate the byte buffer. Use a std::vector for canonicalization, then
  // copy into PSRAM at the end.
  //
  // Resampling per §7.2: "resample each wheel to length L by integer
  // repetition" means stretching each native slot uniformly over the L
  // output slots — output[j] = v[(j * base) / L]. Each native slot
  // spans (L/base) consecutive output slots (the worked example trace
  // in §7.2 confirms this: a 2-slot cam stretches to half-L on, half-L
  // off — NOT tile-repeated to L/2 alternations).
  std::vector<uint8_t> merged(L, 0);
  uint8_t channel_mask = 0;
  for (size_t i = 0; i < wheels.size(); ++i) {
    const uint8_t bit = uint8_t(1u << (wheels[i].pin - 1));
    channel_mask = uint8_t(channel_mask | bit);
    const auto& v = per_wheel[i];
    const uint64_t base = per_wheel_len[i];
    for (uint32_t j = 0; j < L; ++j) {
      const uint32_t idx = uint32_t((uint64_t(j) * base) / L);
      if (v[idx]) merged[j] = uint8_t(merged[j] | bit);
    }
  }

  // Step 5: canonicalize.
  canonicalizeBuffer(merged, channel_mask);

  // Step 6: copy to PSRAM-allocated buffer.
  uint8_t* table = static_cast<uint8_t*>(
      heap_caps_malloc(L, MALLOC_CAP_SPIRAM));
  if (!table) {
    return makeError("compile: heap_caps_malloc(PSRAM) failed", 0);
  }
  memcpy(table, merged.data(), L);

  DslResult r{};
  r.ok = true;
  r.error[0] = '\0';
  r.error_offset = 0;
  r.pattern.table        = table;
  r.pattern.slot_count   = uint16_t(L);
  r.pattern.degrees      = total_degrees;
  r.pattern.rpm_scaler   = float(L) / 120.0f;
  r.pattern.channel_mask = channel_mask;
  r.pattern.name_key     = nullptr;  // M5.7 will fill this in
  return r;
}

DslResult dslCompile(const char* source) {
  if (!source) return makeError("null DSL source", 0);
  const size_t src_len = strlen(source);

  char err_buf[96];
  err_buf[0] = '\0';
  uint16_t err_off = 0;

  ProgramAst* ast = parse(source, err_buf, sizeof(err_buf), &err_off);
  if (!ast) {
    DslResult r = makeError(err_buf[0] ? err_buf : "parse error", err_off);
    return r;
  }

  ValidationResult vr = validate(ast, src_len);
  if (!vr.ok) {
    DslResult r{};
    r.ok = false;
    r.pattern = PatternRef{};
    // Encode rule number into the message for diagnostics.
    char tmp[128];
    snprintf(tmp, sizeof(tmp), "rule#%u: %s",
             unsigned(vr.rule), vr.message);
    size_t n = strlen(tmp);
    if (n >= sizeof(r.error)) n = sizeof(r.error) - 1;
    memcpy(r.error, tmp, n);
    r.error[n] = '\0';
    r.error_offset = vr.src_offset;
    freeProgramAst(ast);
    return r;
  }

  DslResult r = dslCompileAst(ast);
  freeProgramAst(ast);
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// dslFree — release a compiled PatternRef.table.
// ─────────────────────────────────────────────────────────────────────────────

void dslFree(PatternRef& ref) {
  if (ref.table) {
    // Cast away const for the free; the table came from heap_caps_malloc.
    heap_caps_free(const_cast<uint8_t*>(ref.table));
  }
  ref.table        = nullptr;
  ref.slot_count   = 0;
  ref.degrees      = 0;
  ref.rpm_scaler   = 0.0f;
  ref.channel_mask = 0;
  ref.name_key     = nullptr;
}
