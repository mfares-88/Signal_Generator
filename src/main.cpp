// ESP32 Crankshaft Signal Generator — Big Integration (M1.3 + M2.3 + M3.4 +
// M4.5 + M5.7 + M6 + M7 + M9). See _Plans-and-Records/implementation_plan.md.
//
// Agent E (ui-io) custody. This TU wires together every other agent's
// modules and is the single integration point for the manager task.
//
// Build flag contract:
//   -DSIGGEN_BACKEND_TABLE=1  → TableCkpGenerator (M1.1 native byte-table
//                               backend) drives the manager↔generator path.
//                               MSG_SET_PATTERN routes legacy preset indices
//                               through PatternLibrary::builtinByIndex().
//                               This is the S3 production build (always on).
//
// All UI inputs flow through gCtrlQ; UI callbacks never call the generator
// directly (per §6 Agent E hard rules). The LVGL pending-flag pattern in
// ui_lvgl.cpp is the only cross-core sync mechanism.

#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"

#include "CkpGenerator.h"
#include "EdgePulseCapture.h"
#include "LittleFSInit.h"
#include "PatternRef.h"
#include "PatternLibrary.h"
#include "NvsStore.h"
#include "SweepCompression.h"
#include "PatternStorage.h"
#include "Dsl.h"
#include "ctrl_msg.h"
#include "serial_cli.h"
#include "CaptureRecorder.h"
#include "wifi_link.h"   // Cycle 7: WiFi transport (Agent F); hooks populated below.

#if defined(SIGGEN_HAS_DISPLAY)
  // S3 build: LVGL is present, full UI surface is compiled in.
  #include "ui_lvgl.h"
#else
  // WROOM (headless) build: provide no-op stubs so the manager task code
  // stays readable. The control queue + ctrl_msg.h remain unguarded — they
  // are protocol-level and shared with serial_cli.
  typedef void (*ui_on_rpm_cb)(uint32_t rpm);
  typedef void (*ui_on_pattern_cb)(uint8_t pattern_index);
  typedef void (*ui_on_run_cb)(bool running);
  typedef void (*ui_on_invert_cb)(bool inverted);
  static inline bool ui_init(ui_on_rpm_cb, ui_on_pattern_cb, ui_on_run_cb,
                             ui_on_invert_cb) { return true; }
  static inline bool ui_is_ready() { return false; }
  static inline void ui_update_rpm(uint32_t)               {}
  static inline void ui_update_pattern(uint8_t)            {}
  static inline void ui_update_running(bool)               {}
  static inline void ui_update_inverted(bool)              {}
  static inline void ui_show_error(const char*)            {}
  static inline void ui_update_channels(uint8_t, uint8_t)  {}
  static inline void ui_task_handler()                     {}
#endif

#if defined(SIGGEN_BACKEND_TABLE)
  #include "TableCkpGenerator.h"
#endif

// -------------------- Pins --------------------
#define PIN_CKP_OUT        17
#define PIN_CAM1_OUT       9
#define PIN_CAM2_OUT       14
#define PIN_CAPTURE_IN     18

static const char* genErrorString(GenError e) {
  switch (e) {
    case GenError::OK:                return "";
    case GenError::NOT_INITIALIZED:   return "Apply: generator not initialized";
    case GenError::NO_TABLE:          return "Apply: no active pattern table";
    case GenError::BAD_SLOT_COUNT:    return "Apply: bad slot count";
    case GenError::BAD_RPM:           return "Apply: RPM out of range";
    case GenError::TIMER_FAIL:        return "Apply: timer alarm failed";
    case GenError::GPIO_FAIL:         return "Apply: GPIO invalid/reserved";
    case GenError::BUFFER_OVERFLOW:   return "Apply: pattern too large (>24KB)";
  }
  return "Apply: unknown error";
}

// -------------------- Debug -------------------
#define DEBUG 1
#if DEBUG
  #define DBG_BEGIN()     Serial.begin(921600)
  #define DBG_PRINTLN(x)  Serial.println(x)
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN()
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(...)
#endif

// =====================================================
// Control / Manager (Core 1)
// =====================================================

// CtrlMsg / MsgType / MsgPayload are declared in lib/ui_lvgl/ctrl_msg.h —
// shared with serial_cli.cpp so the queue item-size matches across producers.

QueueHandle_t gCtrlQ = nullptr;

static uint32_t gUiMsgDropCount = 0;
static portMUX_TYPE gUiMsgDropMux = portMUX_INITIALIZER_UNLOCKED;

static inline void bumpUiMsgDropCount() {
  portENTER_CRITICAL(&gUiMsgDropMux);
  ++gUiMsgDropCount;
  portEXIT_CRITICAL(&gUiMsgDropMux);
}

// ---- Backend selection -------------------------------------------------
//
// M1.3: the native byte-table backend drives the manager↔generator path.
static TableCkpGenerator gGenInstance;
static IGenerator& gGen = gGenInstance;

static EdgePulseCapture  capRX;

// gCfg / gPatternIdx: gPatternIdx shadows the active builtin index for UI
// sync purposes; gCfg.rpm still represents the operator's "base RPM" target
// (sweep task rides on top).
static SignalConfig gCfg{1000, 60, 1, 2, GAP_AT_END, false};
static uint8_t gPatternIdx = 0;
static volatile bool gRunning = true;
static volatile bool gInverted = false;
static const PatternRef* gActivePattern = nullptr;

// DSL error pipe back to UI (cross-core, polled by LVGL timer per §6 Agent E).
volatile char g_dsl_error[128] = {0};
static portMUX_TYPE g_dsl_error_mux = portMUX_INITIALIZER_UNLOCKED;

#if defined(SIGGEN_HAS_DISPLAY)
// Waveform-canvas hooks (lib/ui_lvgl/ui_lvgl.cpp defines weak fallbacks).
// WROOM build has no display → no need to expose these symbols.
extern "C" const PatternRef* ui_get_active_pattern_for_wave() {
  return gActivePattern ? gActivePattern : PatternLibrary::builtinByIndex(0);
}
extern "C" uint16_t ui_get_edge_counter() {
  return gGen.getEdgeCounter();
}
#endif

// ---- DSL scratch leak management (TODO 2) ------------------------------
//
// The manager owns the lifetime of the single "scratch_dsl" PSRAM table
// produced by MSG_LOAD_DSL. On a
// subsequent compile we must dslFree() the previous table before
// publishing the new one. cleanupScratchDsl() is the single drain point
// for any swap that may abandon the active DSL table.
//
// ESP32 doesn't truly "shutdown" — power loss is the practical exit — so
// the manual call surface exists primarily for defensive correctness:
// every code path that switches gActivePattern away from the DSL scratch
// MUST invoke cleanupScratchDsl() so the PSRAM block is reclaimed.
static PatternRef s_scratch_dsl{};   // valid iff .table != nullptr
static bool       s_scratch_active = false;  // true iff gActivePattern points at s_scratch_dsl

static void cleanupScratchDsl() {
  if (s_scratch_dsl.table) {
    PatternRef tmp = s_scratch_dsl;
    dslFree(tmp);
  }
  s_scratch_dsl = PatternRef{};
  s_scratch_active = false;
}

static void publish_dsl_error(const char* msg) {
  portENTER_CRITICAL(&g_dsl_error_mux);
  if (msg && *msg) {
    strncpy((char*)g_dsl_error, msg, sizeof(g_dsl_error) - 1);
    g_dsl_error[sizeof(g_dsl_error) - 1] = '\0';
  } else {
    g_dsl_error[0] = '\0';
  }
  portEXIT_CRITICAL(&g_dsl_error_mux);
}

bool sendCtrlMsg(const CtrlMsg& msg) {
  if (!gCtrlQ) {
    bumpUiMsgDropCount();
    return false;
  }
  if (xQueueSend(gCtrlQ, &msg, 0) != pdTRUE) {
    bumpUiMsgDropCount();
    return false;
  }
  return true;
}

// --- UI → manager callbacks -------------------------------------------------

static void on_ui_rpm(uint32_t rpm) {
  CtrlMsg m{};
  m.type = MSG_SET_RPM;
  m.payload.val = (int32_t)rpm;
  if (!sendCtrlMsg(m)) {
    ui_show_error("Control queue full");
    ui_update_rpm(gCfg.rpm);
  }
}

static void on_ui_pattern(uint8_t idx) {
  // M3.4: UI dropdown emits the builtin index directly; route through
  // MSG_SELECT_BUILTIN on the TABLE backend.
  CtrlMsg m{};
  m.type = MSG_SELECT_BUILTIN;
  m.payload.val = (int32_t)idx;
  if (!sendCtrlMsg(m)) {
    ui_show_error("Control queue full");
    ui_update_pattern(gPatternIdx);
  }
}

static void on_ui_run(bool running) {
  CtrlMsg m{};
  m.type = running ? MSG_START : MSG_STOP;
  if (!sendCtrlMsg(m)) {
    ui_show_error("Control queue full");
    ui_update_running(gRunning);
  }
}

static void on_ui_invert(bool inverted) {
  CtrlMsg m{};
  m.type = MSG_SET_INVERT;
  m.payload.val = inverted ? 1 : 0;
  if (!sendCtrlMsg(m)) {
    ui_show_error("Control queue full");
    ui_update_inverted(gInverted);
  }
}

// --- Backend application helpers -------------------------------------------

// Try to apply a PatternRef on the TABLE backend; preserves lastGood on
// failure. Returns the ref pointer that is actually active after the call
// (nullptr if rollback occurred and no previous ref was active).
static const PatternRef* applyPatternRef(const PatternRef* ref, uint32_t rpm) {
  if (!ref) return gActivePattern;
  if (gGen.apply(*ref, rpm)) {
    // Successfully switched away from whatever was active. If the prior
    // active table was the DSL scratch (owned by the manager), release
    // its PSRAM now to avoid leaking it.
    if (s_scratch_active && ref != &s_scratch_dsl) {
      cleanupScratchDsl();
    }
    gActivePattern = ref;
    ui_update_channels(ref->channel_mask, gGen.getInverted());
    return ref;
  }
  // Rollback to last good if we have one.
  if (gActivePattern) {
    (void)gGen.apply(*gActivePattern, rpm);
  }
  return gActivePattern;
}

// --- Manager task ----------------------------------------------------------

// Loopback diagnostic message buffer polled by the LVGL UI timer (M6.2).
// Volatile because written by managerTask (Core 1) and read by the LVGL
// thread (Core 0). 95 chars + NUL — enough for the captureLoopbackErrorMsg
// snprintf payload.
volatile char g_loopback_error[96] = {0};

void managerTask(void*) {
  CtrlMsg m{};
  SignalConfig lastGood = gCfg;
  uint8_t lastGoodPattern = gPatternIdx;

  // 100 ms tick cadence for the loopback comparison loop (TODO 3 / M6.2).
  const TickType_t tickInterval = pdMS_TO_TICKS(100);

  for (;;) {
    // Block up to one tickInterval; either dispatch a message or fall
    // through to the periodic-maintenance section.
    const BaseType_t got = xQueueReceive(gCtrlQ, &m, tickInterval);

    if (got == pdTRUE) {
    switch (m.type) {
      case MSG_SET_RPM: {
        const uint32_t requested = (uint32_t)m.payload.val;
        const uint32_t clamped = constrain(requested, 100u, 6000u);

        if (gGen.setRpm(clamped)) {
          gCfg.rpm = clamped;
          g_rpm = clamped;
          sweepSetBaseRpm(clamped);
          NvsStore::setRpmDebounced(clamped);
          lastGood = gCfg;
          ui_show_error("");
          if (clamped != requested) ui_update_rpm(clamped);
        } else {
          ui_update_rpm(gCfg.rpm);
          ui_show_error("Invalid RPM");
        }
        break;
      }

      case MSG_SET_PATTERN: {
        // Legacy preset-index path; routed to MSG_SELECT_BUILTIN on the
        // TABLE backend (5-preset clamp preserved for back-compat callers).
        const uint32_t requested = (uint32_t)m.payload.val;
        const uint8_t idx = (requested > 4u) ? 4u : (uint8_t)requested;

        // Treat as MSG_SELECT_BUILTIN on the TABLE backend.
        const PatternRef* p = PatternLibrary::builtinByIndex(idx);
        if (p && gGen.apply(*p, gCfg.rpm)) {
          gActivePattern = p;
          ui_update_channels(p->channel_mask, gGen.getInverted());
          gPatternIdx = idx;
          if (p->name_key) {
            strncpy(g_pattern_key, p->name_key, sizeof(g_pattern_key) - 1);
            g_pattern_key[sizeof(g_pattern_key) - 1] = '\0';
            (void)NvsStore::setPatternKey(p->name_key);
          }
          lastGood = gCfg;
          lastGoodPattern = gPatternIdx;
          ui_show_error("");
          if (idx != requested) ui_update_pattern(idx);
        } else {
          ui_update_pattern(gPatternIdx);
          ui_show_error("Pattern apply failed");
        }
        break;
      }

      case MSG_START:
        if (gGen.start()) {
          gRunning = true;
          ui_update_running(true);
          ui_show_error("");
        } else {
          ui_show_error(genErrorString(gGen.lastError()));
          Serial.printf("[main] MSG_START rejected: %s\n", genErrorString(gGen.lastError()));
        }
        break;

      case MSG_STOP:
        if (gGen.stop()) {
          gRunning = false;
          ui_update_running(false);
          ui_show_error("");
        } else {
          ui_show_error(genErrorString(gGen.lastError()));
          Serial.printf("[main] MSG_STOP rejected: %s\n", genErrorString(gGen.lastError()));
        }
        break;

      case MSG_SET_INVERT: {
        const bool requested = (m.payload.val != 0);
        gGen.setInverted(requested ? 0x01u : 0x00u);
        gInverted = ((gGen.getInverted() & 0x01u) != 0);
        g_invert_mask = gGen.getInverted();
        (void)NvsStore::setInvertMask(g_invert_mask);
        ui_update_inverted(gInverted);
        ui_show_error("");
        break;
      }

      // ------------------------------------------------------------------
      // M3.4 — string-keyed pattern selection (TABLE backend).
      // ------------------------------------------------------------------
      case MSG_SELECT_BUILTIN: {
        const size_t idx = (size_t)(uint32_t)m.payload.val;
        const PatternRef* p = PatternLibrary::builtinByIndex(idx);
        if (!p) {
          ui_show_error("Bad builtin index");
          break;
        }
        const PatternRef* applied = applyPatternRef(p, gCfg.rpm);
        if (applied == p) {
          gPatternIdx = (uint8_t)idx;
          if (p->name_key) {
            strncpy(g_pattern_key, p->name_key, sizeof(g_pattern_key) - 1);
            g_pattern_key[sizeof(g_pattern_key) - 1] = '\0';
            (void)NvsStore::setPatternKey(p->name_key);
          }
          lastGoodPattern = gPatternIdx;
          ui_show_error("");
          ui_update_pattern(gPatternIdx);
        } else {
          ui_update_pattern(gPatternIdx);
          ui_show_error(genErrorString(gGen.lastError()));
          Serial.printf("[main] pattern apply failed: %s\n", genErrorString(gGen.lastError()));
        }
        break;
      }

      case MSG_SELECT_NAMED: {
        const char* key = m.payload.name;
        if (!key) break;
        const PatternRef* p = PatternLibrary::findByKey(key);
        if (!p) {
          ui_show_error("Unknown pattern key");
          break;
        }
        const PatternRef* applied = applyPatternRef(p, gCfg.rpm);
        if (applied == p) {
          if (p->name_key) {
            strncpy(g_pattern_key, p->name_key, sizeof(g_pattern_key) - 1);
            g_pattern_key[sizeof(g_pattern_key) - 1] = '\0';
            (void)NvsStore::setPatternKey(p->name_key);
          }
          ui_show_error("");
        } else {
          ui_show_error(genErrorString(gGen.lastError()));
          Serial.printf("[main] named pattern apply failed: %s\n", genErrorString(gGen.lastError()));
        }
        // name was heap-allocated by sender; manager frees here.
        // (Senders that pass a static .rodata literal should NOT use this
        //  msg type; use MSG_SELECT_BUILTIN or pass through the heap path.)
        // For safety we DO NOT free here — the convention is .rodata literals.
        break;
      }

      case MSG_LOAD_DSL: {
        const char* src = m.payload.name;
        if (!src) { publish_dsl_error("Empty DSL source"); break; }
        DslResult r = dslCompile(src);
        if (r.ok) {
          if (gGen.apply(r.pattern, gCfg.rpm)) {
            // Manager owns the PSRAM table now; track via PatternLibrary
            // under a transient key so it survives until next overwrite.
            (void)PatternLibrary::unregisterUser("scratch_dsl");
            // Free the prior scratch table (TODO 2) before publishing the
            // new one. cleanupScratchDsl() is also called by applyPatternRef
            // when the user swaps to a builtin.
            cleanupScratchDsl();
            r.pattern.name_key = "scratch_dsl";
            (void)PatternLibrary::registerUserPattern("scratch_dsl", r.pattern);
            s_scratch_dsl   = r.pattern;
            s_scratch_active = true;
            gActivePattern = PatternLibrary::findByKey("scratch_dsl");
            publish_dsl_error("");
            ui_show_error("");
          } else {
            dslFree(r.pattern);
            publish_dsl_error("DSL apply failed");
          }
        } else {
          char buf[128];
          snprintf(buf, sizeof(buf), "%s @%u", r.error, (unsigned)r.error_offset);
          publish_dsl_error(buf);
        }
        // Free the heap-owned DSL source string passed from sender.
        free((void*)src);
        break;
      }

      case MSG_LOAD_TABLE: {
        // Build a PatternRef in-place from raw bytes. Bytes pointer must
        // be lifetime-managed by the sender (e.g. captured buffer in PSRAM
        // tracked by CaptureRecorder).
        PatternRef ref{};
        ref.table        = m.payload.raw.bytes;
        ref.slot_count   = m.payload.raw.len;
        ref.degrees      = m.payload.raw.degrees ? m.payload.raw.degrees : 360;
        ref.rpm_scaler   = (float)ref.slot_count / 120.0f;
        ref.channel_mask = 0x01;  // capture is single-channel
        ref.name_key     = "loaded_table";
        if (gGen.apply(ref, gCfg.rpm)) {
          gActivePattern = nullptr;
          ui_show_error("");
        } else {
          ui_show_error("Table apply failed");
        }
        break;
      }

      case MSG_SET_SWEEP: {
        SweepConfig cfg{};
        cfg.low_rpm        = m.payload.sweep.low_rpm;
        cfg.high_rpm       = m.payload.sweep.high_rpm;
        cfg.mode           = (SweepMode)m.payload.sweep.mode;
        cfg.interval_us    = m.payload.sweep.interval_us;
        cfg.waypoints      = nullptr;
        cfg.waypoint_count = 0;
        sweepSet(cfg);
        g_sweep_low_rpm     = cfg.low_rpm;
        g_sweep_high_rpm    = cfg.high_rpm;
        g_sweep_mode        = (uint8_t)cfg.mode;
        g_sweep_interval_us = cfg.interval_us;
        (void)NvsStore::setSweep(cfg.low_rpm, cfg.high_rpm,
                                 (uint8_t)cfg.mode, cfg.interval_us);
        ui_show_error("");
        break;
      }

      case MSG_SET_COMPRESSION: {
        CompressionConfig c{};
        c.enabled    = m.payload.comp.enabled;
        c.cyl        = m.payload.comp.cyl;
        c.rpm_thresh = m.payload.comp.rpm_thresh;
        c.peak       = m.payload.comp.peak;
        c.dynamic    = m.payload.comp.dynamic;
        c.offset_deg = 0;
        c.custom_curve_256 = nullptr;
        compressionSet(c);
        g_comp_enabled    = c.enabled;
        g_comp_cyl        = c.cyl;
        g_comp_rpm_thresh = c.rpm_thresh;
        g_comp_peak       = c.peak;
        g_comp_dynamic    = c.dynamic;
        (void)NvsStore::setCompression(c.enabled, c.cyl, c.rpm_thresh,
                                        c.peak, c.dynamic);
        ui_show_error("");
        break;
      }

      case MSG_CAPTURE_START: {
        const uint16_t revs = (uint16_t)((uint32_t)m.payload.val ? (uint32_t)m.payload.val : 2u);
        if (!captureStart(PIN_CAPTURE_IN, revs)) {
          ui_show_error("Capture start failed");
        } else {
          ui_show_error("");
        }
        break;
      }

      case MSG_CAPTURE_STOP: {
        PatternRef captured{};
        if (captureFetchPattern(captured)) {
          // Register under timestamped key. Use a static buffer per slot —
          // PatternLibrary stores by value but the key must outlive
          // registration; reserve a small ring of buffers.
          static char s_cap_keys[4][32];
          static uint8_t s_cap_idx = 0;
          snprintf(s_cap_keys[s_cap_idx], sizeof(s_cap_keys[0]),
                   "captured_%lu", (unsigned long)millis());
          const char* key = s_cap_keys[s_cap_idx];
          s_cap_idx = (s_cap_idx + 1) % 4;
          captured.name_key = key;
          (void)PatternLibrary::registerUserPattern(key, captured);
          ui_show_error("");
        } else {
          ui_show_error("No capture data");
        }
        break;
      }

      case MSG_SAVE_USER: {
        // Payload carries BOTH the target key and the DSL source
        // (heap-owned by sender; manager frees both after consuming).
        const char* name = m.payload.save.name;
        const char* dsl  = m.payload.save.dsl_source;
        if (!name) {
          // No name → nothing to save. Free whatever source slipped through.
          if (dsl) free((void*)dsl);
          break;
        }
        if (dsl && dsl[0] != '\0') {
          // Preferred path: caller (DSL editor) supplied the canonical source.
          if (!PatternStorage::saveDsl(name, dsl)) {
            ui_show_error("Save failed");
          } else {
            ui_show_error("");
          }
        } else if (gActivePattern) {
          // Fallback (serial CLI path with no source on hand): write a
          // minimal placeholder DSL that references the active key. This
          // is best-effort — a real round-trip requires the source.
          char placeholder[96];
          snprintf(placeholder, sizeof(placeholder),
                   "// saved alias of %s\n", gActivePattern->name_key
                                              ? gActivePattern->name_key
                                              : "(active)");
          (void)PatternStorage::saveDsl(name, placeholder);
        }
        free((void*)name);
        if (dsl) free((void*)dsl);
        break;
      }
    }  // switch (m.type)
    }  // if (got == pdTRUE)

    // ---- Periodic maintenance (runs once per tickInterval) ----
    //
    // Debounced RPM commit (Agent C): commits the most recent setRpmDebounced()
    // value to NVS only after kRpmDebounceMs has elapsed since the last call.
    NvsStore::tickRpmDebounce();

    // Loopback validator (TODO 3 / M6.2): on each tick, compare the most
    // recent captured edge deltas against the expected pattern's slot
    // period at the live RPM. Sticky error → publish to the UI via the
    // g_loopback_error volatile buffer (polled by LVGL timer).
    captureLoopbackTick(g_rpm);
    if (captureLoopbackHasError()) {
      const char* msg = captureLoopbackErrorMsg();
      if (msg && *msg) {
        // Lock-free volatile copy; LVGL reader tolerates partial updates
        // because we only publish complete NUL-terminated strings.
        size_t i = 0;
        for (; i < sizeof(g_loopback_error) - 1 && msg[i]; ++i) {
          g_loopback_error[i] = msg[i];
        }
        g_loopback_error[i] = '\0';
      }
    }

    (void)lastGoodPattern;
  }
}

// =====================================================
// Cycle 7 — WiFi link telemetry hooks (Agent E)
// =====================================================
//
// The wifi_link task (Core 0, prio 1) NEVER reaches into this TU's globals.
// It reads telemetry exclusively through the function-pointer hooks below
// (WifiLinkTelemetry, declared in lib/wifi_link/wifi_link.h). Each getter
// returns a single naturally-aligned scalar (or a stable .rodata pointer),
// so a per-frame snapshot assembled by the wifi task is consistent enough
// for 5–10 Hz telemetry (no safety-critical field; D8).
//
// Semantics notes for the protocol (§4.0 / D15):
//   - edgeCounter() is a FREE-RUNNING uint16 that WRAPS every 65536 edges
//     with NO defined zero-phase origin (TableCkpGenerator.h:97). The phone
//     uses it only for an RPM-SIMULATED live cursor (board-phase-lock is a
//     future cycle).
//   - cycleDurationUs() is the FULL-TABLE period: a 720° table spans two
//     crank revolutions per cycle, so the phone's cursor math divides by the
//     pattern's `degrees` field, not by 360.

static bool        wlIsRunning()        { return gRunning; }
static uint32_t    wlCurrentRpm()       { return sweepCurrentRpm(); }
static uint32_t    wlBaseRpm()          { return g_rpm; }
static const char* wlActivePatternKey() {
  // null-guard: fall back to empty string so snprintf("%s") is always safe.
  return (gActivePattern && gActivePattern->name_key) ? gActivePattern->name_key
                                                      : "";
}
static uint16_t    wlActiveDegrees()    { return gActivePattern ? gActivePattern->degrees      : 0; }
static uint8_t     wlChannelMask()      { return gActivePattern ? gActivePattern->channel_mask : 0x01; }
static bool        wlInverted()         { return gInverted; }
static uint16_t    wlEdgeCounter()      { return gGen.getEdgeCounter(); }
static uint32_t    wlCycleDurationUs()  { return gGen.getCycleDurationUs(); }
static uint32_t    wlDropCount() {
  uint32_t n;
  portENTER_CRITICAL(&gUiMsgDropMux);
  n = gUiMsgDropCount;
  portEXIT_CRITICAL(&gUiMsgDropMux);
  return n;
}
static const char* wlDslError()         { return (const char*)g_dsl_error; }

// onLink: wifi_link pushes IP/SSID/mode here on every interface IP change.
// Forward to the additive LVGL link-status label (display builds only). The
// LVGL updater is itself cross-core-safe (stashes under s_ui_mux, applies on
// the LVGL thread) — we never touch LVGL objects from the wifi task here.
static void wlOnLink(const char* ip, const char* ssid, const char* mode) {
#if defined(SIGGEN_HAS_DISPLAY)
  ui_update_link(ip, ssid, mode);
#else
  (void)ip; (void)ssid; (void)mode;
#endif
}

static const WifiLinkTelemetry gWifiHooks = {
  wlIsRunning,
  wlCurrentRpm,
  wlBaseRpm,
  wlActivePatternKey,
  wlActiveDegrees,
  wlChannelMask,
  wlInverted,
  wlEdgeCounter,
  wlCycleDurationUs,
  wlDropCount,
  wlDslError,
  wlOnLink,
};

// =====================================================
// Arduino setup/loop
// =====================================================

void setup() {
  DBG_BEGIN();
  delay(250);

  Serial.println(F("\n[boot] === ESP32-S3 Signal Generator ==="));
  Serial.printf("[boot] flash:   %u bytes\n", (unsigned)ESP.getFlashChipSize());
  Serial.printf("[boot] psram:   found=%d size=%u bytes\n",
                (int)psramFound(), (unsigned)ESP.getPsramSize());
  Serial.printf("[boot] heap:    free_internal=%u free_psram=%u\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());

  // ---- LittleFS (gated on -DSIGGEN_USE_LITTLEFS=1; no-op on WROOM) ----
  if (!initLittleFS()) {
    DBG_PRINTLN("[FS] LittleFS mount failed");
  }
  if (!littleFsSmokeTest()) {
    DBG_PRINTLN("[FS] LittleFS smoke test failed");
  }

  // ---- NVS: load globals before applying anything ----
  if (!NvsStore::begin()) {
    DBG_PRINTLN("[NVS] begin failed; defaults will apply");
  }
  NvsStore::loadAllToGlobals();

  // ---- Generator init ----
  // ============================================================
  // ====== USER: SELECT BACKEND OUTPUT PINS ====================
  // All three channels — crank + cam1 + cam2 — are driven via the
  // begin() call below. The pins themselves are assigned ONCE, at
  // the top of this file:
  //   #define PIN_CKP_OUT / PIN_CAM1_OUT / PIN_CAM2_OUT (lines 74-76)
  // To re-map a channel, edit those macros — they are the single
  // user-editable source of truth for output pins.
  //
  // Each pin is validated inside TableCkpGenerator::begin() by the
  // isValidEsp32S3OutputPin() helper; any pin rejected as
  // invalid/reserved is logged to Serial with the exact reason and
  // the call fails (genOk == false).
  // ============================================================
  const bool genOk = gGen.begin(PIN_CKP_OUT, PIN_CAM1_OUT, PIN_CAM2_OUT);
  Serial.printf("[boot] generator init: %s (crank pin=%d cam1 pin=%d cam2 pin=%d)\n",
                genOk ? "OK" : "FAILED",
                (int)PIN_CKP_OUT, (int)PIN_CAM1_OUT, (int)PIN_CAM2_OUT);
  if (!genOk) {
    Serial.printf("[boot] generator error: %s\n",
                  genErrorString(gGen.lastError()));
  }
  if (!genOk) {
    ui_show_error("Generator init failed");
  }

  // Sweep / compression task lives at priority 2 on Core 1.
  if (!sweepCompressionInit(&gGen)) {
    DBG_PRINTLN("[SWP] sweep/compression init failed");
  }

  const bool capOk = capRX.begin(PIN_CAPTURE_IN);
  if (!capOk) {
    DBG_PRINTLN("[CAP] queue/interrupt init failed");
  }

  const bool uiOk = ui_init(on_ui_rpm, on_ui_pattern, on_ui_run,
                            on_ui_invert);
  if (!uiOk) {
    DBG_PRINTLN("[UI] init failed; running defaults only");
  }

  gCtrlQ = xQueueCreate(16, sizeof(CtrlMsg));
  if (!gCtrlQ) {
    ui_show_error("Queue alloc failed");
  } else {
    const BaseType_t ok = xTaskCreatePinnedToCore(managerTask, "managerTask",
                                                   8192, nullptr, 3, nullptr, 1);
    if (ok != pdPASS) {
      ui_show_error("Task create failed");
    }
  }

  // ---- Restore last applied pattern from NVS ----
  gCfg.rpm = g_rpm;
  const PatternRef* p = nullptr;
  if (g_pattern_key[0] != '\0') {
    p = PatternLibrary::findByKey(g_pattern_key);
  }
  if (!p) {
    p = PatternLibrary::builtinByIndex(0);
  }
  if (p && gGen.apply(*p, gCfg.rpm)) {
    gActivePattern = p;
    if (p->name_key) {
      strncpy(g_pattern_key, p->name_key, sizeof(g_pattern_key) - 1);
      g_pattern_key[sizeof(g_pattern_key) - 1] = '\0';
    }
    // Try to locate index for UI sync (linear search; 64 entries max).
    for (size_t i = 0; i < PatternLibrary::builtinCount(); ++i) {
      if (PatternLibrary::builtinByIndex(i) == p) {
        gPatternIdx = (uint8_t)i;
        break;
      }
    }
  } else {
    ui_show_error("Restore pattern failed");
  }

  // ---- Restore sweep + compression from NVS ----
  {
    SweepConfig sc{};
    sc.low_rpm        = g_sweep_low_rpm;
    sc.high_rpm       = g_sweep_high_rpm;
    sc.mode           = (SweepMode)g_sweep_mode;
    sc.interval_us    = g_sweep_interval_us;
    sc.waypoints      = nullptr;
    sc.waypoint_count = 0;
    sweepSet(sc);

    CompressionConfig cc{};
    cc.enabled    = g_comp_enabled;
    cc.cyl        = g_comp_cyl;
    cc.rpm_thresh = g_comp_rpm_thresh;
    cc.peak       = g_comp_peak;
    cc.dynamic    = g_comp_dynamic;
    cc.offset_deg = 0;
    cc.custom_curve_256 = nullptr;
    compressionSet(cc);
  }

  // ---- Restore invert mask + start ----
  gGen.setInverted(g_invert_mask);
  gInverted = ((g_invert_mask & 0x01u) != 0);

  const bool startOk = gGen.start();
  gRunning = startOk;
  if (!startOk) {
    Serial.printf("[boot] start failed: %s\n", genErrorString(gGen.lastError()));
  }

  // ---- Serial CLI ----
  serialCliBegin();

  // ---- Cycle 7: WiFi companion-app transport ----
  // Spawns the wifi_link task (Core 0, prio 1), brings up WiFi (STA-with-
  // timeout or SoftAP fallback), advertises mDNS, and serves NDJSON over TCP
  // 3333. A SECOND transport onto the SAME gCtrlQ — never a new backend.
  // Must run AFTER gCtrlQ exists + the generator is started (above) so the
  // first inbound control frame has a live queue and the first telemetry
  // frame reads a valid snapshot.
  wifiLinkInit(&gWifiHooks);

  ui_update_rpm(gCfg.rpm);
  ui_update_pattern(gPatternIdx);
  ui_update_running(true);
  ui_update_inverted(gInverted);
  ui_update_channels(gActivePattern ? gActivePattern->channel_mask : 0x01,
                     gGen.getInverted());
}

void loop() {
  ui_task_handler();

  CaptureReport r{};
  (void)capRX.fetch(r, 0);

  serialCliPoll();

  vTaskDelay(pdMS_TO_TICKS(10));
}
