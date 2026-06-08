# ESP32 Signal Generator — Orchestrated 5-Agent Implementation Plan

> **Audience:** the orchestrator that schedules and monitors 5 parallel agents.
> **Goal:** drive the project from its current state (5 algorithmic patterns, 1 channel, no sweep/compression/DSL/persistence) to the strict-superset target defined in `integration_report.md`.
> **Source of truth for behavior:** `References/ardustim.ino`, `References/wheel_defs.h`, `References/globals.h`, `References/comms.cpp`, `References/enums.h`, `References/storage.h`, `References/Wheel_Notes`, `References/TODO`, `References/CLAUDE.md`.

---

## 0. Quick-Reference Inputs (Cross-Validated Against References)

These constants are referenced repeatedly below. They are the **non-negotiable correctness anchors** — agents must verify against `References/` before deviating.

| Item | Value / Source |
|---|---|
| Ardu-Stim ISR | `References/ardustim.ino:260-283` — 3 instructions of real work |
| `Wheels[]` shape | `{name_ptr, edges_ptr, rpm_scaler, wheel_max_edges, wheel_degrees}` — `References/ardustim.ino:59-125` |
| Byte encoding | bit0=crank (PB0), bit1=cam1 (PB1), bit2=cam2 (PB2), bit3=knock (reserved) — `References/CLAUDE.md` |
| AVR timer formula | `OCR1A = 8000000 / (rpm_scaler * rpm)` — `References/ardustim.ino:408` |
| `rpm_scaler` convention | `edges / 120` (so 60-2 = 1.0, Optispark 720-edge = 3.0) — verified via `References/Wheels[]` |
| Sweep | `loop()` increments ±1 RPM per `config.sweep_interval` µs; ping-pong between low/high — `References/ardustim.ino:302-318` |
| Compression sin tables | `sin_100_180`, `sin_100_120`, `sin_100_90` in `References/globals.h:80-120` |
| Compression dynamic gate | Only when `base_rpm < 655U` (AVR overflow guard) — `References/ardustim.ino:372` |
| `configTable` wire format | Packed, version-tagged, 22 bytes — `References/globals.h:40-56` |
| Serial opcodes (legacy) | `a`/`c`/`C`/`L`/`n`/`N`/`p`/`P`/`R`/`r`/`s`/`S`/`X` — `References/comms.cpp:62-158` |
| Serial `r` byte order gotcha | `word(hi, lo)` — bytes arrive HI-first (big-endian over wire) — `References/comms.cpp:128-130` |
| Crank-angle math depends on | `cycleStartTime`, `cycleDuration` updated by ISR on wrap — `References/ardustim.ino:268-271`, `:381-388` |
| Max edge rate (Optispark) | 18000 Hz at 6000 RPM; ~15k RPM AVR ISR ceiling — `References/Wheel_Notes:57-60` |

---

## 1. Architecture Locked In

These decisions are final. Agents do not re-litigate.

1. **Backend swap, not interface change.** `IGenerator` is widened (3 pins, `PatternRef`, `setRpm` fast path, per-channel invert mask) but the manager↔generator contract stays the same shape. Phase 1 ships behind a build flag with the algorithmic backend as fallback.
2. **Byte-packed multi-channel tables.** The Ardu-Stim format (`bit0=crank, bit1=cam1, bit2=cam2, bit3=knock-reserved`) becomes the universal in-memory representation. DSL, builtin library, serial uploads, captures all converge to this format.
3. **Primary backend = `GPTimer ISR` + `dedic_gpio` bundle.** RMT and LCD_CAM are explicitly NOT primary; LCD_CAM held in reserve (Phase 8 only).
4. **`SignalConfig` survives** as the input type for the existing "Symmetric/Missing" UI modal; the same compiler that consumes DSL text also consumes `SignalConfig` and emits a `PatternRef`. The slot-machine ISR is deleted at end of Phase 3.
5. **Validation + `lastGood` rollback** in `managerTask` is preserved exactly. Every new message type must integrate into this loop.
6. **LVGL pending-flag sync** (Core 0 ↔ Core 1) is the only UI sync mechanism. No new mechanism is allowed.
7. **String-keyed pattern selection.** Wheel index is NOT the persistent key — string keys (e.g. `"sixty_minus_two"`) are. A legacy-index → key migration table is maintained for any Ardu-Stim wire-protocol compatibility.
8. **Dual-mode serial.** Legacy single-byte Ardu-Stim opcodes coexist with the new text protocol. Detection rule: first byte is alphabetic + space → text; else legacy.

---

## 2. Agent Roster & Ownership Map

Five agents, one orchestrator. Each agent owns a directory tree end-to-end across phases. Cross-agent edits go through the orchestrator.

| Agent | Codename | Owns | Out of bounds |
|---|---|---|---|
| **A** | `gen-backend` | `lib/ckp_gen/`, `include/dedic_gpio_*`, ISR memory hygiene, `IGenerator` interface | UI, patterns, DSL |
| **B** | `pattern-lib` | `lib/patterns/`, `tools/convert_ardustim_wheels.py`, `References/wheel_defs.h` consumption | ISR internals, UI |
| **C** | `sweep-store` | `lib/sweep_compression/`, NVS schema, LittleFS partition, `platformio.ini` partitions | DSL grammar, UI widgets |
| **D** | `dsl-compiler` | `lib/dsl/` (lexer/parser/compiler/validator), DSL ↔ byte-table conversion | UI, persistence |
| **E** | `ui-io` | `lib/ui_lvgl/`, `lib/ckp_capture/`, serial CLI, waveform viewer | Generator internals, DSL parsing |

`src/main.cpp` is jointly owned but **only Agent E edits it**, on PRs from other agents — this prevents merge churn on the message-queue protocol.

---

## 3. Interface Contracts (Authoritative)

These contracts are frozen at the start of the project. Agents may not change them without an orchestrator-mediated amendment.

### 3.1 `PatternRef` (the universal pattern handle)

```cpp
// lib/ckp_gen/PatternRef.h (NEW — Agent A authors, Agents B/D/E consume)
struct PatternRef {
  const uint8_t* table;       // bit0=crank, bit1=cam1, bit2=cam2, bit3=knock(rsvd)
  uint16_t       slot_count;  // length of table[]
  uint16_t       degrees;     // 360 or 720
  float          rpm_scaler;  // slot_count / 120.0  (Ardu-Stim convention)
  uint8_t        channel_mask;// bits set = channels actually used by this pattern
  const char*    name_key;    // stable string key in .rodata, used for NVS persistence
};
```

### 3.2 `IGenerator` (the manager↔backend contract)

```cpp
// lib/ckp_gen/CkpGenerator.h (Agent A authors)
struct IGenerator {
  virtual bool begin(int pin_crank, int pin_cam1 = -1, int pin_cam2 = -1) = 0;
  virtual bool apply(const PatternRef& ref, uint32_t rpm) = 0;
  virtual bool setRpm(uint32_t rpm) = 0;                   // fast path — used in sweep
  virtual void setInverted(uint8_t channel_mask) = 0;      // per-channel XOR
  virtual uint8_t getInverted() const = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual uint16_t getEdgeCounter() const = 0;             // atomic read for waveform cursor
  virtual ~IGenerator() = default;
};
```

### 3.3 Message-queue protocol extension

```cpp
// src/main.cpp (Agent E edits, others PR through)
enum MsgType : uint8_t {
  MSG_SET_RPM, MSG_START, MSG_STOP, MSG_SET_INVERT,
  MSG_SELECT_BUILTIN,    // payload.val = builtin index
  MSG_SELECT_NAMED,      // payload.name = const char* string key
  MSG_SET_CUSTOM,        // payload.cfg = SignalConfig (legacy modal, goes through DSL)
  MSG_LOAD_DSL,          // payload.dsl_source = const char*  (RAM-resident, manager owns)
  MSG_LOAD_TABLE,        // payload.raw = {const uint8_t* bytes, uint16_t len, uint16_t degrees}
  MSG_SET_SWEEP,         // payload.sweep = {low, high, mode, interval_us}
  MSG_SET_COMPRESSION,   // payload.comp  = {enabled, cyl, rpm_thresh, peak, dynamic}
  MSG_CAPTURE_START,
  MSG_CAPTURE_STOP,
  MSG_SAVE_USER          // payload.name = target user-key
};
```

### 3.4 Pattern library API (Agent B authors, Agents C/D/E consume)

```cpp
// lib/patterns/PatternLibrary.h
namespace PatternLibrary {
  size_t builtinCount();
  const PatternRef* builtinByIndex(size_t i);
  const PatternRef* findByKey(const char* key);              // searches builtin + user + scratch
  bool registerUserPattern(const char* key, PatternRef ref); // ref.table must outlive registration
  bool unregisterUser(const char* key);
  size_t userCount();
  const PatternRef* userByIndex(size_t i);
  // Iteration helpers for the LVGL dropdown
  void forEach(void(*fn)(const PatternRef*, const char* tier, void*), void* user);
}
```

### 3.5 DSL compiler API (Agent D authors, Agent E consumes)

```cpp
// lib/dsl/Dsl.h
struct DslResult {
  bool          ok;
  PatternRef    pattern;        // valid only if ok; .table is malloc'd in PSRAM
  char          error[96];      // populated when !ok
  uint16_t      error_offset;   // index into source where parsing failed
};
DslResult dslCompile(const char* source);             // string DSL → PatternRef
DslResult dslCompileSignalConfig(const SignalConfig& cfg); // legacy UI modal path
void      dslFree(PatternRef& ref);                   // deallocate .table
```

### 3.6 Sweep/compression task API (Agent C authors, Agent E consumes)

```cpp
// lib/sweep_compression/SweepCompression.h
enum SweepMode : uint8_t { SWEEP_OFF, SWEEP_LINEAR, SWEEP_LOG, SWEEP_WAYPOINT };
struct SweepConfig { uint16_t low_rpm, high_rpm; SweepMode mode; uint32_t interval_us;
                     const uint32_t* waypoints; uint8_t waypoint_count; };
struct CompressionConfig { bool enabled; uint8_t cyl; uint16_t rpm_thresh;
                           uint8_t peak; bool dynamic; uint16_t offset_deg;
                           const uint8_t* custom_curve_256; };
bool sweepCompressionInit(IGenerator* gen);
void sweepSet(const SweepConfig&);
void compressionSet(const CompressionConfig&);
uint32_t sweepCurrentRpm();   // for UI display
```

### 3.7 Capture/loopback API (Agent E authors)

```cpp
// lib/ckp_capture/CaptureRecorder.h
bool   captureStart(uint8_t pin, uint16_t revolutions);
bool   captureFetchPattern(PatternRef& out);          // valid after captureStart completes
bool   loopbackEnable(const PatternRef& expected, uint32_t tolerance_us);
void   loopbackDisable();
```

---

## 4. Phase / Milestone Matrix

Each row is a **work-package** the orchestrator dispatches. Columns: assigned agent, prerequisites, deliverables, exit criteria (objective, measurable), estimated effort.

### MILESTONE M0 — Interface freeze & scaffolding (≤2 days, **gating**)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M0.1 | A | — | `PatternRef.h`, new `IGenerator` interface, **legacy `TimerCkpGenerator` adapter** that synthesizes a byte table from `SignalConfig` at runtime | Project compiles unchanged on both `esp32-s3-n4r8` and `esp32-wroom32d` envs; existing 5 patterns still run via adapter; logic-analyzer trace matches pre-refactor exactly. |
| M0.2 | E | M0.1 | Updated `src/main.cpp` message enum + dispatch (stubs for new types); UI `on_*` callbacks unchanged | `pio run` clean both envs; UI behavior unchanged. |
| M0.3 | C | — | New `partitions_signalgen.csv` (app 3 MB / nvs 24 KB / littlefs 1 MB); `board_build.partitions` updated for S3 env only (WROOM keeps `huge_app.csv`); LittleFS init code in `setup()` | Filesystem mounts; `LittleFS.format()` available; smoke test writes/reads `/test.txt`. |

> Orchestrator gate: **all of M0 must merge before any other agent starts.** The interface freeze is the keystone.

---

### MILESTONE M1 — Native byte-table backend (1-channel parity) (~1 week)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M1.1 | A | M0 | `TableCkpGenerator` (single-channel `dedic_gpio` bundle), `GPTimer` @ 1 µs tick, ISR with `IRAM_ATTR` | Scope of `TableCkpGenerator` 60-2 @ 1000 RPM is bit-identical to legacy backend; `setRpm()` proven not to rebuild buffers. |
| M1.2 | B | M0 | `tools/convert_ardustim_wheels.py` MVP: parses `References/wheel_defs.h`, emits two patterns (`sixty_minus_two`, `thirty_six_minus_one`) into `lib/patterns/builtin_tables_generated.h`; hand-validates byte equivalence | `diff` against reference bytes: zero differences. |
| M1.3 | A+B | M1.1, M1.2 | Build flag `-DSIGGEN_BACKEND=TABLE`; manager task wires `MSG_SELECT_BUILTIN` through `PatternLibrary::builtinByIndex` | With flag on, the two converted patterns scope-match within ±2 µs of legacy at all RPMs 100–6000. |

---

### MILESTONE M2 — Three-channel synchronized output (~1 week)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M2.1 | A | M1 | `TableCkpGenerator::begin(crank, cam1, cam2)`, bundle width 3; per-channel invert mask | Logic-analyzer confirms cam edges land on the exact crank slot specified by source byte (within one tick). |
| M2.2 | B | M1.2 | `convert_ardustim_wheels.py` extended to emit `sixty_minus_two_with_cam`, `thirty_six_minus_one_with_cam` | Bytes match reference. |
| M2.3 | E | M0.2, M2.1 | Three small LEDs on LVGL main screen reflecting live channel state; `channel_mask` greys out unused channels in dropdown labels | Visual smoke test on hardware. |

---

### MILESTONE M3 — Full 64-pattern library port (~1 week)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M3.1 | B | M1.2 | `convert_ardustim_wheels.py` upgraded to emit **all 64** patterns + the `Wheels[]` metadata table (including `name_key` derived from the C identifier of the array, NOT the `*_friendly_name` string) | 64 entries in `builtin_tables_generated.h`; total `.rodata` ≤ 18 KB per `pio run -t size`. |
| M3.2 | B | M3.1 | Friendly-name strings table (separate `.rodata` array — `name_key` → human label) | Strings present, byte cost confirmed ≤ 2.5 KB. |
| M3.3 | B | M3.1 | Legacy-index → `name_key` migration table mirroring `Wheels[]` order from `References/ardustim.ino:59-125` | Migration unit test in `test/`: pass legacy index → expect specific key. |
| M3.4 | E | M3.1, M3.2 | LVGL dropdown becomes scrollable list with category sections (Distributor / Missing-tooth crank / Crank+Cam / Angular OEM); search filter | All 64 selectable from touch; saved selection persists across reboot via NVS string key. |
| M3.5 | C | M3.1 | NVS schema v1: namespace `siggen`, keys `pattern_key`, `rpm`, `invert_mask`, `sweep_*`, `comp_*`; load on boot, save on apply | `nvs_get_str("pattern_key")` after reboot returns the last-applied key. |
| M3.6 | A | M2.1 | Delete `TimerCkpGenerator` slot-machine implementation; remove build flag | Only `TableCkpGenerator` ships; no algorithmic ISR remains. |

---

### MILESTONE M4 — Sweep + compression simulation (~1 week, parallel with M5)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M4.1 | C | M1, IGenerator::setRpm available | `lib/sweep_compression/`: FreeRTOS task on Core 1, priority 2 (below manager); linear sweep verbatim port from `References/ardustim.ino:302-318` | Scope confirms RPM linearly ramps low→high→low at the configured interval. |
| M4.2 | C | M4.1 | Sin tables `sin_100_180/120/90` into `.rodata`; `calculateCompressionModifier()` port honoring `compressionDynamic`/`compressionOffset`/`compressionRPM` from `References/ardustim.ino:333-389`. Generator publishes `cycleStartTime`/`cycleDuration` via two `volatile uint32_t` accessors (the cheap atomic-32 read pattern), no critical section needed. | Below `compressionRPM`, the captured RPM exhibits the expected sin dip with peak amplitude = 100. Loopback capture (M6) verifies. |
| M4.3 | C | M4.2 | Log-sweep mode (exponential ramp) + waypoint-sweep mode (linear interp between `(rpm, dwell_ms)`) | Both modes scope-verified. |
| M4.4 | C | M4.2 | Per-cyl-1/3/2-stroke compression profiles (sin tables); user-loadable 256-entry custom curve; configurable peak amplitude (replaces hardcoded 100) | 1-cyl and 3-cyl waveforms scope-match reference shapes. |
| M4.5 | E | M4.1–M4.4 | LVGL "Sweep" + "Compression" tabs / modals; serial commands `SWEEP SET …`, `COMP SET …` | UI smoke test; settings persist in NVS via M3.5. |

> Gotcha for Agent C: `compressionDynamic` AVR guard `base_rpm < 655U` is an overflow guard. On 32-bit ESP32 this is unnecessary but the behavior should remain bit-equivalent. **Preserve the gate.**

---

### MILESTONE M5 — DSL compiler + user pattern library (~1–2 weeks, parallel with M4)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M5.1 | D | M0 | `lib/dsl/Lexer.cpp`: tokenizer for grammar in `integration_report.md` §7.1 | Unit tests for all token classes; rejects 5 malformed inputs cleanly. |
| M5.2 | D | M5.1 | `Parser.cpp`: recursive-descent into `WheelDef` AST | Parses the four worked examples (`integration_report.md` §7.1) into expected ASTs. |
| M5.3 | D | M5.2 | `Compiler.cpp`: per-wheel slot expansion (Symmetric / Missing / Angular per §7.2), cam-doubling, **LCM merge** for multi-wheel groups, canonicalization (rotate so slot 0 is rising edge of lowest active pin) | Compiles `"1,C,M,1/2,60,58t,2m : 2,c,S,1/2,1"` to **120-byte** buffer matching `sixty_minus_two_with_halfmoon_cam` from `References/wheel_defs.h`. |
| M5.4 | D | M5.3 | `Validator.cpp`: all 12 rules from `integration_report.md` §7.5 enforced pre-compile | Each rule has a dedicated failing-input test. |
| M5.5 | D | M5.3 | `dslCompileSignalConfig()` adapter so the existing UI "custom" modal goes through the same compiler | Existing modal produces same waveform as before refactor. |
| M5.6 | C | M0.3, M5.3 | LittleFS storage: `/patterns/<key>.dsl` (source) + `/patterns/<key>.bin` (compiled cache, invalidated by source hash) | Round-trip: write DSL → compile → save → reboot → load → identical scope trace. |
| M5.7 | E | M5.5, M5.6 | LVGL modal with textarea, Compile / Save As / Load buttons; serial: `COMPILE <dsl>`, `SAVE user/<name>`, `LOAD user/<name>`, `LIST`, `DELETE user/<name>` | All four worked examples enter via UI and compile. |

---

### MILESTONE M6 — Capture-to-table + loopback validation (~1 week)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M6.1 | E | M0, M3 | `CaptureRecorder`: records N revolutions, derives `slot_count` + `rpm_scaler`, builds `PatternRef`, hands to `PatternLibrary::registerUserPattern("captured_<ts>", …)` | Round-trip: replay 60-2, capture, byte-for-byte match. |
| M6.2 | E | M6.1, M4 | `LoopbackValidator` running in `managerTask`: continuously compares capture period/duty against expected from active `PatternRef` + current RPM; surfaces UI error on > tolerance divergence | With intentional GPIO misconfig, error appears within 1 second. |
| M6.3 | E | M6.1 | LVGL "Capture" page: start/stop, sample count, save as user pattern | Manual smoke test. |

---

### MILESTONE M7 — Waveform visualization (~3 days)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M7.1 | E | M2.1, M3 | LVGL `lv_canvas` widget rendering active `PatternRef.table` as 3-lane horizontal scope strip; cursor follows `IGenerator::getEdgeCounter()` via 50 ms LVGL timer | At 1000 RPM, cursor visibly traverses pattern in 60 ms (360°) or 120 ms (720°) cycles. |
| M7.2 | E | M7.1 | Touch pan / zoom; per-channel toggle via tap on lane | Manual smoke test. |

---

### MILESTONE M8 — Optional LCD_CAM backend (DEFERRED, gated on profiling)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M8.1 | A | M4 complete + profiler shows ISR saturation on Audi 135 + sweep + compression > 8000 RPM | `LcdCamCkpGenerator` behind same `IGenerator`; GDMA descriptor chain; LCD_DATA[2:0] mapped to outputs | Stress test passes. |

> Orchestrator should **NOT** schedule M8 until M4 profiling data justifies it.

---

### MILESTONE M9 — Serial protocol parity (~3 days)

| WP | Agent | Prereq | Deliverable | Exit criterion |
|---|---|---|---|---|
| M9.1 | E | M3, M4 | Legacy byte-opcode parser matching `References/comms.cpp` exactly: `a/c/C/L/n/N/p/P/R/r/s/S/X`. **Honor the `r` big-endian byte order** (`References/comms.cpp:128-130`). `c`/`C` use `configTable` v2 schema with version-tagged migration from v1. | Existing Ardu-Stim Electron GUI (`References/CLAUDE.md`) connects, enumerates wheels, switches wheels, sweeps — without modification. |
| M9.2 | E | M9.1, M5, M6 | New text protocol: `LIST`, `SELECT <key>`, `COMPILE <dsl>`, `SAVE user/<name>`, `LOAD user/<name>`, `DELETE …`, `CAPTURE START/STOP`, `SWEEP …`, `COMP …`. Detection rule: first byte alphabetic + space → text; else legacy. | Both modes parsed correctly in interleaved test. |

---

## 5. Dependency DAG (for the orchestrator scheduler)

```
            M0 (gating)
              │
   ┌──────────┼──────────┬────────────┐
   │          │          │            │
  M1         M3.5       M5.1         (independent: M6.1)
   │       (NVS prep)    │
   ├──── M2 ──┐         M5.2
   │          │          │
   │         M2.3       M5.3
   │          │       ┌──┴───────┐
  M3.1       │       M5.4      M5.5
   ├──> M3.2,3,4,6,5  │          │
   │                  └── M5.6 ──┘
   │                       │
   └───────> M4 ───────────┘
              │             │
              └─── M4.5 ────┤
                            │
                          M5.7
                            │
                  M6 ───────┤
                            │
                          M7 ─── M9
```

Critical path is `M0 → M1 → M2 → M3 → M9`. M4 and M5 fan out in parallel from M0/M1. M6 and M7 land near the end.

---

## 6. Per-Agent Standing Orders

### Agent A (`gen-backend`)

- **Mandate:** correctness and determinism of the ISR.
- **Hard rules:**
  - `IRAM_ATTR` on every function reachable from `gptimer_isr_register`-installed callback.
  - The ISR body MUST be ≤ 5 statements: byte load, XOR mask, `dedic_gpio_bundle_write`, counter advance, wrap check + `cycleStartTime`/`cycleDuration` publish. Anything more is rejected.
  - The byte tables themselves may stay in `.rodata` (flash) — Xtensa caches them — UNLESS profiling Phase 8 reveals cache-miss jitter, in which case the active table is copied into a DRAM buffer at `apply()` time.
  - `getEdgeCounter()` returns a single aligned `uint16_t` — no critical section. Document this invariant in the header.
  - Publish `cycleStartTime` and `cycleDuration` as two `volatile uint32_t` accessors for the compression task — naturally aligned 32-bit reads on Xtensa are atomic.
- **Deliverables across M0-M3 + M8.**

### Agent B (`pattern-lib`)

- **Mandate:** the byte tables are correct, the metadata is correct, and the conversion is reproducible.
- **Hard rules:**
  - `convert_ardustim_wheels.py` is the **single source of truth**. Generated files carry `// AUTO-GENERATED — DO NOT EDIT` header. Hand-editing the generated file is forbidden; orchestrator rejects PRs that do.
  - `name_key` is derived from the **C identifier** of the array in `wheel_defs.h` (e.g. `sixty_minus_two`), NOT the `*_friendly_name` string. Friendly names go in a separate `.rodata` array indexed by `name_key`.
  - `rpm_scaler` is **copied verbatim** from `References/ardustim.ino:59-125`. Do not re-derive — the reference values are the contract.
  - Total `.rodata` budget: hard ceiling 20 KB (target 16 KB). Report actual via `pio run -t size` at every merge.
- **Deliverables across M1-M3.**

### Agent C (`sweep-store`)

- **Mandate:** persistence and timed-RPM workloads.
- **Hard rules:**
  - Sweep/compression task lives at **priority 2** (one below the manager task at priority 3). Sweep calls `gen->setRpm()` ONLY — never `gen->apply()` — to avoid buffer rebuilds.
  - NVS schema is versioned (`uint8_t schema_version` key). `loadConfig()` performs forward migration; mismatches default to factory settings, NEVER block boot.
  - LittleFS partition: 1 MB. Patterns at `/patterns/<key>.dsl` and `/patterns/<key>.bin`. The `.bin` cache is invalidated when SHA-256 of `.dsl` changes.
  - Preserve the AVR `compressionDynamic` overflow guard (`base_rpm < 655U`) for behavioral parity with reference, even though ESP32 doesn't need it.
- **Deliverables across M0.3, M3.5, M4, M5.6.**

### Agent D (`dsl-compiler`)

- **Mandate:** the DSL grammar in `integration_report.md` §7.1 is the contract; the compiler produces canonical, validated `PatternRef`s.
- **Hard rules:**
  - Compiler output buffer ≤ 4096 bytes per pattern (validation rule §7.5 #9).
  - Canonicalization rule: rotate compiled buffer so slot 0 = rising edge of lowest-numbered active pin. (Dedups phase-shifted duplicates.)
  - Allocates `PatternRef.table` from PSRAM (`heap_caps_malloc(len, MALLOC_CAP_SPIRAM)`). `dslFree()` releases.
  - Validate the four worked examples in §7.1 — they form the regression test set. Add to `test/test_dsl.cpp`.
  - `dslCompileSignalConfig()` is **not** an independent code path — it constructs the equivalent DSL string in-memory and feeds it through the normal pipeline. Single source of truth.
- **Deliverables across M5.

### Agent E (`ui-io`)

- **Mandate:** every user-visible surface (LVGL, serial, capture, waveform viewer). Custodian of `src/main.cpp`.
- **Hard rules:**
  - LVGL pending-flag pattern (`s_ui_mux` spinlock) is the ONLY cross-core UI sync mechanism. New widgets extend this — they don't bypass it.
  - All UI inputs flow through `gCtrlQ`. UI callbacks must never call generator functions directly.
  - Serial protocol detection: first byte alphabetic (`isalpha(c)`) followed by space → text mode; else legacy single-byte mode. No protocol negotiation handshake.
  - Legacy `r` command MUST read bytes in HI-LO order per `References/comms.cpp:128-130` — this is a known Ardu-Stim wire-protocol gotcha.
  - Existing Ardu-Stim Electron GUI (`References/CLAUDE.md` describes it at `UI/`) must connect unchanged. This is the M9.1 acceptance test.
- **Deliverables across M0.2, M2.3, M3.4, M4.5, M5.7, M6, M7, M9.

---

## 7. Cross-Cutting Conventions

| Topic | Convention |
|---|---|
| Build envs | `esp32-s3-n4r8` is primary target. `esp32-wroom32d` keeps legacy partitions, no LittleFS, no waveform viewer (no display). Agents flag any feature broken on WROOM. |
| ISR memory | `IRAM_ATTR` on all ISR-reachable functions. Active pattern table in `.rodata` is acceptable; revisit only if M8 triggered. |
| Testing | `test/` directory holds PlatformIO Unity tests. DSL has the densest unit-test budget; ISR is verified by scope only. |
| Commit policy | Each WP ID (e.g. `M3.4`) is a commit-message prefix. Orchestrator gates milestones on green CI + scope traces. |
| Documentation | Every new module includes a one-paragraph header comment citing its parent integration-report section. No README proliferation. |
| Memory budget | `.rodata` ≤ 32 KB total (16 KB patterns + 2 KB names + 8 KB sin tables + slack). Runtime user-pattern PSRAM allocation unlimited but logged. |
| Style | Match existing project style (`AGENTS.md`). Lower-case + underscores for files, PascalCase classes, no smart pointers in ISR-adjacent code. |

---

## 8. Known Risks & Pre-Decided Resolutions

These are not open questions for the agents — they are decisions the orchestrator should remind agents of when relevant.

1. **WROOM-32D scope.** WROOM keeps `huge_app.csv` partitions and runs the M0–M3 single-channel core only. No LittleFS, no DSL persistence, no UI. Serial-only headless mode. Agent C does NOT carve LittleFS into WROOM's partition table.
2. **Partition table change wipes flash.** First-boot detection of new partition layout is logged but does not migrate user state. One-time loss is documented in changelogs.
3. **Knock channel.** Bit 3 of every byte is reserved from day one. ISR ignores it. M2 + future M8 may light it up for the LS1 pattern (per `References/CLAUDE.md` comment at the `gm_ls1_crank_and_cam` row).
4. **Reverse rotation.** Implemented in M2 as a one-line `_edge--` toggle in the ISR. Free feature; expose in UI as a checkbox.
5. **AVR-isms preserved.** `compressionDynamic < 655 RPM` guard, the bidirectional ping-pong sweep, the 100-amplitude sin tables, the `word(hi, lo)` serial byte order — all preserved for behavioral parity. Agents add improvements on top, they don't replace.
6. **`*_friendly_name` strings.** These exist in `References/wheel_defs.h` separately from the byte arrays. Agent B's converter extracts both: byte array → `builtin_tables_generated.h`, friendly strings → `pattern_names_generated.h`.
7. **`SignalConfig` lifetime.** Survives through M4. Removed only if M5.7 ships and the UI custom modal converts to a full DSL editor. Marker for future cleanup.

---

## 9. Orchestrator Operating Procedure

1. **Schedule M0 first, serially.** All three M0 WPs must merge before any other dispatch.
2. **After M0, fan out:** dispatch M1.1 (A), M1.2 (B), M5.1 (D), M3.5 (C) in parallel. M0.3's LittleFS work is a precondition for M5.6 but not for M5.1–5.5.
3. **Gate each milestone on hardware sign-off.** Logic-analyzer traces and scope screenshots are stored in `2.Status_Reports/<milestone>/`.
4. **Conflict resolution.** If two agents need to touch `src/main.cpp` or the `IGenerator` interface in the same window, route through orchestrator-mediated synchronous handoff. Agent E (custodian of `main.cpp`) integrates other agents' changes via PR.
5. **Profiling checkpoint after M4.** Decide whether to schedule M8 based on ISR saturation profile under sweep + compression + Audi 135 / Optispark / Nissan 360 CAS.
6. **Memory checkpoint after M3.** Confirm `.rodata` under 20 KB ceiling before approving M5 storage tier.
7. **Acceptance demo.** End-to-end demo at M9: existing Ardu-Stim Electron GUI drives the ESP32 device. Pattern selection, sweep, RPM display, scope preview all work without GUI modification.

---

*Plan generated 2026-05-20 from analysis of the project codebase, `integration_report.md`, `technical_report.md`, and all Tier 1/2/3 files in `References/`. All file:line references are resolved against the working tree at that date.*
