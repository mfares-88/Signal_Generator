# Implementation-2 — Review Remediation Plan

> **Generated:** 2026-05-22
> **Branch:** Integrate-Ardu-Sim
> **Driver document:** `_Plans-and-Records/implementation-1_Review.md`
> **Orchestration:** 3 of the 5 standing agents (A: gen-backend, C: sweep-store, E: ui-io). Agents B (pattern-lib) and D (dsl-compiler) not engaged this cycle.

---

## 1. Context

The implementation-1 review (2026-05-22) found 12 defects across the ESP32-S3 firmware after first bench bring-up. Symptoms on the JC4827W543 LCD: widgets overlapped, every pattern selection produced `Pattern apply failed`, and Start did not reliably launch the generator. Root causes spanned three layers:

- **Backend** — invalid/conflicting output GPIOs (`PIN_CAM2_OUT=22` is not a valid ESP32-S3 GPIO; `PIN_CAM1_OUT=21` is owned by the LCD QSPI bus), missing start/apply acknowledgement (UI showed optimistic state regardless of failure), generic error reporting that hid the real cause, and ISR reads from pattern tables that live in flash/PSRAM (cache-unsafe during NVS / LittleFS writes).
- **Frontend (LVGL)** — right-column action buttons overlapped RUN / INVERT in absolute-coordinate layout; display rotation was auto-picked while GT911 touch rotation was hardcoded (coordinate systems out of phase); `on_arc_changed` flooded the command queue and NVS; modal spin rows had ambiguous positioning.
- **Build / storage** — partition labelled `littlefs` but typed `spiffs` (build warning); board metadata reported `No PSRAM` instead of N4R8; LVGL minor version drifted from `9.2.2` (config) to `9.5.0` (resolved).

This plan reuses the project's existing 5-agent ownership model. Only the three agents whose territory matched the findings were engaged.

---

## 2. Execution Model

**Hybrid — parallel research, sequential edits.** Each agent first read its target files in parallel. Edits then happened in dependency order to avoid `src/main.cpp` merge conflicts, since per the project's architectural rule (`implementation_plan.md §2`) only the UI / integration agent edits `src/main.cpp`:

1. **Agent A** (backend) — edits in `lib/ckp_gen/`. Defines new interface contract.
2. **Agent C** (storage / build) — edits in `lib/sweep_compression/`, `partitions_signalgen.csv`, `platformio.ini`, plus a new `boards/esp32-s3-n4r8.json`. Runs in parallel with A (disjoint files).
3. **Agent E** (UI + integration) — edits `lib/ui_lvgl/` and `src/main.cpp` to consume A's new interface and C's new helpers, plus rebuilds the main screen. Runs after both A and C complete.
4. **Integration validation** — one `pio run -e esp32-s3-n4r8`. Manual UI verification deferred to bench.

All work stayed on branch `Integrate-Ardu-Sim`. No worktrees.

---

## 3. Decisions Locked With User

| # | Topic | Decision |
|---|---|---|
| 1 | **Pin map (Finding 1)** | Agent A does NOT pick final cam pins. It adds a validation helper, board-reserved pin table, and explicit failure logging. The current invalid pins (`PIN_CKP_OUT=17`, `PIN_CAM1_OUT=21`, `PIN_CAM2_OUT=22`) remain untouched in `src/main.cpp` macros so the user can edit a single line later. For first bring-up the generator initializes **crank-only** (`gGen.begin(PIN_CKP_OUT, -1, -1)`) wrapped in a prominent `USER: SELECT BACKEND OUTPUT PINS` banner so the LCD and crank channel can be validated end-to-end. |
| 2 | **ISR memory safety (Finding 10)** | Agent A pre-allocates **24 KB** of internal DRAM (`heap_caps_malloc(24 * 1024, MALLOC_CAP_INTERNAL \| MALLOC_CAP_8BIT)`) at `begin()` and uses it as the playback buffer. `apply()` `memcpy`s pattern bytes into this buffer; ISR reads only from it. Allocation site is wrapped in a high-visibility banner comment (`// ====== PATTERN PLAYBACK BUFFER ALLOCATION =======`) so the buffer size can be tuned later. Hard-cap `_slot_count ≤ 24576` enforced in `apply()` with `GenError::BUFFER_OVERFLOW`. |
| 3 | **Execution mode** | Hybrid — parallel exploration, sequential edits in dependency order. A and C run in parallel (disjoint files); E waits for both. |
| 4 | **Layout fidelity (Finding 4)** | Agent E uses the review's Section 6 grid as baseline but may tune spacings, font sizes, and widths during implementation. Acceptance is visual: no rectangle intersections on 480×272, every touch target unique. |

---

## 4. Agent Work Packages

### Agent A — Backend (`gen-backend`)

**Files edited:**
- `lib/ckp_gen/CkpGenerator.h` — interface
- `lib/ckp_gen/CkpGenerator.cpp` — legacy `TimerCkpGenerator` shim (signature follow-on)
- `lib/ckp_gen/TableCkpGenerator.h`
- `lib/ckp_gen/TableCkpGenerator.cpp`

**Findings addressed:** 1, 2, 3, 10

**Changes:**

1. **Interface contract** (`CkpGenerator.h`)
   - `IGenerator::start()` / `stop()` return `bool` (was `void`).
   - New enum:
     ```cpp
     enum class GenError : uint8_t {
       OK, NOT_INITIALIZED, NO_TABLE, BAD_SLOT_COUNT,
       BAD_RPM, TIMER_FAIL, GPIO_FAIL, BUFFER_OVERFLOW
     };
     ```
   - New virtual: `GenError lastError() const`, `bool isReady() const`.
   - `TimerCkpGenerator` updated to match (`start`/`stop` return `true` unconditionally — legacy slot-machine path has no visible failure mode).

2. **Pin validation** (`TableCkpGenerator.cpp`)
   - Static helper `isValidEsp32S3OutputPin(int pin)` rejects:
     - pins `< 0` or `> 48`
     - GPIO 22–25 (not present on S3)
     - GPIO 26–32 (flash / PSRAM on N4R8)
     - strapping pins 0, 3, 45, 46
     - JC4827W543 board-reserved set: LCD QSPI (45, 47, 21, 48, 40, 39), backlight (1), GT911 touch (8, 4, 38), SD (10–13), I2S (42, 2, 41)
   - On rejection: `Serial.printf("[gen] begin: pin %d invalid (reason)\n", pin)`, set `_last_error = GPIO_FAIL`, return false **before** `gpio_config` / `dedic_gpio_new_bundle`.

3. **24 KB DRAM playback buffer** (`TableCkpGenerator.h`/`.cpp`)
   - New member: `uint8_t* _playback_buffer = nullptr;`
   - `static constexpr size_t kPlaybackBufferBytes = 24 * 1024;`
   - Allocated once at end of `begin()` with banner comment. Freed in destructor via `heap_caps_free`.
   - `apply()` validates `ref.slot_count ≤ kPlaybackBufferBytes`, then `memcpy(_playback_buffer, ref.table, ref.slot_count)` with the timer paused, sets `_table = _playback_buffer`. ISR contract unchanged (still reads `_table`).

4. **start() / stop() acknowledgement**
   - `start()`: returns false with appropriate `GenError` if `!_initialized` / `!_table` / `_slot_count == 0` / `gptimer_start` non-`ESP_OK`. Treats already-running as success.
   - `stop()`: returns false with `TIMER_FAIL` if `gptimer_stop` errors; treats already-stopped as success. Driving lines low (existing behavior) preserved.

5. **lastError / isReady**
   - `_last_error` is a `mutable GenError` member; set on every error path.
   - `isReady() == _initialized && _table != nullptr && _slot_count > 0`.

### Agent C — Storage / Build / Persistence (`sweep-store`)

**Files edited:**
- `partitions_signalgen.csv`
- `platformio.ini`
- `lib/sweep_compression/LittleFSInit.cpp`
- `lib/sweep_compression/NvsStore.h`
- `lib/sweep_compression/NvsStore.cpp`
- `boards/esp32-s3-n4r8.json` (new file)

**Findings addressed:** 6 (NVS side), 9, 11, 12

**Changes:**

1. **LittleFS partition warning fix** (Finding 9)
   - Partition renamed `littlefs` → `spiffs` in `partitions_signalgen.csv` (name now matches subtype `spiffs`, the arduino-esp32 LittleFS convention; suppresses `gen_esp32part.py` warning).
   - `LittleFS.begin(true, "/littlefs", 10, "spiffs")` — explicit label preserved for clarity.
   - One-line note added to `LittleFSInit.cpp` documenting the choice.

2. **RPM NVS debounce** (Finding 6, persistence side)
   - New public API in `NvsStore`:
     - `void setRpmDebounced(uint32_t rpm)` — records value, restarts 750 ms timer.
     - `void tickRpmDebounce()` — called from manager task loop; commits when timer expires.
     - `void flushPendingRpm()` — forced commit, e.g. before reboot.
   - `kRpmDebounceMs = 750`.
   - Header documents that Agent E must call `tickRpmDebounce()` from manager task loop.

3. **Custom board manifest** (Finding 11)
   - New file `boards/esp32-s3-n4r8.json` declares 4 MB QD flash, 8 MB OPI PSRAM, `partitions_signalgen.csv`, USB CDC on boot. PlatformIO auto-discovers `boards/` in project root.
   - `platformio.ini` `[env:esp32-s3-n4r8]`: `board = esp32-s3-devkitc-1` → `board = esp32-s3-n4r8`.

4. **Version pinning** (Finding 12)
   - `lvgl/lvgl@^9.2.2` → `lvgl/lvgl@9.2.2` (exact).
   - `moononournation/GFX Library for Arduino@^1.5.6` → `@1.6.4` (the version that built successfully).
   - `TAMC_GT911@^1.0.2` left as caret.

### Agent E — UI + Integration (`ui-io`)

**Files edited:**
- `lib/ui_lvgl/ui_lvgl.cpp`
- `src/main.cpp`

**Findings addressed:** 2, 3, 4, 5, 6 (UI side), 7, 8, 11 (boot diagnostics)

**Changes:**

1. **`src/main.cpp` — consume new backend contract** (Findings 2, 3)
   - New file-scope `genErrorString(GenError)` helper produces specific UI strings:
     - `"Apply: generator not initialized"`
     - `"Apply: no active pattern table"`
     - `"Apply: bad slot count"`
     - `"Apply: RPM out of range"`
     - `"Apply: timer alarm failed"`
     - `"Apply: GPIO invalid/reserved"`
     - `"Apply: pattern too large (>24KB)"`
   - `MSG_START` / `MSG_STOP`: check `bool` return; on failure surface `genErrorString(gGen.lastError())` and **do not** flip `gRunning`.
   - `MSG_SELECT_BUILTIN` / `MSG_SELECT_NAMED` / `MSG_SET_CUSTOM`: replace generic "Pattern apply failed" with the specific string from `gGen.lastError()`.

2. **NVS debounce wire-up** (Finding 6, persistence side)
   - `MSG_SET_RPM` (TABLE and LEGACY branches): `NvsStore::setRpm(...)` → `NvsStore::setRpmDebounced(...)`. Live `gGen.setRpm(rpm)` immediate path preserved.
   - `managerTask` loop: `NvsStore::tickRpmDebounce()` called once per iteration (after the 100 ms `xQueueReceive` timeout).

3. **Boot diagnostics** (Finding 11)
   - After `DBG_BEGIN()` in `setup()`:
     ```
     [boot] === ESP32-S3 Signal Generator ===
     [boot] flash:   <bytes>
     [boot] psram:   found=<0|1> size=<bytes>
     [boot] heap:    free_internal=<bytes> free_psram=<bytes>
     ```
   - After `gGen.begin(...)`: print success/failure and `genErrorString(gGen.lastError())`.

4. **Crank-only bring-up** (Finding 1, integration side)
   - `gGen.begin(PIN_CKP_OUT, PIN_CAM1_OUT, PIN_CAM2_OUT)` → `gGen.begin(PIN_CKP_OUT, /*cam1=*/-1, /*cam2=*/-1)` wrapped in the high-visibility `USER: SELECT BACKEND OUTPUT PINS` banner. Macro values themselves untouched.

5. **Main screen layout rebuild** (Finding 4)
   - New layout constants at file scope: `SCREEN_W/H`, `LEFT_X/Y/W/H`, `RIGHT_X/Y/W/H`.
   - Left pane container at `(8, 32, 224, 224)` hosts the RPM arc + value label + caption.
   - Right pane container at `(240, 12, 232, 252)` hosts:
     - PATTERN label + dropdown + filter textarea (top)
     - 2×2 action grid: SWEEP / COMP / DSL / WAVE (middle)
     - INVERT / START-STOP row (bottom)
   - Standalone top-right action column removed.
   - Channel LEDs stay top-left of screen; title stays top-mid.

6. **Display + touch rotation lock** (Finding 5)
   - `pick_display_rotation()` collapsed to a no-op applying `kDisplayRotation = 1`. Auto-loop removed.
   - `kTouchRotation = ROTATION_INVERTED` annotated with mirror-fix guidance.
   - Boot Serial prints: `display rotation=<N> width=<W> height=<H> touch_rotation=<R>`.

7. **RPM arc throttling** (Finding 6, UI side)
   - `on_arc_changed()` updates the local label on every event (visual feedback) but only fires `s_on_rpm(rpm)` on `LV_EVENT_RELEASED` **or** when `(millis() - s_last_send_ms) >= 50` and value changed. `LV_EVENT_RELEASED` listener added explicitly.

8. **Error label long-mode** (Finding 7)
   - `lbl_error`: width 230 px, `LV_LABEL_LONG_DOT`, placed at `x=8, y=252, w=230, h=18`. Hidden by default.

9. **Spin row flex layout** (Finding 8)
   - `make_spin_row()` rebuilt: `LV_FLEX_FLOW_ROW`, column gap 6, label (`flex_grow=1`), minus (32×30), spinbox (76×30), plus (32×30). Right-aligned absolute offsets removed.

---

## 5. Acceptance — Build Phase

Achieved in this orchestration:

| Criterion | Result |
|---|---|
| `pio run -e esp32-s3-n4r8` clean | ✅ SUCCESS in 125.71 s |
| No partition warning | ✅ (CSV name aligned with subtype) |
| LVGL resolved version exactly `9.2.2` | ✅ `lvgl @ 9.2.2` |
| GFX Library resolved `@ 1.6.4` | ✅ |
| Custom board manifest used | ✅ `board: esp32-s3-n4r8` |
| 24 KB DRAM buffer fits | ✅ RAM 31.0% used (101,736 / 327,680 bytes) |
| Flash budget | ✅ Flash 20.0% used (837,203 / 4,194,304 bytes) |

---

## 6. Acceptance — Bench Phase (Deferred to User)

These require physical hardware:

**Boot diagnostics on Serial (must show):**
- Flash size and PSRAM found / size around 8 MB.
- `LittleFS mounted` with total / used bytes.
- Generator init result (expected: FAILED with `GenError::GPIO_FAIL` until user picks valid cam pins, OR OK in crank-only mode).
- Selected display rotation, `gfx->width()` / `gfx->height()`, GT911 rotation.
- Free internal heap > 100 KB.

**UI manual test on LCD:**
- No overlapping rectangles on main screen.
- Pattern dropdown opens; row taps select the touched row.
- INVERT / START have unique non-overlapping touch targets.
- Long error strings don't draw over buttons.
- Modals (SWEEP, COMP, DSL, WAVE) fit within 480×272; spin-row controls don't overlap.

**Backend (logic analyzer / scope):**
- With crank pin alone: pattern `sixty_minus_two` selects without error; START produces a valid 60-2 trace; STOP returns line low.
- Force an invalid cam pin (edit `PIN_CAM1_OUT`): UI shows `Apply: GPIO invalid/reserved`; START rejected with a specific error.
- RPM arc drag across full range: no `Control queue full` in Serial; NVS write count is 1 per settled edit. Scope shows no malformed pulses while dragging.

**Storage:**
- Save DSL pattern → reboot → load → APPLY → identical scope trace.

**ISR safety:**
- Run generator with valid pattern, save NVS repeatedly, trigger LittleFS write. Scope must show no glitches.

---

## 7. Out of Scope (Deferred)

- Final cam pin selection — user decision pending bench.
- Reverse-direction signal validation.
- Capture loopback round-trip with the new bool-return interface.
- Compression dynamic-mode acceptance under the new rotation lock.
- Touch calibration confirmation (`kDisplayRotation = 1` may need flip after bench check).

---

## 8. Files Touched This Orchestration

**Modified:**
```
lib/ckp_gen/CkpGenerator.h
lib/ckp_gen/CkpGenerator.cpp
lib/ckp_gen/TableCkpGenerator.h
lib/ckp_gen/TableCkpGenerator.cpp
lib/sweep_compression/LittleFSInit.cpp
lib/sweep_compression/NvsStore.h
lib/sweep_compression/NvsStore.cpp
lib/ui_lvgl/ui_lvgl.cpp
src/main.cpp
partitions_signalgen.csv
platformio.ini
```

**New:**
```
boards/esp32-s3-n4r8.json
_Plans-and-Records/implementation-2_plan.md   ← this file
```

---

*This plan was generated, approved, and executed in a single autonomous orchestration session driving 3 of the 5 standing sub-agents. Bench verification per §6 remains the next step.*
