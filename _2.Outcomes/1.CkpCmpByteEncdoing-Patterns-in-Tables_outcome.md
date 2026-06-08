# Implementation 1 Review - LVGL and Backend Fix Guide

Generated: 2026-05-22  
Target repository: ESP32 crankshaft signal generator  
Primary build reviewed: `pio run -e esp32-s3-n4r8`  
Review skills applied: `review-lvgl-esp32-ui`, `review-esp32-backend-architecture`

## 1. Purpose

This document is an implementation guide for the team manager and assigned agents. It records the review findings from the current firmware state and turns them into concrete work packages.

The uploaded firmware reportedly booted, rendered LVGL elements, and showed the UI on the ESP32 LCD, but interaction was buggy:

- Icons, labels, dropdowns, and LVGL elements overlapped or appeared randomly positioned.
- Selecting any pattern produced a message similar to `Pattern apply failed`.
- Pressing `Start` produced an error or did not reliably start the signal generator.

The review found both frontend layout defects and backend hardware/interface defects. The backend issues are likely causing most of the pattern/start errors; the LVGL issues explain the visible overlap and unreliable touch interaction.

## 2. Build And Hardware Context

Reviewed project facts:

- PlatformIO environment: `esp32-s3-n4r8`
- Framework: Arduino on pioarduino ESP32 platform
- LVGL dependency requested: `lvgl/lvgl@^9.2.2`
- LVGL actually resolved during build: `9.5.0`
- Display library actually resolved during build: `GFX Library for Arduino 1.6.4`
- Display target: Guiton/JC4827W543, nominal `480x272`
- Touch controller: GT911 over I2C
- UI source: `lib/ui_lvgl/ui_lvgl.cpp`
- Backend source: `src/main.cpp`, `lib/ckp_gen/TableCkpGenerator.cpp`

Build result:

- `pio run -e esp32-s3-n4r8` completed successfully.
- Build emitted a partition warning: partition named `littlefs` uses subtype `spiffs`.
- PlatformIO reported board metadata as `ESP32-S3-DevKitC-1-N8 (8 MB QD, No PSRAM)`, which does not match the intended `ESP32-S3-N4R8` description in the project docs. This must be resolved before relying on PSRAM-heavy features.

## 3. Highest Priority Fix Order

The team should fix these in order:

1. Correct the signal-output pin map and generator initialization failure.
2. Add explicit backend start/apply acknowledgements so the UI cannot show false state.
3. Rebuild the main LVGL screen layout for the exact `480x272` canvas.
4. Lock display/touch rotation and coordinate mapping.
5. Fix RPM event flooding and NVS write timing.
6. Fix LittleFS partition labeling.
7. Move ISR-read pattern tables into internal RAM or otherwise prove cache-safe timing.

Do not start with cosmetic UI changes. If the generator fails to initialize because of invalid or display-conflicting pins, pattern selection and start/stop will keep failing regardless of the UI layout.

## 4. Findings And Fix Instructions

### Finding 1 - Invalid/Conflicting Output Pins Break The Generator

Severity: Critical  
Boundary: Backend / MCU hardware  
Evidence:

- `src/main.cpp` defines:
  - `PIN_CKP_OUT 17`
  - `PIN_CAM1_OUT 21`
  - `PIN_CAM2_OUT 22`
- `TableCkpGenerator::begin()` configures all participating pins as GPIO outputs in one bundle.
- ESP32-S3 SoC caps state: valid GPIOs are `0..48 except 22..25`.
- `PINS_JC4827W543.h` uses GPIO21 for display QSPI D0.

Issue:

- GPIO22 is not valid on ESP32-S3.
- GPIO21 is already owned by the LCD bus on the JC4827W543 board.
- A single invalid/conflicting cam pin can make `gpio_config()` or `dedic_gpio_new_bundle()` fail, leaving `TableCkpGenerator` uninitialized.

Impact:

- `gGen.begin()` can return false.
- `gGen.apply()` later returns false because `_initialized == false`.
- UI pattern selection shows `Pattern apply failed`.
- `Start` cannot truly start because the generator has no valid initialized backend.

Fix/path forward:

- Choose output pins from valid, unused ESP32-S3 GPIOs that do not overlap:
  - LCD QSPI: `45, 47, 21, 48, 40, 39`
  - Backlight: `1`
  - Touch: `8, 4, 3, 38`
  - SD: `10, 11, 12, 13`
  - I2S listed in board header: `42, 2, 41`
  - USB/JTAG/boot-straps should also be reviewed before final selection.
- For first bench recovery, run crank-only:
  - `gGen.begin(PIN_CKP_OUT, -1, -1)`
  - Keep UI channel LEDs but grey cam channels.
- Then add cam pins one at a time after validating the LCD still works.
- Add a pin validation helper before generator init:
  - reject `!digitalPinCanOutput(pin)`
  - reject pins in a board-reserved table
  - log exact failing pin and reason to Serial and UI.

Acceptance tests:

- Boot serial log must not show `Generator init failed`.
- Pattern select `sixty_minus_two` must clear error.
- `Start` must produce output on the crank pin.
- Adding cam pins must not disturb the LCD or touch.

Recommended owner:

- Backend/hardware agent.

### Finding 2 - Start/Stop UI Has No Backend Acknowledgement

Severity: High  
Boundary: Backend/UI contract  
Evidence:

- In `src/main.cpp`, `MSG_START` calls `gGen.start()` and then unconditionally sets:
  - `gRunning = true`
  - `ui_update_running(true)`
- `TableCkpGenerator::start()` returns `void` and silently returns if:
  - generator is not initialized
  - generator is already running
  - active table is null
  - slot count is zero
  - `gptimer_start()` fails

Issue:

The UI is told that the backend is running even when the generator rejected the start request internally.

Impact:

- Pressing `Start` can show the wrong state.
- Existing errors remain visible or unrelated errors appear.
- Debugging becomes misleading because the UI state is optimistic.

Fix/path forward:

- Preferred fix: change `IGenerator::start()` to return `bool`.
  - `true`: timer started or was already running with a valid table.
  - `false`: not initialized, no table, invalid slot count, or driver start failure.
- Add `bool isReady() const` or `GeneratorStatus getStatus()` if the team wants to avoid broad interface changes.
- In `managerTask`:
  - only set `gRunning = true` after success
  - show `Generator not ready` or `Start failed`
  - call `ui_update_running(gRunning)` with backend truth.
- Add serial debug for `gptimer_start()` failure code.

Acceptance tests:

- With no active table, pressing Start must show a clear error and keep button label as `START`.
- With valid table, pressing Start must update button to `STOP`.
- With invalid pins, pattern selection and Start must show different, specific errors.

Recommended owner:

- Backend interface agent, coordinated with UI agent.

### Finding 3 - Pattern Selection Fails Because Apply Error Has No Root Cause

Severity: High  
Boundary: Backend/UI contract  
Evidence:

- `MSG_SELECT_BUILTIN` calls `applyPatternRef()`.
- `applyPatternRef()` only returns active pattern pointer or previous pattern; it does not expose why `gGen.apply()` failed.
- UI receives generic `Pattern apply failed`.

Issue:

The manager cannot distinguish:

- bad builtin index
- generator not initialized
- invalid pattern table
- invalid RPM
- timer/gptimer failure
- pin/bundle failure from setup

Impact:

The screen error is generic and does not direct the operator or developer to the actual defect.

Fix/path forward:

- Add a backend error enum:
  - `GEN_OK`
  - `GEN_ERR_NOT_INITIALIZED`
  - `GEN_ERR_NO_TABLE`
  - `GEN_ERR_BAD_SLOT_COUNT`
  - `GEN_ERR_BAD_RPM`
  - `GEN_ERR_TIMER`
  - `GEN_ERR_GPIO`
- Expose a lightweight `const char* lastError()` or status enum from `TableCkpGenerator`.
- Update UI error strings in manager:
  - `Pattern apply failed: generator not initialized`
  - `Pattern apply failed: timer alarm`
  - `Pattern apply failed: bad RPM`

Acceptance tests:

- Force an invalid cam pin and confirm the UI says generator/GPIO not ready.
- Select a valid pattern with valid pins and confirm no error.
- Select an out-of-range index from serial or UI test and confirm `Bad builtin index`.

Recommended owner:

- Backend interface agent.

### Finding 4 - Main Screen Absolute Coordinates Overlap On 480x272

Severity: High  
Boundary: LVGL layout  
Evidence:

- RPM arc: `200x200`, top-left at `(20,24)`.
- Dropdown: width `200`, top-right at `(x=260,y=24)` on a 480-wide display.
- Run button: `160x44`, bottom-right at `x=300..460`, `y=210..254`.
- Invert button: above run, `x=300..460`, `y=156..200`.
- Action column:
  - buttons `72x28`
  - top-right at `x=398..470`
  - y positions `110,142,174,206`

Issue:

The right action column overlaps the run/invert controls:

- `COMP` button overlaps/contacts invert area.
- `DSL` button overlaps invert area.
- `WAVE` button overlaps run area.

Impact:

The exact reported behavior: labels, icons, dropdowns, and controls appear overlapped and random. Touches in the overlapped region may trigger the wrong object.

Fix/path forward:

Replace the main-screen layout with a fixed 480x272 layout grid:

- Root: `480x272`
- Header/status row: `x=8,y=4,w=464,h=24`
- Left gauge region: `x=8,y=32,w=224,h=224`
- Right controls region: `x=240,y=12,w=232,h=252`
- Pattern label/dropdown:
  - label `x=248,y=14`
  - dropdown `x=248,y=30,w=212,h≈36`
  - filter `x=248,y=74,w=212,h=28`
- Action grid:
  - `SWEEP` `x=248,y=112,w=100,h=34`
  - `COMP` `x=360,y=112,w=100,h=34`
  - `DSL` `x=248,y=154,w=100,h=34`
  - `WAVE` `x=360,y=154,w=100,h=34`
- Bottom run controls:
  - `INVERT` `x=248,y=210,w=100,h=40`
  - `START/STOP` `x=360,y=210,w=100,h=40`
- Error strip:
  - `x=8,y=252,w=230,h=18`
  - set label long mode to dot or scroll, not unlimited draw.

Implementation notes:

- Avoid `LV_ALIGN_TOP_RIGHT` for mixed columns until the final grid is proven.
- Use a single parent container for the right pane with flex or grid layout.
- Create constants:
  - `SCREEN_W = 480`
  - `SCREEN_H = 272`
  - `RIGHT_X = 240`
  - `RIGHT_W = 232`
- If `gfx->width()/height()` is not `480x272`, use an alternate layout or display a diagnostic screen.

Acceptance tests:

- Visual: no object rectangles intersect on `480x272`.
- Touch: each right-side button triggers only its own callback.
- Dropdown list opens without covering the run buttons in a way that traps the user.

Recommended owner:

- LVGL UI agent.

### Finding 5 - Display Rotation And GT911 Rotation Are Not Coupled

Severity: High  
Boundary: LVGL/display/touch  
Evidence:

- Display rotation is auto-picked by trying `gfx->setRotation(0..3)`.
- Touch rotation is hardcoded to `ROTATION_INVERTED`.
- GT911 rotation enum values are not the same semantic system as Arduino_GFX rotation values.
- Project notes already identify touch coordinate mismatch and portrait/landscape mismatch as previous findings.

Issue:

The display and touch coordinate systems can be out of phase. Touch events may land on a different UI object from the visible point.

Impact:

- Dropdown selection can appear random.
- Arc drag can jump.
- Buttons can trigger adjacent controls.

Fix/path forward:

- Stop auto-picking rotation in production.
- Define a board-specific display/touch mapping table:

```cpp
struct RotationMap {
  uint8_t gfxRotation;
  uint8_t gt911Rotation;
  uint16_t width;
  uint16_t height;
};
```

- Select one known-good mapping for JC4827W543.
- At boot print:
  - selected display rotation
  - `gfx->width()`
  - `gfx->height()`
  - GT911 rotation
- Add a temporary touch diagnostic screen:
  - draw crosshairs at four corners and center
  - show raw and transformed touch coordinates
  - pass only when top-left, top-right, bottom-left, bottom-right all map correctly.

Acceptance tests:

- Touch top-left visual target returns near `(0,0)`.
- Touch top-right returns near `(479,0)`.
- Touch bottom-left returns near `(0,271)`.
- Touch bottom-right returns near `(479,271)`.
- Pattern dropdown item taps select the item under the finger.

Recommended owner:

- LVGL/display agent.

### Finding 6 - RPM Arc Floods Queue And NVS

Severity: Medium  
Boundary: UI/backend/persistence  
Evidence:

- `on_arc_changed()` sends an RPM command on every LVGL value change.
- Manager commits every accepted RPM to NVS.

Issue:

Dragging the arc can generate a burst of queue messages and flash writes.

Impact:

- Queue can fill.
- UI shows `Control queue full`.
- NVS writes can block runtime work and wear flash.
- Flash writes can interact badly with ISR cache safety.

Fix/path forward:

- UI should update RPM label locally during drag.
- Send RPM command on:
  - `LV_EVENT_RELEASED`, or
  - a 50 ms coalesced timer while dragging.
- Manager should persist RPM after debounce:
  - commit after 500-1000 ms of no further RPM changes
  - or commit only on Stop/Save.
- Keep generator `setRpm()` fast path immediate for live behavior, but separate it from persistence.

Acceptance tests:

- Drag across full RPM range and inspect queue drop count.
- No `Control queue full` while dragging normally.
- NVS write count should be one per settled edit, not one per arc step.

Recommended owner:

- UI/backend contract agent plus persistence agent.

### Finding 7 - LVGL Error Label Can Overdraw Controls

Severity: Low/Medium  
Boundary: LVGL layout  
Evidence:

- `lbl_error` is aligned bottom-left with no fixed width or long mode.
- Backend error strings can be much longer than the available bottom strip.

Issue:

Long error text can draw underneath controls or outside the intended status area.

Impact:

UI appears visually corrupted when errors are shown.

Fix/path forward:

- Set error label width to fit left pane only.
- Use `LV_LABEL_LONG_DOT` or a small modal/toast.
- Keep status messages short:
  - `GEN: GPIO22 invalid`
  - `GEN: no active table`
  - `Queue full`

Acceptance tests:

- Force every known error string and confirm none overlaps run/invert buttons.

Recommended owner:

- LVGL UI agent.

### Finding 8 - Modal Spin Rows Have Internal Overlap

Severity: Medium  
Boundary: LVGL layout  
Evidence:

- `make_spin_row()` positions:
  - spinbox right with offset `-40`
  - minus button right with offset `-84`
  - plus button right with offset `0`
- The spinbox and minus button rectangles overlap or touch depending on LVGL style padding.

Issue:

The modal controls do not have a real layout model.

Impact:

Sweep, compression, and custom dialogs can have overlapping touch targets and labels.

Fix/path forward:

- Use a flex row:
  - label: fixed or flex-grow
  - minus button: `32x30`
  - spinbox: `76x30`
  - plus button: `32x30`
  - gaps: `6 px`
- Use `lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW)`.
- Set row padding to zero and column gap to `6`.

Acceptance tests:

- Every modal opens on `480x272`.
- No spinbox row controls overlap.
- Plus/minus buttons increment only their associated spinbox.

Recommended owner:

- LVGL UI agent.

### Finding 9 - LittleFS Partition Label/Subtype Mismatch

Severity: Medium  
Boundary: Build/storage  
Evidence:

- `partitions_signalgen.csv` names the partition `littlefs` but uses subtype `spiffs`.
- Build warning: partition has name `littlefs`, which is a partition subtype, but non-matching type/subtype.
- Arduino LittleFS default partition label is `"spiffs"`.

Issue:

The code calls `LittleFS.begin(true)` without passing an explicit partition label. That may mount the default `"spiffs"` label, not the partition named `"littlefs"`.

Impact:

- DSL save/load can fail.
- Smoke test may fail.
- The UI may show `load failed` or `Save failed`.

Fix/path forward:

Choose one consistent scheme:

Option A:

- Rename partition to `spiffs`.
- Keep `LittleFS.begin(true)` default.

Option B:

- Keep partition label `littlefs`.
- Change mount call to:

```cpp
LittleFS.begin(true, "/littlefs", 10, "littlefs")
```

Option B is clearer for this project.

Acceptance tests:

- Build has no partition warning.
- Boot prints successful LittleFS mount.
- Save DSL, reboot, load DSL.

Recommended owner:

- Storage/build agent.

### Finding 10 - Active Pattern Table Is Read From ISR But May Live In Flash/PSRAM

Severity: High for timing robustness  
Boundary: Backend ISR/memory  
Evidence:

- `TableCkpGenerator::onAlarm()` reads `self->_table[e]`.
- Builtin pattern tables live in flash `.rodata`.
- DSL/captured tables may live in PSRAM.
- Runtime NVS/LittleFS writes can occur after user interactions.

Issue:

IRAM ISR code can still read data from memory regions that depend on cache availability. Flash writes can disable cache; PSRAM is also cache-dependent.

Impact:

- Signal generation can glitch or crash during NVS/LittleFS commits.
- RPM dragging is especially risky because it currently commits to NVS repeatedly.

Fix/path forward:

- In `TableCkpGenerator::apply()`, copy the active pattern bytes into an internal DRAM playback buffer.
- ISR reads only the internal buffer.
- Enforce max slot count based on available internal RAM.
- Free/replace the buffer outside ISR after stopping timer or under safe critical section.
- If the team chooses not to copy, then all flash/NVS/LittleFS writes must be blocked while running. That is less flexible and not recommended.

Acceptance tests:

- Run generator while repeatedly changing RPM and pattern.
- Scope must show no malformed pulses.
- NVS/LittleFS save while running must not crash or perturb ISR timing beyond accepted tolerance.

Recommended owner:

- Backend timing agent.

### Finding 11 - Build Environment Does Not Match Stated N4R8 Target

Severity: Medium/High  
Boundary: Build/configuration  
Evidence:

- Project documentation says `ESP32-S3-N4R8`: 4 MB flash, 8 MB OPI PSRAM.
- PlatformIO build output reported `ESP32-S3-DevKitC-1-N8 (8 MB QD, No PSRAM)`.
- `platformio.ini` sets PSRAM flags manually, but board metadata still disagrees.

Issue:

The build may not be accurately describing or configuring the actual module. PSRAM-dependent features such as LVGL buffers, DSL compiled tables, and waveform canvas need deterministic PSRAM availability.

Impact:

- Firmware may work on one board but fail on another.
- Memory placement assumptions are unreliable.
- Flash-size/partition assumptions may diverge from actual hardware.

Fix/path forward:

- Create a custom PlatformIO board manifest for the exact JC4827W543 module or exact ESP32-S3-N4R8 module.
- Confirm:
  - flash size
  - PSRAM type/mode
  - memory mode
  - upload flash size
  - partition table size
- Add boot diagnostics:
  - `ESP.getFlashChipSize()`
  - `psramFound()`
  - `ESP.getPsramSize()`
  - free internal heap
  - free PSRAM

Acceptance tests:

- Boot log must show PSRAM found and size near 8 MB.
- PlatformIO build metadata should no longer claim `No PSRAM`.
- Memory-heavy UI pages must open after boot without allocation failure.

Recommended owner:

- Build/config agent.

### Finding 12 - LVGL Version Drift

Severity: Medium  
Boundary: Build/UI dependency  
Evidence:

- `platformio.ini` requests `lvgl/lvgl@^9.2.2`.
- Build resolved `lvgl 9.5.0`.
- Local `lv_conf.h` header says configuration file for `v9.2.2`.

Issue:

The caret range allows minor-version upgrades. LVGL minor versions can alter widget behavior, defaults, or memory use.

Impact:

- The UI can render differently from the tested baseline.
- Hard-coded sizes become more fragile.

Fix/path forward:

- Pin LVGL exactly for stabilization:
  - `lvgl/lvgl@9.2.2`
- Pin GFX library if the JC4827W543 reference was validated on `1.5.6`, or explicitly retest on `1.6.4`.
- After layout fixes, upgrades can be tested intentionally.

Acceptance tests:

- Dependency graph shows the intended versions.
- UI is validated on the exact dependency set shipped to hardware.

Recommended owner:

- Build/config agent plus LVGL UI agent.

## 5. Proposed Agent Work Packages

### Agent A - Hardware Pin Map And Generator Init

Files:

- `src/main.cpp`
- `lib/ckp_gen/TableCkpGenerator.cpp`
- optionally `lib/ckp_gen/TableCkpGenerator.h`

Tasks:

- Replace invalid/conflicting pin map.
- Add board-reserved pin validation.
- Add exact failure logging for generator init.
- Temporarily support crank-only bring-up if final cam pins are not decided.

Exit criteria:

- Generator initializes.
- Pattern apply succeeds for crank-only pattern.
- Start produces a valid crank output.

### Agent B - Backend Acknowledgement And Error Model

Files:

- `lib/ckp_gen/CkpGenerator.h`
- `lib/ckp_gen/TableCkpGenerator.h`
- `lib/ckp_gen/TableCkpGenerator.cpp`
- `src/main.cpp`
- `lib/ui_lvgl/ui_lvgl.cpp` only if UI string handling must change

Tasks:

- Add start/apply status reporting.
- Stop optimistic `gRunning=true` on failed start.
- Replace generic `Pattern apply failed` where possible.

Exit criteria:

- UI reflects backend truth after start/stop.
- Error strings distinguish GPIO/init/table/RPM failures.

### Agent C - LVGL Main Screen Layout

Files:

- `lib/ui_lvgl/ui_lvgl.cpp`

Tasks:

- Rebuild main screen on fixed `480x272` layout grid.
- Eliminate right-column overlap.
- Add width/long-mode constraints for labels.
- Improve touch target sizes.

Exit criteria:

- No main-screen rectangles overlap.
- Dropdown, filter, action buttons, run/invert, error strip all fit.

### Agent D - Display/Touch Rotation Calibration

Files:

- `lib/ui_lvgl/ui_lvgl.cpp`
- optional temporary diagnostic screen or serial-only diagnostic helper

Tasks:

- Replace auto rotation with explicit board mapping.
- Couple Arduino_GFX rotation to GT911 rotation.
- Add coordinate logging or calibration screen.

Exit criteria:

- Five-point touch test passes: four corners plus center.
- Pattern dropdown selection matches the touched row.

### Agent E - Persistence And Event Rate Limiting

Files:

- `lib/ui_lvgl/ui_lvgl.cpp`
- `src/main.cpp`
- `lib/sweep_compression/NvsStore.cpp`
- `lib/sweep_compression/LittleFSInit.cpp`
- `partitions_signalgen.csv`

Tasks:

- Coalesce RPM commands from the arc.
- Debounce NVS commits.
- Fix LittleFS partition label/mount scheme.

Exit criteria:

- No queue overflow during normal RPM drag.
- NVS writes are bounded.
- LittleFS warning disappears and DSL save/load works after reboot.

### Agent F - ISR Memory Safety

Files:

- `lib/ckp_gen/TableCkpGenerator.h`
- `lib/ckp_gen/TableCkpGenerator.cpp`

Tasks:

- Copy active pattern table to internal RAM playback buffer.
- Ensure ISR never reads flash/PSRAM.
- Preserve table swap safety.

Exit criteria:

- Scope signal remains stable during NVS/LittleFS operations.
- No ISR cache-access crashes during runtime configuration changes.

## 6. LVGL Implementation Instructions For 480x272

Use this as the concrete layout target:

```text
Screen: 480x272

Header:
  title/status LEDs: x=8..472, y=4..28

Left pane:
  x=8, y=32, w=224, h=224
  RPM arc: 190x190 or 196x196 centered at x=120,y=144
  RPM value/caption centered inside arc

Right pane:
  x=240, y=12, w=232, h=252

Pattern:
  label:    x=248, y=14,  w=212, h=14
  dropdown: x=248, y=30,  w=212, h=34
  filter:   x=248, y=74,  w=212, h=28

Actions:
  SWEEP: x=248, y=112, w=100, h=34
  COMP:  x=360, y=112, w=100, h=34
  DSL:   x=248, y=154, w=100, h=34
  WAVE:  x=360, y=154, w=100, h=34

Run controls:
  INVERT: x=248, y=210, w=100, h=40
  RUN:    x=360, y=210, w=100, h=40

Error/status:
  x=8, y=252, w=232, h=18
```

Implementation guidance:

- Do not use a separate absolute-position top-right action column.
- Create a right pane container and place controls relative to it.
- Use `lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE)` for non-scrolling fixed panes.
- Set every label width if it can receive dynamic text.
- Use short button text on this screen. Longer explanations belong in Serial logs or modals.

## 7. Backend/UI Interface Contract To Enforce

Commands from UI to manager:

| Command | Required behavior |
| --- | --- |
| `MSG_SET_RPM` | Fast `setRpm()` now, debounced persistence later |
| `MSG_SELECT_BUILTIN` | Apply pattern; return/emit exact success or failure |
| `MSG_START` | Start only if initialized and table active; UI updates only after backend success |
| `MSG_STOP` | Stop and drive outputs low; UI updates after backend truth |
| `MSG_SET_INVERT` | Store full invert mask and update channel LEDs |

State ownership:

- Backend manager owns truth for:
  - active pattern
  - RPM
  - running state
  - invert mask
  - generator readiness
- LVGL may show pending UI values while editing, but must reconcile with backend status.

Error behavior:

- `ui_show_error("")` clears only after a successful backend command.
- Do not clear a hardware/init error due to unrelated UI events.
- Every backend rejection should have one concise UI message and one detailed Serial message.

Queue behavior:

- Queue depth is 16. High-rate UI widgets must coalesce.
- UI callbacks must not write backend globals directly.
- Backend tasks must not call LVGL object APIs directly; use existing pending-flag update functions.

## 8. Validation Plan

### Build validation

- `pio run -e esp32-s3-n4r8`
- Dependency graph must show pinned intended versions.
- No partition warning.
- Build metadata must match actual module or boot diagnostics must prove flash/PSRAM.

### Boot validation

Serial output must show:

- flash size
- PSRAM found and size
- LittleFS mounted
- generator initialized
- selected pin map
- display width/height
- touch rotation

### UI validation

On the LCD:

- no overlapping main-screen controls
- dropdown opens and row taps select the correct row
- every button has a unique touch target
- every modal fits within 480x272
- error/status text does not draw over buttons

### Backend validation

Using logic analyzer/scope:

- default pattern outputs crank signal at expected RPM
- `Start` and `Stop` are idempotent
- pattern switch from `sixty_minus_two` to a crank+cam pattern succeeds
- invalid pin configuration fails with a clear error
- RPM drag does not create malformed pulse timing

### Storage validation

- Save a DSL pattern.
- Reboot.
- Load it from UI or serial.
- Apply it.
- Confirm signal table matches before and after reboot.

## 9. Manager Notes

The team should treat the current failures as an integration issue, not purely an LVGL issue. The screen overlap is real and must be fixed, but the pattern/start failures are strongly tied to the backend pin map and missing acknowledgement contract.

Recommended first branch:

1. Pin-map correction and crank-only backend bring-up.
2. Start/apply acknowledgement.
3. Main-screen layout grid.

After those land, touch rotation and storage fixes can be validated without the UI being polluted by unrelated backend errors.

