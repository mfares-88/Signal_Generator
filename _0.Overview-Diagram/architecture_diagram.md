# ESP32-S3 Crankshaft Signal Generator — Architectural Overview

> [!NOTE]
> This document provides a comprehensive architectural map of the entire codebase. Each diagram section focuses on a different dimension of the system: class hierarchy, data flow, initialization, task scheduling, and UI event routing. Together they form a complete conceptual reference.

---

## 1. System-Level Architecture — Dual-Core Task Map

This is the top-level view. The ESP32-S3 has two cores, and this firmware deliberately pins work to each core for deterministic timing.

```mermaid
graph TB
    subgraph ESP32_S3["ESP32-S3 Dual-Core SoC"]
        direction TB

        subgraph Core0["🔵 Core 0 — UI & Display"]
            direction TB
            LOOP["<b>Arduino loop()</b><br/>10ms tick cycle"]
            LVGL_PUMP["ui_task_handler()<br/><i>Pumps LVGL rendering</i>"]
            SERIAL_POLL["serialCliPoll()<br/><i>Drains Serial bytes</i>"]
            CAP_FETCH["capRX.fetch()<br/><i>Edge pulse check</i>"]

            LOOP --> LVGL_PUMP
            LOOP --> SERIAL_POLL
            LOOP --> CAP_FETCH
        end

        subgraph Core1["🟢 Core 1 — Real-Time Engine"]
            direction TB
            MGR["<b>managerTask</b><br/>Priority 3 · 8KB stack<br/><i>Message dispatcher</i>"]
            SWP["<b>sweepCompTask</b><br/>Priority 2 · 4KB stack<br/><i>1ms tick · sweep + compression</i>"]
            ISR["<b>GPTimer ISR</b><br/>IRAM_ATTR<br/><i>≤5 statements per alarm</i><br/>dedic_gpio_bundle_write()"]
        end

        subgraph SharedState["📦 Shared State (volatile / portMUX)"]
            QUEUE["gCtrlQ<br/>FreeRTOS Queue<br/>depth=16 × sizeof(CtrlMsg)"]
            GLOBALS["g_rpm · g_pattern_key<br/>g_invert_mask<br/>g_sweep_* · g_comp_*"]
            PENDING["s_pending_rpm<br/>s_pending_pattern<br/>s_pending_running<br/><i>Pending-flag sync pattern</i>"]
        end
    end

    LOOP -.->|"10ms vTaskDelay"| LOOP
    MGR -->|"xQueueReceive<br/>100ms tick"| QUEUE
    LVGL_PUMP -.->|"reads pending flags"| PENDING
    MGR -.->|"writes pending flags"| PENDING
    SWP -->|"gen→setRpm()<br/><i>fast path only</i>"| ISR

    style Core0 fill:#0d1b3e,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
    style Core1 fill:#1a2a0d,stroke:#66FF66,stroke-width:2px,color:#D7E9FF
    style SharedState fill:#2a1a0d,stroke:#FFB020,stroke-width:2px,color:#D7E9FF
    style ESP32_S3 fill:#0B1020,stroke:#555,color:#D7E9FF
```

---

## 2. Class Hierarchy & Interface Contract

The generator backend is polymorphic — an abstract `IGenerator` interface allows swapping between the legacy timer-based generator and the production byte-table backend via a build flag.

```mermaid
classDiagram
    direction TB

    class IGenerator {
        <<interface · abstract>>
        +begin(pin_crank, pin_cam1, pin_cam2) bool
        +apply(PatternRef ref, uint32_t rpm) bool
        +setRpm(uint32_t rpm) bool*
        +setInverted(uint8_t channel_mask) void
        +getInverted() uint8_t
        +start() bool
        +stop() bool
        +getEdgeCounter() uint16_t*
        +lastError() GenError
        +isReady() bool
        +getCycleStartUs() uint32_t*
        +getCycleDurationUs() uint32_t*
    }
    note for IGenerator "§3.2 contract\n─────────────────────\n• setRpm() = fast path (no buffer rebuild)\n  used by sweep task at 1ms cadence\n• apply() = full rebuild, not ISR-safe\n• getEdgeCounter() = lock-free atomic\n  uint16 read (Xtensa aligned)\n• * = pure virtual methods"

    class TimerCkpGenerator {
        <<DEPRECATED · WROOM only>>
        -_pin: int
        -_timer: hw_timer_t*
        -_slotPeriod_us: volatile uint32_t
        -_slotsPerRev: volatile uint32_t
        -_invertMask: volatile uint8_t
        -_edge_counter: volatile uint16_t
        -_synth_table: uint8_t*
        +applySignalConfig(SignalConfig cfg) bool
        -onTimer() void [ISR]
        -writePin(bool level) void
    }
    note for TimerCkpGenerator "Legacy slot-machine ISR\n• Single crank pin only\n• cam1/cam2 accepted but parked\n• Uses Arduino esp32-hal-timer\n• apply(PatternRef) = best-effort stub"

    class TableCkpGenerator {
        <<PRODUCTION · S3>>
        -_pin_crank: int
        -_pin_cam1: int
        -_pin_cam2: int
        -_bundle_width: uint8_t
        -_bundle_mask: uint8_t
        -_table: const uint8_t*
        -_slot_count: uint16_t
        -_edge_counter: volatile uint16_t
        -_invert_mask: volatile uint8_t
        -_reverse: volatile bool
        -_cycle_start_us: volatile uint32_t
        -_cycle_duration_us: volatile uint32_t
        -_timer: gptimer_handle_t
        -_bundle: dedic_gpio_bundle_handle_t
        -_playback_buffer: uint8_t* [24KB]
        +setReverse(bool) void
        -onAlarm() bool [ISR · IRAM]
        -reprogramAlarm(uint32_t rpm) bool
    }
    note for TableCkpGenerator "Native byte-table backend\n─────────────────────────\n• ESP-IDF GPTimer (1MHz tick)\n• dedic_gpio bundle write (atomic\n  3-channel GPIO in single call)\n• ISR ≤ 5 statements rule:\n  1) byte = table[edge] XOR mask\n  2) bundle_write(byte)\n  3) advance + wrap\n  4) commit counter\n  5) return false\n• Timer formula:\n  period_μs = 60,000,000 / (rpm × slots)"

    IGenerator <|-- TimerCkpGenerator : implements
    IGenerator <|-- TableCkpGenerator : implements

    class PatternRef {
        <<struct · POD>>
        +table: const uint8_t*
        +slot_count: uint16_t
        +degrees: uint16_t
        +rpm_scaler: float
        +channel_mask: uint8_t
        +name_key: const char*
    }
    note for PatternRef "Byte encoding per slot:\n  bit0 = crank\n  bit1 = cam1\n  bit2 = cam2\n  bit3 = knock (reserved)\nLifetime:\n  Builtin → .rodata (flash)\n  User/DSL → PSRAM heap"

    class SignalConfig {
        <<struct · legacy POD>>
        +rpm: uint32_t
        +nTeeth: uint16_t
        +pMiss: uint8_t
        +nMiss: uint8_t
        +gapPos: GapPosition
        +gapLvl: bool
    }

    class GenError {
        <<enum : uint8_t>>
        OK
        NOT_INITIALIZED
        NO_TABLE
        BAD_SLOT_COUNT
        BAD_RPM
        TIMER_FAIL
        GPIO_FAIL
        BUFFER_OVERFLOW
    }

    TableCkpGenerator ..> PatternRef : "plays back table[]"
    IGenerator ..> GenError : "returns"
    IGenerator ..> PatternRef : "apply(ref, rpm)"
    TimerCkpGenerator ..> SignalConfig : "applySignalConfig()"
```

---

## 3. Boot & Initialization Sequence

The `setup()` function follows a strict initialization order. Each subsystem must succeed before the next depends on it.

```mermaid
flowchart TD
    BOOT["⚡ Power On / Reset"] --> SERIAL["Serial.begin(921600)<br/><i>Debug output</i>"]
    SERIAL --> DIAG["Boot diagnostics<br/><i>Flash size, PSRAM, heap</i>"]
    DIAG --> FS["initLittleFS()<br/><i>Mount LittleFS partition</i><br/><i>(no-op on WROOM)</i>"]
    FS --> FS_SMOKE["littleFsSmokeTest()<br/><i>Write + readback /test.txt</i>"]

    FS_SMOKE --> NVS_BEGIN["NvsStore::begin()<br/><i>Open 'siggen' namespace</i><br/><i>Schema v1 check + migration</i>"]
    NVS_BEGIN --> NVS_LOAD["NvsStore::loadAllToGlobals()<br/><i>Populate g_rpm, g_pattern_key,</i><br/><i>g_invert_mask, g_sweep_*, g_comp_*</i><br/><i>Missing keys → factory defaults</i>"]

    NVS_LOAD --> GEN_INIT["gGen.begin(PIN17, PIN9, PIN14)<br/><i>Constructor: TableCkpGenerator()</i><br/><i>• Validate all 3 GPIO pins</i><br/><i>• Create GPTimer @ 1MHz</i><br/><i>• Build dedic_gpio bundle</i>"]

    GEN_INIT --> SWP_INIT["sweepCompressionInit(&gGen)<br/><i>• Create mutex</i><br/><i>• Spawn sweepCompTask</i><br/><i>  Core 1, Priority 2, 4KB</i>"]

    SWP_INIT --> CAP_INIT["capRX.begin(PIN18)<br/><i>EdgePulseCapture ISR setup</i>"]

    CAP_INIT --> UI_INIT["ui_init(callbacks...)<br/><i>• lv_init() + display driver</i><br/><i>• Touch controller (GT911)</i><br/><i>• Build LVGL widget tree</i><br/><i>• 480×272 landscape</i>"]

    UI_INIT --> QUEUE_CREATE["xQueueCreate(16, sizeof(CtrlMsg))<br/><i>gCtrlQ — central command queue</i>"]

    QUEUE_CREATE --> TASK_CREATE["xTaskCreatePinnedToCore(<br/>  managerTask, 'managerTask',<br/>  8192, nullptr, <b>priority=3</b>,<br/>  nullptr, <b>core=1</b>)"]

    TASK_CREATE --> RESTORE_PAT["Restore Pattern from NVS<br/><i>findByKey(g_pattern_key)</i><br/><i>→ fallback builtinByIndex(0)</i><br/><i>→ gGen.apply(ref, rpm)</i>"]

    RESTORE_PAT --> RESTORE_SWEEP["Restore Sweep + Compression<br/><i>sweepSet(saved_config)</i><br/><i>compressionSet(saved_config)</i>"]

    RESTORE_SWEEP --> RESTORE_INV["Restore Invert Mask<br/><i>gGen.setInverted(g_invert_mask)</i>"]

    RESTORE_INV --> START["gGen.start()<br/><i>Enable GPTimer alarm</i><br/><i>ISR begins firing</i>"]

    START --> CLI_INIT["serialCliBegin()<br/><i>Serial CLI ready</i>"]

    CLI_INIT --> UI_SYNC["UI sync: update_rpm, pattern,<br/>running, inverted, channels"]

    UI_SYNC --> LOOP_START["→ loop() begins<br/><i>10ms cadence forever</i>"]

    style BOOT fill:#1a3a1a,stroke:#66FF66,stroke-width:2px,color:#D7E9FF
    style LOOP_START fill:#0d2b4e,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
    style GEN_INIT fill:#2a1a0d,stroke:#FFB020,stroke-width:2px,color:#D7E9FF
    style TASK_CREATE fill:#2a1a0d,stroke:#FFB020,stroke-width:2px,color:#D7E9FF
```

---

## 4. Message Queue Control Flow — The `CtrlMsg` Dispatch

All user inputs (touch screen, serial port) are funneled through a single FreeRTOS queue. The manager task on Core 1 is the **sole consumer**. This is the central nervous system of the firmware.

```mermaid
flowchart LR
    subgraph Producers["📤 Message Producers"]
        direction TB
        UI_CB["LVGL UI Callbacks<br/><i>on_ui_rpm()</i><br/><i>on_ui_pattern()</i><br/><i>on_ui_run()</i><br/><i>on_ui_custom()</i><br/><i>on_ui_invert()</i>"]
        SERIAL["Serial CLI<br/><i>Legacy binary opcodes</i><br/><i>Text commands (LIST,</i><br/><i>SELECT, COMPILE, etc.)</i>"]
    end

    QUEUE["gCtrlQ<br/>FreeRTOS Queue<br/>16 slots"]

    subgraph Consumer["📥 managerTask (Core 1, P3)"]
        direction TB
        DISPATCH{{"switch(m.type)"}}

        MSG_RPM["MSG_SET_RPM<br/><i>clamp 100..6000</i><br/><i>gen.setRpm() ←fast path</i><br/><i>NvsStore::setRpmDebounced()</i>"]
        MSG_PAT["MSG_SET_PATTERN<br/><i>Legacy 5-preset path</i><br/><i>(WROOM only)</i>"]
        MSG_BUILTIN["MSG_SELECT_BUILTIN<br/><i>PatternLibrary::builtinByIndex()</i><br/><i>→ applyPatternRef()</i><br/><i>→ gen.apply(ref, rpm)</i>"]
        MSG_NAMED["MSG_SELECT_NAMED<br/><i>PatternLibrary::findByKey()</i><br/><i>→ applyPatternRef()</i>"]
        MSG_CUSTOM["MSG_SET_CUSTOM<br/><i>dslCompileSignalConfig(cfg)</i><br/><i>→ DSL pipeline → PatternRef</i><br/><i>→ gen.apply()</i>"]
        MSG_DSL["MSG_LOAD_DSL<br/><i>dslCompile(source)</i><br/><i>→ register as 'scratch_dsl'</i><br/><i>→ gen.apply()</i>"]
        MSG_START["MSG_START / MSG_STOP<br/><i>gen.start() / gen.stop()</i>"]
        MSG_INV["MSG_SET_INVERT<br/><i>gen.setInverted(mask)</i><br/><i>NvsStore::setInvertMask()</i>"]
        MSG_SWEEP["MSG_SET_SWEEP<br/><i>sweepSet(config)</i><br/><i>NvsStore::setSweep()</i>"]
        MSG_COMP["MSG_SET_COMPRESSION<br/><i>compressionSet(config)</i><br/><i>NvsStore::setCompression()</i>"]
        MSG_CAP["MSG_CAPTURE_START/STOP<br/><i>captureStart(pin, revs)</i><br/><i>captureFetchPattern()</i>"]
        MSG_SAVE["MSG_SAVE_USER<br/><i>PatternStorage::saveDsl()</i>"]
        MSG_TABLE["MSG_LOAD_TABLE<br/><i>Build PatternRef from raw bytes</i><br/><i>→ gen.apply()</i>"]
    end

    UI_CB -->|"sendCtrlMsg()"| QUEUE
    SERIAL -->|"sendCtrlMsg()"| QUEUE
    QUEUE --> DISPATCH

    DISPATCH --> MSG_RPM
    DISPATCH --> MSG_PAT
    DISPATCH --> MSG_BUILTIN
    DISPATCH --> MSG_NAMED
    DISPATCH --> MSG_CUSTOM
    DISPATCH --> MSG_DSL
    DISPATCH --> MSG_START
    DISPATCH --> MSG_INV
    DISPATCH --> MSG_SWEEP
    DISPATCH --> MSG_COMP
    DISPATCH --> MSG_CAP
    DISPATCH --> MSG_SAVE
    DISPATCH --> MSG_TABLE

    style Producers fill:#0d1b3e,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
    style Consumer fill:#1a2a0d,stroke:#66FF66,stroke-width:2px,color:#D7E9FF
```

---

## 5. LVGL UI Architecture — Pages, Events & Cross-Core Sync

The UI runs on Core 0. It **never** calls the generator directly — all commands go through `gCtrlQ`. State updates from the backend use a "pending-flag" pattern protected by `portMUX`.

```mermaid
flowchart TB
    subgraph UI_PAGES["LVGL UI Surface (480×272 LCD + GT911 Touch)"]
        direction TB

        subgraph HOME["🏠 HOME Tab"]
            ARC["RPM Arc<br/><i>Range 100–6000</i><br/><i>on_arc_changed → MSG_SET_RPM</i>"]
            DD["Pattern Dropdown<br/><i>64+ builtin patterns</i><br/><i>with fuzzy search filter</i><br/><i>on_pattern_changed → MSG_SELECT_BUILTIN</i>"]
            BTN_RUN["RUN/STOP Button<br/><i>on_run_clicked → MSG_START/STOP</i>"]
            BTN_INV["INVERT Button<br/><i>on_invert_clicked → MSG_SET_INVERT</i>"]
            LEDS["Channel LEDs<br/><i>Crank · CAM1 · CAM2</i><br/><i>Green=active, Grey=unused</i>"]
            ERR["Error Label<br/><i>Shows gen errors / DSL errors</i>"]
        end

        subgraph ADV["⚙️ ADVANCED Tab → Full-Screen Overlays"]
            direction LR
            SWEEP_PAGE["Sweep Page<br/><i>Low/High RPM spinboxes</i><br/><i>Mode: OFF/Linear/Log/Waypoint</i><br/><i>Interval spinner</i><br/><i>Live RPM arc + status pill</i><br/><i>→ MSG_SET_SWEEP</i>"]
            COMP_PAGE["Compression Page<br/><i>Enable switch</i><br/><i>Cylinders / Threshold / Peak</i><br/><i>Dynamic toggle</i><br/><i>Live RPM arc + status pill</i><br/><i>→ MSG_SET_COMPRESSION</i>"]
            DSL_PAGE["DSL Editor Page<br/><i>Source textarea + keyboard</i><br/><i>Compile / Save As / Load</i><br/><i>Help sub-page overlay</i><br/><i>Error polling timer (200ms)</i><br/><i>→ MSG_LOAD_DSL / MSG_SAVE_USER</i>"]
            WAVE_PAGE["Waveform Canvas<br/><i>3-lane oscilloscope view</i><br/><i>Crank + CAM1 + CAM2</i><br/><i>Pinch-zoom Q24.8 model</i><br/><i>Pan, pause, lane toggle</i><br/><i>→ reads getEdgeCounter()</i>"]
            CUSTOM_PAGE["Custom Pattern Page<br/><i>Teeth/Periods/Missing spinboxes</i><br/><i>Gap position dropdown</i><br/><i>Gap level switch</i><br/><i>→ MSG_SET_CUSTOM</i>"]
        end
    end

    subgraph SYNC["Cross-Core Pending-Flag Sync"]
        direction LR
        BACKEND["managerTask<br/>(Core 1)"]
        FLAGS["volatile bool<br/>s_pending_rpm<br/>s_pending_pattern<br/>s_pending_running<br/>s_pending_error<br/>s_pending_channels"]
        APPLY["apply_pending_updates()<br/><i>Called every ui_task_handler()</i><br/><i>portENTER_CRITICAL / portEXIT_CRITICAL</i>"]

        BACKEND -->|"ui_update_rpm(val)<br/>sets flag + val"| FLAGS
        FLAGS -->|"checked each tick"| APPLY
        APPLY -->|"lv_arc_set_value()<br/>lv_label_set_text()<br/>..."| HOME
    end

    style UI_PAGES fill:#0B1020,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
    style HOME fill:#141C2E,stroke:#00E5FF,stroke-width:1px,color:#D7E9FF
    style ADV fill:#141C2E,stroke:#00E5FF,stroke-width:1px,color:#D7E9FF
    style SYNC fill:#2a1a0d,stroke:#FFB020,stroke-width:2px,color:#D7E9FF
```

---

## 6. DSL Compiler Pipeline

The Domain-Specific Language lets users define arbitrary crank/cam patterns at runtime. The pipeline is a classic compiler chain.

```mermaid
flowchart LR
    SRC["DSL Source String<br/><i>'1,C,M,1/2,60,58t,2m'</i>"]

    subgraph PIPELINE["DSL Compiler Pipeline (lib/dsl/)"]
        direction LR
        LEX["<b>Lexer</b><br/><i>Tokenizer</i><br/>─────────<br/>TOK_INT · TOK_FRACTION<br/>TOK_INT_SUFFIXED<br/>TOK_LETTER · TOK_COMMA<br/>TOK_COLON · TOK_EOF"]

        PARSE["<b>Parser</b><br/><i>Recursive Descent</i><br/>───────────────<br/>ProgramAst {<br/>  wheels: WheelDef[]<br/>}<br/>WheelDef {<br/>  pin, rotation, kind,<br/>  duty, total_teeth,<br/>  runs[], angular[]<br/>}"]

        VAL["<b>Validator</b><br/><i>12 Semantic Rules (§7.5)</i><br/>─────────────────<br/>#1 pin ∈ {1..4}<br/>#2 pins unique<br/>#3 duty 0<n<d, d≤32<br/>#4 teeth > 0<br/>#5 sum(t+m)=teeth<br/>#9 table ≤ 4096 slots<br/>#11 source ≤ 512 chars"]

        COMP["<b>Compiler</b><br/><i>AST → Byte Table</i><br/>─────────────────<br/>• Compute LCM across wheels<br/>• Pack bit0=crank, bit1=cam1<br/>  bit2=cam2 per slot<br/>• Allocate in PSRAM via<br/>  heap_caps_malloc(SPIRAM)"]

        LEX -->|"Token stream"| PARSE
        PARSE -->|"ProgramAst*"| VAL
        VAL -->|"validated AST"| COMP
    end

    SRC --> LEX

    RESULT["DslResult {<br/>  ok: bool<br/>  pattern: PatternRef<br/>  error[96]<br/>  error_offset<br/>}"]

    COMP --> RESULT

    SIGCFG["SignalConfig<br/><i>Legacy custom modal</i>"]
    SHIM["dslCompileSignalConfig()<br/><i>Synthesizes DSL source</i><br/><i>from teeth/miss params</i><br/><i>then feeds dslCompile()</i>"]

    SIGCFG --> SHIM
    SHIM --> SRC

    style PIPELINE fill:#141C2E,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
```

---

## 7. Pattern Library & Storage Tiers

Patterns come from three sources, managed by the `PatternLibrary` namespace and persisted by `PatternStorage` / `NvsStore`.

```mermaid
flowchart TB
    subgraph TIERS["PatternLibrary — 3-Tier Registry"]
        direction TB

        BUILTIN["🔷 <b>Builtin Tier</b><br/><i>Static const PatternRef[]</i><br/><i>in .rodata (flash)</i><br/>──────────────────<br/>Generated by:<br/>tools/convert_ardustim_wheels.py<br/>→ builtin_tables_generated.h<br/>→ pattern_names_generated.h<br/>→ pattern_legacy_index_generated.h<br/>──────────────────<br/>64+ patterns: 60-2, 36-1,<br/>GM LS1, Honda K20, etc.<br/>Immutable at runtime"]

        USER["🟡 <b>User Tier</b><br/><i>Up to 16 runtime entries</i><br/><i>kUserCapacity = 16</i><br/>──────────────────<br/>Sources:<br/>• DSL editor 'Save As'<br/>• Captured waveforms<br/>• Serial CLI 'SAVE' command<br/>──────────────────<br/>PatternRef stored by value<br/>.table → PSRAM heap<br/>.name_key → caller-owned"]

        SCRATCH["🟠 <b>Scratch</b><br/><i>s_scratch_dsl</i><br/>──────────────────<br/>Transient unsaved DSL<br/>compilation result<br/>Manager-owned lifetime<br/>Freed on next compile or<br/>when switching to builtin"]
    end

    subgraph PERSIST["Persistence Backends"]
        direction TB
        NVS["<b>NvsStore</b><br/><i>ESP32 NVS (key-value flash)</i><br/>──────────────────<br/>Namespace: 'siggen'<br/>Schema v1 (versioned)<br/>──────────────────<br/>Keys stored:<br/>• pattern_key (string, 64B)<br/>• rpm (uint32)<br/>• invert_mask (uint8)<br/>• sweep_* (4 keys)<br/>• comp_* (5 keys)<br/>──────────────────<br/>Debounced RPM writes<br/>(750ms window)"]

        LITTLEFS["<b>PatternStorage</b><br/><i>LittleFS partition</i><br/><i>(S3 only, gated on flag)</i><br/>──────────────────<br/>/patterns/‹key›.dsl<br/>  → UTF-8 DSL source<br/>/patterns/‹key›.bin<br/>  → SHA-256 + metadata<br/>    + compiled byte table<br/>──────────────────<br/>Cache invalidation:<br/>SHA-256 mismatch → recompile"]
    end

    BUILTIN -->|"findByKey(key)"| LOOKUP["PatternLibrary::findByKey()<br/><i>Searches: builtin → user → scratch</i>"]
    USER -->|"findByKey(key)"| LOOKUP
    SCRATCH -.->|"(future)"| LOOKUP

    BUILTIN -->|"builtinByIndex(i)"| INDEX["PatternLibrary::builtinByIndex()<br/><i>Direct array index</i>"]

    USER -->|"registerUserPattern()"| LITTLEFS
    LITTLEFS -->|"loadDsl() → dslCompile()"| USER

    LOOKUP -->|"Resolved PatternRef"| APPLY_FN["gGen.apply(ref, rpm)<br/><i>Active playback begins</i>"]

    NVS -->|"g_pattern_key on boot"| LOOKUP

    style TIERS fill:#141C2E,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
    style PERSIST fill:#1a2a0d,stroke:#66FF66,stroke-width:2px,color:#D7E9FF
```

---

## 8. Sweep & Compression Simulation Task

This FreeRTOS task modulates the generator RPM in real-time, simulating engine sweep profiles and compression effects using sin lookup tables.

```mermaid
flowchart TB
    subgraph TASK["sweepCompTask (Core 1, P2, 1ms tick)"]
        direction TB
        TICK["vTaskDelayUntil(1ms)"]
        READ_CFG["cfgLock() → copy SweepConfig + CompressionConfig → cfgUnlock()"]

        IDLE_CHECK{"sweep.mode == OFF<br/>AND !comp.enabled?"}

        subgraph SWEEP["Sweep Phase"]
            direction TB
            SW_DISPATCH{{"sweep.mode?"}}
            LINEAR["sweepStepLinear()<br/><i>±1 RPM per interval_us</i><br/><i>Ping-pong between</i><br/><i>low_rpm ↔ high_rpm</i>"]
            LOG["sweepStepLog()<br/><i>Exponential ramp</i><br/><i>rpm = low × (high/low)^(t/period)</i><br/><i>Uses powf() (HW FP)</i>"]
            WAYPOINT["sweepStepWaypoint()<br/><i>Packed (rpm, dwell_ms) pairs</i><br/><i>Linear interp between segments</i>"]
        end

        subgraph COMPRESSION["Compression Phase"]
            direction TB
            ANGLE["currentCrankAngle(offset_deg)<br/><i>Uses getCycleStartUs() +</i><br/><i>getCycleDurationUs() from ISR</i>"]
            LUT_SELECT{{"comp.cyl?"}}
            SIN180["sin_100_180[180]<br/><i>1-cyl, 2-cyl</i>"]
            SIN120["sin_100_120[120]<br/><i>3-cyl</i>"]
            SIN90["sin_100_90[90]<br/><i>4-cyl, 6+cyl</i>"]
            CUSTOM_CURVE["custom_curve_256[256]<br/><i>User-supplied override</i>"]
            SCALE["Scale by peak (0..100)<br/>Dynamic: × (rpm/thresh)<br/><i>if rpm < 655 (AVR guard)</i>"]
        end

        FINAL["target = base_rpm − modifier<br/>gen→setRpm(target)<br/><i>Fast path, no buffer rebuild</i>"]
    end

    TICK --> READ_CFG --> IDLE_CHECK
    IDLE_CHECK -->|"Yes"| TICK
    IDLE_CHECK -->|"No"| SW_DISPATCH
    SW_DISPATCH -->|"LINEAR"| LINEAR
    SW_DISPATCH -->|"LOG"| LOG
    SW_DISPATCH -->|"WAYPOINT"| WAYPOINT
    SW_DISPATCH -->|"OFF"| ANGLE

    LINEAR --> ANGLE
    LOG --> ANGLE
    WAYPOINT --> ANGLE

    ANGLE --> LUT_SELECT
    LUT_SELECT -->|"1,2"| SIN180
    LUT_SELECT -->|"3"| SIN120
    LUT_SELECT -->|"4+"| SIN90
    LUT_SELECT -->|"custom"| CUSTOM_CURVE

    SIN180 --> SCALE
    SIN120 --> SCALE
    SIN90 --> SCALE
    CUSTOM_CURVE --> SCALE

    SCALE --> FINAL
    FINAL --> TICK

    style TASK fill:#141C2E,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
    style SWEEP fill:#0d1b3e,stroke:#66FF66,stroke-width:1px,color:#D7E9FF
    style COMPRESSION fill:#2a1a0d,stroke:#FFB020,stroke-width:1px,color:#D7E9FF
```

---

## 9. Capture & Loopback Subsystem

The system can capture external CKP signals and validate its own output via a loopback path.

```mermaid
flowchart LR
    subgraph CAPTURE["Capture Subsystem (lib/ckp_capture/)"]
        direction TB

        EDGE["<b>EdgePulseCapture</b><br/><i>Low-level ISR-based</i><br/><i>edge timing capture</i><br/>───────────────<br/>• Pin interrupt handler<br/>• Measures period_us,<br/>  high_us, timestamp_us<br/>• FreeRTOS queue → fetch()"]

        RECORDER["<b>CaptureRecorder</b><br/><i>High-level capture API</i><br/>───────────────<br/>captureStart(pin, revolutions)<br/>  → Arms ISR, records N revs<br/>  → Post-processes into<br/>     byte-packed slot table<br/>captureFetchPattern(out)<br/>  → Returns PatternRef<br/>  → .table in PSRAM"]

        LOOPBACK["<b>Loopback Validator</b><br/>───────────────<br/>loopbackEnable(expected, tol)<br/>captureLoopbackTick(rpm)<br/>  → 100ms poll from manager<br/>  → Compares captured edges<br/>    vs expected pattern<br/>captureLoopbackHasError()<br/>  → Sticky error flag<br/>  → Surfaces to UI via<br/>    g_loopback_error[96]"]
    end

    PIN18["GPIO 18<br/><i>PIN_CAPTURE_IN</i>"] --> EDGE
    EDGE --> RECORDER
    RECORDER -->|"PatternRef"| PLIB["PatternLibrary::<br/>registerUserPattern()"]
    LOOPBACK -.->|"g_loopback_error"| LVGL_POLL["LVGL Timer<br/>(polled for display)"]

    style CAPTURE fill:#141C2E,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
```

---

## 10. Complete Module Dependency Map

This shows which source file depends on which, revealing the layered architecture.

```mermaid
flowchart TB
    subgraph MAIN["src/main.cpp — Integration Point"]
        MAIN_FILE["<b>main.cpp</b><br/><i>setup() + loop() + managerTask()</i><br/><i>Wires all modules together</i><br/><i>Owns gCtrlQ, gGenInstance,</i><br/><i>gActivePattern, gCfg</i>"]
    end

    subgraph GEN["lib/ckp_gen/ — Signal Generation"]
        PREF["PatternRef.h<br/><i>Universal pattern handle</i>"]
        IGEN["CkpGenerator.h/.cpp<br/><i>IGenerator interface +</i><br/><i>TimerCkpGenerator</i>"]
        TGEN["TableCkpGenerator.h/.cpp<br/><i>GPTimer + dedic_gpio</i><br/><i>byte-table playback</i>"]
    end

    subgraph PAT["lib/patterns/ — Pattern Registry"]
        PLIB_SRC["PatternLibrary.h/.cpp<br/><i>3-tier registry</i>"]
        BUILTIN_GEN["builtin_tables_generated.h<br/><i>~127KB .rodata tables</i>"]
        NAMES_GEN["pattern_names_generated.h<br/><i>Friendly display names</i>"]
        LEGACY_GEN["pattern_legacy_index_generated.h<br/><i>Ardu-Stim index migration</i>"]
    end

    subgraph DSL["lib/dsl/ — DSL Compiler"]
        DSL_H["Dsl.h<br/><i>Public API</i>"]
        LEXER["Lexer.h/.cpp<br/><i>Tokenizer</i>"]
        PARSER["Parser.h/.cpp<br/><i>Recursive descent</i>"]
        COMPILER["Compiler.h/.cpp<br/><i>AST → byte table</i>"]
        VALIDATOR["Validator.h/.cpp<br/><i>12 semantic rules</i>"]
    end

    subgraph STORE["lib/sweep_compression/ — Storage & Simulation"]
        NVS_SRC["NvsStore.h/.cpp<br/><i>ESP32 NVS persistence</i>"]
        PSTOR["PatternStorage.h/.cpp<br/><i>LittleFS DSL + cache</i>"]
        SWEEP_SRC["SweepCompression.h/.cpp<br/><i>FreeRTOS sweep/comp task</i>"]
        CTABLES["CompressionTables.h<br/><i>Sin LUTs (90/120/180)</i>"]
        LFS["LittleFSInit.h/.cpp<br/><i>Mount + smoke test</i>"]
    end

    subgraph UI["lib/ui_lvgl/ — User Interface & I/O"]
        UI_SRC["ui_lvgl.h/.cpp<br/><i>2847-line LVGL UI</i><br/><i>Tabs, overlays, events</i>"]
        CTRL["ctrl_msg.h<br/><i>CtrlMsg / MsgType / MsgPayload</i>"]
        CLI_SRC["serial_cli.h/.cpp<br/><i>Dual-mode serial protocol</i>"]
        DSL_HELP["dsl_help.h<br/><i>ASCII grammar reference</i>"]
    end

    subgraph CAP["lib/ckp_capture/ — Signal Capture"]
        EDGE_SRC["EdgePulseCapture.h/.cpp<br/><i>ISR edge timing</i>"]
        REC_SRC["CaptureRecorder.h/.cpp<br/><i>Multi-rev capture + loopback</i>"]
    end

    %% Main dependencies
    MAIN_FILE --> IGEN
    MAIN_FILE --> TGEN
    MAIN_FILE --> PLIB_SRC
    MAIN_FILE --> DSL_H
    MAIN_FILE --> NVS_SRC
    MAIN_FILE --> SWEEP_SRC
    MAIN_FILE --> PSTOR
    MAIN_FILE --> LFS
    MAIN_FILE --> UI_SRC
    MAIN_FILE --> CTRL
    MAIN_FILE --> CLI_SRC
    MAIN_FILE --> EDGE_SRC
    MAIN_FILE --> REC_SRC
    MAIN_FILE --> PREF

    %% Internal deps
    IGEN --> PREF
    TGEN --> IGEN
    PLIB_SRC --> PREF
    PLIB_SRC --> BUILTIN_GEN
    PLIB_SRC --> NAMES_GEN
    PLIB_SRC --> LEGACY_GEN
    DSL_H --> PREF
    COMPILER --> LEXER
    COMPILER --> PARSER
    COMPILER --> VALIDATOR
    SWEEP_SRC --> IGEN
    SWEEP_SRC --> CTABLES
    UI_SRC --> CTRL
    UI_SRC --> PLIB_SRC
    UI_SRC --> PSTOR
    UI_SRC --> SWEEP_SRC
    UI_SRC --> NVS_SRC
    UI_SRC --> DSL_HELP
    CTRL --> IGEN
    CLI_SRC --> CTRL
    REC_SRC --> PREF

    style MAIN fill:#2a1a0d,stroke:#FFB020,stroke-width:2px,color:#D7E9FF
    style GEN fill:#0d1b3e,stroke:#00E5FF,stroke-width:1px,color:#D7E9FF
    style PAT fill:#0d1b3e,stroke:#00E5FF,stroke-width:1px,color:#D7E9FF
    style DSL fill:#141C2E,stroke:#66FF66,stroke-width:1px,color:#D7E9FF
    style STORE fill:#1a2a0d,stroke:#66FF66,stroke-width:1px,color:#D7E9FF
    style UI fill:#1a0d2a,stroke:#BB86FC,stroke-width:1px,color:#D7E9FF
    style CAP fill:#2a0d1a,stroke:#FF6666,stroke-width:1px,color:#D7E9FF
```

---

## 11. Build Configuration & Conditional Compilation

The firmware supports two hardware targets via build flags. This controls which code paths compile.

```mermaid
flowchart TB
    subgraph FLAGS["Build Flags (platformio.ini)"]
        direction LR
        S3["<b>esp32-s3-n4r8 (Production)</b><br/>─────────────────────<br/>SIGGEN_BACKEND_TABLE=1<br/>SIGGEN_HAS_DISPLAY=1<br/>SIGGEN_USE_LITTLEFS=1<br/>BOARD_HAS_PSRAM<br/>─────────────────────<br/>• TableCkpGenerator<br/>• Full LVGL UI (2847 lines)<br/>• LittleFS pattern persistence<br/>• PSRAM for DSL tables<br/>• 3-channel GPIO bundle"]
        WROOM["<b>esp32-wroom32d (Legacy)</b><br/><i>(currently commented out)</i><br/>─────────────────────<br/>(flags unset)<br/>─────────────────────<br/>• TimerCkpGenerator<br/>• No display (no-op stubs)<br/>• No LittleFS<br/>• No PSRAM<br/>• Single crank pin only<br/>• Serial CLI only"]
    end

    S3 -->|"compiles"| FULL["Full Feature Set<br/>TableCkpGenerator + LVGL + DSL + Sweep + Capture"]
    WROOM -->|"compiles"| MINIMAL["Minimal Feature Set<br/>TimerCkpGenerator + Serial CLI<br/>Legacy 5-preset SignalConfig path"]

    style FLAGS fill:#0B1020,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
    style S3 fill:#1a3a1a,stroke:#66FF66,stroke-width:2px,color:#D7E9FF
    style WROOM fill:#3a1a1a,stroke:#FF6666,stroke-width:2px,color:#D7E9FF
```

---

## Key C++ Concepts At Work

| Concept | Where | What It Does |
|---------|-------|-------------|
| **Abstract Interface (vtable)** | `IGenerator` | Defines the generator contract; `virtual` methods enable runtime polymorphism between `TimerCkpGenerator` and `TableCkpGenerator` |
| **Constructor / Destructor** | `TableCkpGenerator()` / `~TableCkpGenerator()` | Constructor zeros all fields; destructor frees PSRAM `_playback_buffer` and tears down GPTimer + dedic_gpio handles |
| **`static` Singleton Pattern** | `TimerCkpGenerator::s_inst` | ISR callback cannot capture `this` — a file-scope static pointer bridges the C callback to the C++ method |
| **`volatile` Qualifier** | `_edge_counter`, `_invert_mask`, `_cycle_start_us` | Tells the compiler not to optimize away reads — these are written by the ISR and read by other tasks/cores |
| **`portMUX_TYPE` Spinlock** | `s_ui_mux`, `g_dsl_error_mux` | ESP-IDF critical sections for cross-core access to small shared state (lighter than a mutex) |
| **FreeRTOS Queue** | `gCtrlQ` | Type-safe, ISR-safe message passing — the `CtrlMsg` union carries different payload types per `MsgType` |
| **`IRAM_ATTR`** | `TableCkpGenerator::onAlarm()` | Forces the ISR into internal RAM (not flash) — eliminates SPI flash cache-miss jitter during timing-critical operation |
| **`heap_caps_malloc(MALLOC_CAP_SPIRAM)`** | DSL Compiler output | Allocates the compiled byte table in PSRAM (8MB on S3), keeping internal SRAM free for stack and ISR buffers |
| **`constexpr` Arrays** | `CompressionTables.h`, `builtin_tables_generated.h` | Sin lookup tables and pattern byte arrays placed in `.rodata` at compile time — zero runtime initialization cost |
| **Tagged Union** | `MsgPayload` (union in `CtrlMsg`) | A single queue slot carries RPM values, SignalConfig structs, string pointers, raw byte tables, or sweep configs depending on `MsgType` |
| **Pending-Flag Sync Pattern** | `s_pending_rpm`, `s_pending_*` | Backend (Core 1) sets a volatile flag + value; UI thread (Core 0) reads and clears it inside a `portMUX` critical section — avoids LVGL reentrancy |
| **`extern "C"`** | `ui_get_active_pattern_for_wave()` | Prevents C++ name mangling so the LVGL `.c` callbacks can call into C++ code |
| **Build-Flag Polymorphism** | `#if defined(SIGGEN_BACKEND_TABLE)` | Compile-time selection of code paths — replaces the runtime cost of another vtable dispatch in the hot MSG handler |
| **`std::vector` in AST** | `WheelDef::runs`, `WheelDef::angular` | Parser uses heap-allocated dynamic arrays for variable-length DSL inputs; freed via `freeProgramAst()` |
