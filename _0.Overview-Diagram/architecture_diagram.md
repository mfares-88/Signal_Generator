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
            LOOP["`**Arduino loop()**
            10ms tick cycle`"]
            LVGL_PUMP["`**ui_task_handler()**
            Pumps LVGL rendering`"]
            SERIAL_POLL["`**serialCliPoll()**
            Drains Serial bytes`"]
            CAP_FETCH["`**capRX.fetch()**
            Edge pulse check`"]

            LOOP --> LVGL_PUMP
            LOOP --> SERIAL_POLL
            LOOP --> CAP_FETCH
        end

        subgraph Core1["🟢 Core 1 — Real-Time Engine"]
            direction TB
            MGR["`**managerTask**
            Priority 3 - 8KB stack
            Message dispatcher`"]
            SWP["`**sweepCompTask**
            Priority 2 - 4KB stack
            1ms tick - sweep + compression`"]
            ISR["`**GPTimer ISR**
            IRAM_ATTR
            <=5 statements per alarm
            dedic_gpio_bundle_write()`"]
        end

        subgraph SharedState["📦 Shared State (volatile / portMUX)"]
            QUEUE["`**gCtrlQ**
            FreeRTOS Queue
            depth=16 x sizeof(CtrlMsg)`"]
            GLOBALS["`**Shared Globals**
            g_rpm - g_pattern_key
            g_invert_mask
            g_sweep_* - g_comp_*`"]
            PENDING["`**Pending Flags**
            s_pending_rpm
            s_pending_pattern
            s_pending_running
            Pending-flag sync pattern`"]
        end
    end

    LOOP -.->|"10ms vTaskDelay"| LOOP
    MGR -->|"`xQueueReceive
    100ms tick`"| QUEUE
    LVGL_PUMP -.->|"reads pending flags"| PENDING
    MGR -.->|"writes pending flags"| PENDING
    SWP -->|"`gen->setRpm()
    fast path only`"| ISR

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
    BOOT["⚡ **Power On / Reset**"] --> SERIAL["`**Serial.begin(921600)**
    Debug output`"]
    SERIAL --> DIAG["`**Boot diagnostics**
    Flash size, PSRAM, heap`"]
    DIAG --> FS["`**initLittleFS()**
    Mount LittleFS partition
    (no-op on WROOM)`"]
    FS --> FS_SMOKE["`**littleFsSmokeTest()**
    Write + readback /test.txt`"]

    FS_SMOKE --> NVS_BEGIN["`**NvsStore::begin()**
    Open 'siggen' namespace
    Schema v1 check + migration`"]
    NVS_BEGIN --> NVS_LOAD["`**NvsStore::loadAllToGlobals()**
    Populate g_rpm, g_pattern_key,
    g_invert_mask, g_sweep_*, g_comp_*
    Missing keys -> factory defaults`"]

    NVS_LOAD --> GEN_INIT["`**gGen.begin(PIN17, PIN9, PIN14)**
    Constructor: TableCkpGenerator()
    - Validate all 3 GPIO pins
    - Create GPTimer @ 1MHz
    - Build dedic_gpio bundle`"]

    GEN_INIT --> SWP_INIT["`**sweepCompressionInit(&gGen)**
    - Create mutex
    - Spawn sweepCompTask
      Core 1, Priority 2, 4KB`"]

    SWP_INIT --> CAP_INIT["`**capRX.begin(PIN18)**
    EdgePulseCapture ISR setup`"]

    CAP_INIT --> UI_INIT["`**ui_init(callbacks...)**
    - lv_init() + display driver
    - Touch controller (GT911)
    - Build LVGL widget tree
    - 480x272 landscape`"]

    UI_INIT --> QUEUE_CREATE["`**xQueueCreate(16, sizeof(CtrlMsg))**
    gCtrlQ - central command queue`"]

    QUEUE_CREATE --> TASK_CREATE["`**xTaskCreatePinnedToCore()**
    managerTask, 'managerTask',
    8192, nullptr, priority=3,
    nullptr, core=1`"]

    TASK_CREATE --> RESTORE_PAT["`**Restore Pattern from NVS**
    findByKey(g_pattern_key)
    -> fallback builtinByIndex(0)
    -> gGen.apply(ref, rpm)`"]

    RESTORE_PAT --> RESTORE_SWEEP["`**Restore Sweep + Compression**
    sweepSet(saved_config)
    compressionSet(saved_config)`"]

    RESTORE_SWEEP --> RESTORE_INV["`**Restore Invert Mask**
    gGen.setInverted(g_invert_mask)`"]

    RESTORE_INV --> START["`**gGen.start()**
    Enable GPTimer alarm
    ISR begins firing`"]

    START --> CLI_INIT["`**serialCliBegin()**
    Serial CLI ready`"]

    CLI_INIT --> UI_SYNC["`**UI sync**
    update_rpm, pattern,
    running, inverted, channels`"]

    UI_SYNC --> LOOP_START["`**-> loop() begins**
    10ms cadence forever`"]

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
        UI_CB["`**LVGL UI Callbacks**
        on_ui_rpm()
        on_ui_pattern()
        on_ui_run()
        on_ui_custom()
        on_ui_invert()`"]
        SERIAL["`**Serial CLI**
        Legacy binary opcodes
        Text commands (LIST,
        SELECT, COMPILE, etc.)`"]
    end

    QUEUE["`**gCtrlQ**
    FreeRTOS Queue
    16 slots`"]

    subgraph Consumer["📥 managerTask (Core 1, P3)"]
        direction TB
        DISPATCH{{"switch(m.type)"}}

        MSG_RPM["`**MSG_SET_RPM**
        clamp 100..6000
        gen.setRpm() <- fast path
        NvsStore::setRpmDebounced()`"]
        MSG_PAT["`**MSG_SET_PATTERN**
        Legacy 5-preset path
        (WROOM only)`"]
        MSG_BUILTIN["`**MSG_SELECT_BUILTIN**
        PatternLibrary::builtinByIndex()
        -> applyPatternRef()
        -> gen.apply(ref, rpm)`"]
        MSG_NAMED["`**MSG_SELECT_NAMED**
        PatternLibrary::findByKey()
        -> applyPatternRef()`"]
        MSG_CUSTOM["`**MSG_SET_CUSTOM**
        dslCompileSignalConfig(cfg)
        -> DSL pipeline -> PatternRef
        -> gen.apply()`"]
        MSG_DSL["`**MSG_LOAD_DSL**
        dslCompile(source)
        -> register as 'scratch_dsl'
        -> gen.apply()`"]
        MSG_START["`**MSG_START / MSG_STOP**
        gen.start() / gen.stop()`"]
        MSG_INV["`**MSG_SET_INVERT**
        gen.setInverted(mask)
        NvsStore::setInvertMask()`"]
        MSG_SWEEP["`**MSG_SET_SWEEP**
        sweepSet(config)
        NvsStore::setSweep()`"]
        MSG_COMP["`**MSG_SET_COMPRESSION**
        compressionSet(config)
        NvsStore::setCompression()`"]
        MSG_CAP["`**MSG_CAPTURE_START/STOP**
        captureStart(pin, revs)
        captureFetchPattern()`"]
        MSG_SAVE["`**MSG_SAVE_USER**
        PatternStorage::saveDsl()`"]
        MSG_TABLE["`**MSG_LOAD_TABLE**
        Build PatternRef from raw bytes
        -> gen.apply()`"]
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
            ARC["`**RPM Arc**
            Range 100-6000
            on_arc_changed -> MSG_SET_RPM`"]
            DD["`**Pattern Dropdown**
            64+ builtin patterns
            with fuzzy search filter
            on_pattern_changed -> MSG_SELECT_BUILTIN`"]
            BTN_RUN["`**RUN/STOP Button**
            on_run_clicked -> MSG_START/STOP`"]
            BTN_INV["`**INVERT Button**
            on_invert_clicked -> MSG_SET_INVERT`"]
            LEDS["`**Channel LEDs**
            Crank - CAM1 - CAM2
            Green=active, Grey=unused`"]
            ERR["`**Error Label**
            Shows gen errors / DSL errors`"]
        end

        subgraph ADV["⚙️ ADVANCED Tab → Full-Screen Overlays"]
            direction LR
            SWEEP_PAGE["`**Sweep Page**
            Low/High RPM spinboxes
            Mode: OFF/Linear/Log/Waypoint
            Interval spinner
            Live RPM arc + status pill
            -> MSG_SET_SWEEP`"]
            COMP_PAGE["`**Compression Page**
            Enable switch
            Cylinders / Threshold / Peak
            Dynamic toggle
            Live RPM arc + status pill
            -> MSG_SET_COMPRESSION`"]
            DSL_PAGE["`**DSL Editor Page**
            Source textarea + keyboard
            Compile / Save As / Load
            Help sub-page overlay
            Error polling timer (200ms)
            -> MSG_LOAD_DSL / MSG_SAVE_USER`"]
            WAVE_PAGE["`**Waveform Canvas**
            3-lane oscilloscope view
            Crank + CAM1 + CAM2
            Pinch-zoom Q24.8 model
            Pan, pause, lane toggle
            -> reads getEdgeCounter()`"]
            CUSTOM_PAGE["`**Custom Pattern Page**
            Teeth/Periods/Missing spinboxes
            Gap position dropdown
            Gap level switch
            -> MSG_SET_CUSTOM`"]
        end
    end

    subgraph SYNC["Cross-Core Pending-Flag Sync"]
        direction LR
        BACKEND["`**managerTask**
        (Core 1)`"]
        FLAGS["`**Volatile Flags**
        volatile bool
        s_pending_rpm
        s_pending_pattern
        s_pending_running
        s_pending_error
        s_pending_channels`"]
        APPLY["`**apply_pending_updates()**
        Called every ui_task_handler()
        portENTER_CRITICAL / portEXIT_CRITICAL`"]

        BACKEND -->|"`ui_update_rpm(val)
        sets flag + val`"| FLAGS
        FLAGS -->|"checked each tick"| APPLY
        APPLY -->|"`lv_arc_set_value()
        lv_label_set_text()
        ...`"| HOME
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
    SRC["`**DSL Source String**
    '1,C,M,1/2,60,58t,2m'`"]

    subgraph PIPELINE["DSL Compiler Pipeline (lib/dsl/)"]
        direction LR
        LEX["`**Lexer**
        Tokenizer
        ---------
        TOK_INT - TOK_FRACTION
        TOK_INT_SUFFIXED
        TOK_LETTER - TOK_COMMA
        TOK_COLON - TOK_EOF`"]

        PARSE["`**Parser**
        Recursive Descent
        ---------------
        ProgramAst {
          wheels: WheelDef[]
        }
        WheelDef {
          pin, rotation, kind,
          duty, total_teeth,
          runs[], angular[]
        }`"]

        VAL["`**Validator**
        12 Semantic Rules (section 7.5)
        -----------------
        #1 pin in {1..4}
        #2 pins unique
        #3 duty 0 < n < d, d <= 32
        #4 teeth > 0
        #5 sum(t+m)=teeth
        #9 table <= 4096 slots
        #11 source <= 512 chars`"]

        COMP["`**Compiler**
        AST -> Byte Table
        -----------------
        - Compute LCM across wheels
        - Pack bit0=crank, bit1=cam1
          bit2=cam2 per slot
        - Allocate in PSRAM via
          heap_caps_malloc(SPIRAM)`"]

        LEX -->|"Token stream"| PARSE
        PARSE -->|"ProgramAst*"| VAL
        VAL -->|"validated AST"| COMP
    end

    SRC --> LEX

    RESULT["`**DslResult**
    {
      ok: bool
      pattern: PatternRef
      error[96]
      error_offset
    }`"]

    COMP --> RESULT

    SIGCFG["`**SignalConfig**
    Legacy custom modal`"]
    SHIM["`**dslCompileSignalConfig()**
    Synthesizes DSL source
    from teeth/miss params
    then feeds dslCompile()`"]

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

        BUILTIN["`**Builtin Tier**
        Static const PatternRef[]
        in .rodata (flash)
        ------------------
        Generated by:
        tools/convert_ardustim_wheels.py
        -> builtin_tables_generated.h
        -> pattern_names_generated.h
        -> pattern_legacy_index_generated.h
        ------------------
        64+ patterns: 60-2, 36-1,
        GM LS1, Honda K20, etc.
        Immutable at runtime`"]

        USER["`**User Tier**
        Up to 16 runtime entries
        kUserCapacity = 16
        ------------------
        Sources:
        - DSL editor 'Save As'
        - Captured waveforms
        - Serial CLI 'SAVE' command
        - PatternRef stored by value
        - table -> PSRAM heap
        - name_key -> caller-owned`"]

        SCRATCH["`**Scratch**
        s_scratch_dsl
        ------------------
        Transient unsaved DSL
        compilation result
        Manager-owned lifetime
        Freed on next compile or
        when switching to builtin`"]
    end

    subgraph PERSIST["Persistence Backends"]
        direction TB
        NVS["`**NvsStore**
        ESP32 NVS (key-value flash)
        ------------------
        Namespace: 'siggen'
        Schema v1 (versioned)
        ------------------
        Keys stored:
        - pattern_key (string, 64B)
        - rpm (uint32)
        - invert_mask (uint8)
        - sweep_* (4 keys)
        - comp_* (5 keys)
        ------------------
        Debounced RPM writes
        (750ms window)`"]

        LITTLEFS["`**PatternStorage**
        LittleFS partition
        (S3 only, gated on flag)
        ------------------
        /patterns/key.dsl
          -> UTF-8 DSL source
        /patterns/key.bin
          -> SHA-256 + metadata
            + compiled byte table
        ------------------
        Cache invalidation:
        SHA-256 mismatch -> recompile`"]
    end

    BUILTIN -->|"findByKey(key)"| LOOKUP["`**PatternLibrary::findByKey()**
    Searches: builtin -> user -> scratch`"]
    USER -->|"findByKey(key)"| LOOKUP
    SCRATCH -.->|"(future)"| LOOKUP

    BUILTIN -->|"builtinByIndex(i)"| INDEX["`**PatternLibrary::builtinByIndex()**
    Direct array index`"]

    USER -->|"registerUserPattern()"| LITTLEFS
    LITTLEFS -->|"loadDsl() -> dslCompile()"| USER

    LOOKUP -->|"Resolved PatternRef"| APPLY_FN["`**gGen.apply(ref, rpm)**
    Active playback begins`"]

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
        READ_CFG["cfgLock() -> copy SweepConfig + CompressionConfig -> cfgUnlock()"]

        IDLE_CHECK{"sweep.mode == OFF\nAND !comp.enabled?"}

        subgraph SWEEP["Sweep Phase"]
            direction TB
            SW_DISPATCH{{"sweep.mode?"}}
            LINEAR["`**sweepStepLinear()**
            +/-1 RPM per interval_us
            Ping-pong between
            low_rpm <-> high_rpm`"]
            LOG["`**sweepStepLog()**
            Exponential ramp
            rpm = low x (high/low)^(t/period)
            Uses powf() (HW FP)`"]
            WAYPOINT["`**sweepStepWaypoint()**
            Packed (rpm, dwell_ms) pairs
            Linear interp between segments`"]
        end

        subgraph COMPRESSION["Compression Phase"]
            direction TB
            ANGLE["`**currentCrankAngle(offset_deg)**
            Uses getCycleStartUs() +
            getCycleDurationUs() from ISR`"]
            LUT_SELECT{{"comp.cyl?"}}
            SIN180["`**sin_100_180[180]**
            1-cyl, 2-cyl`"]
            SIN120["`**sin_100_120[120]**
            3-cyl`"]
            SIN90["`**sin_100_90[90]**
            4-cyl, 6+cyl`"]
            CUSTOM_CURVE["`**custom_curve_256[256]**
            User-supplied override`"]
            SCALE["`**Scale by peak (0..100)**
            Dynamic: x (rpm/thresh)
            if rpm < 655 (AVR guard)`"]
        end

        FINAL["`**target = base_rpm - modifier**
        gen->setRpm(target)
        Fast path, no buffer rebuild`"]
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

        EDGE["`**EdgePulseCapture**
        Low-level ISR-based
        edge timing capture
        ---------------
        - Pin interrupt handler
        - Measures period_us,
          high_us, timestamp_us
        - FreeRTOS queue -> fetch()`"]

        RECORDER["`**CaptureRecorder**
        High-level capture API
        ---------------
        captureStart(pin, revolutions)
          -> Arms ISR, records N revs
          -> Post-processes into
             byte-packed slot table
        captureFetchPattern(out)
          -> Returns PatternRef
          -> .table in PSRAM`"]

        LOOPBACK["`**Loopback Validator**
        ---------------
        loopbackEnable(expected, tol)
        captureLoopbackTick(rpm)
          -> 100ms poll from manager
          -> Compares captured edges
            vs expected pattern
        captureLoopbackHasError()
          -> Sticky error flag
          -> Surfaces to UI via
            g_loopback_error[96]`"]
    end

    PIN18["`**GPIO 18**
    PIN_CAPTURE_IN`"] --> EDGE
    EDGE --> RECORDER
    RECORDER -->|"PatternRef"| PLIB["`**PatternLibrary::**
    registerUserPattern()`"]
    LOOPBACK -.->|"g_loopback_error"| LVGL_POLL["`**LVGL Timer**
    (polled for display)`"]

    style CAPTURE fill:#141C2E,stroke:#00E5FF,stroke-width:2px,color:#D7E9FF
```

---

## 10. Complete Module Dependency Map

This shows which source file depends on which, revealing the layered architecture.

```mermaid
flowchart TB
    subgraph MAIN["src/main.cpp — Integration Point"]
        MAIN_FILE["`**main.cpp**
        setup() + loop() + managerTask()
        Wires all modules together
        Owns gCtrlQ, gGenInstance,
        gActivePattern, gCfg`"]
    end

    subgraph GEN["lib/ckp_gen/ — Signal Generation"]
        PREF["`**PatternRef.h**
        Universal pattern handle`"]
        IGEN["`**CkpGenerator.h/.cpp**
        IGenerator interface +
        TimerCkpGenerator`"]
        TGEN["`**TableCkpGenerator.h/.cpp**
        GPTimer + dedic_gpio
        byte-table playback`"]
    end

    subgraph PAT["lib/patterns/ — Pattern Registry"]
        PLIB_SRC["`**PatternLibrary.h/.cpp**
        3-tier registry`"]
        BUILTIN_GEN["`**builtin_tables_generated.h**
        ~127KB .rodata tables`"]
        NAMES_GEN["`**pattern_names_generated.h**
        Friendly display names`"]
        LEGACY_GEN["`**pattern_legacy_index_generated.h**
        Ardu-Stim index migration`"]
    end

    subgraph DSL["lib/dsl/ — DSL Compiler"]
        DSL_H["`**Dsl.h**
        Public API`"]
        LEXER["`**Lexer.h/.cpp**
        Tokenizer`"]
        PARSER["`**Parser.h/.cpp**
        Recursive descent`"]
        COMPILER["`**Compiler.h/.cpp**
        AST -> byte table`"]
        VALIDATOR["`**Validator.h/.cpp**
        12 semantic rules`"]
    end

    subgraph STORE["lib/sweep_compression/ — Storage & Simulation"]
        NVS_SRC["`**NvsStore.h/.cpp**
        ESP32 NVS persistence`"]
        PSTOR["`**PatternStorage.h/.cpp**
        LittleFS DSL + cache`"]
        SWEEP_SRC["`**SweepCompression.h/.cpp**
        FreeRTOS sweep/comp task`"]
        CTABLES["`**CompressionTables.h**
        Sin LUTs (90/120/180)`"]
        LFS["`**LittleFSInit.h/.cpp**
        Mount + smoke test`"]
    end

    subgraph UI["lib/ui_lvgl/ — User Interface & I/O"]
        UI_SRC["`**ui_lvgl.h/.cpp**
        2847-line LVGL UI
        Tabs, overlays, events`"]
        CTRL["`**ctrl_msg.h**
        CtrlMsg / MsgType / MsgPayload`"]
        CLI_SRC["`**serial_cli.h/.cpp**
        Dual-mode serial protocol`"]
        DSL_HELP["`**dsl_help.h**
        ASCII grammar reference`"]
    end

    subgraph CAP["lib/ckp_capture/ — Signal Capture"]
        EDGE_SRC["`**EdgePulseCapture.h/.cpp**
        ISR edge timing`"]
        REC_SRC["`**CaptureRecorder.h/.cpp**
        Multi-rev capture + loopback`"]
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
        S3["`**esp32-s3-n4r8 (Production)**
        ---------------------
        SIGGEN_BACKEND_TABLE=1
        SIGGEN_HAS_DISPLAY=1
        SIGGEN_USE_LITTLEFS=1
        BOARD_HAS_PSRAM
        ---------------------
        - TableCkpGenerator
        - Full LVGL UI (2847 lines)
        - LittleFS pattern persistence
        - PSRAM for DSL tables
        - 3-channel GPIO bundle`"]
        WROOM["`**esp32-wroom32d (Legacy)**
        (currently commented out)
        ---------------------
        (flags unset)
        ---------------------
        - TimerCkpGenerator
        - No display (no-op stubs)
        - No LittleFS
        - No PSRAM
        - Single crank pin only
        - Serial CLI only`"]
    end

    S3 -->|"compiles"| FULL["Full Feature Set\nTableCkpGenerator + LVGL + DSL + Sweep + Capture"]
    WROOM -->|"compiles"| MINIMAL["Minimal Feature Set\nTimerCkpGenerator + Serial CLI\nLegacy 5-preset SignalConfig path"]

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
