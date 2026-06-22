package com.signalgen.companion.dsl

/**
 * DslModel — the editable builder state, mirroring the on-board LVGL Custom-tab
 * builder (lib/ui_lvgl/ui_lvgl.cpp ~2419-2660) and the firmware DSL grammar
 * (lib/dsl/Parser.cpp / Dsl.h).
 *
 * Channel layout (FROZEN — matches the firmware):
 *   channel 0 = CKP  (crank, pin 1, bit0, rotation 'C' / CW,  360 deg)
 *   channel 1 = CMP1 (cam1,  pin 2, bit1, rotation 'c' / CCW, 720 deg group)
 *   channel 2 = CMP2 (cam2,  pin 3, bit2, rotation 'c' / CCW, 720 deg group)
 *
 * Any enabled cam (channel 1 or 2) promotes the WHOLE pattern to a 720 deg group;
 * see [BuilderState.hasCam].
 *
 * The three subtypes map 1:1 onto the firmware [WheelKind]:
 *   SYM  -> Symmetric : duty n/d, total teeth
 *   MISS -> Missing   : duty n/d, total teeth, ordered run-list (Nt / Nm)
 *   ANG  -> Angular   : alternating HIGH/LOW degree durations (starts HIGH)
 */

/** Wheel subtype. Mirrors firmware `enum class WheelKind { Symmetric, Missing, Angular }`. */
enum class WheelKind { SYM, MISS, ANG }

/** Rotation tag. 'C' = CW (360 deg crank); 'c' = CCW (720 deg cam). */
enum class Rotation { CW, CCW }

/** Per-channel hard caps lifted from the firmware (ui_lvgl.cpp #defines + Validator). */
object DslLimits {
    const val MAX_DYN_ROWS = 12     // CB_MAX_DYN_ROWS — per-channel runs[]/ang[] cap
    const val TEETH_MAX = 65535     // Parser.cpp total-teeth 0..0xFFFF
    const val SLOT_MAX = 4096       // Validator/Compiler per-wheel & LCM cap
    const val DUTY_DEN_MAX = 32     // Validator rule #4: d <= 32
    const val SRC_MAX = 512         // Validator rule #11: DSL source <= 512 chars
    const val N_CHANNELS = 3
}

/**
 * One Missing-wheel run row: `count` teeth of type 't' (present) or 'm' (gap).
 * Mirrors firmware `struct CbRun { int32_t count; char t; }` / `DslRunEntry`.
 */
data class RunRow(val count: Int, val suffix: Char) {
    val isMissing: Boolean get() = suffix == 'm'
    val isTeeth: Boolean get() = suffix == 't'
}

/**
 * A single channel's editable parameters.
 *
 * @property enabled     whether this channel participates (channel 0 is always on).
 * @property kind        S / M / A subtype.
 * @property dutyNum     duty numerator n (Symmetric/Missing).
 * @property dutyDen     duty denominator d (Symmetric/Missing).
 * @property teeth       total teeth (Symmetric/Missing).
 * @property runs        ordered run-list rows (Missing only).
 * @property angular     alternating degree durations starting HIGH (Angular only).
 */
data class ChannelSpec(
    val enabled: Boolean,
    val kind: WheelKind,
    val dutyNum: Int,
    val dutyDen: Int,
    val teeth: Int,
    val runs: List<RunRow>,
    val angular: List<Int>,
) {
    companion object {
        /** Default CKP (channel 0): Missing 60-2 — runs {58t},{2m}, duty 1/2. */
        fun defaultCrank(): ChannelSpec = ChannelSpec(
            enabled = true,
            kind = WheelKind.MISS,
            dutyNum = 1,
            dutyDen = 2,
            teeth = 60,
            runs = listOf(RunRow(58, 't'), RunRow(2, 'm')),
            angular = listOf(120, 60, 180),
        )

        /** Default cam (channels 1/2): Symmetric 1/2, single tooth, disabled. */
        fun defaultCam(): ChannelSpec = ChannelSpec(
            enabled = false,
            kind = WheelKind.SYM,
            dutyNum = 1,
            dutyDen = 2,
            teeth = 1,
            runs = listOf(RunRow(1, 't')),
            angular = listOf(360, 360),
        )
    }
}

/**
 * The complete builder model: the three channels plus stepper position.
 * Equivalent to the firmware `CbState`.
 */
data class BuilderState(
    val channels: List<ChannelSpec> = listOf(
        ChannelSpec.defaultCrank(),
        ChannelSpec.defaultCam(),
        ChannelSpec.defaultCam(),
    ),
    val step: Int = 0,
) {
    init {
        require(channels.size == DslLimits.N_CHANNELS) {
            "BuilderState requires exactly ${DslLimits.N_CHANNELS} channels"
        }
    }

    /** True when any cam channel (1 or 2) is enabled — promotes the group to 720 deg. */
    fun hasCam(): Boolean = channels[1].enabled || channels[2].enabled

    /** Channel 0 is always considered enabled; others honor their flag. */
    fun isEnabled(ch: Int): Boolean = ch == 0 || channels[ch].enabled

    /** Angular span this channel must sum to: 360 for the crank, 720 for a cam. */
    fun spanFor(ch: Int): Int = if (ch == 0) 360 else 720

    /** Group span: 720 if any cam is enabled, else 360. */
    fun groupSpan(): Int = if (hasCam()) 720 else 360

    fun withChannel(ch: Int, spec: ChannelSpec): BuilderState =
        copy(channels = channels.toMutableList().also { it[ch] = spec })

    companion object {
        /** Physical pin for a channel index (1-based): ch0->1, ch1->2, ch2->3. */
        fun pinOf(ch: Int): Int = ch + 1

        /** Channel-mask bit for a channel index: bit0 crank, bit1 cam1, bit2 cam2. */
        fun bitOf(ch: Int): Int = 1 shl ch

        /** Rotation tag for a channel: crank is CW ('C'), cams are CCW ('c'). */
        fun rotationOf(ch: Int): Rotation = if (ch == 0) Rotation.CW else Rotation.CCW

        /** Display label for a channel index. */
        fun labelOf(ch: Int): String = when (ch) {
            0 -> "CKP"
            1 -> "CMP1"
            else -> "CMP2"
        }
    }
}
