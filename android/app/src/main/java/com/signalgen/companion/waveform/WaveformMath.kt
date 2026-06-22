package com.signalgen.companion.waveform

import kotlin.math.roundToInt

/**
 * WaveformMath — the timing/angle math for the interactive waveform (plan §4.7).
 *
 * The compiled DSL table is a sequence of `slotCount` slots that together span
 * `spanDeg` crank degrees of ONE table cycle:
 *   - a 360 deg (crank-only) table spans ONE crank revolution per cycle,
 *   - a 720 deg (cam present) table spans TWO crank revolutions per cycle.
 *
 * Each slot therefore covers `spanDeg / slotCount` crank degrees. At a given RPM
 * the crank turns 360 deg in `60e6 / rpm` microseconds, so one slot lasts:
 *
 *     periodUsPerSlot = 60e6 * spanDeg / (360 * rpm * slotCount)
 *
 * KDoc on `cycUs` (the telemetry field, §4.0): it is the FULL table period — for a
 * 720 deg table that is TWO crank revolutions per cycle, so cursor/crank-angle math
 * must always use the `deg` (spanDeg) field, never assume 360.
 *
 * IMPORTANT — these are DIGITAL logic levels (HIGH/LOW = 1/0). There is no analog
 * voltage amplitude; the only "amplitude-like" measurement is DUTY%.
 */
object WaveformMath {

    private const val US_PER_MINUTE = 60_000_000.0  // 60e6 microseconds per minute

    /**
     * Microseconds per single slot at a given RPM.
     *
     *   periodUsPerSlot = 60e6 * spanDeg / (360 * rpm * slotCount)
     *
     * @param rpm        crank RPM (>0).
     * @param spanDeg    table span in crank degrees (360 or 720).
     * @param slotCount  number of slots in the compiled table (1..4096).
     */
    fun periodUsPerSlot(rpm: Double, spanDeg: Int, slotCount: Int): Double {
        if (rpm <= 0.0 || slotCount <= 0) return 0.0
        return US_PER_MINUTE * spanDeg / (360.0 * rpm * slotCount)
    }

    /** Crank degrees covered by one slot: spanDeg / slotCount. */
    fun degPerSlot(spanDeg: Int, slotCount: Int): Double {
        if (slotCount <= 0) return 0.0
        return spanDeg.toDouble() / slotCount.toDouble()
    }

    /** Convert a (fractional) slot index to table-space degrees [0, spanDeg). */
    fun slotToDeg(slot: Double, spanDeg: Int, slotCount: Int): Double =
        slot * degPerSlot(spanDeg, slotCount)

    /** Convert table-space degrees to a (fractional) slot index. */
    fun degToSlot(deg: Double, spanDeg: Int, slotCount: Int): Double {
        if (spanDeg <= 0) return 0.0
        return deg / spanDeg.toDouble() * slotCount.toDouble()
    }

    /**
     * Elapsed time in microseconds between two (fractional) slot positions at a
     * given RPM. Sign follows (s1 - s0).
     */
    fun deltaTimeUs(s0: Double, s1: Double, rpm: Double, spanDeg: Int, slotCount: Int): Double =
        (s1 - s0) * periodUsPerSlot(rpm, spanDeg, slotCount)

    /**
     * Tooth/edge rate in Hz — the rate at which SLOTS advance, i.e. 1 / slotPeriod.
     *
     *     frequencyHz = 1 / (periodUsPerSlot * 1e-6)
     *
     * NOTE (crank-vs-tooth distinction, plan §4.7 / §9): this is the TOOTH/EDGE rate
     * (slots per second), NOT the crank rotational frequency. The crank rotational
     * frequency is `rpm / 60` Hz; see [crankFreqHz]. The UI labels this value
     * "tooth/edge rate" and shows the crank RPM as a secondary readout.
     */
    fun frequencyHz(rpm: Double, spanDeg: Int, slotCount: Int): Double {
        val periodUs = periodUsPerSlot(rpm, spanDeg, slotCount)
        if (periodUs <= 0.0) return 0.0
        return 1.0 / (periodUs * 1e-6)
    }

    /** Crank rotational frequency in Hz = rpm / 60 (secondary readout for the cursor panel). */
    fun crankFreqHz(rpm: Double): Double = rpm / 60.0

    /**
     * Crank angle in degrees for an edge/slot position:
     *
     *     crankAngleDeg(edge) = (edge / slotCount) * spanDeg
     *
     * For a 720 deg table this yields 0..720; the 360 deg dial divides by 2 (and
     * everything is shown mod 360). See [crankAngleForDial].
     */
    fun crankAngleDeg(edge: Double, slotCount: Int, spanDeg: Int): Double {
        if (slotCount <= 0) return 0.0
        return edge / slotCount.toDouble() * spanDeg.toDouble()
    }

    /**
     * Crank angle mapped onto the 360 deg dial: a 720 deg table is divided by 2 so a
     * full table cycle maps to a single 0..360 sweep; result is taken mod 360.
     */
    fun crankAngleForDial(edge: Double, slotCount: Int, spanDeg: Int): Double {
        val raw = crankAngleDeg(edge, slotCount, spanDeg)
        val onCrank = if (spanDeg >= 720) raw / 2.0 else raw
        var m = onCrank % 360.0
        if (m < 0) m += 360.0
        return m
    }

    /**
     * DUTY% of a given channel bit over a slot window [s0, s1) of the compiled table.
     *
     * Counts the fraction of slots in the (wrapping) window where `bit` is HIGH.
     * `s0`/`s1` are fractional slot positions; we sample whole slots in the window.
     * Returns 0..100. This is the only "amplitude-like" measurement for these
     * digital signals (plan D4).
     *
     * @param table  the bit-packed compiled table (bit0 crank / bit1 cam1 / bit2 cam2).
     * @param bit    the channel bit mask (1, 2, or 4).
     * @param s0     window start (fractional slot).
     * @param s1     window end   (fractional slot).
     */
    fun dutyOverWindow(table: ByteArray, bit: Int, s0: Double, s1: Double): Double {
        val n = table.size
        if (n == 0) return 0.0
        val lo = minOf(s0, s1)
        val hi = maxOf(s0, s1)
        val first = kotlin.math.floor(lo).toInt()
        val last = kotlin.math.ceil(hi).toInt()
        var high = 0
        var total = 0
        var i = first
        while (i < last) {
            val idx = ((i % n) + n) % n
            if ((table[idx].toInt() and bit) != 0) high++
            total++
            i++
        }
        if (total == 0) return 0.0
        return high.toDouble() / total.toDouble() * 100.0
    }

    /**
     * Whether `bit` is HIGH at a (wrapping) integer slot index. Used by the lane
     * renderer's envelope sampler.
     */
    fun levelAt(table: ByteArray, bit: Int, slot: Int): Boolean {
        val n = table.size
        if (n == 0) return false
        val idx = ((slot % n) + n) % n
        return (table[idx].toInt() and bit) != 0
    }

    /**
     * Fixed-point (1/256-slot resolution) column -> [s0, s1] slot-window mapper.
     *
     * Given a pixel/canvas column over a visible slot window, returns the inclusive
     * slot range that column covers so the lane renderer can collapse sub-slot
     * detail into a HIGH/LOW envelope (avoids aliasing when zoomed out). The 1/256
     * granularity matches the firmware's sub-slot envelope resolution.
     *
     * @param col            current column (0..widthPx-1).
     * @param widthPx        canvas width in pixels.
     * @param visStartSlot   left edge of the visible window in slots (fractional).
     * @param visSlots       number of slots visible across the canvas (fractional).
     * @return Pair(s0, s1) fractional slot positions for this column.
     */
    fun colToSlotWindow(
        col: Int,
        widthPx: Int,
        visStartSlot: Double,
        visSlots: Double,
    ): Pair<Double, Double> {
        if (widthPx <= 0) return visStartSlot to visStartSlot
        val slotsPerPx = visSlots / widthPx.toDouble()
        // Quantize to 1/256 slot so neighboring columns share consistent boundaries.
        fun q(x: Double): Double = (x * 256.0).roundToInt() / 256.0
        val s0 = q(visStartSlot + col * slotsPerPx)
        val s1 = q(visStartSlot + (col + 1) * slotsPerPx)
        return s0 to s1
    }

    /**
     * Envelope (anyHigh, anyLow) of a channel bit over a fractional slot window —
     * used to draw a thick HIGH band, a thin LOW line, or a transition region when
     * the window straddles an edge (zoomed-out anti-aliasing).
     */
    fun envelopeOverWindow(table: ByteArray, bit: Int, s0: Double, s1: Double): Pair<Boolean, Boolean> {
        val n = table.size
        if (n == 0) return false to true
        val lo = kotlin.math.floor(minOf(s0, s1)).toInt()
        val hi = maxOf(kotlin.math.ceil(maxOf(s0, s1)).toInt(), lo + 1)
        var anyHigh = false
        var anyLow = false
        var i = lo
        while (i < hi) {
            if (levelAt(table, bit, i)) anyHigh = true else anyLow = true
            i++
        }
        return anyHigh to anyLow
    }
}
