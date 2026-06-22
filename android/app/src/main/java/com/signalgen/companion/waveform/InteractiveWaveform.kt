package com.signalgen.companion.waveform

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.runtime.withFrameMillis
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.signalgen.companion.dsl.DslCompiler
import com.signalgen.companion.ui.theme.HudMuted
import com.signalgen.companion.ui.theme.HudSunken
import com.signalgen.companion.ui.theme.HudText
import com.signalgen.companion.ui.theme.LocalWaveformColors
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.compositionLocalOf
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * InteractiveWaveform — the interactive Canvas waveform (plan §4.7).
 *
 * Renders N enabled channel lanes as DIGITAL HIGH/LOW traces derived from the
 * compiled slot table's envelope, overlays 90 deg crank gridlines + a TDC marker
 * at 0 deg + dim-red gap bands over the crank's missing ('m') runs, and supports:
 *   - pinch-zoom (1x .. 32x) + clamped horizontal pan (Modifier.pointerInput),
 *   - PAUSE / PLAY of the RPM-simulated live cursor (D15 — labeled "simulated"),
 *   - up to two draggable measurement cursors,
 *   - a measurement panel: Delta time / tooth-edge freq (+ crank RPM) /
 *     Delta crank-angle / DUTY%, with an explicit
 *     "digital logic levels — no voltage amplitude" note.
 *
 * The signals are DIGITAL: lanes are HIGH/LOW (1/0); there is no voltage amplitude.
 */

/** Lanes derived from the compiled table — one per enabled channel. */
data class WaveformLane(
    val channel: Int,      // 0 crank / 1 cam1 / 2 cam2
    val bit: Int,          // 1 / 2 / 4
    val label: String,     // "CKP" / "CMP1" / "CMP2"
)

private const val MIN_ZOOM = 1f
private const val MAX_ZOOM = 32f

@Composable
fun InteractiveWaveform(
    stats: DslCompiler.CompiledStats,
    lanes: List<WaveformLane>,
    rpm: Double,
    /** Crank's missing-run gap windows in slot space (start, endExclusive) for the dim-red bands. */
    gapWindows: List<Pair<Int, Int>>,
    modifier: Modifier = Modifier,
) {
    val waveColors = LocalWaveformColors.current
    val slotCount = stats.slotCount.coerceAtMost(com.signalgen.companion.dsl.DslLimits.SLOT_MAX)
    val spanDeg = stats.degrees

    // View transform: zoom factor and pan (left-edge slot).
    var zoom by remember(slotCount) { mutableStateOf(1f) }
    var panSlot by remember(slotCount) { mutableStateOf(0.0) }

    // Live RPM-simulated cursor (D15) and pause state.
    var paused by remember { mutableStateOf(false) }
    var liveSlot by remember(slotCount) { mutableStateOf(0.0) }

    // Two draggable measurement cursors (in slot space). Null = not placed.
    var cursorA by remember(slotCount) { mutableStateOf<Double?>(slotCount * 0.25) }
    var cursorB by remember(slotCount) { mutableStateOf<Double?>(slotCount * 0.5) }

    // Canvas size captured for pointer math.
    var canvasW by remember { mutableStateOf(1f) }
    var canvasH by remember { mutableStateOf(1f) }

    // Advance the live cursor from RPM while playing (one full table cycle per cycUs).
    LaunchedEffect(paused, rpm, slotCount, spanDeg) {
        if (paused || rpm <= 0.0 || slotCount <= 0) return@LaunchedEffect
        var last = withFrameMillis { it }
        while (true) {
            val now = withFrameMillis { it }
            val dtMs = (now - last).coerceAtLeast(0)
            last = now
            val periodUsPerSlot = WaveformMath.periodUsPerSlot(rpm, spanDeg, slotCount)
            if (periodUsPerSlot > 0.0) {
                val advancedSlots = (dtMs * 1000.0) / periodUsPerSlot
                liveSlot = (liveSlot + advancedSlots) % slotCount
            }
        }
    }

    fun visibleSlots(): Double = slotCount / zoom.toDouble()

    fun clampPan() {
        val maxPan = (slotCount - visibleSlots()).coerceAtLeast(0.0)
        panSlot = panSlot.coerceIn(0.0, maxPan)
    }

    Column(modifier = modifier) {
        Canvas(
            modifier = Modifier
                .fillMaxWidth()
                .height(220.dp)
                .background(HudSunken, RoundedCornerShape(8.dp))
                .pointerInput(slotCount) {
                    // Pinch-zoom + clamped pan.
                    detectTransformGestures { _, pan, gestureZoom, _ ->
                        val newZoom = (zoom * gestureZoom).coerceIn(MIN_ZOOM, MAX_ZOOM)
                        zoom = newZoom
                        val slotsPerPx = visibleSlots() / max(1f, canvasW)
                        panSlot -= pan.x * slotsPerPx
                        clampPan()
                    }
                }
                .pointerInput(slotCount) {
                    // Drag the nearest cursor (A/B) under the finger.
                    detectDragNearestCursor(
                        widthProvider = { canvasW },
                        visStart = { panSlot },
                        visSlots = { visibleSlots() },
                        getA = { cursorA },
                        getB = { cursorB },
                        setA = { cursorA = it },
                        setB = { cursorB = it },
                        slotCount = slotCount,
                    )
                },
        ) {
            canvasW = size.width
            canvasH = size.height
            val visStart = panSlot
            val visSlots = visibleSlots()

            drawGridAndGaps(
                slotCount = slotCount,
                spanDeg = spanDeg,
                visStart = visStart,
                visSlots = visSlots,
                gapWindows = gapWindows,
                gridColor = waveColors.grid,
                tdcColor = waveColors.tdc,
                gapColor = waveColors.gap,
            )

            drawLanes(
                table = stats.table,
                lanes = lanes,
                visStart = visStart,
                visSlots = visSlots,
                laneColorOf = { waveColors.laneColor(it) },
            )

            // Live (simulated) cursor.
            drawCursor(liveSlot, visStart, visSlots, waveColors.tdc, dashed = true)
            // Measurement cursors.
            cursorA?.let { drawCursor(it, visStart, visSlots, waveColors.cursor, dashed = false) }
            cursorB?.let { drawCursor(it, visStart, visSlots, waveColors.cursor, dashed = false) }
        }

        Spacer(Modifier.height(8.dp))

        // Transport + zoom controls.
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Button(onClick = { paused = !paused }) {
                Text(if (paused) "PLAY" else "PAUSE")
            }
            Button(onClick = {
                zoom = (zoom * 2f).coerceAtMost(MAX_ZOOM); clampPan()
            }) { Text("ZOOM +") }
            Button(onClick = {
                zoom = (zoom / 2f).coerceAtLeast(MIN_ZOOM); clampPan()
            }) { Text("ZOOM -") }
            Text(
                "${"%.1f".format(zoom)}x",
                color = HudMuted,
                fontFamily = FontFamily.Monospace,
                fontSize = 12.sp,
            )
        }

        Spacer(Modifier.height(8.dp))

        MeasurementPanel(
            stats = stats,
            lanes = lanes,
            rpm = rpm,
            liveSlot = liveSlot,
            cursorA = cursorA,
            cursorB = cursorB,
            paused = paused,
        )
    }
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------

private fun DrawScope.slotToX(slot: Double, visStart: Double, visSlots: Double): Float {
    if (visSlots <= 0.0) return 0f
    return (((slot - visStart) / visSlots) * size.width).toFloat()
}

private fun DrawScope.drawGridAndGaps(
    slotCount: Int,
    spanDeg: Int,
    visStart: Double,
    visSlots: Double,
    gapWindows: List<Pair<Int, Int>>,
    gridColor: androidx.compose.ui.graphics.Color,
    tdcColor: androidx.compose.ui.graphics.Color,
    gapColor: androidx.compose.ui.graphics.Color,
) {
    // Dim-red gap bands over missing-tooth runs (crank).
    for ((g0, g1) in gapWindows) {
        val x0 = slotToX(g0.toDouble(), visStart, visSlots)
        val x1 = slotToX(g1.toDouble(), visStart, visSlots)
        if (x1 < 0f || x0 > size.width) continue
        drawRect(
            color = gapColor,
            topLeft = Offset(max(0f, x0), 0f),
            size = androidx.compose.ui.geometry.Size(
                (min(size.width, x1) - max(0f, x0)).coerceAtLeast(0f), size.height,
            ),
        )
    }

    // 90 deg crank gridlines. For a 720 deg table, gridlines step every 90 crank
    // degrees across the full span (so 0,90,...,630,720 -> 8 intervals).
    var deg = 0
    while (deg <= spanDeg) {
        val slot = WaveformMath.degToSlot(deg.toDouble(), spanDeg, slotCount)
        val x = slotToX(slot, visStart, visSlots)
        if (x in 0f..size.width) {
            val isTdc = (deg % 360) == 0
            drawLine(
                color = if (isTdc) tdcColor else gridColor,
                start = Offset(x, 0f),
                end = Offset(x, size.height),
                strokeWidth = if (isTdc) 2f else 1f,
            )
        }
        deg += 90
    }
}

private fun DrawScope.drawLanes(
    table: ByteArray,
    lanes: List<WaveformLane>,
    visStart: Double,
    visSlots: Double,
    laneColorOf: (Int) -> androidx.compose.ui.graphics.Color,
) {
    if (lanes.isEmpty() || table.isEmpty()) return
    val laneH = size.height / lanes.size
    val widthPx = size.width.roundToInt().coerceAtLeast(1)

    lanes.forEachIndexed { idx, lane ->
        val top = idx * laneH
        val highY = top + laneH * 0.20f
        val lowY = top + laneH * 0.80f
        val color = laneColorOf(lane.channel)

        var prevY = Float.NaN
        for (col in 0 until widthPx) {
            val (s0, s1) = WaveformMath.colToSlotWindow(col, widthPx, visStart, visSlots)
            val (anyHigh, anyLow) = WaveformMath.envelopeOverWindow(table, lane.bit, s0, s1)
            val x = col.toFloat()
            when {
                anyHigh && anyLow -> {
                    // Transition column: draw a vertical edge spanning HIGH..LOW.
                    drawLine(color, Offset(x, highY), Offset(x, lowY), strokeWidth = 1.5f)
                    prevY = highY
                }
                anyHigh -> {
                    val y = highY
                    if (!prevY.isNaN() && prevY != y) drawLine(color, Offset(x, prevY), Offset(x, y), 1.5f)
                    drawLine(color, Offset(x, y), Offset(x + 1, y), strokeWidth = 1.5f)
                    prevY = y
                }
                else -> {
                    val y = lowY
                    if (!prevY.isNaN() && prevY != y) drawLine(color, Offset(x, prevY), Offset(x, y), 1.5f)
                    drawLine(color, Offset(x, y), Offset(x + 1, y), strokeWidth = 1.5f)
                    prevY = y
                }
            }
        }

        // Lane baseline separator.
        drawLine(
            color = laneColorOf(lane.channel).copy(alpha = 0.15f),
            start = Offset(0f, top + laneH),
            end = Offset(size.width, top + laneH),
            strokeWidth = 1f,
        )
    }
}

private fun DrawScope.drawCursor(
    slot: Double,
    visStart: Double,
    visSlots: Double,
    color: androidx.compose.ui.graphics.Color,
    dashed: Boolean,
) {
    val x = slotToX(slot, visStart, visSlots)
    if (x < 0f || x > size.width) return
    if (dashed) {
        var y = 0f
        while (y < size.height) {
            drawLine(color, Offset(x, y), Offset(x, min(y + 6f, size.height)), strokeWidth = 1.5f)
            y += 12f
        }
    } else {
        drawLine(color, Offset(x, 0f), Offset(x, size.height), strokeWidth = 2f)
    }
}

// ---------------------------------------------------------------------------
// Pointer: drag the nearest cursor
// ---------------------------------------------------------------------------

private suspend fun androidx.compose.ui.input.pointer.PointerInputScope.detectDragNearestCursor(
    widthProvider: () -> Float,
    visStart: () -> Double,
    visSlots: () -> Double,
    getA: () -> Double?,
    getB: () -> Double?,
    setA: (Double) -> Unit,
    setB: (Double) -> Unit,
    slotCount: Int,
) {
    var dragging: Int = 0  // 1 = A, 2 = B
    detectDragGestures(
        onDragStart = { offset: Offset ->
            val w = widthProvider().coerceAtLeast(1f)
            val slotAtPx = visStart() + (offset.x / w) * visSlots()
            val da = getA()?.let { abs(it - slotAtPx) } ?: Double.MAX_VALUE
            val db = getB()?.let { abs(it - slotAtPx) } ?: Double.MAX_VALUE
            dragging = if (da <= db) 1 else 2
        },
        onDrag = { change, _ ->
            val w = widthProvider().coerceAtLeast(1f)
            val slotAtPx = (visStart() + (change.position.x / w) * visSlots())
                .coerceIn(0.0, slotCount.toDouble())
            if (dragging == 1) setA(slotAtPx) else setB(slotAtPx)
        },
    )
}

// ---------------------------------------------------------------------------
// Measurement panel
// ---------------------------------------------------------------------------

@Composable
private fun MeasurementPanel(
    stats: DslCompiler.CompiledStats,
    lanes: List<WaveformLane>,
    rpm: Double,
    liveSlot: Double,
    cursorA: Double?,
    cursorB: Double?,
    paused: Boolean,
) {
    val slotCount = stats.slotCount
    val spanDeg = stats.degrees

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(HudSunken, RoundedCornerShape(8.dp))
            .padding(12.dp),
    ) {
        Text(
            "MEASUREMENTS",
            color = HudMuted,
            fontSize = 11.sp,
            fontFamily = FontFamily.Monospace,
        )
        Spacer(Modifier.height(4.dp))

        if (cursorA != null && cursorB != null) {
            val dtUs = WaveformMath.deltaTimeUs(cursorA, cursorB, rpm, spanDeg, slotCount)
            val toothFreq = WaveformMath.frequencyHz(rpm, spanDeg, slotCount)
            val angA = WaveformMath.crankAngleDeg(cursorA, slotCount, spanDeg)
            val angB = WaveformMath.crankAngleDeg(cursorB, slotCount, spanDeg)
            val dAng = angB - angA

            MeasureRow("Δ time", "${"%.1f".format(abs(dtUs))} µs")
            MeasureRow("tooth/edge rate", "${"%.1f".format(toothFreq)} Hz")
            MeasureRow("crank rate", "${"%.2f".format(WaveformMath.crankFreqHz(rpm))} Hz  (${rpm.roundToInt()} RPM)")
            MeasureRow("Δ crank angle", "${"%.1f".format(dAng)}°")

            // DUTY% for each lane over the A..B window.
            for (lane in lanes) {
                val duty = WaveformMath.dutyOverWindow(stats.table, lane.bit, cursorA, cursorB)
                MeasureRow("${lane.label} DUTY", "${"%.1f".format(duty)} %")
            }
        } else {
            Text("Place both cursors to measure.", color = HudMuted, fontSize = 12.sp)
        }

        Spacer(Modifier.height(6.dp))
        val liveDial = WaveformMath.crankAngleForDial(liveSlot, slotCount, spanDeg)
        MeasureRow(
            "live cursor (simulated)",
            if (paused) "paused @ ${"%.0f".format(liveDial)}°"
            else "${"%.0f".format(liveDial)}° crank",
        )

        Spacer(Modifier.height(6.dp))
        Text(
            "Digital logic levels — no voltage amplitude (HIGH/LOW = 1/0). " +
                "Live cursor is RPM-simulated, not board-phase-locked.",
            color = HudMuted,
            fontSize = 10.sp,
        )
    }
}

@Composable
private fun MeasureRow(label: String, value: String) {
    Row(
        modifier = Modifier.fillMaxWidth().padding(vertical = 1.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        Text(label, color = HudMuted, fontSize = 12.sp)
        Text(value, color = HudText, fontSize = 12.sp, fontFamily = FontFamily.Monospace)
    }
}

// Unused placeholder local to keep the compositionLocalOf import tidy if needed by callers.
internal val LocalWaveformZoom = compositionLocalOf { 1f }

@Composable
internal fun WaveformZoomProvider(zoom: Float, content: @Composable () -> Unit) {
    CompositionLocalProvider(LocalWaveformZoom provides zoom, content = content)
}
