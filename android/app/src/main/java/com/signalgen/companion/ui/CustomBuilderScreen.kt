package com.signalgen.companion.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Checkbox
import androidx.compose.material3.FilterChip
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.signalgen.companion.dsl.ChannelSpec
import com.signalgen.companion.dsl.BuilderState
import com.signalgen.companion.dsl.DslCompiler
import com.signalgen.companion.dsl.DslLimits
import com.signalgen.companion.dsl.RunRow
import com.signalgen.companion.dsl.WheelKind
import com.signalgen.companion.ui.theme.HudAccent
import com.signalgen.companion.ui.theme.HudBorder
import com.signalgen.companion.ui.theme.HudError
import com.signalgen.companion.ui.theme.HudMuted
import com.signalgen.companion.ui.theme.HudSuccess
import com.signalgen.companion.ui.theme.HudSunken
import com.signalgen.companion.ui.theme.HudSurface
import com.signalgen.companion.ui.theme.HudText
import com.signalgen.companion.ui.theme.HudWarn
import com.signalgen.companion.ui.theme.LocalWaveformColors
import com.signalgen.companion.waveform.InteractiveWaveform
import com.signalgen.companion.waveform.WaveformLane
import kotlinx.coroutines.launch

/**
 * CustomBuilderScreen — the 5-step guided DSL builder (plan §4.7), the phone port
 * of the on-board LVGL Custom tab.
 *
 * Steps: Channels -> Subtype (S/M/A) -> Parameters (live validation, add/remove
 * rows, max 12/channel) -> Preview (embedded [InteractiveWaveform] from the LOCAL
 * compile + slots/span/char-count) -> Apply (recap + emitted DSL + APPLY /
 * APPLY+START).
 *
 * APPLY sends the emitted DSL to the board: the caller wires [onApply] to
 * `DeviceRepository.loadDsl(dsl)` and [onApplyStart] to `loadDsl(dsl)` then
 * `start()` (consumed via the ViewModel/ServiceLocator N2 defines). The screen
 * itself is repository-agnostic so it links regardless of N1/N2 wiring order.
 */
@Composable
fun CustomBuilderScreen(
    onApply: suspend (dsl: String) -> Unit,
    onApplyStart: suspend (dsl: String) -> Unit,
    modifier: Modifier = Modifier,
    initialState: BuilderState = BuilderState(),
) {
    var state by remember { mutableStateOf(initialState) }
    val scope = rememberCoroutineScope()
    var applyMsg by remember { mutableStateOf<String?>(null) }

    val validation = remember(state) { DslCompiler.validate(state) }
    val dsl = remember(state) { DslCompiler.emitDsl(state) }

    Column(modifier = modifier.fillMaxSize().padding(12.dp)) {
        StepStrip(step = state.step)
        Spacer(Modifier.height(8.dp))

        Box(
            modifier = Modifier
                .weight(1f)
                .fillMaxWidth()
                .verticalScroll(rememberScrollState()),
        ) {
            when (state.step) {
                0 -> ChannelsStep(state) { state = it }
                1 -> SubtypeStep(state) { state = it }
                2 -> ParametersStep(state, validation) { state = it }
                3 -> PreviewStep(state, validation, dsl)
                else -> ApplyStep(
                    state = state,
                    validation = validation,
                    dsl = dsl,
                    applyMsg = applyMsg,
                    onApply = {
                        scope.launch {
                            runCatching { onApply(dsl) }
                                .onSuccess { applyMsg = "Applied ${dsl.length} chars" }
                                .onFailure { applyMsg = "Apply failed: ${it.message}" }
                        }
                    },
                    onApplyStart = {
                        scope.launch {
                            runCatching { onApplyStart(dsl) }
                                .onSuccess { applyMsg = "Applied + started" }
                                .onFailure { applyMsg = "Apply failed: ${it.message}" }
                        }
                    },
                )
            }
        }

        Spacer(Modifier.height(8.dp))
        NavBar(
            step = state.step,
            canAdvance = state.step < 2 || validation.ok,
            onBack = { if (state.step > 0) state = state.copy(step = state.step - 1) },
            onNext = { if (state.step < DslLimits.N_CHANNELS + 2) state = state.copy(step = state.step + 1) },
        )
    }
}

private val STEP_LABELS = listOf("Channels", "Subtype", "Parameters", "Preview", "Apply")

@Composable
private fun StepStrip(step: Int) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(6.dp),
    ) {
        STEP_LABELS.forEachIndexed { i, label ->
            val active = i == step
            val done = i < step
            Box(
                modifier = Modifier
                    .weight(1f)
                    .clip(RoundedCornerShape(8.dp))
                    .background(if (active) HudAccent else HudSunken)
                    .padding(vertical = 6.dp),
                contentAlignment = Alignment.Center,
            ) {
                Text(
                    "${i + 1}·$label",
                    color = when {
                        active -> com.signalgen.companion.ui.theme.HudOnAccent
                        done -> HudSuccess
                        else -> HudMuted
                    },
                    fontSize = 11.sp,
                )
            }
        }
    }
}

@Composable
private fun NavBar(step: Int, canAdvance: Boolean, onBack: () -> Unit, onNext: () -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        OutlinedButton(onClick = onBack, enabled = step > 0, modifier = Modifier.weight(1f)) {
            Text("BACK")
        }
        Button(
            onClick = onNext,
            enabled = step < STEP_LABELS.size - 1 && canAdvance,
            modifier = Modifier.weight(1f),
        ) {
            Text("NEXT")
        }
    }
}

// ---------------------------------------------------------------------------
// Step 1 — Channels
// ---------------------------------------------------------------------------

@Composable
private fun ChannelsStep(state: BuilderState, onChange: (BuilderState) -> Unit) {
    SunkenCard {
        Text("Enable channels", color = HudText, fontSize = 16.sp)
        Spacer(Modifier.height(8.dp))
        for (ch in 0 until DslLimits.N_CHANNELS) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Checkbox(
                    checked = state.isEnabled(ch),
                    enabled = ch != 0, // crank always on
                    onCheckedChange = { on ->
                        onChange(state.withChannel(ch, state.channels[ch].copy(enabled = on)))
                    },
                )
                Text(
                    "${BuilderState.labelOf(ch)}  (pin ${BuilderState.pinOf(ch)}, " +
                        "bit ${BuilderState.bitOf(ch)}, ${if (ch == 0) "CW 360°" else "CCW 720°"})",
                    color = if (state.isEnabled(ch)) HudText else HudMuted,
                    fontSize = 13.sp,
                )
            }
        }
        Spacer(Modifier.height(8.dp))
        Text(
            "Any cam (CMP1/CMP2) makes the whole pattern 720° (two crank revolutions).",
            color = HudMuted, fontSize = 11.sp,
        )
    }
}

// ---------------------------------------------------------------------------
// Step 2 — Subtype
// ---------------------------------------------------------------------------

@Composable
private fun SubtypeStep(state: BuilderState, onChange: (BuilderState) -> Unit) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        for (ch in 0 until DslLimits.N_CHANNELS) {
            if (!state.isEnabled(ch)) continue
            SunkenCard {
                Text(BuilderState.labelOf(ch), color = HudAccent, fontSize = 14.sp)
                Spacer(Modifier.height(6.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    WheelKind.entries.forEach { k ->
                        FilterChip(
                            selected = state.channels[ch].kind == k,
                            onClick = { onChange(state.withChannel(ch, state.channels[ch].copy(kind = k))) },
                            label = { Text(subtypeLabel(k)) },
                        )
                    }
                }
            }
        }
    }
}

private fun subtypeLabel(k: WheelKind) = when (k) {
    WheelKind.SYM -> "Symmetric"
    WheelKind.MISS -> "Missing"
    WheelKind.ANG -> "Angular"
}

// ---------------------------------------------------------------------------
// Step 3 — Parameters (live validation + add/remove rows)
// ---------------------------------------------------------------------------

@Composable
private fun ParametersStep(
    state: BuilderState,
    validation: DslCompiler.ValidationResult,
    onChange: (BuilderState) -> Unit,
) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        for (ch in 0 until DslLimits.N_CHANNELS) {
            if (!state.isEnabled(ch)) continue
            val spec = state.channels[ch]
            SunkenCard {
                Text("${BuilderState.labelOf(ch)} — ${subtypeLabel(spec.kind)}",
                    color = HudAccent, fontSize = 14.sp)
                Spacer(Modifier.height(6.dp))

                if (spec.kind == WheelKind.SYM || spec.kind == WheelKind.MISS) {
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        NumField("n", spec.dutyNum, Modifier.weight(1f)) {
                            onChange(state.withChannel(ch, spec.copy(dutyNum = it)))
                        }
                        NumField("d", spec.dutyDen, Modifier.weight(1f)) {
                            onChange(state.withChannel(ch, spec.copy(dutyDen = it)))
                        }
                        NumField("teeth", spec.teeth, Modifier.weight(1f)) {
                            onChange(state.withChannel(ch, spec.copy(teeth = it)))
                        }
                    }
                }

                if (spec.kind == WheelKind.MISS) {
                    Spacer(Modifier.height(6.dp))
                    Text("Run rows (t = teeth, m = gap)", color = HudMuted, fontSize = 11.sp)
                    spec.runs.forEachIndexed { i, r ->
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            NumField("#${i + 1}", r.count, Modifier.width(110.dp)) { c ->
                                onChange(state.withChannel(ch, spec.copy(
                                    runs = spec.runs.toMutableList().also { it[i] = r.copy(count = c) },
                                )))
                            }
                            Spacer(Modifier.width(6.dp))
                            FilterChip(
                                selected = r.suffix == 't',
                                onClick = {
                                    val s = if (r.suffix == 't') 'm' else 't'
                                    onChange(state.withChannel(ch, spec.copy(
                                        runs = spec.runs.toMutableList().also { it[i] = r.copy(suffix = s) },
                                    )))
                                },
                                label = { Text(if (r.suffix == 't') "t" else "m") },
                            )
                            Spacer(Modifier.width(6.dp))
                            OutlinedButton(
                                onClick = {
                                    if (spec.runs.size > 1) onChange(state.withChannel(ch, spec.copy(
                                        runs = spec.runs.toMutableList().also { it.removeAt(i) },
                                    )))
                                },
                                contentPadding = androidx.compose.foundation.layout.PaddingValues(8.dp),
                            ) { Text("–") }
                        }
                    }
                    AddRowButton(enabled = spec.runs.size < DslLimits.MAX_DYN_ROWS) {
                        onChange(state.withChannel(ch, spec.copy(runs = spec.runs + RunRow(1, 't'))))
                    }
                }

                if (spec.kind == WheelKind.ANG) {
                    Spacer(Modifier.height(6.dp))
                    Text("Degree rows (alternating HIGH/LOW, starts HIGH; sum = ${state.spanFor(ch)}°)",
                        color = HudMuted, fontSize = 11.sp)
                    spec.angular.forEachIndexed { i, deg ->
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            NumField(if (i % 2 == 0) "HIGH ${i + 1}" else "LOW ${i + 1}", deg, Modifier.width(150.dp)) { v ->
                                onChange(state.withChannel(ch, spec.copy(
                                    angular = spec.angular.toMutableList().also { it[i] = v },
                                )))
                            }
                            Spacer(Modifier.width(6.dp))
                            OutlinedButton(
                                onClick = {
                                    if (spec.angular.size > 1) onChange(state.withChannel(ch, spec.copy(
                                        angular = spec.angular.toMutableList().also { it.removeAt(i) },
                                    )))
                                },
                                contentPadding = androidx.compose.foundation.layout.PaddingValues(8.dp),
                            ) { Text("–") }
                        }
                    }
                    AddRowButton(enabled = spec.angular.size < DslLimits.MAX_DYN_ROWS) {
                        onChange(state.withChannel(ch, spec.copy(angular = spec.angular + 0)))
                    }
                }

                // Live per-channel validation message.
                val chErr = validation.errors.firstOrNull { it.channel == ch }
                if (chErr != null) {
                    Spacer(Modifier.height(4.dp))
                    Text("⚠ ${chErr.message}", color = HudWarn, fontSize = 11.sp)
                }
            }
        }

        // Group-level / char-count footer.
        CharCounterRow(validation)
    }
}

@Composable
private fun AddRowButton(enabled: Boolean, onClick: () -> Unit) {
    OutlinedButton(onClick = onClick, enabled = enabled) {
        Text(if (enabled) "+ Add row" else "max ${DslLimits.MAX_DYN_ROWS} rows")
    }
}

@Composable
private fun NumField(label: String, value: Int, modifier: Modifier = Modifier, onValue: (Int) -> Unit) {
    OutlinedTextField(
        value = value.toString(),
        onValueChange = { txt -> onValue(txt.filter { it.isDigit() }.toIntOrNull() ?: 0) },
        label = { Text(label, fontSize = 11.sp) },
        singleLine = true,
        modifier = modifier,
    )
}

// ---------------------------------------------------------------------------
// Step 4 — Preview
// ---------------------------------------------------------------------------

@Composable
private fun PreviewStep(
    state: BuilderState,
    validation: DslCompiler.ValidationResult,
    dsl: String,
) {
    val stats = remember(state) { DslCompiler.compiledStats(state) }
    val lanes = remember(state, stats) { buildLanes(state) }
    val gaps = remember(state, stats) { gapWindows(state, stats) }

    Column {
        SunkenCard {
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                StatChip("slots", if (stats.ok) "${stats.slotCount}" else "—")
                StatChip("span", "${stats.degrees}°")
                StatChip("chars", "${validation.dslLength}/${DslLimits.SRC_MAX}")
            }
        }
        Spacer(Modifier.height(8.dp))

        if (stats.ok && validation.ok) {
            InteractiveWaveform(
                stats = stats,
                lanes = lanes,
                rpm = 1000.0,            // preview reference RPM (timing is RPM-scaled).
                gapWindows = gaps,
                modifier = Modifier.fillMaxWidth(),
            )
        } else {
            SunkenCard {
                Text(
                    validation.firstError?.let { "⚠ ${labelFor(it.channel)} ${it.message}" }
                        ?: stats.error ?: "Pattern not valid yet",
                    color = HudError, fontSize = 13.sp,
                )
            }
        }
    }
}

@Composable
private fun StatChip(label: String, value: String) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(label, color = HudMuted, fontSize = 10.sp)
        Text(value, color = HudText, fontSize = 16.sp, fontFamily = FontFamily.Monospace)
    }
}

// ---------------------------------------------------------------------------
// Step 5 — Apply
// ---------------------------------------------------------------------------

@Composable
private fun ApplyStep(
    state: BuilderState,
    validation: DslCompiler.ValidationResult,
    dsl: String,
    applyMsg: String?,
    onApply: () -> Unit,
    onApplyStart: () -> Unit,
) {
    val stats = remember(state) { DslCompiler.compiledStats(state) }
    Column {
        SunkenCard {
            Text("Recap", color = HudAccent, fontSize = 14.sp)
            Spacer(Modifier.height(4.dp))
            for (ch in 0 until DslLimits.N_CHANNELS) {
                if (!state.isEnabled(ch)) continue
                Text("${BuilderState.labelOf(ch)}: ${subtypeLabel(state.channels[ch].kind)}",
                    color = HudText, fontSize = 12.sp)
            }
            Spacer(Modifier.height(6.dp))
            Text("${stats.slotCount} slots · ${stats.degrees}° · ${validation.dslLength}/${DslLimits.SRC_MAX} chars",
                color = HudMuted, fontSize = 12.sp)
        }
        Spacer(Modifier.height(8.dp))
        SunkenCard {
            Text("Emitted DSL", color = HudMuted, fontSize = 11.sp)
            Spacer(Modifier.height(4.dp))
            Text(dsl, color = HudText, fontSize = 12.sp, fontFamily = FontFamily.Monospace)
        }
        Spacer(Modifier.height(8.dp))

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onApply, enabled = validation.ok, modifier = Modifier.weight(1f)) {
                Text("APPLY")
            }
            Button(
                onClick = onApplyStart,
                enabled = validation.ok,
                colors = ButtonDefaults.buttonColors(containerColor = HudSuccess),
                modifier = Modifier.weight(1f),
            ) { Text("APPLY + START") }
        }

        if (!validation.ok) {
            Spacer(Modifier.height(6.dp))
            Text("⚠ ${validation.firstError?.message ?: "invalid"}", color = HudError, fontSize = 12.sp)
        }
        applyMsg?.let {
            Spacer(Modifier.height(6.dp))
            Text(it, color = HudSuccess, fontSize = 12.sp)
        }
    }
}

// ---------------------------------------------------------------------------
// Shared bits
// ---------------------------------------------------------------------------

@Composable
private fun CharCounterRow(validation: DslCompiler.ValidationResult) {
    val over = validation.dslLength > DslLimits.SRC_MAX
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        Text(if (validation.ok) "✓ valid" else "⚠ ${validation.errors.size} issue(s)",
            color = if (validation.ok) HudSuccess else HudWarn, fontSize = 12.sp)
        Text("${validation.dslLength}/${DslLimits.SRC_MAX} chars",
            color = if (over) HudError else HudMuted, fontSize = 12.sp, fontFamily = FontFamily.Monospace)
    }
}

@Composable
private fun SunkenCard(content: @Composable androidx.compose.foundation.layout.ColumnScope.() -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(12.dp))
            .background(HudSurface)
            .border(1.dp, HudBorder, RoundedCornerShape(12.dp))
            .padding(12.dp),
        content = content,
    )
}

private fun labelFor(channel: Int): String =
    if (channel in 0 until DslLimits.N_CHANNELS) BuilderState.labelOf(channel) else "group"

/** Build the visible lanes for the enabled channels. */
internal fun buildLanes(state: BuilderState): List<WaveformLane> =
    (0 until DslLimits.N_CHANNELS)
        .filter { state.isEnabled(it) }
        .map { WaveformLane(it, BuilderState.bitOf(it), BuilderState.labelOf(it)) }

/**
 * Compute the crank's missing-run gap windows in COMPILED slot space for the
 * dim-red bands. We re-expand the crank wheel (channel 0) the same way the
 * compiler does, then map its native gap ranges onto the LCM-stretched table.
 *
 * Because the compiler canonicalizes (rotates) the merged table, exact alignment
 * to the rotated table is approximate; the bands are a visual aid over the crank
 * low-level (gap) regions, which is their intended purpose.
 */
internal fun gapWindows(state: BuilderState, stats: DslCompiler.CompiledStats): List<Pair<Int, Int>> {
    if (!stats.ok || stats.slotCount <= 0) return emptyList()
    val crank = state.channels[0]
    if (crank.kind != WheelKind.MISS) return emptyList()
    val is720 = state.hasCam()
    val crankVec = DslCompiler.expandWheel(0, state, is720)
    if (crankVec.isEmpty()) return emptyList()

    // Map native crank-low runs onto the LCM table length by uniform stretch.
    val l = stats.slotCount
    val base = crankVec.size
    val windows = mutableListOf<Pair<Int, Int>>()
    var i = 0
    while (i < base) {
        if (crankVec[i] == 0) {
            var j = i
            while (j < base && crankVec[j] == 0) j++
            val s0 = (i.toLong() * l / base).toInt()
            val s1 = (j.toLong() * l / base).toInt()
            if (s1 > s0) windows.add(s0 to s1)
            i = j
        } else i++
    }
    return windows
}
