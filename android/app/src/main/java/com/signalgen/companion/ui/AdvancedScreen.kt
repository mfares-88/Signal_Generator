package com.signalgen.companion.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.selection.selectable
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.RadioButton
import androidx.compose.material3.Slider
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.ui.unit.dp
import com.signalgen.companion.data.DeviceState
import com.signalgen.companion.data.PatternEntry
import kotlin.math.roundToInt

// Sweep mode codes — MUST match firmware SweepMode enum
// (SweepCompression.h): OFF=0 LINEAR=1 LOG=2 WAYPOINT=3.
private val SWEEP_MODES = listOf("OFF" to 0, "LINEAR" to 1, "LOG" to 2, "WAYPOINT" to 3)

@Composable
fun AdvancedScreen(
    state: DeviceState,
    catalog: List<PatternEntry>,
    onSweep: (low: Int, high: Int, mode: Int, intervalUs: Long) -> Unit,
    onCompression: (enabled: Boolean, cyl: Int, thresh: Int, peak: Int, dynamic: Boolean) -> Unit,
    onSelectBuiltin: (Int) -> Unit,
    onSelectNamed: (String) -> Unit,
    onCaptureStart: (Int) -> Unit,
    onCaptureStop: () -> Unit,
    onOpenWaveform: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        SweepCard(onSweep = onSweep)
        CompressionCard(onCompression = onCompression)
        PatternLibraryCard(
            catalog = catalog,
            activeKey = state.pattern,
            onSelectBuiltin = onSelectBuiltin,
            onSelectNamed = onSelectNamed,
        )
        CaptureCard(
            running = state.run,
            onCaptureStart = onCaptureStart,
            onCaptureStop = onCaptureStop,
        )
        Button(
            onClick = onOpenWaveform,
            modifier = Modifier
                .fillMaxWidth()
                .height(52.dp),
            colors = ButtonDefaults.buttonColors(
                containerColor = MaterialTheme.colorScheme.primary,
                contentColor = MaterialTheme.colorScheme.onPrimary,
            ),
        ) { Text("LIVE WAVEFORM / CUSTOM BUILDER", fontWeight = FontWeight.Bold) }
    }
}

@Composable
private fun SectionCard(title: String, content: @Composable () -> Unit) {
    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        shape = RoundedCornerShape(12.dp),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Text(title, color = MaterialTheme.colorScheme.primary, style = MaterialTheme.typography.titleLarge)
            content()
        }
    }
}

@Composable
private fun SweepCard(onSweep: (Int, Int, Int, Long) -> Unit) {
    var lo by remember { mutableIntStateOf(600) }
    var hi by remember { mutableIntStateOf(6000) }
    var mode by remember { mutableIntStateOf(0) }
    var intervalMs by remember { mutableStateOf("2000") }

    SectionCard("Sweep") {
        Text("Low: $lo RPM", color = MaterialTheme.colorScheme.onSurface)
        Slider(
            value = lo.toFloat(),
            onValueChange = { lo = it.roundToInt().coerceIn(100, hi) },
            valueRange = 100f..6000f,
        )
        Text("High: $hi RPM", color = MaterialTheme.colorScheme.onSurface)
        Slider(
            value = hi.toFloat(),
            onValueChange = { hi = it.roundToInt().coerceIn(lo, 6000) },
            valueRange = 100f..6000f,
        )
        Text("Mode", color = MaterialTheme.colorScheme.onSurfaceVariant)
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            SWEEP_MODES.forEach { (label, code) ->
                FilterChip(
                    selected = mode == code,
                    onClick = { mode = code },
                    label = { Text(label) },
                )
            }
        }
        if (mode == 3) {
            Text(
                "WAYPOINT is configured on-device (deferred over the wire in v1).",
                color = MaterialTheme.colorScheme.secondary,
                style = MaterialTheme.typography.labelLarge,
            )
        }
        OutlinedTextField(
            value = intervalMs,
            onValueChange = { intervalMs = it.filter(Char::isDigit) },
            label = { Text("Step interval (ms)") },
            singleLine = true,
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            modifier = Modifier.fillMaxWidth(),
        )
        Button(
            onClick = {
                val ivUs = (intervalMs.toLongOrNull() ?: 2000L) * 1000L
                onSweep(lo, hi, mode, ivUs)
            },
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.buttonColors(
                containerColor = MaterialTheme.colorScheme.primary,
                contentColor = MaterialTheme.colorScheme.onPrimary,
            ),
        ) { Text("APPLY SWEEP", fontWeight = FontWeight.Bold) }
    }
}

@Composable
private fun CompressionCard(onCompression: (Boolean, Int, Int, Int, Boolean) -> Unit) {
    var enabled by remember { mutableStateOf(false) }
    var cyl by remember { mutableIntStateOf(4) }
    var thresh by remember { mutableIntStateOf(3000) }
    var peak by remember { mutableIntStateOf(100) }
    var dynamic by remember { mutableStateOf(false) }

    SectionCard("Compression") {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("Enabled", color = MaterialTheme.colorScheme.onSurface, modifier = Modifier.weight(1f))
            Switch(checked = enabled, onCheckedChange = { enabled = it })
        }
        Text("Cylinders: $cyl", color = MaterialTheme.colorScheme.onSurface)
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            (1..4).forEach { c ->
                FilterChip(selected = cyl == c, onClick = { cyl = c }, label = { Text("$c") })
            }
        }
        Text("RPM threshold: $thresh", color = MaterialTheme.colorScheme.onSurface)
        Slider(
            value = thresh.toFloat(),
            onValueChange = { thresh = it.roundToInt() },
            valueRange = 100f..6000f,
        )
        Text("Peak: $peak%", color = MaterialTheme.colorScheme.onSurface)
        Slider(
            value = peak.toFloat(),
            onValueChange = { peak = it.roundToInt() },
            valueRange = 0f..100f,
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("Dynamic", color = MaterialTheme.colorScheme.onSurface, modifier = Modifier.weight(1f))
            Switch(checked = dynamic, onCheckedChange = { dynamic = it })
        }
        Button(
            onClick = { onCompression(enabled, cyl, thresh, peak, dynamic) },
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.buttonColors(
                containerColor = MaterialTheme.colorScheme.primary,
                contentColor = MaterialTheme.colorScheme.onPrimary,
            ),
        ) { Text("APPLY COMPRESSION", fontWeight = FontWeight.Bold) }
    }
}

@Composable
private fun PatternLibraryCard(
    catalog: List<PatternEntry>,
    activeKey: String,
    onSelectBuiltin: (Int) -> Unit,
    onSelectNamed: (String) -> Unit,
) {
    SectionCard("Pattern Library") {
        if (catalog.isEmpty()) {
            Text(
                "No catalog yet — connect and refresh.",
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        } else {
            // Bounded height so it scrolls inside the outer column.
            LazyColumn(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(220.dp),
                verticalArrangement = Arrangement.spacedBy(4.dp),
            ) {
                items(catalog, key = { it.key.ifBlank { "b${it.index}" } }) { p ->
                    val selected = p.key == activeKey
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .selectable(
                                selected = selected,
                                onClick = {
                                    if (p.tier == "builtin" && p.index >= 0) onSelectBuiltin(p.index)
                                    else onSelectNamed(p.key)
                                },
                            )
                            .padding(vertical = 6.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        RadioButton(selected = selected, onClick = null)
                        Column(Modifier.padding(start = 8.dp)) {
                            Text(p.name, color = MaterialTheme.colorScheme.onSurface)
                            Text(
                                "${p.tier} · ${p.deg}° · mask ${p.mask} · slots ${p.slots}",
                                color = MaterialTheme.colorScheme.onSurfaceVariant,
                                style = MaterialTheme.typography.labelLarge,
                            )
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun CaptureCard(
    running: Boolean,
    onCaptureStart: (Int) -> Unit,
    onCaptureStop: () -> Unit,
) {
    var revs by remember { mutableIntStateOf(2) }
    SectionCard("Capture") {
        Text("Revolutions: $revs", color = MaterialTheme.colorScheme.onSurface)
        Slider(
            value = revs.toFloat(),
            onValueChange = { revs = it.roundToInt().coerceIn(1, 16) },
            valueRange = 1f..16f,
        )
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Button(
                onClick = { onCaptureStart(revs) },
                enabled = running,
                modifier = Modifier.weight(1f),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    contentColor = MaterialTheme.colorScheme.onPrimary,
                ),
            ) { Text("CAPTURE") }
            OutlinedButton(
                onClick = onCaptureStop,
                modifier = Modifier.weight(1f),
            ) { Text("STOP CAPTURE") }
        }
        if (!running) {
            Text(
                "Start the generator first to capture.",
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                style = MaterialTheme.typography.labelLarge,
            )
        }
    }
}
