package com.signalgen.companion.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.ExposedDropdownMenuBox
import androidx.compose.material3.ExposedDropdownMenuDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.signalgen.companion.data.DeviceState
import com.signalgen.companion.data.PatternEntry
import com.signalgen.companion.ui.theme.HudLedOff
import com.signalgen.companion.ui.theme.LocalWaveformColors
import kotlin.math.atan2
import kotlin.math.roundToInt

private const val RPM_MIN = 100
private const val RPM_MAX = 6000

@Composable
fun HomeScreen(
    state: DeviceState,
    catalog: List<PatternEntry>,
    onRpm: (Int) -> Unit,
    onStart: () -> Unit,
    onStop: () -> Unit,
    onInvert: (Boolean) -> Unit,
    onSelectBuiltin: (Int) -> Unit,
    onSelectNamed: (String) -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        // ---- RPM dial + readout ------------------------------------------
        Card(
            colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
            shape = RoundedCornerShape(12.dp),
            modifier = Modifier.fillMaxWidth(),
        ) {
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(16.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                RpmDial(
                    rpm = state.rpm,
                    onRpm = onRpm,
                    modifier = Modifier
                        .fillMaxWidth(0.72f)
                        .aspectRatio(1f),
                )
                Spacer(Modifier.height(8.dp))
                Text(
                    text = "${state.rpm}",
                    color = MaterialTheme.colorScheme.primary,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 44.sp,
                )
                Text(
                    text = "RPM   base ${state.baseRpm}",
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    style = MaterialTheme.typography.labelLarge,
                )
            }
        }

        // ---- Channel LEDs from mask --------------------------------------
        ChannelLeds(mask = state.mask, modifier = Modifier.fillMaxWidth())

        // ---- Pattern selector + filter -----------------------------------
        PatternSelector(
            catalog = catalog,
            activeKey = state.pattern,
            onSelectBuiltin = onSelectBuiltin,
            onSelectNamed = onSelectNamed,
        )

        // ---- Invert toggle -----------------------------------------------
        Card(
            colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
            shape = RoundedCornerShape(12.dp),
            modifier = Modifier.fillMaxWidth(),
        ) {
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 16.dp, vertical = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Text(
                    "INVERT outputs",
                    color = MaterialTheme.colorScheme.onSurface,
                    style = MaterialTheme.typography.titleLarge,
                    modifier = Modifier.weight(1f),
                )
                Switch(checked = state.inv, onCheckedChange = onInvert)
            }
        }

        Spacer(Modifier.weight(1f))

        // ---- START / STOP ------------------------------------------------
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Button(
                onClick = onStart,
                enabled = !state.run,
                modifier = Modifier
                    .weight(1f)
                    .height(56.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    contentColor = MaterialTheme.colorScheme.onPrimary,
                ),
            ) { Text("START", fontWeight = FontWeight.Bold) }

            Button(
                onClick = onStop,
                enabled = state.run,
                modifier = Modifier
                    .weight(1f)
                    .height(56.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.error,
                    contentColor = MaterialTheme.colorScheme.onError,
                ),
            ) { Text("STOP", fontWeight = FontWeight.Bold) }
        }
    }
}

@Composable
private fun RpmDial(
    rpm: Int,
    onRpm: (Int) -> Unit,
    modifier: Modifier = Modifier,
) {
    val accent = MaterialTheme.colorScheme.primary
    val track = MaterialTheme.colorScheme.outlineVariant
    val frac = ((rpm - RPM_MIN).toFloat() / (RPM_MAX - RPM_MIN)).coerceIn(0f, 1f)

    // Sweep 270° arc from 135° (bottom-left) clockwise to 45° (bottom-right).
    val startAngle = 135f
    val sweepRange = 270f

    Box(modifier = modifier, contentAlignment = Alignment.Center) {
        Canvas(
            modifier = Modifier
                .fillMaxSize()
                .pointerInput(Unit) {
                    detectDragGestures { change, _ ->
                        change.consume()
                        val cx = size.width / 2f
                        val cy = size.height / 2f
                        val dx = change.position.x - cx
                        val dy = change.position.y - cy
                        // angle in degrees, 0 at +x, growing clockwise (screen y down)
                        var ang = Math.toDegrees(atan2(dy.toDouble(), dx.toDouble())).toFloat()
                        if (ang < 0) ang += 360f
                        // map onto the 135°..405° sweep
                        var rel = ang - startAngle
                        if (rel < 0) rel += 360f
                        if (rel <= sweepRange) {
                            val f = (rel / sweepRange).coerceIn(0f, 1f)
                            val v = (RPM_MIN + f * (RPM_MAX - RPM_MIN)).roundToInt()
                            onRpm(v.coerceIn(RPM_MIN, RPM_MAX))
                        }
                    }
                },
        ) {
            val stroke = size.minDimension * 0.10f
            val inset = stroke / 2f
            val arcSize = androidx.compose.ui.geometry.Size(
                size.width - stroke,
                size.height - stroke,
            )
            val topLeft = Offset(inset, inset)
            // track
            drawArc(
                color = track,
                startAngle = startAngle,
                sweepAngle = sweepRange,
                useCenter = false,
                topLeft = topLeft,
                size = arcSize,
                style = Stroke(width = stroke, cap = StrokeCap.Round),
            )
            // value
            drawArc(
                brush = Brush.sweepGradient(listOf(accent.copy(alpha = 0.4f), accent)),
                startAngle = startAngle,
                sweepAngle = sweepRange * frac,
                useCenter = false,
                topLeft = topLeft,
                size = arcSize,
                style = Stroke(width = stroke, cap = StrokeCap.Round),
            )
        }
        Text(
            text = "drag",
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            style = MaterialTheme.typography.labelLarge,
        )
    }
}

@Composable
private fun ChannelLeds(mask: Int, modifier: Modifier = Modifier) {
    val wc = LocalWaveformColors.current
    val labels = listOf("CKP", "CMP1", "CMP2")
    val colors = listOf(wc.crank, wc.cam1, wc.cam2)
    Row(
        modifier = modifier,
        horizontalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        labels.forEachIndexed { i, label ->
            val on = (mask shr i) and 1 == 1
            Card(
                colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
                shape = RoundedCornerShape(12.dp),
                modifier = Modifier.weight(1f),
            ) {
                Row(
                    modifier = Modifier.padding(horizontal = 12.dp, vertical = 10.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Box(
                        modifier = Modifier
                            .size(14.dp)
                            .clip(CircleShape)
                            .background(if (on) colors[i] else HudLedOff),
                    )
                    Text(
                        label,
                        color = if (on) MaterialTheme.colorScheme.onSurface
                        else MaterialTheme.colorScheme.onSurfaceVariant,
                        style = MaterialTheme.typography.labelLarge,
                    )
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun PatternSelector(
    catalog: List<PatternEntry>,
    activeKey: String,
    onSelectBuiltin: (Int) -> Unit,
    onSelectNamed: (String) -> Unit,
) {
    var filter by remember { mutableStateOf("") }
    var expanded by remember { mutableStateOf(false) }

    val filtered = remember(catalog, filter) {
        if (filter.isBlank()) catalog
        else catalog.filter {
            it.name.contains(filter, ignoreCase = true) ||
                it.key.contains(filter, ignoreCase = true)
        }
    }
    val activeName = remember(catalog, activeKey) {
        catalog.firstOrNull { it.key == activeKey }?.name ?: activeKey.ifBlank { "—" }
    }

    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        shape = RoundedCornerShape(12.dp),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(
                "Pattern: $activeName",
                color = MaterialTheme.colorScheme.onSurface,
                style = MaterialTheme.typography.titleLarge,
            )
            OutlinedTextField(
                value = filter,
                onValueChange = { filter = it },
                singleLine = true,
                label = { Text("Filter patterns") },
                modifier = Modifier.fillMaxWidth(),
            )
            ExposedDropdownMenuBox(
                expanded = expanded,
                onExpandedChange = { expanded = it },
            ) {
                OutlinedTextField(
                    value = if (filtered.isEmpty()) "no match" else "${filtered.size} pattern(s)",
                    onValueChange = {},
                    readOnly = true,
                    label = { Text("Select") },
                    trailingIcon = { ExposedDropdownMenuDefaults.TrailingIcon(expanded = expanded) },
                    modifier = Modifier
                        .menuAnchor()
                        .fillMaxWidth(),
                )
                DropdownMenu(
                    expanded = expanded,
                    onDismissRequest = { expanded = false },
                ) {
                    filtered.forEach { p ->
                        DropdownMenuItem(
                            text = {
                                Column {
                                    Text(p.name, color = MaterialTheme.colorScheme.onSurface)
                                    Text(
                                        "${p.tier} · ${p.deg}° · slots ${p.slots}",
                                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                                        style = MaterialTheme.typography.labelLarge,
                                    )
                                }
                            },
                            onClick = {
                                expanded = false
                                if (p.tier == "builtin" && p.index >= 0) onSelectBuiltin(p.index)
                                else onSelectNamed(p.key)
                            },
                        )
                    }
                }
            }
        }
    }
}
