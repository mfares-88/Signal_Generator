package com.signalgen.companion.ui.theme

import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Shapes
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.Immutable
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp

// ---------------------------------------------------------------------------
// WaveformColors — shared lane/overlay palette for the interactive waveform.
// Agent N3's InteractiveWaveform imports this object (top-level singleton) and
// may also pull the per-composition instance via LocalWaveformColors.
// Values are LOCKED by plan §4.6 / §4.7.
// ---------------------------------------------------------------------------
@Immutable
data class WaveformColors(
    val crank: Color = Color(0xFF00E5FF),   // channel 0 / CKP
    val cam1: Color = Color(0xFFFFB020),    // channel 1 / CMP1
    val cam2: Color = Color(0xFF7CFFB0),    // channel 2 / CMP2
    val cursor: Color = Color(0xFFFF4060),
    val gap: Color = Color(0xFF3A2030),     // dim-red gap bands over missing runs
    val grid: Color = Color(0xFF1B2438),    // 90° gridlines
    val tdc: Color = Color(0xFF00E5FF),     // TDC marker at 0° == accent
) {
    /** Lane color by channel index (0=crank, 1=cam1, 2=cam2); wraps for safety. */
    fun laneColor(channel: Int): Color = when (channel % 3) {
        0 -> crank
        1 -> cam1
        else -> cam2
    }
}

/** Top-level default instance — importable directly by N3 if no composition is in scope. */
val DefaultWaveformColors = WaveformColors()

/** CompositionLocal so composables can read the active waveform palette. */
val LocalWaveformColors = staticCompositionLocalOf { DefaultWaveformColors }

// ---------------------------------------------------------------------------
// Material3 dark color scheme (cyan-HUD).
// ---------------------------------------------------------------------------
private val SignalGenColorScheme = darkColorScheme(
    primary = HudAccent,
    onPrimary = HudOnAccent,
    primaryContainer = HudSurface,
    onPrimaryContainer = HudText,
    secondary = HudWarn,
    onSecondary = HudOnAccent,
    background = HudBackground,
    onBackground = HudText,
    surface = HudSurface,
    onSurface = HudText,
    surfaceVariant = HudSunken,
    onSurfaceVariant = HudMuted,
    outline = HudBorder,
    outlineVariant = HudLedOff,
    error = HudError,
    onError = HudOnAccent,
)

private val SignalGenShapes = Shapes(
    extraSmall = RoundedCornerShape(8.dp),
    small = RoundedCornerShape(8.dp),
    medium = RoundedCornerShape(12.dp),
    large = RoundedCornerShape(12.dp),
)

@Composable
fun SignalGenTheme(
    content: @Composable () -> Unit,
) {
    androidx.compose.runtime.CompositionLocalProvider(
        LocalWaveformColors provides DefaultWaveformColors,
    ) {
        MaterialTheme(
            colorScheme = SignalGenColorScheme,
            typography = HudTypography,
            shapes = SignalGenShapes,
            content = content,
        )
    }
}
