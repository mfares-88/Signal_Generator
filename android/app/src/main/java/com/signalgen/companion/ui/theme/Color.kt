package com.signalgen.companion.ui.theme

import androidx.compose.ui.graphics.Color

// ---------------------------------------------------------------------------
// Cyan-HUD palette (plan §4.6). These mirror the on-board LVGL theme so the
// companion app feels like a natural extension of the device.
// ---------------------------------------------------------------------------
val HudBackground = Color(0xFF0B1020)
val HudSurface = Color(0xFF141C2E)
val HudSunken = Color(0xFF0F1628)
val HudAccent = Color(0xFF00E5FF)
val HudWarn = Color(0xFFFFB020)
val HudText = Color(0xFFD7E9FF)
val HudMuted = Color(0xFF7C8DB0)
val HudLedOff = Color(0xFF37425A)

// Derived helper tints
val HudOnAccent = Color(0xFF03161B)      // dark text on the filled-cyan primary
val HudError = Color(0xFFFF4060)
val HudSuccess = Color(0xFF7CFFB0)
val HudBorder = Color(0x6600E5FF)        // 40% accent for sunken-card borders
