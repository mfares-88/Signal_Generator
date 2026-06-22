// lib/wifi_link/wifi_link.h — Cycle 7, Agent F.
//
// Public API for the WiFi companion-link transport. This is a SECOND
// transport onto the existing manager queue (gCtrlQ) — never a new backend.
// It mirrors serial_cli.cpp's role over TCP: inbound NDJSON command frames
// become sendCtrlMsg() calls; outbound NDJSON telemetry is pushed at 5–10 Hz.
//
// The header is intentionally self-contained: it includes ONLY <stdint.h>
// and references NO main.cpp globals. All telemetry is read through the
// function-pointer hooks in WifiLinkTelemetry, populated by Agent E.
//
// Threading: wifiLinkInit() spawns ONE FreeRTOS task (Core 0, prio 1) that
// owns all WiFi / NetworkServer / NetworkClient / mDNS state. No other code
// touches that state. The generator is a hardware gptimer IRAM ISR and is
// immune to this task's scheduling (plan §2.1).

#pragma once

#include <stdint.h>

// Telemetry + link hooks. Every field is a function pointer that the
// wifi_link task calls (from Core 0) to read a consistent-enough snapshot
// of generator state without reaching into main.cpp globals directly.
//
// Reads are unguarded single-word loads (plan D8); per-frame snapshot
// consistency at 5–10 Hz is acceptable for these non-safety-critical fields.
//
// Any hook MAY be null; the task must null-check before calling.
struct WifiLinkTelemetry {
  bool        (*isRunning)();         // gRunning
  uint32_t    (*currentRpm)();        // sweepCurrentRpm() (live, post-sweep)
  uint32_t    (*baseRpm)();           // g_rpm (commanded base)
  const char* (*activePatternKey)();  // gActivePattern->name_key (.rodata, may be null)
  uint16_t    (*activeDegrees)();     // gActivePattern->degrees (360/720)
  uint8_t     (*channelMask)();       // gActivePattern->channel_mask
  bool        (*inverted)();          // gInverted
  uint16_t    (*edgeCounter)();       // gGen.getEdgeCounter() (wrapping u16, D15)
  uint32_t    (*cycleDurationUs)();   // gGen.getCycleDurationUs() (full-table period)
  uint32_t    (*dropCount)();         // gUiMsgDropCount (read under mux by hook)
  const char* (*dslError)();          // g_dsl_error ("" when none)

  // Pushes the live link state to the on-board LCD label without this lib
  // ever including LVGL headers. Called on every interface IP change.
  //   ip   : dotted-quad string ("" while connecting)
  //   ssid : home SSID (STA) or SoftAP SSID (AP)
  //   mode : "AP" | "STA" | "CONNECTING" | "DISCONNECTED"
  void        (*onLink)(const char* ip, const char* ssid, const char* mode);
};

// One-shot init. Copies the hook struct (the pointer need not outlive the
// call), spawns the wifi_link task (Core 0, prio 1), and brings up WiFi
// (STA-with-timeout from stored creds, else SoftAP fallback). Safe to call
// exactly once from setup() after gCtrlQ exists and the generator started.
void wifiLinkInit(const WifiLinkTelemetry* hooks);
