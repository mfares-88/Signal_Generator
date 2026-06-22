// lib/wifi_link/wifi_creds.h — Cycle 7, Agent F.
//
// Thin Preferences (NVS) wrapper for WiFi STA credentials, stored in a
// namespace ("wifi") DISTINCT from the existing "siggen" namespace (plan D6).
// ~100 B; fits the existing `nvs` partition — no resize.

#pragma once

#include <stddef.h>

namespace WifiCreds {

// Max stored lengths (SSID <= 32, WPA2 passphrase <= 63 per 802.11) + NUL.
static constexpr size_t SSID_BUFLEN = 33;
static constexpr size_t PASS_BUFLEN = 64;

// Load the provisioned SSID/pass into the caller's buffers. Returns true
// only when a provisioned flag is set AND a non-empty SSID was read.
// On false, ssid/pass are set to empty strings.
bool load(char* ssid, size_t ssidLen, char* pass, size_t passLen);

// Persist SSID + pass and set provisioned=true. Returns true on success.
bool save(const char* ssid, const char* pass);

// Clear all stored creds and the provisioned flag (used by forgetWifi).
void clear();

// True iff a provisioned flag is currently set in NVS.
bool isProvisioned();

}  // namespace WifiCreds
