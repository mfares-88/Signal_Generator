// lib/wifi_link/wifi_creds.cpp — Cycle 7, Agent F. See wifi_creds.h.

#include "wifi_creds.h"

#include <string.h>

#include <Preferences.h>

namespace {
constexpr const char* kNs        = "wifi";   // distinct from "siggen" (D6)
constexpr const char* kKeySsid   = "ssid";
constexpr const char* kKeyPass   = "pass";
constexpr const char* kKeyProv   = "prov";
}  // namespace

namespace WifiCreds {

bool load(char* ssid, size_t ssidLen, char* pass, size_t passLen) {
  if (ssid && ssidLen) ssid[0] = '\0';
  if (pass && passLen) pass[0] = '\0';

  Preferences prefs;
  if (!prefs.begin(kNs, /*readOnly=*/true)) return false;

  const bool provisioned = prefs.getBool(kKeyProv, false);
  if (!provisioned) { prefs.end(); return false; }

  // getString writes a NUL-terminated string and returns the byte count.
  if (ssid && ssidLen) prefs.getString(kKeySsid, ssid, ssidLen);
  if (pass && passLen) prefs.getString(kKeyPass, pass, passLen);
  prefs.end();

  return ssid && ssid[0] != '\0';
}

bool save(const char* ssid, const char* pass) {
  if (!ssid || ssid[0] == '\0') return false;

  Preferences prefs;
  if (!prefs.begin(kNs, /*readOnly=*/false)) return false;

  prefs.putString(kKeySsid, ssid);
  prefs.putString(kKeyPass, pass ? pass : "");
  prefs.putBool(kKeyProv, true);
  prefs.end();
  return true;
}

void clear() {
  Preferences prefs;
  if (!prefs.begin(kNs, /*readOnly=*/false)) return;
  prefs.clear();
  prefs.end();
}

bool isProvisioned() {
  Preferences prefs;
  if (!prefs.begin(kNs, /*readOnly=*/true)) return false;
  const bool p = prefs.getBool(kKeyProv, false);
  prefs.end();
  return p;
}

}  // namespace WifiCreds
