// lib/wifi_link/wifi_link.cpp — Cycle 7, Agent F. See wifi_link.h.
//
// One FreeRTOS task (Core 0, prio 1) owns ALL WiFi / NetworkServer /
// NetworkClient / mDNS state. Inbound NDJSON command frames become
// sendCtrlMsg() calls (via cmd_dispatch); link/query frames are handled here;
// outbound telemetry is pushed at ~8 Hz. Zero per-tick heap.
//
// WiFi bring-up is EVENT-DRIVEN (WiFi.onEvent), not status-polling: the event
// handler sets file-static flags that the task loop observes. STA join has a
// ~15 s timeout; on timeout/disconnect we fall back to SoftAP. mDNS is started
// ONLY after the active interface has an IP (GOT_IP handler for STA; right
// after softAP() for AP), per plan §4.1.
//
// Single-radio reality (plan §2.2/§2.4): provisioning ACKs IMMEDIATELY, then
// WiFi.begin(STA); the SoftAP client socket is EXPECTED to drop at STA
// associate — that is normal and handled.
//
// API verified against arduino-esp32 3.3.4 framework sources:
//   NetworkServer::accept()/hasClient()/setNoDelay() (NetworkServer.h:50-58)
//   NetworkClient::connected()/available()/read()/write()/stop()/setNoDelay()
//   WiFi.softAP(ssid,pass,...) return-checked (WiFiAP.h:84-93, D14)
//   WiFi.onEvent(WiFiEventFuncCb, id) (WiFiGeneric.h:91)
//   MDNS.begin/addService/addServiceTxt (ESPmDNS.h)

#include "wifi_link.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <Arduino.h>
#include <WiFi.h>
#include <esp_mac.h>   // esp_read_mac() — efuse MAC, valid before WiFi init
#include <ESPmDNS.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cmd_dispatch.h"
#include "wifi_creds.h"

// Forward decl (definition at global scope below) so handleHello() can emit an
// immediate telemetry frame on connect.
void wifiLinkEmitTelemetryNow();

namespace {

// ---- Config constants -----------------------------------------------------
constexpr uint16_t kTcpPort       = 3333;
constexpr uint32_t kStaJoinMs     = 15000;  // STA join timeout (plan §2.3)
constexpr uint32_t kTelPeriodMs   = 125;    // ~8 Hz telemetry (5–10 Hz band)
constexpr const char* kMdnsHost   = "siggen";

// ---- Hooks (copied from caller) -------------------------------------------
WifiLinkTelemetry s_hooks{};

// ---- Link state -----------------------------------------------------------
enum class Mode : uint8_t { CONNECTING, STA, AP, DISCONNECTED };
Mode  s_mode = Mode::DISCONNECTED;
char  s_ssid[WifiCreds::SSID_BUFLEN] = {0};
char  s_ip[20]   = {0};
char  s_appw[24] = {0};   // SoftAP passphrase (AP mode only, D14)

// Event-driven flags (set in the WiFi event handler, observed by the task).
volatile bool s_evtGotIp        = false;
volatile bool s_evtStaDiscon    = false;

// ---- Server / client ------------------------------------------------------
NetworkServer s_server(kTcpPort);
NetworkClient s_client;

// ---- Buffers (static — zero per-tick heap) --------------------------------
char   s_line[256];
size_t s_lineLen = 0;
char   s_tel[256];

// ---------------------------------------------------------------------------
// Socket emit sink passed to cmd_dispatch. Writes a complete frame (which
// already carries its own '\n') to the current client if connected.
// ---------------------------------------------------------------------------
void socketEmit(const char* frame) {
  if (s_client && s_client.connected()) {
    s_client.write((const uint8_t*)frame, strlen(frame));
  }
}

// ---------------------------------------------------------------------------
// Minimal flat-frame helpers (file-local; mirror cmd_dispatch's scanner but
// kept independent so cmd_dispatch.cpp includes only its 4 allowed headers).
// ---------------------------------------------------------------------------
const char* jsonFindKey(const char* json, const char* key) {
  const size_t klen = strlen(key);
  const char* p = json;
  while ((p = strchr(p, '"')) != nullptr) {
    const char* keyStart = p + 1;
    if (strncmp(keyStart, key, klen) == 0 && keyStart[klen] == '"') {
      const char* q = keyStart + klen + 1;
      while (*q == ' ' || *q == '\t') ++q;
      if (*q == ':') { ++q; while (*q == ' ' || *q == '\t') ++q; return q; }
    }
    const char* r = keyStart;
    while (*r && *r != '"') { if (*r == '\\' && r[1]) ++r; ++r; }
    p = (*r == '"') ? r + 1 : r;
  }
  return nullptr;
}

bool jsonGetStr(const char* json, const char* key, char* buf, size_t bufLen) {
  const char* v = jsonFindKey(json, key);
  if (!v || *v != '"') { if (bufLen) buf[0] = '\0'; return false; }
  ++v;
  size_t n = 0;
  while (*v && *v != '"' && n + 1 < bufLen) {
    char c = *v;
    if (c == '\\' && v[1]) {
      ++v;
      switch (*v) {
        case 'n': c = '\n'; break; case 't': c = '\t'; break;
        case 'r': c = '\r'; break; case '"': c = '"';  break;
        case '\\': c = '\\'; break; case '/': c = '/'; break;
        default: c = *v; break;
      }
    }
    buf[n++] = c; ++v;
  }
  buf[n] = '\0';
  return true;
}

bool jsonTypeIs(const char* json, const char* type) {
  char t[24];
  if (!jsonGetStr(json, "t", t, sizeof(t))) return false;
  return strcmp(t, type) == 0;
}

// ---------------------------------------------------------------------------
// Derive the SoftAP SSID ("SignalGen-XXXX", XXXX = last 2 MAC bytes) and a
// guaranteed 8..63 printable-ASCII WPA2 passphrase from the MAC (plan D14):
//   "siggen-" + last 4 MAC bytes as 8 hex digits  (15 chars, always >= 8).
// ---------------------------------------------------------------------------
void deriveApIdentity(char* ssidOut, size_t ssidLen, char* pwOut, size_t pwLen) {
  uint8_t mac[6] = {0};
  // Read the factory MAC from efuse — WiFi.macAddress() returns all-zeros
  // until the WiFi driver is initialized, and deriveApIdentity() runs before
  // WiFi.mode()/softAP(). esp_read_mac() works regardless of WiFi state.
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(ssidOut, ssidLen, "SignalGen-%02X%02X", mac[4], mac[5]);
  snprintf(pwOut, pwLen, "siggen-%02X%02X%02X%02X",
           mac[2], mac[3], mac[4], mac[5]);
}

// ---------------------------------------------------------------------------
// Push the current link state to the LCD label (via hook) and emit a `link`
// telemetry frame to the client (if any). `appw` is emitted only in AP mode.
// ---------------------------------------------------------------------------
const char* modeStr(Mode m) {
  switch (m) {
    case Mode::STA:        return "STA";
    case Mode::AP:         return "AP";
    case Mode::CONNECTING: return "CONNECTING";
    default:               return "DISCONNECTED";
  }
}

void emitLinkFrame() {
  const int8_t rssi = (s_mode == Mode::STA) ? WiFi.RSSI() : 0;
  char out[256];
  if (s_mode == Mode::AP) {
    snprintf(out, sizeof(out),
             "{\"t\":\"link\",\"mode\":\"AP\",\"ssid\":\"%s\",\"ip\":\"%s\","
             "\"rssi\":%d,\"mdns\":\"%s\",\"appw\":\"%s\"}\n",
             s_ssid, s_ip, (int)rssi, kMdnsHost, s_appw);
  } else {
    snprintf(out, sizeof(out),
             "{\"t\":\"link\",\"mode\":\"%s\",\"ssid\":\"%s\",\"ip\":\"%s\","
             "\"rssi\":%d,\"mdns\":\"%s\"}\n",
             modeStr(s_mode), s_ssid, s_ip, (int)rssi, kMdnsHost);
  }
  socketEmit(out);
}

void pushLink() {
  if (s_hooks.onLink) s_hooks.onLink(s_ip, s_ssid, modeStr(s_mode));
  emitLinkFrame();
}

// ---------------------------------------------------------------------------
// Start mDNS (idempotent-ish: end() first to allow re-advertise on STA join).
// Called ONLY when the active interface already has an IP.
// ---------------------------------------------------------------------------
void startMdns() {
  MDNS.end();
  if (MDNS.begin(kMdnsHost)) {
    MDNS.addService("siggen", "tcp", kTcpPort);
    MDNS.addServiceTxt("siggen", "tcp", "ver", "1");
  }
}

// ---------------------------------------------------------------------------
// WiFi event handler (runs in the framework WiFi-event task context). Keep it
// trivial: just set flags. The wifi_link task does the heavy lifting.
// ---------------------------------------------------------------------------
void onWifiEvent(arduino_event_id_t event, arduino_event_info_t /*info*/) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      s_evtGotIp = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      s_evtStaDiscon = true;
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Bring up SoftAP with a guaranteed-valid WPA2 passphrase (D14). Records the
// AP identity, starts mDNS, and pushes the link state.
// ---------------------------------------------------------------------------
void startSoftAp() {
  char ssid[WifiCreds::SSID_BUFLEN];
  char pw[24];
  deriveApIdentity(ssid, sizeof(ssid), pw, sizeof(pw));

  // D14 assertion: passphrase MUST be 8..63 chars or softAP() silently
  // downgrades to OPEN. Our derivation yields 15 chars; guard anyway.
  if (strlen(pw) < 8) {
    // Should be unreachable; fall back to a fixed valid literal.
    strncpy(pw, "siggen12345", sizeof(pw) - 1);
    pw[sizeof(pw) - 1] = '\0';
  }

  WiFi.mode(WIFI_AP);
  const bool ok = WiFi.softAP(ssid, pw);  // channel 1 default (WiFiAP.h:84)
  if (!ok) {
    log_e("[wifi_link] softAP() failed");
  }

  strncpy(s_ssid, ssid, sizeof(s_ssid) - 1); s_ssid[sizeof(s_ssid) - 1] = '\0';
  strncpy(s_appw, pw,   sizeof(s_appw) - 1); s_appw[sizeof(s_appw) - 1] = '\0';
  const IPAddress apIp = WiFi.softAPIP();
  snprintf(s_ip, sizeof(s_ip), "%u.%u.%u.%u", apIp[0], apIp[1], apIp[2], apIp[3]);
  s_mode = Mode::AP;

  // Print the SoftAP credentials so bring-up is observable on the serial
  // monitor and the user can join manually (the app derives the same pw).
  Serial.printf("[wifi_link] SoftAP %s : ssid=\"%s\" pass=\"%s\" ip=%s tcp=%d\n",
                ok ? "up" : "FAILED", ssid, pw, s_ip, (int)kTcpPort);

  startMdns();   // AP IP exists immediately after softAP()
  pushLink();
}

// ---------------------------------------------------------------------------
// Begin a STA join attempt for the given creds (non-blocking; GOT_IP /
// DISCONNECTED arrive via events). Returns immediately.
// ---------------------------------------------------------------------------
void beginStaJoin(const char* ssid, const char* pass) {
  s_evtGotIp = false;
  s_evtStaDiscon = false;
  strncpy(s_ssid, ssid, sizeof(s_ssid) - 1); s_ssid[sizeof(s_ssid) - 1] = '\0';
  s_ip[0] = '\0';
  s_mode = Mode::CONNECTING;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  pushLink();  // report CONNECTING (no IP yet)
}

// Promote a successful STA join: capture IP, start mDNS, push link.
void onStaGotIp() {
  const IPAddress ip = WiFi.localIP();
  snprintf(s_ip, sizeof(s_ip), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  // SSID() reflects the joined network.
  const String ssid = WiFi.SSID();
  strncpy(s_ssid, ssid.c_str(), sizeof(s_ssid) - 1);
  s_ssid[sizeof(s_ssid) - 1] = '\0';
  s_mode = Mode::STA;
  startMdns();  // STA IP now exists
  pushLink();
}

// ---------------------------------------------------------------------------
// Link/query frame handlers (NOT queued — handled inside wifi_link).
// ---------------------------------------------------------------------------
void handleScan() {
  const int16_t n = WiFi.scanNetworks();  // blocking, but only on explicit req
  // Build incrementally; one scanRes line. Cap to keep within s_tel-ish size.
  // We stream into a heap-free static buffer via chunked writes.
  socketEmit("{\"t\":\"scanRes\",\"nets\":[");
  for (int16_t i = 0; i < n; ++i) {
    char row[160];
    String ssid = WiFi.SSID(i);
    // Escape minimal: SSIDs rarely contain quotes; replace any '"' with ' '.
    char ss[40]; size_t k = 0;
    for (const char* c = ssid.c_str(); *c && k + 1 < sizeof(ss); ++c) {
      ss[k++] = (*c == '"' || *c == '\\') ? ' ' : *c;
    }
    ss[k] = '\0';
    snprintf(row, sizeof(row),
             "%s{\"ssid\":\"%s\",\"rssi\":%d,\"sec\":%d,\"ch\":%d}",
             (i == 0) ? "" : ",",
             ss, (int)WiFi.RSSI(i),
             (int)WiFi.encryptionType(i), (int)WiFi.channel(i));
    socketEmit(row);
  }
  socketEmit("]}\n");
  WiFi.scanDelete();
}

void handleProvision(const char* line) {
  char ssid[WifiCreds::SSID_BUFLEN] = {0};
  char pass[WifiCreds::PASS_BUFLEN] = {0};
  if (!jsonGetStr(line, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
    socketEmit("{\"t\":\"ack\",\"ok\":false,\"err\":\"bad ssid\"}\n");
    return;
  }
  jsonGetStr(line, "pass", pass, sizeof(pass));  // pass may be empty (open AP)

  // Persist, then ACK IMMEDIATELY (do NOT wait for the join). The SoftAP
  // socket is EXPECTED to drop once we WiFi.begin(STA) — plan §2.4.
  const bool saved = WifiCreds::save(ssid, pass);
  socketEmit(saved ? "{\"t\":\"ack\",\"ok\":true}\n"
                   : "{\"t\":\"ack\",\"ok\":false,\"err\":\"nvs\"}\n");
  if (!saved) return;

  // Switch to STA. The AP client socket will likely die here — that is normal.
  beginStaJoin(ssid, pass);
}

void handleForget() {
  WifiCreds::clear();
  socketEmit("{\"t\":\"ack\",\"ok\":true}\n");
  // Give the ACK a moment to flush, then reboot to SoftAP (plan §2.4 step 7).
  vTaskDelay(pdMS_TO_TICKS(150));
  ESP.restart();
}

void handleHello() {
  // hello -> reply link + a fresh telemetry frame + full catalog.
  emitLinkFrame();
  // telemetry is pushed by the periodic path; emit one now for immediacy.
  wifiLinkEmitTelemetryNow();
  cmd_emit_catalog(socketEmit);
}

// Returns true if `line` was a link/query frame handled here.
bool handleLinkFrame(const char* line) {
  if (jsonTypeIs(line, "hello"))      { handleHello();          return true; }
  if (jsonTypeIs(line, "list"))       { cmd_emit_catalog(socketEmit); return true; }
  if (jsonTypeIs(line, "scan"))       { handleScan();           return true; }
  if (jsonTypeIs(line, "provision"))  { handleProvision(line);  return true; }
  if (jsonTypeIs(line, "forgetWifi")) { handleForget();         return true; }
  return false;
}

// ---------------------------------------------------------------------------
// Telemetry frame builder. Reads each hook ONCE into a local; snprintf into
// the static s_tel buffer; client.write(). Zero heap.
// ---------------------------------------------------------------------------
void buildAndSendTelemetry() {
  if (!(s_client && s_client.connected())) return;

  const bool        run    = s_hooks.isRunning        ? s_hooks.isRunning()        : false;
  const uint32_t    rpm    = s_hooks.currentRpm       ? s_hooks.currentRpm()       : 0;
  const uint32_t    base   = s_hooks.baseRpm          ? s_hooks.baseRpm()          : 0;
  const char*       pat    = s_hooks.activePatternKey ? s_hooks.activePatternKey() : nullptr;
  const uint16_t    deg    = s_hooks.activeDegrees    ? s_hooks.activeDegrees()    : 0;
  const uint8_t     mask   = s_hooks.channelMask      ? s_hooks.channelMask()      : 0;
  const bool        inv    = s_hooks.inverted         ? s_hooks.inverted()         : false;
  const uint16_t    edge   = s_hooks.edgeCounter      ? s_hooks.edgeCounter()      : 0;
  const uint32_t    cycUs  = s_hooks.cycleDurationUs  ? s_hooks.cycleDurationUs()  : 0;
  const uint32_t    drop   = s_hooks.dropCount        ? s_hooks.dropCount()        : 0;
  if (!pat) pat = "";

  snprintf(s_tel, sizeof(s_tel),
           "{\"t\":\"tel\",\"rpm\":%lu,\"baseRpm\":%lu,\"run\":%s,\"pat\":\"%s\","
           "\"deg\":%u,\"mask\":%u,\"inv\":%s,\"edge\":%u,\"cycUs\":%lu,\"drop\":%lu}\n",
           (unsigned long)rpm, (unsigned long)base, run ? "true" : "false", pat,
           (unsigned)deg, (unsigned)mask, inv ? "true" : "false",
           (unsigned)edge, (unsigned long)cycUs, (unsigned long)drop);
  s_client.write((const uint8_t*)s_tel, strlen(s_tel));
}

// ---------------------------------------------------------------------------
// Inbound byte pump: drain available bytes, assemble lines into s_line, and
// dispatch on '\n'. Non-blocking; processes whatever is buffered this tick.
// ---------------------------------------------------------------------------
void pumpInbound() {
  if (!(s_client && s_client.connected())) { s_lineLen = 0; return; }
  while (s_client.available() > 0) {
    const int ci = s_client.read();
    if (ci < 0) break;
    const char c = (char)ci;
    if (c == '\r') continue;
    if (c == '\n') {
      s_line[s_lineLen] = '\0';
      if (s_lineLen > 0) {
        // Link/query frames handled here; everything else -> control dispatch.
        if (!handleLinkFrame(s_line)) {
          cmd_dispatch(s_line, socketEmit);
        }
      }
      s_lineLen = 0;
      continue;
    }
    if (s_lineLen < sizeof(s_line) - 1) {
      s_line[s_lineLen++] = c;
    } else {
      // Overlong line — reset to avoid wedging on a garbage stream.
      s_lineLen = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Accept a (single) client; a new accept() drops the previous one.
// ---------------------------------------------------------------------------
void acceptClient() {
  if (s_server.hasClient()) {
    NetworkClient next = s_server.accept();  // 3.3.4: available() deprecated
    if (next) {
      if (s_client) s_client.stop();
      s_client = next;
      s_client.setNoDelay(true);
      s_lineLen = 0;
      // Greet the new client with the current link state.
      emitLinkFrame();
    }
  }
  // Reap a dead client.
  if (s_client && !s_client.connected()) {
    s_client.stop();
  }
}

// ---------------------------------------------------------------------------
// The task body.
// ---------------------------------------------------------------------------
[[noreturn]] void wifiLinkTask(void* /*arg*/) {
  // Event handler must be registered before any WiFi.begin/softAP.
  WiFi.onEvent(onWifiEvent);
  WiFi.persistent(false);   // we manage creds in our own NVS namespace

  // ---- Bring-up: STA-with-timeout from stored creds, else SoftAP ----
  char ssid[WifiCreds::SSID_BUFLEN] = {0};
  char pass[WifiCreds::PASS_BUFLEN] = {0};
  bool started = false;
  bool staAttempted = false;

  if (WifiCreds::load(ssid, sizeof(ssid), pass, sizeof(pass))) {
    staAttempted = true;
    beginStaJoin(ssid, pass);
    const uint32_t t0 = millis();
    while (millis() - t0 < kStaJoinMs) {
      if (s_evtGotIp)     { onStaGotIp(); started = true; break; }
      if (s_evtStaDiscon) { break; }   // failed early — fall back
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
  if (!started) {
    // Only tear down a half-open STA if we actually attempted one — calling
    // WiFi.disconnect() before any WiFi.mode()/begin() errors with
    // ESP_ERR_WIFI_NOT_INIT (harmless, but noisy). startSoftAp() sets the mode.
    if (staAttempted) WiFi.disconnect(true);
    startSoftAp();
  }

  // ---- Server up ----
  s_server.begin();
  s_server.setNoDelay(true);

  uint32_t lastTel = millis();

  for (;;) {
    // Handle late STA events (e.g. provisioning join completing after ACK, or
    // a roaming disconnect). GOT_IP after provisioning re-advertises mDNS.
    if (s_evtGotIp) {
      s_evtGotIp = false;
      onStaGotIp();
    }
    if (s_evtStaDiscon) {
      s_evtStaDiscon = false;
      if (s_mode == Mode::STA || s_mode == Mode::CONNECTING) {
        // Lost / failed STA. Report; the framework auto-reconnects for STA,
        // but if creds are gone we stay disconnected. Keep serving on whatever
        // interface remains; do NOT tear down the server.
        s_mode = Mode::DISCONNECTED;
        s_ip[0] = '\0';
        pushLink();
      }
    }

    acceptClient();
    pumpInbound();

    const uint32_t now = millis();
    if (now - lastTel >= kTelPeriodMs) {
      lastTel = now;
      buildAndSendTelemetry();
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // ~100 Hz service loop; telemetry gated above
  }
}

}  // namespace

// Defined out-of-namespace so handleHello()'s extern fwd-decl resolves.
void wifiLinkEmitTelemetryNow() {
  buildAndSendTelemetry();
}

// ---------------------------------------------------------------------------
// Public init: copy hooks, spawn the task on Core 0 prio 1.
// ---------------------------------------------------------------------------
void wifiLinkInit(const WifiLinkTelemetry* hooks) {
  if (hooks) s_hooks = *hooks;

  xTaskCreatePinnedToCore(
      wifiLinkTask,
      "wifi_link",
      8192,                      // stack: WiFi/mDNS/snprintf headroom
      nullptr,
      tskIDLE_PRIORITY + 1,      // prio 1 (below LVGL), plan §2.1
      nullptr,
      0);                        // Core 0 (never Core 1)
}
