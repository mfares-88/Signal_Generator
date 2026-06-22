// lib/wifi_link/cmd_dispatch.cpp — Cycle 7, Agent F. See cmd_dispatch.h.
//
// Hand-rolled, zero-heap (except the malloc'd DSL/save strings the manager
// frees) flat-frame NDJSON scanner + control-verb dispatcher. Mirrors
// serial_cli.cpp dispatch_text() semantics 1:1 onto the SAME 14 MsgTypes.
//
// Includes ONLY the contract + pattern headers (plan §4.1) — never gGen/gCfg.

#include "cmd_dispatch.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ctrl_msg.h"
#include "PatternLibrary.h"
#include "PatternStorage.h"
#include "PatternRef.h"

// ---------------------------------------------------------------------------
// Flat-frame JSON scanner (file-local). Frames are flat: top-level keys map to
// a string or a number (the sweep/comp fields are flat too, e.g. "lo","hi").
// We never allocate; string values are copied into caller-supplied buffers.
// ---------------------------------------------------------------------------

namespace {

// Find the value position for "key" in a flat JSON object. Returns a pointer
// just past the ':' (skipping whitespace), or nullptr if the key is absent.
// Matches keys only when they are JSON object keys ("key":) — not substrings
// of a string value — by requiring the match to be a quoted token followed by
// optional whitespace and a ':'.
const char* findKey(const char* json, const char* key) {
  const size_t klen = strlen(key);
  const char* p = json;
  while ((p = strchr(p, '"')) != nullptr) {
    const char* keyStart = p + 1;
    if (strncmp(keyStart, key, klen) == 0 && keyStart[klen] == '"') {
      const char* q = keyStart + klen + 1;
      while (*q == ' ' || *q == '\t') ++q;
      if (*q == ':') {
        ++q;
        while (*q == ' ' || *q == '\t') ++q;
        return q;
      }
    }
    // Skip to the end of this string token so we don't match inside values.
    // Advance past the closing quote (best-effort; handles simple escapes).
    const char* r = keyStart;
    while (*r && *r != '"') {
      if (*r == '\\' && r[1]) ++r;
      ++r;
    }
    p = (*r == '"') ? r + 1 : r;
  }
  return nullptr;
}

// Read a number value for "key". Returns true and sets *out on success.
bool getLong(const char* json, const char* key, long* out) {
  const char* v = findKey(json, key);
  if (!v) return false;
  if (*v == 't') { *out = 1; return true; }   // true
  if (*v == 'f') { *out = 0; return true; }   // false
  char* end = nullptr;
  const long n = strtol(v, &end, 10);
  if (end == v) return false;
  *out = n;
  return true;
}

// Read a string value for "key" into buf (NUL-terminated, decoding \" \\ \n
// \t \/). Returns true and sets *outLen on success. Truncates to bufLen-1.
bool getStr(const char* json, const char* key, char* buf, size_t bufLen,
            size_t* outLen) {
  const char* v = findKey(json, key);
  if (!v || *v != '"') return false;
  ++v;  // past opening quote
  size_t n = 0;
  while (*v && *v != '"' && n + 1 < bufLen) {
    char c = *v;
    if (c == '\\' && v[1]) {
      ++v;
      switch (*v) {
        case 'n': c = '\n'; break;
        case 't': c = '\t'; break;
        case 'r': c = '\r'; break;
        case '"': c = '"';  break;
        case '\\': c = '\\'; break;
        case '/': c = '/';  break;
        default:  c = *v;   break;
      }
    }
    buf[n++] = c;
    ++v;
  }
  buf[n] = '\0';
  if (outLen) *outLen = n;
  return true;
}

// True iff the frame's "t" value equals `type`.
bool typeIs(const char* json, const char* type) {
  char t[24];
  size_t tl = 0;
  if (!getStr(json, "t", t, sizeof(t), &tl)) return false;
  return strcmp(t, type) == 0;
}

// Optional client sequence id, echoed in ack. -1 when absent.
long getId(const char* json) {
  long id = -1;
  getLong(json, "id", &id);
  return id;
}

// Emit an ack frame. err==nullptr -> ok:true. id<0 -> omit id field.
void ack(void (*emit)(const char*), long id, bool ok, const char* err) {
  char out[160];
  if (ok) {
    if (id >= 0) snprintf(out, sizeof(out), "{\"t\":\"ack\",\"id\":%ld,\"ok\":true}\n", id);
    else         snprintf(out, sizeof(out), "{\"t\":\"ack\",\"ok\":true}\n");
  } else {
    // Keep err short and free of quotes/newlines (our strings are literals).
    if (id >= 0) snprintf(out, sizeof(out),
                          "{\"t\":\"ack\",\"id\":%ld,\"ok\":false,\"err\":\"%s\"}\n",
                          id, err ? err : "error");
    else         snprintf(out, sizeof(out),
                          "{\"t\":\"ack\",\"ok\":false,\"err\":\"%s\"}\n",
                          err ? err : "error");
  }
  emit(out);
}

// Enqueue helpers (mirror serial_cli.cpp enq*). Return false on queue-full.
bool enqVal(MsgType type, int32_t v) {
  CtrlMsg m{};
  m.type = type;
  m.payload.val = v;
  return sendCtrlMsg(m);
}

bool enqName(MsgType type, const char* name) {
  CtrlMsg m{};
  m.type = type;
  m.payload.name = name;
  return sendCtrlMsg(m);
}

}  // namespace

// ---------------------------------------------------------------------------
// Catalog emission (the `list` reply). Builtins via builtinByIndex(i) so the
// emitted `i` matches MSG_SELECT_BUILTIN's consumer (main.cpp builtinByIndex).
// User rows: i=-1, selected via selNamed by key. friendlyName null-guarded.
// ---------------------------------------------------------------------------

static void emitPatRow(void (*emit)(const char*), const char* tier, int i,
                       const PatternRef* p) {
  if (!p) return;
  const char* key  = p->name_key ? p->name_key : "";
  const char* name = PatternLibrary::friendlyName(p->name_key);
  if (!name) name = key;  // null-guard -> name_key (serial_cli.cpp:88-89)
  char out[256];
  snprintf(out, sizeof(out),
           "{\"t\":\"pat\",\"tier\":\"%s\",\"i\":%d,\"key\":\"%s\",\"name\":\"%s\","
           "\"deg\":%u,\"mask\":%u,\"slots\":%u}\n",
           tier, i, key, name,
           (unsigned)p->degrees, (unsigned)p->channel_mask,
           (unsigned)p->slot_count);
  emit(out);
}

void cmd_emit_catalog(void (*emit)(const char*)) {
  const size_t nb = PatternLibrary::builtinCount();
  const size_t nu = PatternLibrary::userCount();

  char hdr[48];
  snprintf(hdr, sizeof(hdr), "{\"t\":\"catBegin\",\"n\":%u}\n",
           (unsigned)(nb + nu));
  emit(hdr);

  for (size_t i = 0; i < nb; ++i) {
    emitPatRow(emit, "builtin", (int)i, PatternLibrary::builtinByIndex(i));
  }
  for (size_t i = 0; i < nu; ++i) {
    emitPatRow(emit, "user", -1, PatternLibrary::userByIndex(i));
  }

  emit("{\"t\":\"catEnd\"}\n");
}

// ---------------------------------------------------------------------------
// Control-frame dispatch.
// ---------------------------------------------------------------------------

void cmd_dispatch(const char* jsonLine, void (*emit)(const char*)) {
  const long id = getId(jsonLine);

  // ---- rpm ----
  if (typeIs(jsonLine, "rpm")) {
    long v = 0;
    if (!getLong(jsonLine, "v", &v)) { ack(emit, id, false, "bad rpm"); return; }
    ack(emit, id, enqVal(MSG_SET_RPM, (int32_t)v), "queue full");
    return;
  }
  // ---- start / stop ----
  if (typeIs(jsonLine, "start")) {
    ack(emit, id, enqVal(MSG_START, 0), "queue full");
    return;
  }
  if (typeIs(jsonLine, "stop")) {
    ack(emit, id, enqVal(MSG_STOP, 0), "queue full");
    return;
  }
  // ---- invert ----
  if (typeIs(jsonLine, "invert")) {
    long v = 0;
    getLong(jsonLine, "v", &v);
    ack(emit, id, enqVal(MSG_SET_INVERT, v ? 1 : 0), "queue full");
    return;
  }
  // ---- selBuiltin ----
  if (typeIs(jsonLine, "selBuiltin")) {
    long i = -1;
    if (!getLong(jsonLine, "i", &i) || i < 0) {
      ack(emit, id, false, "bad index"); return;
    }
    ack(emit, id, enqVal(MSG_SELECT_BUILTIN, (int32_t)i), "queue full");
    return;
  }
  // ---- selNamed ----
  if (typeIs(jsonLine, "selNamed")) {
    char key[PatternStorage::KEY_BUFLEN];
    if (!getStr(jsonLine, "key", key, sizeof(key), nullptr)) {
      ack(emit, id, false, "bad key"); return;
    }
    // D11: resolve to the .rodata name_key — NEVER strdup a network buffer.
    const PatternRef* p = PatternLibrary::findByKey(key);
    if (!p) { ack(emit, id, false, "unknown key"); return; }
    ack(emit, id, enqName(MSG_SELECT_NAMED, p->name_key), "queue full");
    return;
  }
  // ---- loadDsl ----
  if (typeIs(jsonLine, "loadDsl")) {
    // The DSL source may be up to 512 chars (Validator.cpp rule #11) plus
    // builder formatting; allocate generously. Manager frees.
    static constexpr size_t kDslMax = 2048;
    char tmp[kDslMax];
    size_t n = 0;
    if (!getStr(jsonLine, "src", tmp, sizeof(tmp), &n)) {
      ack(emit, id, false, "bad src"); return;
    }
    char* src = (char*)malloc(n + 1);
    if (!src) { ack(emit, id, false, "oom"); return; }
    memcpy(src, tmp, n + 1);
    if (!enqName(MSG_LOAD_DSL, src)) {
      free(src);
      ack(emit, id, false, "queue full");
    } else {
      ack(emit, id, true, nullptr);
    }
    return;
  }
  // ---- sweep ---- (flat: lo, hi, mode, iv)
  if (typeIs(jsonLine, "sweep")) {
    long lo = 0, hi = 0, mode = 0, iv = 0;
    getLong(jsonLine, "lo",   &lo);
    getLong(jsonLine, "hi",   &hi);
    getLong(jsonLine, "mode", &mode);
    getLong(jsonLine, "iv",   &iv);
    CtrlMsg m{};
    m.type = MSG_SET_SWEEP;
    m.payload.sweep.low_rpm     = (uint16_t)lo;
    m.payload.sweep.high_rpm    = (uint16_t)hi;
    m.payload.sweep.mode        = (uint8_t)mode;
    m.payload.sweep.interval_us = (uint32_t)iv;
    ack(emit, id, sendCtrlMsg(m), "queue full");
    return;
  }
  // ---- comp ---- (flat: on, cyl, thr, peak, dyn)
  if (typeIs(jsonLine, "comp")) {
    long on = 0, cyl = 0, thr = 0, peak = 0, dyn = 0;
    getLong(jsonLine, "on",   &on);
    getLong(jsonLine, "cyl",  &cyl);
    getLong(jsonLine, "thr",  &thr);
    getLong(jsonLine, "peak", &peak);
    getLong(jsonLine, "dyn",  &dyn);
    CtrlMsg m{};
    m.type = MSG_SET_COMPRESSION;
    m.payload.comp.enabled    = on != 0;
    m.payload.comp.cyl        = (uint8_t)cyl;
    m.payload.comp.rpm_thresh = (uint16_t)thr;
    m.payload.comp.peak       = (uint8_t)peak;
    m.payload.comp.dynamic    = dyn != 0;
    ack(emit, id, sendCtrlMsg(m), "queue full");
    return;
  }
  // ---- capStart ---- (revs optional; absent/0 -> val=0 -> manager defaults 2)
  if (typeIs(jsonLine, "capStart")) {
    long revs = 0;
    getLong(jsonLine, "revs", &revs);
    if (revs < 0) revs = 0;
    ack(emit, id, enqVal(MSG_CAPTURE_START, (int32_t)revs), "queue full");
    return;
  }
  if (typeIs(jsonLine, "capStop")) {
    ack(emit, id, enqVal(MSG_CAPTURE_STOP, 0), "queue full");
    return;
  }
  // ---- save ---- (key + src; both heap-copied, manager frees)
  if (typeIs(jsonLine, "save")) {
    char keyTmp[PatternStorage::KEY_BUFLEN];
    size_t keyLen = 0;
    if (!getStr(jsonLine, "key", keyTmp, sizeof(keyTmp), &keyLen) || keyLen == 0) {
      ack(emit, id, false, "bad key"); return;
    }
    static constexpr size_t kDslMax = 2048;
    char srcTmp[kDslMax];
    size_t srcLen = 0;
    const bool haveSrc = getStr(jsonLine, "src", srcTmp, sizeof(srcTmp), &srcLen);

    char* keyHeap = (char*)malloc(keyLen + 1);
    if (!keyHeap) { ack(emit, id, false, "oom"); return; }
    memcpy(keyHeap, keyTmp, keyLen + 1);

    char* srcHeap = nullptr;
    if (haveSrc) {
      srcHeap = (char*)malloc(srcLen + 1);
      if (!srcHeap) { free(keyHeap); ack(emit, id, false, "oom"); return; }
      memcpy(srcHeap, srcTmp, srcLen + 1);
    }

    CtrlMsg m{};
    m.type = MSG_SAVE_USER;
    m.payload.save.name       = keyHeap;
    m.payload.save.dsl_source = srcHeap;  // null -> manager falls back (main.cpp)
    if (!sendCtrlMsg(m)) {
      free(keyHeap);
      if (srcHeap) free(srcHeap);
      ack(emit, id, false, "queue full");
    } else {
      ack(emit, id, true, nullptr);
    }
    return;
  }

  // Unrecognized as a control frame — caller handles link/query frames.
  // (We deliberately emit nothing here.)
}
