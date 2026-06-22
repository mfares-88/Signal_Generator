// lib/wifi_link/cmd_dispatch.h — Cycle 7, Agent F.
//
// Transport-agnostic NDJSON-command -> sendCtrlMsg() translator. This is a
// MIRROR (not a share) of serial_cli.cpp's dispatch_text() (plan D7): it
// re-implements ONLY the control verbs, with a `void emit(const char*)` sink
// (the socket writer) instead of Serial.print. serial_cli.cpp is NOT touched.
//
// Handles ONLY the §4.0 CONTROL frames (those that become gCtrlQ messages):
//   rpm start stop invert selBuiltin selNamed loadDsl sweep comp
//   capStart capStop save
// LINK/QUERY frames (hello/list/provision/scan/forgetWifi) are handled by the
// caller (wifi_link.cpp), NOT here — except `list`, whose catalog enumeration
// lives here (cmd_emit_catalog) because it needs PatternLibrary ordering that
// matches selBuiltin.
//
// `jsonLine` is one NUL-terminated NDJSON object (no trailing newline). `emit`
// receives complete frames to write to the client (each already includes its
// own trailing '\n'). Malformed frames -> an ack error via emit.

#pragma once

// Parse one §4.0 control frame and enqueue the matching CtrlMsg. Emits an
// `ack` (ok:true / ok:false+err) for every recognized frame. Unknown `t` is
// ignored here (the caller decides whether it is a link/query frame).
void cmd_dispatch(const char* jsonLine, void (*emit)(const char*));

// Emit the full pattern catalog as catBegin / pat… / catEnd frames (the
// `list` reply). Builtins enumerated via builtinByIndex(i) so the emitted `i`
// matches what selBuiltin's MSG_SELECT_BUILTIN consumes; user rows i=-1+key.
void cmd_emit_catalog(void (*emit)(const char*));
