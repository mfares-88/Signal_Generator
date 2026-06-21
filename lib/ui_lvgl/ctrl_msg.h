// lib/ui_lvgl/ctrl_msg.h — shared MsgType / CtrlMsg definitions for
// main.cpp + serial_cli.cpp (and any other TU that needs to enqueue onto
// the manager queue). Keeping the type in a single header guarantees
// queue item-size parity between producers and consumer.

#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "CkpGenerator.h"  // SignalConfig (POD)

enum MsgType : uint8_t {
  MSG_SET_RPM,
  MSG_SET_PATTERN,
  MSG_START,
  MSG_STOP,
  MSG_SET_INVERT,
  MSG_SELECT_BUILTIN,
  MSG_SELECT_NAMED,
  MSG_LOAD_DSL,
  MSG_LOAD_TABLE,
  MSG_SET_SWEEP,
  MSG_SET_COMPRESSION,
  MSG_CAPTURE_START,
  MSG_CAPTURE_STOP,
  MSG_SAVE_USER
};

union MsgPayload {
  int32_t      val;
  const char*  name;
  struct {
    const uint8_t* bytes;
    uint16_t       len;
    uint16_t       degrees;
  } raw;
  struct {
    uint16_t low_rpm;
    uint16_t high_rpm;
    uint8_t  mode;
    uint32_t interval_us;
  } sweep;
  struct {
    bool     enabled;
    uint8_t  cyl;
    uint16_t rpm_thresh;
    uint8_t  peak;
    bool     dynamic;
  } comp;
  // MSG_SAVE_USER: carries both the target user-pattern key AND the DSL
  // source string. Both pointers are heap-allocated by the sender and the
  // manager task is responsible for free()'ing them after consuming.
  struct {
    const char* name;
    const char* dsl_source;
  } save;
};

struct CtrlMsg {
  MsgType    type;
  MsgPayload payload;
};

// Manager queue handle, defined in src/main.cpp.
extern QueueHandle_t gCtrlQ;

// Non-blocking enqueue; bumps a drop counter on failure (definition in main.cpp).
bool sendCtrlMsg(const CtrlMsg& msg);
