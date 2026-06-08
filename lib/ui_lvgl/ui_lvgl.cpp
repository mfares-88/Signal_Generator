// LVGL v9 UI + display/touch port for JC4827W543 (ESP32-S3)
//
// Phase 4..7 / 9 (Agent E): adds channel-state LEDs (M2.3), the full
// 64-pattern scrollable dropdown with search (M3.4), Sweep / Compression
// tabs (M4.5), DSL editor modal (M5.7), Capture page (M6), and the 3-lane
// waveform canvas (M7). All cross-core updates continue to use the
// existing pending-flag pattern (s_ui_mux) — no new sync mechanism.
//
// This TU is gated on SIGGEN_HAS_DISPLAY so the WROOM (headless) build
// can skip LVGL entirely. PlatformIO's lib-dep finder still scans this
// folder for `ctrl_msg.h`/`serial_cli.{h,cpp}`, hence the file-level
// guard rather than a build_src_filter.
#if defined(SIGGEN_HAS_DISPLAY)

#include "ui_lvgl.h"

#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>
#include <esp_heap_caps.h>
#include <Arduino_GFX_Library.h>
#include <PINS_JC4827W543.h>
#include "TAMC_GT911.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>  // INT_MIN — fuzzy scorer "no match" sentinel

#include "PatternLibrary.h"
#include "PatternStorage.h"
#include "SweepCompression.h"
#include "ctrl_msg.h"


// Touch controller configuration (GT911)
static constexpr uint8_t  kTouchSda = I2C_SDA;
static constexpr uint8_t  kTouchScl = I2C_SCL;
static constexpr uint8_t  kTouchInt = TOUCH_INT;
static constexpr uint8_t  kTouchRst = TOUCH_RES;
static constexpr uint16_t kPanelWidth = 480;
static constexpr uint16_t kPanelHeight = 272;
// Hardcoded landscape rotation. If touch and visuals are mirrored, swap to
// ROTATION_NORMAL or ROTATION_LEFT / ROTATION_RIGHT.
static constexpr auto    kTouchRotation = ROTATION_INVERTED;
static constexpr uint8_t  kDisplayRotation = 0;
static constexpr uint16_t kTouchWidth = kPanelWidth;
static constexpr uint16_t kTouchHeight = kPanelHeight;
static constexpr bool kArcReverse = false;

// 480x272 layout constants for the main screen pane structure.
static constexpr int SCREEN_W = 480;
static constexpr int SCREEN_H = 272;
static constexpr int LEFT_X = 8;
static constexpr int LEFT_Y = 32;
static constexpr int LEFT_W = 224;
static constexpr int LEFT_H = 224;
static constexpr int RIGHT_X = 240;
static constexpr int RIGHT_Y = 12;
static constexpr int RIGHT_W = 232;
static constexpr int RIGHT_H = 252;

TAMC_GT911 touchController(kTouchSda, kTouchScl, kTouchInt, kTouchRst, kTouchWidth, kTouchHeight);

// ---- Callbacks provided by application ----
static ui_on_rpm_cb     s_on_rpm = nullptr;
static ui_on_pattern_cb s_on_pattern = nullptr;
static ui_on_run_cb     s_on_run = nullptr;
static ui_on_custom_cb  s_on_custom = nullptr;
static ui_on_invert_cb  s_on_invert = nullptr;


// ---- LVGL objects ----
static lv_obj_t* screen_main = nullptr;
static lv_obj_t* arc_rpm = nullptr;
static lv_obj_t* lbl_rpm_value = nullptr;
static lv_obj_t* lbl_rpm_caption = nullptr;
static lv_obj_t* lbl_title = nullptr;
static lv_obj_t* lbl_pattern = nullptr;
static lv_obj_t* dd_patterns = nullptr;
static lv_obj_t* btn_run = nullptr;
static lv_obj_t* lbl_run = nullptr;
static lv_obj_t* btn_invert = nullptr;
static lv_obj_t* lbl_invert = nullptr;
static lv_obj_t* lbl_error = nullptr;

// M2.3: channel LEDs (crank, cam1, cam2)
static lv_obj_t* led_crank = nullptr;
static lv_obj_t* led_cam1  = nullptr;
static lv_obj_t* led_cam2  = nullptr;
// Cached visible state to avoid redundant style updates
static uint8_t s_visible_channel_mask = 0x01;
static uint8_t s_visible_invert_mask  = 0x00;

// M3.4: 64-pattern dropdown — we substitute lv_dropdown's "options"
// string with all builtin patterns, prefixed by category section
// markers. The mapping s_pattern_dd_to_builtin[] resolves a dropdown
// selection (incl. section-marker offsets we skip via re-selection
// logic) to a PatternLibrary::builtinByIndex(...) index. We hold up to
// 128 entries to leave room for user patterns later.
// 128 builtins + 4 category headers = 132 possible rows; +4 slack.
#define UI_PATTERN_DD_CAP 136
static int16_t s_pattern_dd_to_builtin[UI_PATTERN_DD_CAP];
static uint8_t s_pattern_dd_entry_count = 0;
static lv_obj_t* ta_pattern_filter = nullptr;
static char s_pattern_filter[32] = {0};

// On-screen keyboard for the filter / DSL textareas. Lazily created on
// lv_layer_top() the first time a textarea is focused (see ui_get_keyboard).
static lv_obj_t* kb_filter = nullptr;

// M4.5: Sweep + compression panels (modals).
static lv_obj_t* overlay_sweep = nullptr;
static lv_obj_t* spin_sweep_low = nullptr;
static lv_obj_t* spin_sweep_high = nullptr;
static lv_obj_t* dd_sweep_mode  = nullptr;
static lv_obj_t* spin_sweep_iv  = nullptr;
static lv_obj_t* lbl_sweep_live = nullptr;
static lv_timer_t* tmr_sweep_live = nullptr;

static lv_obj_t* overlay_comp = nullptr;
static lv_obj_t* sw_comp_en   = nullptr;
static lv_obj_t* spin_comp_cyl = nullptr;
static lv_obj_t* spin_comp_thr = nullptr;
static lv_obj_t* spin_comp_peak = nullptr;
static lv_obj_t* sw_comp_dyn   = nullptr;

// M5.7: DSL editor modal.
static lv_obj_t* overlay_dsl = nullptr;
static lv_obj_t* ta_dsl_src  = nullptr;
static lv_obj_t* lbl_dsl_err = nullptr;
static lv_timer_t* tmr_dsl_err = nullptr;

// M7: waveform canvas.
static lv_obj_t*   overlay_wave   = nullptr;
static lv_obj_t*   canvas_wave    = nullptr;
static lv_color_t* canvas_wave_buf = nullptr;
static lv_timer_t* tmr_wave        = nullptr;
static int         s_wave_zoom    = 1;     // pixels per slot
static uint8_t     s_wave_lane_mask = 0x07; // bit0..2: lane visibility

// Cross-TU hooks (defined in main.cpp).
extern volatile char g_dsl_error[];
// All globals below are declared in NvsStore.h (we include it transitively
// via ctrl_msg.h? — actually we don't; pull NvsStore directly):
#include "NvsStore.h"

// ---- Custom pattern modal ----
static lv_obj_t* overlay_custom = nullptr;
static lv_obj_t* panel_custom = nullptr;
static lv_obj_t* spin_teeth = nullptr;
static lv_obj_t* spin_pmiss = nullptr;
static lv_obj_t* spin_nmiss = nullptr;
static lv_obj_t* dd_gap_pos = nullptr;
static lv_obj_t* sw_gap_lvl = nullptr;
static lv_obj_t* lbl_custom_error = nullptr;
static uint8_t s_last_preset_pattern = 0;


// ---- LVGL display state ----
static lv_display_t* s_disp = nullptr;
static lv_indev_t* s_indev = nullptr;
static lv_color_t* s_draw_buf = nullptr;
static uint32_t s_draw_buf_px = 0;
static uint32_t s_screen_w = 480;
static uint32_t s_screen_h = 272;
static bool s_lvgl_ready = false;

static portMUX_TYPE s_ui_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool s_pending_rpm = false;
static volatile uint32_t s_pending_rpm_val = 0;
static volatile bool s_pending_pattern = false;
static volatile uint8_t s_pending_pattern_val = 0;
static volatile bool s_pending_running = false;
static volatile bool s_pending_running_val = false;
static volatile bool s_pending_inverted = false;
static volatile bool s_pending_inverted_val = false;
static volatile bool s_pending_error = false;
static char s_pending_error_msg[96];

// M2.3 — pending channel-state update (bit0=crank,1=cam1,2=cam2 for both
// `channels` (which channels the active pattern uses) and `inverts`
// (per-channel XOR mask). Applied on the LVGL thread.
static volatile bool    s_pending_channels = false;
static volatile uint8_t s_pending_channel_mask = 0x01;
static volatile uint8_t s_pending_invert_mask  = 0x00;

static bool s_suppress_rpm_cb = false;
static bool s_suppress_pattern_cb = false;
static bool s_suppress_run_cb = false;
static bool s_suppress_invert_cb = false;

static bool s_running = true;
static bool s_inverted = false;
static uint32_t s_rpm_flash_until_ms = 0;


// ---- Styles ----
static lv_style_t style_bg;
static lv_style_t style_title;
static lv_style_t style_caption;
static lv_style_t style_value;
static lv_style_t style_arc_main;
static lv_style_t style_arc_indic;
static lv_style_t style_dropdown;

static uint8_t pick_display_rotation();
static void init_styles();
static void create_main_screen();
static void on_arc_changed(lv_event_t* e);
static void on_pattern_changed(lv_event_t* e);
static void on_pattern_open(lv_event_t* e);
static void on_run_clicked(lv_event_t* e);
static void on_invert_clicked(lv_event_t* e);
static void refresh_run_label();
static void refresh_invert_label();
static void update_rpm_label(int32_t rpm);
static void apply_pending_updates();

static SignalConfig presetCfgFromIndex(uint8_t idx, uint32_t rpm);
static void open_custom_panel();
static void close_custom_panel();
static void set_custom_error(const char* msg);
static void on_spin_inc(lv_event_t* e);
static void on_spin_dec(lv_event_t* e);
static void on_custom_apply(lv_event_t* e);
static void on_custom_cancel(lv_event_t* e);

// M3.4 / M4.5 / M5.7 / M7 forward decls.
static void rebuild_pattern_dropdown_options(const char* filter);
static void on_pattern_filter_changed(lv_event_t* e);
static const char* category_for_pattern(const PatternRef* p);

// On-screen keyboard lifecycle (R3) — shared by the filter + DSL textareas.
static lv_obj_t* ui_get_keyboard();
static void kb_show_for(lv_obj_t* ta);
static void kb_hide();
static void on_ta_focused(lv_event_t* e);
static void on_ta_defocused(lv_event_t* e);

static void open_sweep_panel(lv_event_t* e);
static void close_sweep_panel(lv_event_t* e);
static void on_sweep_apply(lv_event_t* e);
static void on_sweep_live_tick(lv_timer_t* t);

static void open_comp_panel(lv_event_t* e);
static void close_comp_panel(lv_event_t* e);
static void on_comp_apply(lv_event_t* e);

static void open_dsl_panel(lv_event_t* e);
static void close_dsl_panel(lv_event_t* e);
static void on_dsl_compile(lv_event_t* e);
static void on_dsl_saveas(lv_event_t* e);
static void on_dsl_load(lv_event_t* e);
static void on_dsl_err_tick(lv_timer_t* t);

static void open_wave_panel(lv_event_t* e);
static void close_wave_panel(lv_event_t* e);
static void on_wave_tick(lv_timer_t* t);


// ---- LVGL callbacks ----
static void my_print(lv_log_level_t level, const char* buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

static uint32_t millis_cb(void) {
  return millis();
}

static void my_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);

  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t*)px_map, w, h);
  lv_disp_flush_ready(disp);
}

static void my_touchpad_read(lv_indev_t* indev, lv_indev_data_t* data) {
  LV_UNUSED(indev);

  static uint32_t last_ms = 0;
  static lv_point_t last_pt{0, 0};
  static bool last_pressed = false;

  const uint32_t now_ms = millis();

  touchController.read();
  const bool pressed = (touchController.isTouched && touchController.touches > 0);
  if (!pressed) {
    data->state = LV_INDEV_STATE_RELEASED;
    last_pressed = false;
    return;
  }

  const lv_point_t pt{(lv_coord_t)touchController.points[0].x, (lv_coord_t)touchController.points[0].y};

  if (last_pressed && (now_ms - last_ms) < 50) {
    const int dx = (int)pt.x - (int)last_pt.x;
    const int dy = (int)pt.y - (int)last_pt.y;
    if ((dx * dx + dy * dy) < 9) {
      data->point = last_pt;
      data->state = LV_INDEV_STATE_PRESSED;
      return;
    }
  }

  last_ms = now_ms;
  last_pt = pt;
  last_pressed = true;

  data->point = pt;
  data->state = LV_INDEV_STATE_PRESSED;
}


static uint8_t pick_display_rotation() {
  gfx->setRotation(kDisplayRotation);
  return kDisplayRotation;
}

static void init_styles() {
  static bool inited = false;
  if (inited) return;
  inited = true;

  lv_style_init(&style_bg);
  lv_style_set_bg_color(&style_bg, lv_color_hex(0x0B1020));
  lv_style_set_bg_grad_color(&style_bg, lv_color_hex(0x141C2E));
  lv_style_set_bg_grad_dir(&style_bg, LV_GRAD_DIR_VER);
  lv_style_set_bg_opa(&style_bg, LV_OPA_COVER);

  lv_style_init(&style_title);
  lv_style_set_text_color(&style_title, lv_color_hex(0xD7E9FF));
  lv_style_set_text_letter_space(&style_title, 2);
  lv_style_set_text_font(&style_title, &lv_font_montserrat_14);

  lv_style_init(&style_caption);
  lv_style_set_text_color(&style_caption, lv_color_hex(0x7C8DB0));
  lv_style_set_text_font(&style_caption, &lv_font_montserrat_10);

  lv_style_init(&style_value);
  lv_style_set_text_color(&style_value, lv_color_hex(0x00E5FF));
  lv_style_set_text_font(&style_value, &lv_font_montserrat_14);

  lv_style_init(&style_arc_main);
  lv_style_set_arc_width(&style_arc_main, 18);
  lv_style_set_arc_color(&style_arc_main, lv_color_hex(0x1B2438));
  lv_style_set_arc_rounded(&style_arc_main, true);

  lv_style_init(&style_arc_indic);
  lv_style_set_arc_width(&style_arc_indic, 18);
  lv_style_set_arc_color(&style_arc_indic, lv_color_hex(0x00E5FF));
  lv_style_set_arc_rounded(&style_arc_indic, true);
  lv_style_set_shadow_width(&style_arc_indic, 18);
  lv_style_set_shadow_color(&style_arc_indic, lv_color_hex(0x00E5FF));
  lv_style_set_shadow_opa(&style_arc_indic, LV_OPA_40);
  lv_style_set_shadow_spread(&style_arc_indic, 2);

  lv_style_init(&style_dropdown);
  lv_style_set_bg_color(&style_dropdown, lv_color_hex(0x0F1628));
  lv_style_set_bg_grad_color(&style_dropdown, lv_color_hex(0x101F34));
  lv_style_set_bg_grad_dir(&style_dropdown, LV_GRAD_DIR_VER);
  lv_style_set_bg_opa(&style_dropdown, LV_OPA_COVER);
  lv_style_set_border_color(&style_dropdown, lv_color_hex(0x00E5FF));
  lv_style_set_border_width(&style_dropdown, 1);
  lv_style_set_border_opa(&style_dropdown, LV_OPA_50);
  lv_style_set_text_color(&style_dropdown, lv_color_hex(0xD7E9FF));
  lv_style_set_pad_all(&style_dropdown, 6);
  lv_style_set_shadow_width(&style_dropdown, 10);
  lv_style_set_shadow_color(&style_dropdown, lv_color_hex(0x00E5FF));
  lv_style_set_shadow_opa(&style_dropdown, LV_OPA_20);
}

static SignalConfig presetCfgFromIndex(uint8_t idx, uint32_t rpm) {
  SignalConfig c{rpm, 60, 1, 2, GAP_AT_END, false};
  switch (idx) {
    case 0: c = {rpm, 60, 1, 2, GAP_AT_END, false}; break;
    case 1: c = {rpm, 36, 1, 1, GAP_AT_END, false}; break;
    case 2: c = {rpm, 36, 1, 2, GAP_AT_END, false}; break;
    case 3: c = {rpm, 36, 2, 1, GAP_AT_END, false}; break;
    case 4: c = {rpm, 12, 1, 1, GAP_AT_START, true}; break;
  }
  return c;
}

static void set_custom_error(const char* msg) {
  if (!lbl_custom_error) return;
  if (!msg || msg[0] == '\0') {
    lv_obj_add_flag(lbl_custom_error, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_label_set_text(lbl_custom_error, msg);
  lv_obj_clear_flag(lbl_custom_error, LV_OBJ_FLAG_HIDDEN);
}

static void on_spin_inc(lv_event_t* e) {
  lv_obj_t* spin = (lv_obj_t*)lv_event_get_user_data(e);
  if (!spin) return;
  lv_spinbox_increment(spin);
}

static void on_spin_dec(lv_event_t* e) {
  lv_obj_t* spin = (lv_obj_t*)lv_event_get_user_data(e);
  if (!spin) return;
  lv_spinbox_decrement(spin);
}

static lv_obj_t* make_spin_row(lv_obj_t* parent, const char* caption, lv_obj_t** out_spin, int32_t min, int32_t max, int32_t initial) {
  lv_obj_t* row = lv_obj_create(parent);
  lv_obj_set_size(row, lv_pct(100), 34);
  lv_obj_set_style_pad_all(row, 0, 0);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row, 0, 0);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 6, 0);

  lv_obj_t* lbl = lv_label_create(row);
  lv_label_set_text(lbl, caption);
  lv_obj_add_style(lbl, &style_caption, 0);
  lv_obj_set_flex_grow(lbl, 1);

  lv_obj_t* btn_minus = lv_btn_create(row);
  lv_obj_set_size(btn_minus, 32, 30);
  lv_obj_t* lbl_minus = lv_label_create(btn_minus);
  lv_label_set_text(lbl_minus, "-");
  lv_obj_center(lbl_minus);

  lv_obj_t* spin = lv_spinbox_create(row);
  lv_spinbox_set_range(spin, min, max);
  lv_spinbox_set_value(spin, initial);
  lv_spinbox_set_digit_format(spin, 4, 0);
  lv_obj_set_size(spin, 76, 30);

  lv_obj_t* btn_plus = lv_btn_create(row);
  lv_obj_set_size(btn_plus, 32, 30);
  lv_obj_t* lbl_plus = lv_label_create(btn_plus);
  lv_label_set_text(lbl_plus, "+");
  lv_obj_center(lbl_plus);

  lv_obj_add_event_cb(btn_minus, on_spin_dec, LV_EVENT_CLICKED, spin);
  lv_obj_add_event_cb(btn_plus,  on_spin_inc, LV_EVENT_CLICKED, spin);

  if (out_spin) *out_spin = spin;
  return row;
}

static void close_custom_panel() {
  if (overlay_custom) {
    lv_obj_del(overlay_custom);
  }
  overlay_custom = nullptr;
  panel_custom = nullptr;
  spin_teeth = nullptr;
  spin_pmiss = nullptr;
  spin_nmiss = nullptr;
  dd_gap_pos = nullptr;
  sw_gap_lvl = nullptr;
  lbl_custom_error = nullptr;
}

static void on_custom_cancel(lv_event_t* e) {
  LV_UNUSED(e);
  close_custom_panel();
}

static void on_custom_apply(lv_event_t* e) {
  LV_UNUSED(e);
  if (!spin_teeth || !spin_pmiss || !spin_nmiss || !dd_gap_pos || !sw_gap_lvl) return;

  const uint32_t rpm = arc_rpm ? (uint32_t)lv_arc_get_value(arc_rpm) : 1000u;

  SignalConfig cfg{};
  cfg.rpm = rpm;
  cfg.nTeeth = (uint16_t)lv_spinbox_get_value(spin_teeth);
  cfg.pMiss = (uint8_t)lv_spinbox_get_value(spin_pmiss);
  cfg.nMiss = (uint8_t)lv_spinbox_get_value(spin_nmiss);

  const uint16_t posSel = lv_dropdown_get_selected(dd_gap_pos);
  cfg.gapPos = (posSel == 1) ? GAP_AT_START : GAP_AT_END;
  cfg.gapLvl = lv_obj_has_state(sw_gap_lvl, LV_STATE_CHECKED);

  if (!validateSignalConfig(cfg)) {
    set_custom_error("Invalid combination");
    return;
  }

  if (!s_on_custom) {
    set_custom_error("Custom callback missing");
    return;
  }

  ui_show_error("");
  set_custom_error("");
  close_custom_panel();
  s_on_custom(cfg);
}

static void open_custom_panel() {
  if (!screen_main) return;
  if (overlay_custom) return;

  overlay_custom = lv_obj_create(screen_main);
  lv_obj_set_size(overlay_custom, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(overlay_custom, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(overlay_custom, LV_OPA_70, 0);
  lv_obj_set_style_border_width(overlay_custom, 0, 0);
  lv_obj_set_style_pad_all(overlay_custom, 0, 0);
  lv_obj_clear_flag(overlay_custom, LV_OBJ_FLAG_SCROLLABLE);

  panel_custom = lv_obj_create(overlay_custom);
  lv_obj_set_size(panel_custom, 380, 240);
  lv_obj_center(panel_custom);
  lv_obj_add_style(panel_custom, &style_dropdown, 0);
  lv_obj_set_style_pad_all(panel_custom, 10, 0);

  lv_obj_t* title = lv_label_create(panel_custom);
  lv_label_set_text(title, "CUSTOM PATTERN");
  lv_obj_add_style(title, &style_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

  lv_obj_t* hint = lv_label_create(panel_custom);
  lv_label_set_text(hint, "RPM uses the dial on the left");
  lv_obj_add_style(hint, &style_caption, 0);
  lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 18);

  const uint32_t rpm = arc_rpm ? (uint32_t)lv_arc_get_value(arc_rpm) : 1000u;
  const SignalConfig seed = presetCfgFromIndex(s_last_preset_pattern, rpm);

  lv_obj_t* rows = lv_obj_create(panel_custom);
  lv_obj_set_size(rows, lv_pct(100), 140);
  lv_obj_align(rows, LV_ALIGN_TOP_MID, 0, 40);
  lv_obj_set_style_bg_opa(rows, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rows, 0, 0);
  lv_obj_set_style_pad_all(rows, 0, 0);
  lv_obj_set_style_pad_row(rows, 6, 0);
  lv_obj_set_flex_flow(rows, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(rows, LV_OBJ_FLAG_SCROLLABLE);

  (void)make_spin_row(rows, "Teeth", &spin_teeth, 1, 120, seed.nTeeth);
  (void)make_spin_row(rows, "Periods/Rev", &spin_pmiss, 1, 10, seed.pMiss);
  (void)make_spin_row(rows, "Missing/Period", &spin_nmiss, 1, 60, seed.nMiss);

  lv_obj_t* rowPos = lv_obj_create(rows);
  lv_obj_set_size(rowPos, lv_pct(100), 34);
  lv_obj_set_style_bg_opa(rowPos, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rowPos, 0, 0);
  lv_obj_set_style_pad_all(rowPos, 0, 0);

  lv_obj_t* lblPos = lv_label_create(rowPos);
  lv_label_set_text(lblPos, "Gap Pos");
  lv_obj_add_style(lblPos, &style_caption, 0);
  lv_obj_align(lblPos, LV_ALIGN_LEFT_MID, 0, 0);

  dd_gap_pos = lv_dropdown_create(rowPos);
  lv_dropdown_set_options(dd_gap_pos, "END\nSTART");
  lv_obj_set_width(dd_gap_pos, 140);
  lv_obj_align(dd_gap_pos, LV_ALIGN_RIGHT_MID, 0, 0);
  lv_obj_add_style(dd_gap_pos, &style_dropdown, LV_PART_MAIN);
  lv_dropdown_set_selected(dd_gap_pos, (seed.gapPos == GAP_AT_START) ? 1 : 0);

  lv_obj_t* rowLvl = lv_obj_create(rows);
  lv_obj_set_size(rowLvl, lv_pct(100), 34);
  lv_obj_set_style_bg_opa(rowLvl, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rowLvl, 0, 0);
  lv_obj_set_style_pad_all(rowLvl, 0, 0);

  lv_obj_t* lblLvl = lv_label_create(rowLvl);
  lv_label_set_text(lblLvl, "Gap HIGH");
  lv_obj_add_style(lblLvl, &style_caption, 0);
  lv_obj_align(lblLvl, LV_ALIGN_LEFT_MID, 0, 0);

  sw_gap_lvl = lv_switch_create(rowLvl);
  lv_obj_align(sw_gap_lvl, LV_ALIGN_RIGHT_MID, 0, 0);
  if (seed.gapLvl) lv_obj_add_state(sw_gap_lvl, LV_STATE_CHECKED);

  lbl_custom_error = lv_label_create(panel_custom);
  lv_obj_add_style(lbl_custom_error, &style_caption, 0);
  lv_label_set_text(lbl_custom_error, "");
  lv_obj_align(lbl_custom_error, LV_ALIGN_BOTTOM_LEFT, 0, -48);
  lv_obj_add_flag(lbl_custom_error, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* btnCancel = lv_btn_create(panel_custom);
  lv_obj_set_size(btnCancel, 120, 38);
  lv_obj_align(btnCancel, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_event_cb(btnCancel, on_custom_cancel, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lblCancel = lv_label_create(btnCancel);
  lv_label_set_text(lblCancel, "CANCEL");
  lv_obj_center(lblCancel);

  lv_obj_t* btnApply = lv_btn_create(panel_custom);
  lv_obj_set_size(btnApply, 120, 38);
  lv_obj_align(btnApply, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_add_event_cb(btnApply, on_custom_apply, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lblApply = lv_label_create(btnApply);
  lv_label_set_text(lblApply, "APPLY");
  lv_obj_center(lblApply);
}

static void style_pane(lv_obj_t* pane) {
  lv_obj_set_style_bg_opa(pane, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(pane, 0, 0);
  lv_obj_set_style_pad_all(pane, 0, 0);
  lv_obj_clear_flag(pane, LV_OBJ_FLAG_SCROLLABLE);
}

static void create_main_screen() {
  screen_main = lv_screen_active();
  lv_obj_remove_style_all(screen_main);
  lv_obj_add_style(screen_main, &style_bg, 0);
  lv_obj_clear_flag(screen_main, LV_OBJ_FLAG_SCROLLABLE);

  // Channel LEDs (top-left, on the root screen, outside panes).
  led_crank = lv_led_create(screen_main);
  lv_obj_set_size(led_crank, 14, 14);
  lv_obj_align(led_crank, LV_ALIGN_TOP_LEFT, 8, 8);
  lv_led_set_color(led_crank, lv_color_hex(0x00E5FF));
  lv_led_on(led_crank);

  led_cam1 = lv_led_create(screen_main);
  lv_obj_set_size(led_cam1, 14, 14);
  lv_obj_align(led_cam1, LV_ALIGN_TOP_LEFT, 28, 8);
  lv_led_set_color(led_cam1, lv_color_hex(0x37425A));
  lv_led_set_brightness(led_cam1, 60);
  lv_led_on(led_cam1);

  led_cam2 = lv_led_create(screen_main);
  lv_obj_set_size(led_cam2, 14, 14);
  lv_obj_align(led_cam2, LV_ALIGN_TOP_LEFT, 48, 8);
  lv_led_set_color(led_cam2, lv_color_hex(0x37425A));
  lv_led_set_brightness(led_cam2, 60);
  lv_led_on(led_cam2);

  // Title centered at top.
  lbl_title = lv_label_create(screen_main);
  lv_label_set_text(lbl_title, "CKP SIGNAL");
  lv_obj_add_style(lbl_title, &style_title, 0);
  lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 10);

  // ---- LEFT PANE: RPM arc ----
  lv_obj_t* left_pane = lv_obj_create(screen_main);
  lv_obj_set_pos(left_pane, LEFT_X, LEFT_Y);
  lv_obj_set_size(left_pane, LEFT_W, LEFT_H);
  style_pane(left_pane);

  arc_rpm = lv_arc_create(left_pane);
  lv_obj_set_size(arc_rpm, 196, 196);
  lv_arc_set_rotation(arc_rpm, 135);
  lv_arc_set_bg_angles(arc_rpm, 0, 270);
  lv_arc_set_mode(arc_rpm, kArcReverse ? LV_ARC_MODE_REVERSE : LV_ARC_MODE_NORMAL);
  lv_arc_set_range(arc_rpm, 100, 6000);
  lv_arc_set_value(arc_rpm, 1000);
  lv_obj_add_style(arc_rpm, &style_arc_main, LV_PART_MAIN);
  lv_obj_add_style(arc_rpm, &style_arc_indic, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(arc_rpm, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_set_style_border_width(arc_rpm, 0, LV_PART_KNOB);
  lv_obj_add_flag(arc_rpm, LV_OBJ_FLAG_ADV_HITTEST);
  lv_obj_align(arc_rpm, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(arc_rpm, on_arc_changed, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(arc_rpm, on_arc_changed, LV_EVENT_RELEASED, NULL);

  lbl_rpm_value = lv_label_create(left_pane);
  lv_obj_add_style(lbl_rpm_value, &style_value, 0);
  lv_label_set_text(lbl_rpm_value, "1000");
  lv_obj_align_to(lbl_rpm_value, arc_rpm, LV_ALIGN_CENTER, 0, -6);

  lbl_rpm_caption = lv_label_create(left_pane);
  lv_obj_add_style(lbl_rpm_caption, &style_caption, 0);
  lv_label_set_text(lbl_rpm_caption, "RPM");
  lv_obj_align_to(lbl_rpm_caption, arc_rpm, LV_ALIGN_CENTER, 0, 16);

  // ---- RIGHT PANE: controls ----
  lv_obj_t* right_pane = lv_obj_create(screen_main);
  lv_obj_set_pos(right_pane, RIGHT_X, RIGHT_Y);
  lv_obj_set_size(right_pane, RIGHT_W, RIGHT_H);
  style_pane(right_pane);

  // Pattern label
  lbl_pattern = lv_label_create(right_pane);
  lv_obj_add_style(lbl_pattern, &style_caption, 0);
  lv_label_set_text(lbl_pattern, "PATTERN");
  lv_obj_set_pos(lbl_pattern, 8, 2);

  // Pattern dropdown
  dd_patterns = lv_dropdown_create(right_pane);
  rebuild_pattern_dropdown_options(nullptr);
  lv_obj_set_pos(dd_patterns, 8, 18);
  lv_obj_set_size(dd_patterns, 212, 34);
  lv_obj_add_style(dd_patterns, &style_dropdown, LV_PART_MAIN);
  lv_obj_t* list = lv_dropdown_get_list(dd_patterns);
  if (list) {
    lv_obj_add_style(list, &style_dropdown, LV_PART_MAIN);
    // Capped so the open list (top ~y 64) stays partially visible above the
    // 116 px on-screen keyboard that rises from the bottom when filtering.
    lv_obj_set_height(list, 120);
  }
  lv_obj_add_event_cb(dd_patterns, on_pattern_changed, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(dd_patterns, on_pattern_open, LV_EVENT_CLICKED, NULL);

  // Filter textarea
  ta_pattern_filter = lv_textarea_create(right_pane);
  lv_textarea_set_one_line(ta_pattern_filter, true);
  lv_textarea_set_placeholder_text(ta_pattern_filter, "filter...");
  lv_obj_set_pos(ta_pattern_filter, 8, 62);
  lv_obj_set_size(ta_pattern_filter, 212, 28);
  lv_obj_add_event_cb(ta_pattern_filter, on_pattern_filter_changed,
                      LV_EVENT_VALUE_CHANGED, NULL);
  // On-screen keyboard: raise on focus/click, dismiss on defocus or the
  // keyboard's OK / close keys (forwarded as READY / CANCEL).
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_focused, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_focused, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_defocused, LV_EVENT_DEFOCUSED, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_defocused, LV_EVENT_READY, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_defocused, LV_EVENT_CANCEL, NULL);

  // 2x2 action grid: SWEEP / COMP / DSL / WAVE
  struct BtnSpec { const char* text; int x; int y; lv_event_cb_t cb; };
  const BtnSpec grid_specs[] = {
    {"SWEEP",   8, 100, open_sweep_panel},
    {"COMP",  120, 100, open_comp_panel},
    {"DSL",     8, 142, open_dsl_panel},
    {"WAVE",  120, 142, open_wave_panel},
  };
  for (size_t i = 0; i < sizeof(grid_specs) / sizeof(grid_specs[0]); ++i) {
    lv_obj_t* b = lv_btn_create(right_pane);
    lv_obj_set_pos(b, grid_specs[i].x, grid_specs[i].y);
    lv_obj_set_size(b, 100, 34);
    lv_obj_add_event_cb(b, grid_specs[i].cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl = lv_label_create(b);
    lv_label_set_text(lbl, grid_specs[i].text);
    lv_obj_center(lbl);
  }

  // INVERT + START/STOP bottom row
  btn_invert = lv_btn_create(right_pane);
  lv_obj_set_pos(btn_invert, 8, 198);
  lv_obj_set_size(btn_invert, 100, 40);
  lv_obj_add_event_cb(btn_invert, on_invert_clicked, LV_EVENT_CLICKED, NULL);
  lbl_invert = lv_label_create(btn_invert);
  refresh_invert_label();
  lv_obj_center(lbl_invert);

  btn_run = lv_btn_create(right_pane);
  lv_obj_set_pos(btn_run, 120, 198);
  lv_obj_set_size(btn_run, 100, 40);
  lv_obj_add_event_cb(btn_run, on_run_clicked, LV_EVENT_CLICKED, NULL);
  lbl_run = lv_label_create(btn_run);
  lv_label_set_text(lbl_run, s_running ? "STOP" : "START");
  lv_obj_center(lbl_run);

  // Error label — bottom of screen, spans the left half (under the arc).
  lbl_error = lv_label_create(screen_main);
  lv_obj_add_style(lbl_error, &style_caption, 0);
  lv_label_set_text(lbl_error, "");
  lv_obj_set_pos(lbl_error, 8, 252);
  lv_obj_set_size(lbl_error, 230, 18);
  lv_label_set_long_mode(lbl_error, LV_LABEL_LONG_DOT);
  lv_obj_add_flag(lbl_error, LV_OBJ_FLAG_HIDDEN);

  update_rpm_label(lv_arc_get_value(arc_rpm));
}


static void update_rpm_label(int32_t rpm) {
  if (!lbl_rpm_value) return;
  lv_label_set_text_fmt(lbl_rpm_value, "%ld", (long)rpm);
}

static void on_arc_changed(lv_event_t* e) {
  lv_obj_t* arc = lv_event_get_target_obj(e);
  int32_t rpm = lv_arc_get_value(arc);
  update_rpm_label(rpm);
  if (s_suppress_rpm_cb || !s_on_rpm) return;

  static int32_t  s_last_sent_rpm = -1;
  static uint32_t s_last_send_ms  = 0;

  const lv_event_code_t code = lv_event_get_code(e);
  const uint32_t now_ms = millis();
  const bool released = (code == LV_EVENT_RELEASED);
  const bool throttled_ok = (now_ms - s_last_send_ms) >= 50;

  if (released || (throttled_ok && rpm != s_last_sent_rpm)) {
    s_last_sent_rpm = rpm;
    s_last_send_ms  = now_ms;
    s_on_rpm((uint32_t)rpm);
  }
}

static void on_pattern_changed(lv_event_t* e) {
  lv_obj_t* dd = lv_event_get_target_obj(e);
  uint16_t sel = lv_dropdown_get_selected(dd);

  if (s_suppress_pattern_cb) return;
  if (sel >= s_pattern_dd_entry_count) return;

  // Resolve through the dd->builtin mapping. Category headers map to -1
  // and the next real entry is auto-selected.
  int16_t builtin_idx = s_pattern_dd_to_builtin[sel];
  if (builtin_idx < 0) {
    // Header row — advance to next valid entry.
    for (uint8_t i = sel + 1; i < s_pattern_dd_entry_count; ++i) {
      if (s_pattern_dd_to_builtin[i] >= 0) {
        s_suppress_pattern_cb = true;
        lv_dropdown_set_selected(dd, i);
        s_suppress_pattern_cb = false;
        builtin_idx = s_pattern_dd_to_builtin[i];
        break;
      }
    }
    if (builtin_idx < 0) return;
  }
  s_last_preset_pattern = (uint8_t)builtin_idx;
  if (s_on_pattern) s_on_pattern((uint8_t)builtin_idx);
}


static void on_pattern_open(lv_event_t* e) {
  lv_obj_t* dd = lv_event_get_target_obj(e);
  lv_obj_t* list = lv_dropdown_get_list(dd);
  if (list) lv_obj_move_foreground(list);
}

static void refresh_run_label() {
  if (!lbl_run) return;
  lv_label_set_text(lbl_run, s_running ? "STOP" : "START");
}

static void refresh_invert_label() {
  if (!lbl_invert) return;
  lv_label_set_text(lbl_invert, s_inverted ? "INVERT ON" : "INVERT OFF");
}

static void on_run_clicked(lv_event_t* e) {
  LV_UNUSED(e);
  if (s_suppress_run_cb) return;

  s_running = !s_running;
  refresh_run_label();

  if (s_on_run) s_on_run(s_running);
}

static void on_invert_clicked(lv_event_t* e) {
  LV_UNUSED(e);
  if (s_suppress_invert_cb) return;

  s_inverted = !s_inverted;
  refresh_invert_label();

  if (s_on_invert) s_on_invert(s_inverted);
}

bool ui_init(ui_on_rpm_cb on_rpm, ui_on_pattern_cb on_pattern, ui_on_run_cb on_run, ui_on_custom_cb on_custom, ui_on_invert_cb on_invert) {
  s_on_rpm = on_rpm;
  s_on_pattern = on_pattern;
  s_on_run = on_run;
  s_on_custom = on_custom;
  s_on_invert = on_invert;
  if (s_lvgl_ready) return true;


  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
    return false;
  }

  const uint8_t chosen_rotation = pick_display_rotation();

  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
  gfx->setRotation(chosen_rotation);
  gfx->fillScreen(RGB565_BLACK);

  Wire.begin(kTouchSda, kTouchScl);
  touchController.begin();
  touchController.setRotation(kTouchRotation);

  Serial.printf("[ui] display rotation=%u width=%d height=%d touch_rotation=%d\n",
                (unsigned)chosen_rotation,
                (int)gfx->width(), (int)gfx->height(),
                (int)kTouchRotation);

  lv_init();
  lv_tick_set_cb(millis_cb);

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  s_screen_w = gfx->width();
  s_screen_h = gfx->height();
  touchController.setResolution(s_screen_w, s_screen_h);

  const uint8_t row_candidates[] = {40, 20, 10, 5};
  s_draw_buf = nullptr;
  size_t buf_bytes = 0;

  for (uint8_t i = 0; i < sizeof(row_candidates); ++i) {
    s_draw_buf_px = s_screen_w * row_candidates[i];
    buf_bytes = s_draw_buf_px * sizeof(lv_color_t);

    s_draw_buf = static_cast<lv_color_t*>(heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!s_draw_buf) {
      s_draw_buf = static_cast<lv_color_t*>(heap_caps_malloc(buf_bytes, MALLOC_CAP_8BIT));
    }
    if (s_draw_buf) break;
  }

  if (!s_draw_buf) {
    Serial.println("LVGL draw buffer alloc failed!");
    return false;
  }

  s_disp = lv_display_create(s_screen_w, s_screen_h);
  lv_display_set_flush_cb(s_disp, my_disp_flush);
  lv_display_set_buffers(s_disp, s_draw_buf, NULL, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);

  s_indev = lv_indev_create();
  lv_indev_set_type(s_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(s_indev, my_touchpad_read);
  lv_indev_set_display(s_indev, s_disp);

  init_styles();
  create_main_screen();
  s_lvgl_ready = true;
  return true;
}


bool ui_is_ready() {
  return s_lvgl_ready;
}

void ui_update_rpm(uint32_t rpm) {
  portENTER_CRITICAL(&s_ui_mux);
  s_pending_rpm_val = rpm;
  s_pending_rpm = true;
  portEXIT_CRITICAL(&s_ui_mux);
}

void ui_update_pattern(uint8_t pattern_index) {
  portENTER_CRITICAL(&s_ui_mux);
  s_pending_pattern_val = pattern_index;
  s_pending_pattern = true;
  portEXIT_CRITICAL(&s_ui_mux);
}

void ui_update_running(bool running) {
  portENTER_CRITICAL(&s_ui_mux);
  s_pending_running_val = running;
  s_pending_running = true;
  portEXIT_CRITICAL(&s_ui_mux);
}

void ui_update_inverted(bool inverted) {
  portENTER_CRITICAL(&s_ui_mux);
  s_pending_inverted_val = inverted;
  s_pending_inverted = true;
  portEXIT_CRITICAL(&s_ui_mux);
}

void ui_show_error(const char* msg) {
  portENTER_CRITICAL(&s_ui_mux);
  strncpy(s_pending_error_msg, msg ? msg : "", sizeof(s_pending_error_msg));
  s_pending_error_msg[sizeof(s_pending_error_msg) - 1] = '\0';
  s_pending_error = true;
  portEXIT_CRITICAL(&s_ui_mux);
}

void ui_update_channels(uint8_t channel_mask, uint8_t invert_mask) {
  portENTER_CRITICAL(&s_ui_mux);
  s_pending_channel_mask = channel_mask;
  s_pending_invert_mask  = invert_mask;
  s_pending_channels = true;
  portEXIT_CRITICAL(&s_ui_mux);
}

// LED apply — runs on LVGL thread (called from apply_pending_updates).
static void apply_channel_leds(uint8_t channel_mask, uint8_t invert_mask) {
  struct LedRow { lv_obj_t* led; uint8_t bit; };
  LedRow rows[3] = {
    { led_crank, 0x01 },
    { led_cam1,  0x02 },
    { led_cam2,  0x04 },
  };
  for (int i = 0; i < 3; ++i) {
    if (!rows[i].led) continue;
    const bool active   = (channel_mask & rows[i].bit) != 0;
    const bool inverted = (invert_mask  & rows[i].bit) != 0;
    if (!active) {
      // Greyed-out — channel unused by current pattern.
      lv_led_set_color(rows[i].led, lv_color_hex(0x37425A));
      lv_led_set_brightness(rows[i].led, 60);
    } else {
      lv_led_set_color(rows[i].led,
                       inverted ? lv_color_hex(0xFFB020)
                                : lv_color_hex(0x00E5FF));
      lv_led_set_brightness(rows[i].led, 220);
    }
  }
  s_visible_channel_mask = channel_mask;
  s_visible_invert_mask  = invert_mask;
}

static void apply_pending_updates() {
  bool hasRpm = false;
  uint32_t rpm = 0;
  bool hasPattern = false;
  uint8_t pattern = 0;
  bool hasRunning = false;
  bool running = false;
  bool hasInverted = false;
  bool inverted = false;
  bool hasError = false;
  char errorMsg[sizeof(s_pending_error_msg)];

  bool hasChannels = false;
  uint8_t chan_mask = 0x01;
  uint8_t inv_mask  = 0x00;

  portENTER_CRITICAL(&s_ui_mux);
  if (s_pending_rpm) { hasRpm = true; rpm = s_pending_rpm_val; s_pending_rpm = false; }
  if (s_pending_pattern) { hasPattern = true; pattern = s_pending_pattern_val; s_pending_pattern = false; }
  if (s_pending_running) { hasRunning = true; running = s_pending_running_val; s_pending_running = false; }
  if (s_pending_inverted) { hasInverted = true; inverted = s_pending_inverted_val; s_pending_inverted = false; }
  if (s_pending_error) {
    hasError = true;
    strncpy(errorMsg, s_pending_error_msg, sizeof(errorMsg));
    errorMsg[sizeof(errorMsg) - 1] = '\0';
    s_pending_error = false;
  }
  if (s_pending_channels) {
    hasChannels = true;
    chan_mask = s_pending_channel_mask;
    inv_mask  = s_pending_invert_mask;
    s_pending_channels = false;
  }
  portEXIT_CRITICAL(&s_ui_mux);

  if (hasChannels) apply_channel_leds(chan_mask, inv_mask);

  if (hasRpm && arc_rpm) {
    s_suppress_rpm_cb = true;
    lv_arc_set_value(arc_rpm, (int32_t)rpm);
    update_rpm_label((int32_t)rpm);
    s_suppress_rpm_cb = false;

    if (lbl_rpm_value) {
      lv_obj_set_style_text_color(lbl_rpm_value, lv_color_hex(0xFFB020), 0);
      s_rpm_flash_until_ms = millis() + 600;
    }
  }

  if (hasPattern && dd_patterns) {
    // M3.4: reverse-map builtin index -> dd row.
    int row = -1;
    for (uint8_t i = 0; i < s_pattern_dd_entry_count; ++i) {
      if (s_pattern_dd_to_builtin[i] == (int16_t)pattern) { row = i; break; }
    }
    if (row >= 0) {
      s_last_preset_pattern = pattern;
      s_suppress_pattern_cb = true;
      lv_dropdown_set_selected(dd_patterns, (uint16_t)row);
      s_suppress_pattern_cb = false;
    }
  }

  if (hasRunning) {
    s_suppress_run_cb = true;
    s_running = running;
    refresh_run_label();
    s_suppress_run_cb = false;
  }

  if (hasInverted) {
    s_suppress_invert_cb = true;
    s_inverted = inverted;
    refresh_invert_label();
    s_suppress_invert_cb = false;
    // Mirror crank-channel invert state into the LED row.
    uint8_t new_inv = (uint8_t)((s_visible_invert_mask & 0xFEu) | (inverted ? 0x01u : 0x00u));
    apply_channel_leds(s_visible_channel_mask, new_inv);
  }

  if (hasError && lbl_error) {
    if (errorMsg[0] == '\0') {
      lv_obj_add_flag(lbl_error, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_label_set_text(lbl_error, errorMsg);
      lv_obj_clear_flag(lbl_error, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (lbl_rpm_value && s_rpm_flash_until_ms != 0 && (int32_t)(millis() - s_rpm_flash_until_ms) >= 0) {
    lv_obj_set_style_text_color(lbl_rpm_value, lv_color_hex(0x00E5FF), 0);
    s_rpm_flash_until_ms = 0;
  }
}

void ui_task_handler() {
  if (!s_lvgl_ready) return;
  lv_timer_handler();
  apply_pending_updates();
}

// =====================================================
// M3.4 — 64-pattern dropdown with category headers + search
// =====================================================

static const char* category_for_pattern(const PatternRef* p) {
  if (!p || !p->name_key) return "Other";
  if (strncmp(p->name_key, "dizzy_", 6) == 0) return "Distributor";
  if (p->channel_mask & 0x06) return "Crank+Cam";
  if (p->channel_mask == 0x01) {
    // Heuristic: missing-tooth wheels have name keys containing "_minus_"
    // (e.g. sixty_minus_two, thirty_six_minus_one) or numeric "X_minus_Y".
    if (strstr(p->name_key, "_minus_") != nullptr) return "Missing-tooth";
    return "Angular OEM";
  }
  return "Angular OEM";
}

// ---- Fuzzy search (R3) ----
//
// fzf-lite subsequence scorer. `name_matches_filter` is kept as a thin
// wrapper so the two existing call sites in the dropdown rebuild are
// unchanged; the rebuild itself uses fuzzy_score() to rank rows.

static inline char ci_lower(char c) {
  return (char)tolower((unsigned char)c);
}

// Case-insensitive substring test — lifted from the old matcher loop so we
// don't depend on the non-portable strcasestr.
static bool ci_substr(const char* hay, const char* needle) {
  if (!hay || !needle) return false;
  if (!*needle) return true;
  const size_t nlen = strlen(needle);
  for (const char* p = hay; *p; ++p) {
    size_t i = 0;
    for (; i < nlen; ++i) {
      char a = ci_lower(p[i]);
      char b = ci_lower(needle[i]);
      if (!a || a != b) break;
    }
    if (i == nlen) return true;
  }
  return false;
}

// True if `c` begins a "word" relative to the preceding char `prev`:
// after a separator, or on a digit<->alpha transition.
static inline bool is_word_boundary(char prev, char c) {
  if (prev == '_' || prev == '-' || prev == ' ') return true;
  const bool prev_d = isdigit((unsigned char)prev) != 0;
  const bool cur_d  = isdigit((unsigned char)c) != 0;
  const bool prev_a = isalpha((unsigned char)prev) != 0;
  const bool cur_a  = isalpha((unsigned char)c) != 0;
  if (prev_d && cur_a) return true;
  if (prev_a && cur_d) return true;
  return false;
}

// Case-insensitive subsequence walk of `needle` over `hay`. Returns INT_MIN
// when `needle` is not a subsequence; otherwise a positive-ish score that
// rewards consecutive / word-boundary / start-of-string matches and
// penalises gaps. Allocation-free, O(|hay|).
static int fuzzy_score_one(const char* hay, const char* needle) {
  if (!hay) return INT_MIN;
  if (!needle || !*needle) return 0;

  int score = 0;
  bool prev_matched = false;   // was the previous hay char a match?
  size_t ni = 0;               // index into needle
  const char* h = hay;
  char prev_c = '\0';

  for (size_t hi = 0; h[hi]; ++hi) {
    char hc = ci_lower(h[hi]);
    char nc = ci_lower(needle[ni]);
    if (hc == nc) {
      score += 4;                                  // MATCH
      if (prev_matched) score += 8;                // CONSECUTIVE
      if (hi == 0) score += 12;                     // START-OF-STRING
      else if (is_word_boundary(prev_c, h[hi])) score += 10;  // WORD-BOUNDARY
      prev_matched = true;
      if (!needle[++ni]) return score;             // consumed whole needle
    } else {
      if (prev_matched == false) score -= 1;       // GAP (only between hits)
      prev_matched = false;
    }
    prev_c = h[hi];
  }
  return INT_MIN;  // needle not fully consumed → not a subsequence
}

// Best score over the friendly name and the raw key, with a strong bonus
// when either contains the filter as a contiguous (case-insensitive)
// substring so exact hits rank above scattered subsequences.
static int fuzzy_score(const PatternRef* p, const char* friendly,
                       const char* filter) {
  if (!filter || !*filter) return 0;
  const char* key = p ? p->name_key : nullptr;
  int best = INT_MIN;
  const int sf = fuzzy_score_one(friendly, filter);
  const int sk = fuzzy_score_one(key, filter);
  if (sf > best) best = sf;
  if (sk > best) best = sk;
  if (best == INT_MIN) return INT_MIN;
  if (ci_substr(friendly, filter) || ci_substr(key, filter)) best += 100;
  return best;
}

static bool name_matches_filter(const char* friendly, const char* key,
                                 const char* filter) {
  if (!filter || !*filter) return true;
  // Wrapper over the scorer so existing call sites stay unchanged: a row
  // matches iff the filter is a (fuzzy) subsequence of name or key.
  PatternRef tmp{};
  tmp.name_key = key;
  return fuzzy_score(&tmp, friendly, filter) > INT_MIN;
}

// One scored dropdown candidate. Kept tiny + plain-old-data so the working
// array below can live in .bss (see note on the static array).
struct Cand {
  int16_t     idx;    // builtin index (always a real pattern, never a header)
  int         score;  // fuzzy_score(); 0 when no filter is active
  const char* label;  // friendly name (or raw key) — points into .rodata
};

static void rebuild_pattern_dropdown_options(const char* filter) {
  if (!dd_patterns) return;
  // Build options string in 4 category-ordered passes, ranking rows within
  // each category by fuzzy score when a filter is active.
  static char opts[4096];
  size_t off = 0;
  s_pattern_dd_entry_count = 0;

  // Scratch candidate buffer for the current category. STATIC on purpose:
  // ~136 * 16 B ≈ 2.2 KB belongs in .bss, NOT on the 8 KB loopTask stack
  // that LVGL event dispatch already burdens.
  static Cand cands[UI_PATTERN_DD_CAP];

  const char* cats[] = { "Distributor", "Missing-tooth", "Crank+Cam", "Angular OEM" };
  const size_t n = PatternLibrary::builtinCount();
  const bool has_filter = (filter && *filter);

  for (int ci = 0; ci < 4; ++ci) {
    // Collect (and score) every matching pattern in this category.
    int cand_count = 0;
    for (size_t i = 0; i < n; ++i) {
      const PatternRef* p = PatternLibrary::builtinByIndex(i);
      if (!p) continue;
      if (strcmp(category_for_pattern(p), cats[ci]) != 0) continue;
      const char* friendly = PatternLibrary::friendlyName(p->name_key);
      const char* label = friendly ? friendly : p->name_key;
      if (!label) continue;
      const int score = fuzzy_score(p, friendly, filter);  // 0 when no filter
      if (score == INT_MIN) continue;                       // not a match
      if (cand_count >= UI_PATTERN_DD_CAP) break;
      cands[cand_count].idx   = (int16_t)i;
      cands[cand_count].score = score;
      cands[cand_count].label = label;
      ++cand_count;
    }
    if (cand_count == 0) continue;

    // When filtering, insertion-sort descending by score (stable: equal
    // scores keep library order). With no filter, all scores are 0 so the
    // list stays in its current, stable library order — skip the sort.
    if (has_filter) {
      for (int a = 1; a < cand_count; ++a) {
        const Cand key = cands[a];
        int b = a - 1;
        while (b >= 0 && cands[b].score < key.score) {
          cands[b + 1] = cands[b];
          --b;
        }
        cands[b + 1] = key;
      }
    }

    // Header row (maps to -1). Preserve the existing capacity guard.
    if (s_pattern_dd_entry_count >= UI_PATTERN_DD_CAP) break;
    if (off > 0 && off + 1 < sizeof(opts)) opts[off++] = '\n';
    {
      const char* hdr_prefix = "-- ";
      const char* hdr_suffix = " --";
      const size_t need = strlen(hdr_prefix) + strlen(cats[ci]) + strlen(hdr_suffix);
      if (off + need + 1 >= sizeof(opts)) break;
      memcpy(opts + off, hdr_prefix, strlen(hdr_prefix)); off += strlen(hdr_prefix);
      memcpy(opts + off, cats[ci], strlen(cats[ci]));    off += strlen(cats[ci]);
      memcpy(opts + off, hdr_suffix, strlen(hdr_suffix)); off += strlen(hdr_suffix);
    }
    s_pattern_dd_to_builtin[s_pattern_dd_entry_count++] = -1;

    // Emit labels and the builtin map in the SAME (ranked) order.
    for (int k = 0; k < cand_count; ++k) {
      if (s_pattern_dd_entry_count >= UI_PATTERN_DD_CAP) break;
      const char* label = cands[k].label;
      const size_t lab_len = strlen(label);
      if (off + 1 + lab_len + 1 >= sizeof(opts)) break;
      opts[off++] = '\n';
      memcpy(opts + off, label, lab_len);
      off += lab_len;
      s_pattern_dd_to_builtin[s_pattern_dd_entry_count++] = cands[k].idx;
    }
  }
  if (off < sizeof(opts)) opts[off] = '\0';
  else opts[sizeof(opts) - 1] = '\0';

  lv_dropdown_set_options(dd_patterns, opts);
}

static void on_pattern_filter_changed(lv_event_t* e) {
  lv_obj_t* ta = lv_event_get_target_obj(e);
  const char* txt = lv_textarea_get_text(ta);
  strncpy(s_pattern_filter, txt ? txt : "", sizeof(s_pattern_filter) - 1);
  s_pattern_filter[sizeof(s_pattern_filter) - 1] = '\0';
  rebuild_pattern_dropdown_options(s_pattern_filter);
}

// ---- On-screen keyboard (R3) ----
//
// One lazily-created keyboard on lv_layer_top(), shared by the filter and
// DSL textareas. The project default font (dejavu_16_persian_hebrew,
// lv_conf.h:522) lacks the LV_SYMBOL_* glyphs, so the keyboard's special
// keys render as tofu — we MUST force montserrat_14 which carries them.
static lv_obj_t* ui_get_keyboard() {
  if (kb_filter) return kb_filter;
  kb_filter = lv_keyboard_create(lv_layer_top());
  lv_keyboard_set_mode(kb_filter, LV_KEYBOARD_MODE_TEXT_LOWER);
  lv_obj_set_size(kb_filter, 480, 116);
  lv_obj_align(kb_filter, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_text_font(kb_filter, &lv_font_montserrat_14, 0);
  lv_obj_add_flag(kb_filter, LV_OBJ_FLAG_HIDDEN);
  return kb_filter;
}

// Bind the textarea BEFORE unhiding: the keyboard only forwards READY/CANCEL
// (its OK / close keys) to a bound textarea (lv_keyboard.c:356-374).
static void kb_show_for(lv_obj_t* ta) {
  lv_obj_t* kb = ui_get_keyboard();
  if (!kb || !ta) return;
  lv_keyboard_set_textarea(kb, ta);
  lv_obj_move_foreground(kb);
  lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
}

static void kb_hide() {
  if (!kb_filter) return;
  lv_keyboard_set_textarea(kb_filter, NULL);  // drop the binding first
  lv_obj_add_flag(kb_filter, LV_OBJ_FLAG_HIDDEN);
}

static void on_ta_focused(lv_event_t* e) {
  lv_obj_t* ta = lv_event_get_target_obj(e);
  kb_show_for(ta);
}

static void on_ta_defocused(lv_event_t* e) {
  LV_UNUSED(e);
  kb_hide();
}

// =====================================================
// M4.5 — Sweep + Compression modals
// =====================================================

static void close_sweep_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (tmr_sweep_live) { lv_timer_del(tmr_sweep_live); tmr_sweep_live = nullptr; }
  if (overlay_sweep) { lv_obj_del(overlay_sweep); overlay_sweep = nullptr; }
  spin_sweep_low = spin_sweep_high = spin_sweep_iv = nullptr;
  dd_sweep_mode = lbl_sweep_live = nullptr;
}

static void on_sweep_live_tick(lv_timer_t* t) {
  LV_UNUSED(t);
  if (!lbl_sweep_live) return;
  lv_label_set_text_fmt(lbl_sweep_live, "Live: %u RPM", (unsigned)sweepCurrentRpm());
}

static void on_sweep_apply(lv_event_t* e) {
  LV_UNUSED(e);
  if (!spin_sweep_low || !spin_sweep_high || !dd_sweep_mode || !spin_sweep_iv) return;
  CtrlMsg m{};
  m.type = MSG_SET_SWEEP;
  m.payload.sweep.low_rpm     = (uint16_t)lv_spinbox_get_value(spin_sweep_low);
  m.payload.sweep.high_rpm    = (uint16_t)lv_spinbox_get_value(spin_sweep_high);
  m.payload.sweep.mode        = (uint8_t)lv_dropdown_get_selected(dd_sweep_mode);
  m.payload.sweep.interval_us = (uint32_t)lv_spinbox_get_value(spin_sweep_iv);
  if (gCtrlQ) (void)xQueueSend(gCtrlQ, &m, 0);
  close_sweep_panel(nullptr);
}

static void open_sweep_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_sweep) return;
  overlay_sweep = lv_obj_create(screen_main);
  lv_obj_set_size(overlay_sweep, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(overlay_sweep, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(overlay_sweep, LV_OPA_70, 0);
  lv_obj_set_style_border_width(overlay_sweep, 0, 0);
  lv_obj_clear_flag(overlay_sweep, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* panel = lv_obj_create(overlay_sweep);
  lv_obj_set_size(panel, 380, 240);
  lv_obj_center(panel);
  lv_obj_add_style(panel, &style_dropdown, 0);
  lv_obj_set_style_pad_all(panel, 10, 0);

  lv_obj_t* title = lv_label_create(panel);
  lv_label_set_text(title, "SWEEP");
  lv_obj_add_style(title, &style_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

  lv_obj_t* rows = lv_obj_create(panel);
  lv_obj_set_size(rows, lv_pct(100), 140);
  lv_obj_align(rows, LV_ALIGN_TOP_MID, 0, 26);
  lv_obj_set_style_bg_opa(rows, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rows, 0, 0);
  lv_obj_set_style_pad_all(rows, 0, 0);
  lv_obj_set_style_pad_row(rows, 6, 0);
  lv_obj_set_flex_flow(rows, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(rows, LV_OBJ_FLAG_SCROLLABLE);

  make_spin_row(rows, "Low RPM", &spin_sweep_low, 100, 6000, g_sweep_low_rpm);
  make_spin_row(rows, "High RPM", &spin_sweep_high, 100, 6000, g_sweep_high_rpm);
  // Mode dropdown
  lv_obj_t* rowMode = lv_obj_create(rows);
  lv_obj_set_size(rowMode, lv_pct(100), 34);
  lv_obj_set_style_bg_opa(rowMode, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rowMode, 0, 0);
  lv_obj_set_style_pad_all(rowMode, 0, 0);
  lv_obj_t* lblMode = lv_label_create(rowMode);
  lv_label_set_text(lblMode, "Mode");
  lv_obj_add_style(lblMode, &style_caption, 0);
  lv_obj_align(lblMode, LV_ALIGN_LEFT_MID, 0, 0);
  dd_sweep_mode = lv_dropdown_create(rowMode);
  lv_dropdown_set_options(dd_sweep_mode, "OFF\nLINEAR\nLOG\nWAYPOINT");
  lv_dropdown_set_selected(dd_sweep_mode, g_sweep_mode);
  lv_obj_set_width(dd_sweep_mode, 140);
  lv_obj_align(dd_sweep_mode, LV_ALIGN_RIGHT_MID, 0, 0);

  make_spin_row(rows, "Interval us", &spin_sweep_iv, 100, 100000, (int)g_sweep_interval_us);

  lbl_sweep_live = lv_label_create(panel);
  lv_obj_add_style(lbl_sweep_live, &style_caption, 0);
  lv_label_set_text(lbl_sweep_live, "Live: ---");
  lv_obj_align(lbl_sweep_live, LV_ALIGN_BOTTOM_LEFT, 0, -48);

  lv_obj_t* btnCancel = lv_btn_create(panel);
  lv_obj_set_size(btnCancel, 100, 36);
  lv_obj_align(btnCancel, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_event_cb(btnCancel, close_sweep_panel, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l1 = lv_label_create(btnCancel); lv_label_set_text(l1, "CANCEL"); lv_obj_center(l1);

  lv_obj_t* btnApply = lv_btn_create(panel);
  lv_obj_set_size(btnApply, 100, 36);
  lv_obj_align(btnApply, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_add_event_cb(btnApply, on_sweep_apply, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l2 = lv_label_create(btnApply); lv_label_set_text(l2, "APPLY"); lv_obj_center(l2);

  tmr_sweep_live = lv_timer_create(on_sweep_live_tick, 100, NULL);
}

static void close_comp_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (overlay_comp) { lv_obj_del(overlay_comp); overlay_comp = nullptr; }
  sw_comp_en = sw_comp_dyn = nullptr;
  spin_comp_cyl = spin_comp_thr = spin_comp_peak = nullptr;
}

static void on_comp_apply(lv_event_t* e) {
  LV_UNUSED(e);
  if (!sw_comp_en || !sw_comp_dyn || !spin_comp_cyl || !spin_comp_thr || !spin_comp_peak) return;
  CtrlMsg m{};
  m.type = MSG_SET_COMPRESSION;
  m.payload.comp.enabled    = lv_obj_has_state(sw_comp_en, LV_STATE_CHECKED);
  m.payload.comp.cyl        = (uint8_t)lv_spinbox_get_value(spin_comp_cyl);
  m.payload.comp.rpm_thresh = (uint16_t)lv_spinbox_get_value(spin_comp_thr);
  m.payload.comp.peak       = (uint8_t)lv_spinbox_get_value(spin_comp_peak);
  m.payload.comp.dynamic    = lv_obj_has_state(sw_comp_dyn, LV_STATE_CHECKED);
  if (gCtrlQ) (void)xQueueSend(gCtrlQ, &m, 0);
  close_comp_panel(nullptr);
}

static void open_comp_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_comp) return;
  overlay_comp = lv_obj_create(screen_main);
  lv_obj_set_size(overlay_comp, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(overlay_comp, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(overlay_comp, LV_OPA_70, 0);
  lv_obj_set_style_border_width(overlay_comp, 0, 0);
  lv_obj_clear_flag(overlay_comp, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* panel = lv_obj_create(overlay_comp);
  lv_obj_set_size(panel, 380, 240);
  lv_obj_center(panel);
  lv_obj_add_style(panel, &style_dropdown, 0);
  lv_obj_set_style_pad_all(panel, 10, 0);

  lv_obj_t* title = lv_label_create(panel);
  lv_label_set_text(title, "COMPRESSION");
  lv_obj_add_style(title, &style_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

  lv_obj_t* rows = lv_obj_create(panel);
  lv_obj_set_size(rows, lv_pct(100), 150);
  lv_obj_align(rows, LV_ALIGN_TOP_MID, 0, 26);
  lv_obj_set_style_bg_opa(rows, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rows, 0, 0);
  lv_obj_set_style_pad_all(rows, 0, 0);
  lv_obj_set_style_pad_row(rows, 4, 0);
  lv_obj_set_flex_flow(rows, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(rows, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* rowEn = lv_obj_create(rows);
  lv_obj_set_size(rowEn, lv_pct(100), 30);
  lv_obj_set_style_bg_opa(rowEn, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rowEn, 0, 0);
  lv_obj_set_style_pad_all(rowEn, 0, 0);
  lv_obj_t* lblEn = lv_label_create(rowEn);
  lv_label_set_text(lblEn, "Enabled");
  lv_obj_add_style(lblEn, &style_caption, 0);
  lv_obj_align(lblEn, LV_ALIGN_LEFT_MID, 0, 0);
  sw_comp_en = lv_switch_create(rowEn);
  lv_obj_align(sw_comp_en, LV_ALIGN_RIGHT_MID, 0, 0);
  if (g_comp_enabled) lv_obj_add_state(sw_comp_en, LV_STATE_CHECKED);

  make_spin_row(rows, "Cylinders", &spin_comp_cyl, 1, 12, g_comp_cyl);
  make_spin_row(rows, "RPM Thresh", &spin_comp_thr, 100, 6000, g_comp_rpm_thresh);
  make_spin_row(rows, "Peak", &spin_comp_peak, 0, 255, g_comp_peak);

  lv_obj_t* rowDyn = lv_obj_create(rows);
  lv_obj_set_size(rowDyn, lv_pct(100), 30);
  lv_obj_set_style_bg_opa(rowDyn, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rowDyn, 0, 0);
  lv_obj_set_style_pad_all(rowDyn, 0, 0);
  lv_obj_t* lblDyn = lv_label_create(rowDyn);
  lv_label_set_text(lblDyn, "Dynamic");
  lv_obj_add_style(lblDyn, &style_caption, 0);
  lv_obj_align(lblDyn, LV_ALIGN_LEFT_MID, 0, 0);
  sw_comp_dyn = lv_switch_create(rowDyn);
  lv_obj_align(sw_comp_dyn, LV_ALIGN_RIGHT_MID, 0, 0);
  if (g_comp_dynamic) lv_obj_add_state(sw_comp_dyn, LV_STATE_CHECKED);

  lv_obj_t* btnCancel = lv_btn_create(panel);
  lv_obj_set_size(btnCancel, 100, 36);
  lv_obj_align(btnCancel, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_event_cb(btnCancel, close_comp_panel, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l1 = lv_label_create(btnCancel); lv_label_set_text(l1, "CANCEL"); lv_obj_center(l1);

  lv_obj_t* btnApply = lv_btn_create(panel);
  lv_obj_set_size(btnApply, 100, 36);
  lv_obj_align(btnApply, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_add_event_cb(btnApply, on_comp_apply, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l2 = lv_label_create(btnApply); lv_label_set_text(l2, "APPLY"); lv_obj_center(l2);
}

// =====================================================
// M5.7 — DSL editor modal
// =====================================================

static void on_dsl_err_tick(lv_timer_t* t) {
  LV_UNUSED(t);
  if (!lbl_dsl_err) return;
  if (g_dsl_error[0]) {
    lv_label_set_text(lbl_dsl_err, (const char*)g_dsl_error);
  } else {
    lv_label_set_text(lbl_dsl_err, "OK");
  }
}

static void close_dsl_panel(lv_event_t* e) {
  LV_UNUSED(e);
  // Hide + unbind the keyboard BEFORE deleting the modal so it never holds
  // a dangling pointer to the about-to-be-freed ta_dsl_src.
  kb_hide();
  if (tmr_dsl_err) { lv_timer_del(tmr_dsl_err); tmr_dsl_err = nullptr; }
  if (overlay_dsl) { lv_obj_del(overlay_dsl); overlay_dsl = nullptr; }
  ta_dsl_src = nullptr;
  lbl_dsl_err = nullptr;
}

static void on_dsl_compile(lv_event_t* e) {
  LV_UNUSED(e);
  if (!ta_dsl_src) return;
  const char* src = lv_textarea_get_text(ta_dsl_src);
  if (!src) return;
  // Heap-copy the source; manager will free.
  char* heap = (char*)malloc(strlen(src) + 1);
  if (!heap) return;
  strcpy(heap, src);
  CtrlMsg m{};
  m.type = MSG_LOAD_DSL;
  m.payload.name = heap;
  if (!gCtrlQ || xQueueSend(gCtrlQ, &m, 0) != pdTRUE) {
    free(heap);
  }
}

static void on_dsl_saveas(lv_event_t* e) {
  LV_UNUSED(e);
  // Route through MSG_SAVE_USER so the manager task owns persistence.
  // We heap-allocate copies of BOTH the key and the DSL source; the
  // manager free()'s them after PatternStorage::saveDsl().
  if (!ta_dsl_src) return;
  const char* src = lv_textarea_get_text(ta_dsl_src);
  if (!src) return;

  char key[32];
  snprintf(key, sizeof(key), "scratch_%lu", (unsigned long)millis());

  char* name_heap = (char*)malloc(strlen(key) + 1);
  char* src_heap  = (char*)malloc(strlen(src) + 1);
  if (!name_heap || !src_heap) {
    free(name_heap);
    free(src_heap);
    if (lbl_dsl_err) lv_label_set_text(lbl_dsl_err, "save: oom");
    return;
  }
  strcpy(name_heap, key);
  strcpy(src_heap,  src);

  CtrlMsg m{};
  m.type = MSG_SAVE_USER;
  m.payload.save.name       = name_heap;
  m.payload.save.dsl_source = src_heap;
  if (!sendCtrlMsg(m)) {
    free(name_heap);
    free(src_heap);
    if (lbl_dsl_err) lv_label_set_text(lbl_dsl_err, "save: queue full");
    return;
  }
  if (lbl_dsl_err) lv_label_set_text_fmt(lbl_dsl_err, "saving %s", key);
}

static void on_dsl_load(lv_event_t* e) {
  LV_UNUSED(e);
  // List patterns; pick the first one (stub for full file-picker).
  char keys[8][PatternStorage::KEY_BUFLEN];
  size_t n = PatternStorage::listPatterns(keys, 8);
  if (n == 0) {
    if (lbl_dsl_err) lv_label_set_text(lbl_dsl_err, "no saved patterns");
    return;
  }
  char buf[2048];
  if (!PatternStorage::loadDsl(keys[0], buf, sizeof(buf))) {
    if (lbl_dsl_err) lv_label_set_text(lbl_dsl_err, "load failed");
    return;
  }
  if (ta_dsl_src) lv_textarea_set_text(ta_dsl_src, buf);
  if (lbl_dsl_err) lv_label_set_text_fmt(lbl_dsl_err, "loaded %s", keys[0]);
}

static void open_dsl_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_dsl) return;
  overlay_dsl = lv_obj_create(screen_main);
  lv_obj_set_size(overlay_dsl, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(overlay_dsl, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(overlay_dsl, LV_OPA_80, 0);
  lv_obj_set_style_border_width(overlay_dsl, 0, 0);
  lv_obj_clear_flag(overlay_dsl, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* panel = lv_obj_create(overlay_dsl);
  lv_obj_set_size(panel, 460, 250);
  lv_obj_center(panel);
  lv_obj_add_style(panel, &style_dropdown, 0);
  lv_obj_set_style_pad_all(panel, 8, 0);

  lv_obj_t* title = lv_label_create(panel);
  lv_label_set_text(title, "DSL");
  lv_obj_add_style(title, &style_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

  ta_dsl_src = lv_textarea_create(panel);
  lv_obj_set_size(ta_dsl_src, 440, 140);
  lv_obj_align(ta_dsl_src, LV_ALIGN_TOP_LEFT, 0, 20);
  lv_textarea_set_placeholder_text(ta_dsl_src, "wheel DSL source...");
  // Reuse the shared on-screen keyboard for DSL entry.
  lv_obj_add_event_cb(ta_dsl_src, on_ta_focused, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_focused, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_defocused, LV_EVENT_DEFOCUSED, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_defocused, LV_EVENT_READY, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_defocused, LV_EVENT_CANCEL, NULL);

  lbl_dsl_err = lv_label_create(panel);
  lv_obj_add_style(lbl_dsl_err, &style_caption, 0);
  lv_label_set_text(lbl_dsl_err, "");
  lv_obj_align(lbl_dsl_err, LV_ALIGN_BOTTOM_LEFT, 0, -42);

  // Buttons row
  lv_obj_t* btnCompile = lv_btn_create(panel);
  lv_obj_set_size(btnCompile, 90, 32);
  lv_obj_align(btnCompile, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_event_cb(btnCompile, on_dsl_compile, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l1 = lv_label_create(btnCompile); lv_label_set_text(l1, "COMPILE"); lv_obj_center(l1);

  lv_obj_t* btnSave = lv_btn_create(panel);
  lv_obj_set_size(btnSave, 90, 32);
  lv_obj_align(btnSave, LV_ALIGN_BOTTOM_LEFT, 95, 0);
  lv_obj_add_event_cb(btnSave, on_dsl_saveas, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l2 = lv_label_create(btnSave); lv_label_set_text(l2, "SAVE"); lv_obj_center(l2);

  lv_obj_t* btnLoad = lv_btn_create(panel);
  lv_obj_set_size(btnLoad, 90, 32);
  lv_obj_align(btnLoad, LV_ALIGN_BOTTOM_LEFT, 190, 0);
  lv_obj_add_event_cb(btnLoad, on_dsl_load, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l3 = lv_label_create(btnLoad); lv_label_set_text(l3, "LOAD"); lv_obj_center(l3);

  lv_obj_t* btnClose = lv_btn_create(panel);
  lv_obj_set_size(btnClose, 90, 32);
  lv_obj_align(btnClose, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_add_event_cb(btnClose, close_dsl_panel, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l4 = lv_label_create(btnClose); lv_label_set_text(l4, "CLOSE"); lv_obj_center(l4);

  tmr_dsl_err = lv_timer_create(on_dsl_err_tick, 250, NULL);
}

// =====================================================
// M7 — Waveform canvas
// =====================================================

// Active pattern access — main.cpp maintains gActivePattern; we hook
// a getter via an extern. For independence, fall back to the first
// builtin if unset.
extern "C" const PatternRef* ui_get_active_pattern_for_wave();

// Default hook returns first builtin if main.cpp's symbol isn't linked
// (e.g. unit-test stub).
__attribute__((weak)) const PatternRef* ui_get_active_pattern_for_wave() {
  return PatternLibrary::builtinByIndex(0);
}

// At 1000 RPM the cursor must traverse pattern in
//    60 ms (360 deg)  or 120 ms (720 deg).
// Math: rev_us = 60 / RPM seconds = 60_000_000/RPM us. For RPM=1000 →
// 60_000 us = 60 ms. A 720-degree pattern covers 2 revs → 120 ms.
// Cursor index = (now_us - cycle_start_us) / period_us; period_us =
// rev_us / slot_count. We compute it from PatternRef + IGenerator state
// (gGen.getEdgeCounter() gives exact slot — preferred).
extern "C" uint16_t ui_get_edge_counter();
__attribute__((weak)) uint16_t ui_get_edge_counter() { return 0; }

// ---- Bounded direct-to-canvas pixel helpers (R4) ----
//
// These write RGB565 (LV_COLOR_FORMAT_NATIVE at LV_COLOR_DEPTH 16) straight
// into the canvas draw buffer — ZERO lv_draw_* task allocation, so nothing
// can exhaust the 64 KB LVGL pool the way the old lv_draw_line path did.
//
// CRITICAL: all clamps are against the draw-buf header (db->header.w/h) and
// the row stride is db->header.stride which is in BYTES — never lv_obj
// geometry, never a hardcoded w*2. lv_canvas_set_buffer() sizes data to
// exactly stride*h with no slack, so out-of-range clamping is mandatory.

static inline uint16_t* wave_row(lv_draw_buf_t* db, int y) {
  return (uint16_t*)(db->data + (uint32_t)y * db->header.stride);
}

// Horizontal run [x0..x1] on row y, color c.
static void wave_hfill(lv_draw_buf_t* db, int y, int x0, int x1, uint16_t c) {
  const int W = (int)db->header.w, H = (int)db->header.h;
  if (y < 0 || y >= H) return;
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  if (x0 < 0) x0 = 0;
  if (x1 >= W) x1 = W - 1;
  uint16_t* row = wave_row(db, y);
  for (int x = x0; x <= x1; ++x) row[x] = c;
}

// Vertical run [y0..y1] on column x, color c.
static void wave_vfill(lv_draw_buf_t* db, int x, int y0, int y1, uint16_t c) {
  const int W = (int)db->header.w, H = (int)db->header.h;
  if (x < 0 || x >= W) return;
  if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
  if (y0 < 0) y0 = 0;
  if (y1 >= H) y1 = H - 1;
  for (int y = y0; y <= y1; ++y) wave_row(db, y)[x] = c;
}

// Fill the whole buffer with color c.
static void wave_clear(lv_draw_buf_t* db, uint16_t c) {
  const int W = (int)db->header.w, H = (int)db->header.h;
  for (int y = 0; y < H; ++y) {
    uint16_t* row = wave_row(db, y);
    for (int x = 0; x < W; ++x) row[x] = c;
  }
}

static void on_wave_tick(lv_timer_t* t) {
  LV_UNUSED(t);
  if (!canvas_wave) return;
  lv_draw_buf_t* db = lv_canvas_get_draw_buf(canvas_wave);
  if (!db || !db->data) return;
  const PatternRef* p = ui_get_active_pattern_for_wave();
  if (!p || !p->table || p->slot_count == 0) return;

  const int w = (int)db->header.w;
  const int h = (int)db->header.h;
  if (w <= 0 || h <= 0) return;
  if (db->header.cf != LV_COLOR_FORMAT_NATIVE) return;  // RGB565 only

  // Pre-pack colors once (lv_color_to_u16 takes an lv_color_t struct, so the
  // hex literals must go through lv_color_hex first).
  const uint16_t bg_c = lv_color_to_u16(lv_color_hex(0x0B1020));
  const uint16_t lane_c[3] = {
    lv_color_to_u16(lv_color_hex(0x00E5FF)),
    lv_color_to_u16(lv_color_hex(0xFFB020)),
    lv_color_to_u16(lv_color_hex(0x7CFFB0)),
  };
  const uint16_t cursor_c = lv_color_to_u16(lv_color_hex(0xFF4060));
  const uint8_t lane_bits[3] = { 0x01, 0x02, 0x04 };

  wave_clear(db, bg_c);

  const int lane_h = h / 3;
  const int slot_count = (int)p->slot_count;
  const int zoom = (s_wave_zoom > 0 ? s_wave_zoom : 1);
  // Fit mode: when the zoomed pattern is wider than the canvas, decimate
  // each column over a slot RANGE so the whole wheel stays visible.
  const bool fit = ((long)slot_count * zoom > (long)w);

  for (int lane = 0; lane < 3; ++lane) {
    if (!(s_wave_lane_mask & lane_bits[lane])) continue;
    const uint16_t c = lane_c[lane];
    const int y_lo = lane * lane_h + lane_h - 4;   // logic LOW row
    const int y_hi = lane * lane_h + 4;            // logic HIGH row
    int prev_y = y_lo;

    for (int x = 0; x < w; ++x) {
      // Map this column to a slot range [s0..s1].
      int s0, s1;
      if (fit) {
        s0 = (int)((long)x * slot_count / w);
        s1 = (int)(((long)(x + 1) * slot_count) / w) - 1;
        if (s1 < s0) s1 = s0;
      } else {
        s0 = x / zoom;
        s1 = s0;
      }
      if (s0 >= slot_count) break;
      if (s1 >= slot_count) s1 = slot_count - 1;

      // Envelope over the slot range: any HIGH / any LOW.
      bool any_hi = false, any_lo = false;
      for (int s = s0; s <= s1; ++s) {
        if (p->table[s] & lane_bits[lane]) any_hi = true;
        else any_lo = true;
        if (any_hi && any_lo) break;
      }
      const int y = any_hi ? y_hi : y_lo;

      // 2 px vertical edge on level change.
      if (x > 0 && y != prev_y) {
        wave_vfill(db, x,     prev_y, y, c);
        wave_vfill(db, x + 1, prev_y, y, c);
      }
      // 2 px horizontal level for this column.
      wave_hfill(db, y,     x, x, c);
      wave_hfill(db, y + 1, x, x, c);
      // Sub-pixel toggling within the range: draw the full envelope.
      if (any_hi && any_lo) {
        wave_vfill(db, x, y_hi, y_lo, c);
      }
      prev_y = y;
    }
  }

  // Cursor — 1 px, follows getEdgeCounter (M7.1).
  const uint16_t cur = ui_get_edge_counter();
  if (cur < slot_count) {
    const int cx = fit ? (int)((long)cur * w / slot_count) : (int)cur * zoom;
    if (cx < w) wave_vfill(db, cx, 0, h - 1, cursor_c);
  }

  lv_obj_invalidate(canvas_wave);
}

static void close_wave_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (tmr_wave) { lv_timer_del(tmr_wave); tmr_wave = nullptr; }
  if (canvas_wave_buf) { heap_caps_free(canvas_wave_buf); canvas_wave_buf = nullptr; }
  if (overlay_wave) { lv_obj_del(overlay_wave); overlay_wave = nullptr; }
  canvas_wave = nullptr;
}

static void open_wave_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_wave) return;
  overlay_wave = lv_obj_create(screen_main);
  lv_obj_set_size(overlay_wave, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(overlay_wave, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(overlay_wave, LV_OPA_90, 0);
  lv_obj_clear_flag(overlay_wave, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* panel = lv_obj_create(overlay_wave);
  lv_obj_set_size(panel, 460, 240);
  lv_obj_center(panel);
  lv_obj_add_style(panel, &style_dropdown, 0);
  lv_obj_set_style_pad_all(panel, 6, 0);

  lv_obj_t* title = lv_label_create(panel);
  lv_label_set_text(title, "WAVEFORM");
  lv_obj_add_style(title, &style_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

  // 432x144 → lane_h = 48 exact (old 440x160 left a 160/3 remainder row).
  const int cw = 432;
  const int ch = 144;
  const size_t buf_bytes = LV_CANVAS_BUF_SIZE(cw, ch, LV_COLOR_DEPTH, LV_DRAW_BUF_STRIDE_ALIGN);
  // PSRAM-first: relieves internal system-heap pressure while WAVE is open
  // (~124 KB). A PSRAM software-render target is fine at 20 Hz.
  canvas_wave_buf = (lv_color_t*)heap_caps_malloc(buf_bytes,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!canvas_wave_buf) {
    canvas_wave_buf = (lv_color_t*)heap_caps_malloc(buf_bytes,
                                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (canvas_wave_buf) {
    canvas_wave = lv_canvas_create(panel);
    lv_canvas_set_buffer(canvas_wave, canvas_wave_buf, cw, ch, LV_COLOR_FORMAT_NATIVE);
    lv_obj_set_size(canvas_wave, cw, ch);
    lv_obj_align(canvas_wave, LV_ALIGN_TOP_LEFT, 0, 22);
  }

  lv_obj_t* btnClose = lv_btn_create(panel);
  lv_obj_set_size(btnClose, 90, 30);
  lv_obj_align(btnClose, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_add_event_cb(btnClose, close_wave_panel, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l1 = lv_label_create(btnClose); lv_label_set_text(l1, "CLOSE"); lv_obj_center(l1);

  // 50 ms (20 Hz) — at 1000 RPM the 60-2 cursor (120 slots, 500 us/slot)
  // hops ~100 slots between frames; entire pattern visibly traverses
  // in 60 ms for a 360-degree wheel, 120 ms for a 720-degree wheel.
  tmr_wave = lv_timer_create(on_wave_tick, 50, NULL);
}

#endif  // SIGGEN_HAS_DISPLAY

