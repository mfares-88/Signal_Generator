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
#include "dsl_help.h"


// =====================================================
// Unified cyan-accent HUD palette (Implementation-5, D11).
// Distinct surface above bg; FILLED accent buttons (dark text);
// primary actions get an inline cyan glow (not in shared style).
// =====================================================
#define COL_BG       0x0B1020
#define COL_SURFACE  0x141C2E   // now DISTINCT from bg
#define COL_SUNKEN   0x0F1628
#define COL_ACCENT   0x00E5FF
#define COL_WARN     0xFFB020
#define COL_TEXT     0xD7E9FF
#define COL_MUTED    0x7C8DB0
#define COL_LED_OFF  0x37425A


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
// 2-tab navigation (Decision 1). HOME = index 0 (default), ADVANCED = 1.
static lv_obj_t* tabview = nullptr;
static lv_obj_t* tab_home = nullptr;
static lv_obj_t* tab_adv  = nullptr;
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

// M4.5: Sweep + compression pages (full-screen overlays).
static lv_obj_t* overlay_sweep = nullptr;
static lv_obj_t* spin_sweep_low = nullptr;
static lv_obj_t* spin_sweep_high = nullptr;
static lv_obj_t* dd_sweep_mode  = nullptr;
static lv_obj_t* spin_sweep_iv  = nullptr;
static lv_obj_t* lbl_sweep_live = nullptr;
static lv_timer_t* tmr_sweep_live = nullptr;
static lv_obj_t* arc_sweep_live = nullptr;   // page-local live RPM arc (Decision 14)

static lv_obj_t* overlay_comp = nullptr;
static lv_obj_t* sw_comp_en   = nullptr;
static lv_obj_t* spin_comp_cyl = nullptr;
static lv_obj_t* spin_comp_thr = nullptr;
static lv_obj_t* spin_comp_peak = nullptr;
static lv_obj_t* sw_comp_dyn   = nullptr;
static lv_obj_t* arc_comp_live = nullptr;    // page-local live RPM arc
static lv_timer_t* tmr_comp_live = nullptr;  // Comp page lazy 100ms arc timer

// Page-local live RPM arcs for WAVE / CUSTOM (Decision 14). DSL arc/timer
// removed (E2-6/D12): the DSL page now has no live arc; the editor textarea
// claims the freed width.
static lv_obj_t* arc_wave_live = nullptr;
static lv_timer_t* tmr_wave_live = nullptr;
static lv_obj_t* arc_custom_live = nullptr;
static lv_timer_t* tmr_custom_live = nullptr;

// ON/OFF status pills below each page arc (D7). Created in open_*_panel,
// nulled in close_*_panel; driven from each 100ms tick + START/STOP handlers.
static lv_obj_t* lbl_sweep_status  = nullptr;
static lv_obj_t* lbl_comp_status   = nullptr;
static lv_obj_t* lbl_custom_status = nullptr;

// Sweep arc center value + direction glyph (D17). Updated each tick from
// sweepCurrentRpm(); the glyph compares against the previous tick's RPM.
static lv_obj_t* lbl_sweep_arc_val = nullptr;
static lv_obj_t* lbl_sweep_arc_dir = nullptr;
static uint32_t  s_sweep_prev_rpm  = 0;
// Comp arc center value (D17).
static lv_obj_t* lbl_comp_arc_val  = nullptr;

// Comp arc dual-mode edge guard (D5): comp_arc_set_interactive(bool) early-
// returns when already in the requested state so repeated STOP taps never
// stack event descriptors / grow the 64KB pool. Reset in close_comp_panel.
static bool s_comp_arc_interactive = false;

// Modal numeric keypad (D8) — replaces spinbox steppers. The map MUST be a
// file-scope `static const char* const`: lv_buttonmatrix_set_map STORES the
// pointer (does NOT copy), so a stack-local array would dangle -> crash.
static const char* const kKeypadMap[] = {
  "1", "2", "3", "\n",
  "4", "5", "6", "\n",
  "7", "8", "9", "\n",
  "Clear", "0", LV_SYMBOL_BACKSPACE, "\n",
  "Back", "OK", ""
};
static lv_obj_t* overlay_keypad    = nullptr;
static lv_obj_t* lbl_keypad_value  = nullptr;
static struct { lv_obj_t* target; int32_t min, max; char buf[16]; uint8_t len; } s_kp;

// Per-value-box keypad field descriptor (D9). lv_spinbox_get_range/get_min/
// get_max DO NOT EXIST in 9.2.2, so min/max/name are bound per box via
// lv_obj_set_user_data and read back in on_value_box_clicked. <=3 boxes/page,
// one page open at a time -> the array stays valid while the keypad is up.
struct KpField { int32_t min, max; const char* name; };
static KpField  s_kp_fields[8];
static uint8_t  s_kp_field_n = 0;

// M5.7: DSL editor page + Help sub-page (Ref4).
static lv_obj_t* overlay_dsl = nullptr;
static lv_obj_t* overlay_dsl_help = nullptr;  // help sub-page overlay (above DSL page)
static lv_obj_t* ta_dsl_src  = nullptr;
static lv_obj_t* lbl_dsl_err = nullptr;
static lv_timer_t* tmr_dsl_err = nullptr;

// M7: waveform canvas.
static lv_obj_t*   overlay_wave   = nullptr;
static lv_obj_t*   canvas_wave    = nullptr;
static lv_color_t* canvas_wave_buf = nullptr;
static lv_timer_t* tmr_wave        = nullptr;
// Q24.8 continuous view model (E-wave-1, Decision 9b). All slot-space
// quantities are in 1/256-slot fixed point; 256 == 1.0 slot.
//   s_wave_zoom_x256 : 256 == 1.0x == full-fit; clamp [256, 256*32].
//   s_wave_panL_x256 : left edge in 1/256-slot units; clamp [0, full-visible].
static int      s_wave_zoom_x256    = 256;       // 256 == 1.0x (full-fit)
static long     s_wave_panL_x256    = 0;         // left edge, 1/256-slot units
static bool     s_wave_paused       = false;     // PAUSE freezes cursor only
static uint16_t s_wave_frozen_cursor = 0;        // cursor slot captured on pause
static bool     s_wave_dirty        = true;      // input cbs set; timer renders
static uint8_t  s_wave_lane_mask    = 0x07;      // bit0..2: lane visibility
// SLOW-PAN FIX (E-wave-6): set true while the WAVE page is open so
// my_touchpad_read skips its <3px/50ms move-coalescing — slow drags then
// report a real vector and can pan. False elsewhere (arc coalescing stays).
static bool     s_wave_drag_coalesce_off = false;

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
static lv_style_t style_dropdown;   // generalized: card/pane/input accent border + COL_SUNKEN fill
static lv_style_t style_btn;        // shared OUTLINE button (1px accent border, transparent fill)

static uint8_t pick_display_rotation();
static void init_styles();
static void create_main_screen();
static void build_home_tab(lv_obj_t* page);
static void build_advanced_tab(lv_obj_t* page);
static void on_tabview_changed(lv_event_t* e);
static void ui_force_output_off();
static void ui_force_output_on();
// Shared page helpers (full-screen overlay chrome + live RPM arc, Decision 14).
static lv_obj_t* make_page_overlay(lv_obj_t** slot, uint32_t bg_opa);
static lv_obj_t* make_back_header(lv_obj_t* parent, const char* title, lv_event_cb_t back_cb);
static lv_obj_t* make_page_live_arc(lv_obj_t* parent);
static void update_live_arc(lv_obj_t* arc);
static void on_adv_row_sweep(lv_event_t* e);
static void on_adv_row_comp(lv_event_t* e);
static void on_adv_row_dsl(lv_event_t* e);
static void on_adv_row_wave(lv_event_t* e);
static void on_adv_row_custom(lv_event_t* e);
static void on_sweep_back(lv_event_t* e);
static void on_sweep_stop(lv_event_t* e);
static void on_comp_back(lv_event_t* e);
static void on_comp_stop(lv_event_t* e);
static void on_dsl_back(lv_event_t* e);
static void on_wave_back(lv_event_t* e);
static void on_custom_back(lv_event_t* e);
static void on_comp_live_tick(lv_timer_t* t);
static void on_wave_live_tick(lv_timer_t* t);
static void on_custom_live_tick(lv_timer_t* t);
static void open_dsl_help(lv_event_t* e);
static void close_dsl_help(lv_event_t* e);
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
static void on_custom_apply(lv_event_t* e);
static void on_custom_cancel(lv_event_t* e);

// ON/OFF status pill + modal numeric keypad + value-box infra (D7/D8/D9).
static void update_status_pill(lv_obj_t* lbl);
static lv_obj_t* make_status_pill(lv_obj_t* parent, int x, int y);
static void open_numeric_keypad(lv_obj_t* target_spin, const char* field_name, int32_t min, int32_t max);
static void close_numeric_keypad(bool commit);
static void keypad_update_display();
static void on_keypad_button_click(lv_event_t* e);
static void make_value_box(lv_obj_t* parent, const char* caption, lv_obj_t** out_spin,
                           int32_t min, int32_t max, int32_t initial,
                           int cap_x, int cap_y, int box_x, int box_y, int box_w, int box_h);
static void on_value_box_clicked(lv_event_t* e);

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
static void comp_arc_set_interactive(bool on);

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

  // Slow-drag suppression: coalesce sub-3px moves within 50ms by reporting the
  // stale point (keeps the RPM arc from jittering). SKIP this on the WAVE page
  // (s_wave_drag_coalesce_off) so slow fine drags report a real vector and pan
  // (E-wave-6 / plan §10 budget MAJOR fix).
  if (!s_wave_drag_coalesce_off && last_pressed && (now_ms - last_ms) < 50) {
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

  // Flat surface == bg (Decision 10): no gradient, no glow anywhere.
  lv_style_init(&style_bg);
  lv_style_set_bg_color(&style_bg, lv_color_hex(COL_BG));
  lv_style_set_bg_grad_dir(&style_bg, LV_GRAD_DIR_NONE);
  lv_style_set_bg_opa(&style_bg, LV_OPA_COVER);

  // HUD header weight (page titles / keypad header), D11.
  lv_style_init(&style_title);
  lv_style_set_text_color(&style_title, lv_color_hex(COL_TEXT));
  lv_style_set_text_letter_space(&style_title, 2);
  lv_style_set_text_font(&style_title, &lv_font_montserrat_20);

  lv_style_init(&style_caption);
  lv_style_set_text_color(&style_caption, lv_color_hex(COL_MUTED));
  lv_style_set_text_font(&style_caption, &lv_font_montserrat_12);

  // Bold cyan HUD number (reference screenshot), D11.
  lv_style_init(&style_value);
  lv_style_set_text_color(&style_value, lv_color_hex(COL_ACCENT));
  lv_style_set_text_font(&style_value, &lv_font_montserrat_28);

  lv_style_init(&style_arc_main);
  lv_style_set_arc_width(&style_arc_main, 18);
  lv_style_set_arc_color(&style_arc_main, lv_color_hex(COL_SUNKEN));
  lv_style_set_arc_rounded(&style_arc_main, true);

  // Indicator: bright cyan fill, NO shadow (D11 — lv_draw_arc_dsc_t has no
  // shadow field; a true arc shadow-glow is not achievable in LVGL 9.2.2).
  lv_style_init(&style_arc_indic);
  lv_style_set_arc_width(&style_arc_indic, 18);
  lv_style_set_arc_color(&style_arc_indic, lv_color_hex(COL_ACCENT));
  lv_style_set_arc_rounded(&style_arc_indic, true);

  // Generalized card/pane/input: COL_SUNKEN fill + 1px accent-tinted border
  // (Decision 11), NO shadow.
  lv_style_init(&style_dropdown);
  lv_style_set_bg_color(&style_dropdown, lv_color_hex(COL_SUNKEN));
  lv_style_set_bg_grad_dir(&style_dropdown, LV_GRAD_DIR_NONE);
  lv_style_set_bg_opa(&style_dropdown, LV_OPA_COVER);
  lv_style_set_border_color(&style_dropdown, lv_color_hex(COL_ACCENT));
  lv_style_set_border_width(&style_dropdown, 1);
  lv_style_set_border_opa(&style_dropdown, LV_OPA_40);
  lv_style_set_text_color(&style_dropdown, lv_color_hex(COL_TEXT));
  lv_style_set_pad_all(&style_dropdown, 6);

  // Shared FILLED button (D11): cyan fill, dark-on-cyan label, 1px accent
  // border. NO shadow in the shared style — glow is applied INLINE only to
  // primary actions (review budget#5: confines per-draw sh_buf malloc/free to
  // ~2 buttons/screen since LV_DRAW_SW_SHADOW_CACHE_SIZE==0).
  lv_style_init(&style_btn);
  lv_style_set_bg_color(&style_btn, lv_color_hex(COL_ACCENT));
  lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);
  lv_style_set_border_color(&style_btn, lv_color_hex(COL_ACCENT));
  lv_style_set_border_width(&style_btn, 1);
  lv_style_set_border_opa(&style_btn, LV_OPA_COVER);
  lv_style_set_text_color(&style_btn, lv_color_hex(COL_BG));
  lv_style_set_shadow_width(&style_btn, 0);
  lv_style_set_radius(&style_btn, 8);
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

// Decision 12b: `left_justified` drops the label flex_grow so the control
// sits adjacent at the page gap (Sweep/Comp pass true; CUSTOM false).
// D9: steppers REMOVED — the spinbox is a tap-to-keypad value box. Caret is
// suppressed and a KpField is bound on user_data so on_value_box_clicked can
// open the modal keypad with this box's min/max/name. Signature unchanged
// (call sites depend on it byte-for-byte).
static lv_obj_t* make_spin_row(lv_obj_t* parent, const char* caption, lv_obj_t** out_spin, int32_t min, int32_t max, int32_t initial, bool left_justified = false) {
  lv_obj_t* row = lv_obj_create(parent);
  lv_obj_set_size(row, lv_pct(100), 42);
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
  if (!left_justified) lv_obj_set_flex_grow(lbl, 1);

  lv_obj_t* spin = lv_spinbox_create(row);
  lv_spinbox_set_range(spin, min, max);
  lv_spinbox_set_value(spin, initial);
  lv_spinbox_set_digit_format(spin, 4, 0);
  lv_obj_set_size(spin, 120, 40);
  lv_obj_add_style(spin, &style_dropdown, 0);

  // Tap-to-keypad: suppress the edit caret and bind this box's field
  // descriptor (D9). on_*_apply still reads via lv_spinbox_get_value.
  lv_textarea_set_cursor_click_pos(spin, false);
  if (s_kp_field_n < (uint8_t)(sizeof(s_kp_fields) / sizeof(s_kp_fields[0]))) {
    KpField* f = &s_kp_fields[s_kp_field_n++];
    f->min = min; f->max = max; f->name = caption;
    lv_obj_set_user_data(spin, f);
    lv_obj_add_event_cb(spin, on_value_box_clicked, LV_EVENT_CLICKED, NULL);
  }

  if (out_spin) *out_spin = spin;
  return row;
}

// Absolute-layout value box (D9/D18): a caption + a tap-to-keypad spinbox,
// both DIRECT children of `parent` (no flex row). Used by Sweep/Comp.
static void make_value_box(lv_obj_t* parent, const char* caption, lv_obj_t** out_spin,
                           int32_t min, int32_t max, int32_t initial,
                           int cap_x, int cap_y, int box_x, int box_y, int box_w, int box_h) {
  lv_obj_t* lbl = lv_label_create(parent);
  lv_label_set_text(lbl, caption);
  lv_obj_add_style(lbl, &style_caption, 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, cap_x, cap_y);

  lv_obj_t* spin = lv_spinbox_create(parent);
  lv_spinbox_set_range(spin, min, max);
  lv_spinbox_set_value(spin, initial);
  lv_spinbox_set_digit_format(spin, 4, 0);
  lv_obj_set_size(spin, box_w, box_h);
  lv_obj_align(spin, LV_ALIGN_TOP_LEFT, box_x, box_y);
  lv_obj_add_style(spin, &style_dropdown, 0);

  // Suppress the edit caret + bind keypad (D9).
  lv_textarea_set_cursor_click_pos(spin, false);
  if (s_kp_field_n < (uint8_t)(sizeof(s_kp_fields) / sizeof(s_kp_fields[0]))) {
    KpField* f = &s_kp_fields[s_kp_field_n++];
    f->min = min; f->max = max; f->name = caption;
    lv_obj_set_user_data(spin, f);
    lv_obj_add_event_cb(spin, on_value_box_clicked, LV_EVENT_CLICKED, NULL);
  }

  if (out_spin) *out_spin = spin;
}

// Tap a value box -> open the modal keypad with the bound field's limits/name.
static void on_value_box_clicked(lv_event_t* e) {
  lv_obj_t* spin = lv_event_get_target_obj(e);
  KpField* f = (KpField*)lv_obj_get_user_data(spin);
  if (f) open_numeric_keypad(spin, f->name, f->min, f->max);
}

static void close_custom_panel() {
  if (tmr_custom_live) { lv_timer_del(tmr_custom_live); tmr_custom_live = nullptr; }
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
  arc_custom_live = nullptr;
  lbl_custom_status = nullptr;        // E2-5: null the pill pointer
}

static void on_custom_live_tick(lv_timer_t* t) {
  LV_UNUSED(t);
  update_live_arc(arc_custom_live);
  update_status_pill(lbl_custom_status);
}

// CANCEL renamed -> Back: stop output, tear down. E2-5: defensively del any
// orphaned keypad first.
static void on_custom_back(lv_event_t* e) {
  LV_UNUSED(e);
  if (overlay_keypad) { lv_obj_del(overlay_keypad); overlay_keypad = nullptr; }
  ui_force_output_off();
  close_custom_panel();
}

static void on_custom_cancel(lv_event_t* e) {
  LV_UNUSED(e);
  on_custom_back(e);
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

  // D2: START = config-then-start, page STAYS OPEN (no close_custom_panel()).
  // Order on success: clear error -> push custom config -> start output ->
  // refresh pill. Validation failure already returned early above.
  ui_show_error("");
  set_custom_error("");
  s_on_custom(cfg);
  ui_force_output_on();
  update_status_pill(lbl_custom_status);
}

static void open_custom_panel() {
  if (!screen_main) return;
  if (overlay_custom) return;

  s_kp_field_n = 0;   // D9: reset keypad-field cursor (parity with Sweep/Comp)
  panel_custom = make_page_overlay(&overlay_custom, LV_OPA_COVER);
  make_back_header(panel_custom, "CUSTOM PATTERN", on_custom_back);

  lv_obj_t* hint = lv_label_create(panel_custom);
  lv_label_set_text(hint, "RPM uses the HOME dial");
  lv_obj_add_style(hint, &style_caption, 0);
  lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 28);

  const uint32_t rpm = arc_rpm ? (uint32_t)lv_arc_get_value(arc_rpm) : 1000u;
  const SignalConfig seed = presetCfgFromIndex(s_last_preset_pattern, rpm);

  // LEFT column: rows (CUSTOM keeps make_spin_row left_justified=false).
  lv_obj_t* rows = lv_obj_create(panel_custom);
  lv_obj_set_size(rows, 280, 196);
  lv_obj_align(rows, LV_ALIGN_TOP_LEFT, 0, 44);
  lv_obj_set_style_bg_opa(rows, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rows, 0, 0);
  lv_obj_set_style_pad_all(rows, 0, 0);
  lv_obj_set_style_pad_row(rows, 2, 0);
  lv_obj_set_flex_flow(rows, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(rows, LV_OBJ_FLAG_SCROLLABLE);

  (void)make_spin_row(rows, "Teeth", &spin_teeth, 1, 120, seed.nTeeth, false);
  (void)make_spin_row(rows, "Periods/Rev", &spin_pmiss, 1, 10, seed.pMiss, false);
  (void)make_spin_row(rows, "Missing/Period", &spin_nmiss, 1, 60, seed.nMiss, false);

  lv_obj_t* rowPos = lv_obj_create(rows);
  lv_obj_set_size(rowPos, lv_pct(100), 34);
  lv_obj_set_style_bg_opa(rowPos, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rowPos, 0, 0);
  lv_obj_set_style_pad_all(rowPos, 0, 0);
  lv_obj_clear_flag(rowPos, LV_OBJ_FLAG_SCROLLABLE);

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
  lv_obj_clear_flag(rowLvl, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* lblLvl = lv_label_create(rowLvl);
  lv_label_set_text(lblLvl, "Gap HIGH");
  lv_obj_add_style(lblLvl, &style_caption, 0);
  lv_obj_align(lblLvl, LV_ALIGN_LEFT_MID, 0, 0);

  sw_gap_lvl = lv_switch_create(rowLvl);
  lv_obj_align(sw_gap_lvl, LV_ALIGN_RIGHT_MID, 0, 0);
  if (seed.gapLvl) lv_obj_add_state(sw_gap_lvl, LV_STATE_CHECKED);

  // RIGHT: lazy live RPM arc (Decision 14).
  arc_custom_live = make_page_live_arc(panel_custom);
  lv_obj_set_size(arc_custom_live, 130, 130);
  lv_obj_align(arc_custom_live, LV_ALIGN_RIGHT_MID, -8, -10);

  // ON/OFF status pill below the custom arc (D7).
  lbl_custom_status = make_status_pill(panel_custom, 340, 188);

  lbl_custom_error = lv_label_create(panel_custom);
  lv_obj_add_style(lbl_custom_error, &style_caption, 0);
  lv_label_set_text(lbl_custom_error, "");
  lv_obj_align(lbl_custom_error, LV_ALIGN_BOTTOM_LEFT, 0, -48);
  lv_obj_add_flag(lbl_custom_error, LV_OBJ_FLAG_HIDDEN);

  // Bottom row (D10): Back 120x40 @(0,0), START 120x40 @(164,0) = 44px gap.
  lv_obj_t* btnCancel = lv_btn_create(panel_custom);
  lv_obj_set_size(btnCancel, 120, 40);
  lv_obj_align(btnCancel, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_style(btnCancel, &style_btn, 0);
  lv_obj_add_event_cb(btnCancel, on_custom_back, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lblCancel = lv_label_create(btnCancel);
  lv_obj_set_style_text_font(lblCancel, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblCancel, "Back");
  lv_obj_center(lblCancel);

  // START (renamed from APPLY) -> primary action, inline cyan glow (D11).
  lv_obj_t* btnApply = lv_btn_create(panel_custom);
  lv_obj_set_size(btnApply, 120, 40);
  lv_obj_align(btnApply, LV_ALIGN_BOTTOM_LEFT, 164, 0);
  lv_obj_add_style(btnApply, &style_btn, 0);
  lv_obj_set_style_shadow_width(btnApply, 8, 0);
  lv_obj_set_style_shadow_color(btnApply, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_shadow_opa(btnApply, LV_OPA_50, 0);
  lv_obj_add_event_cb(btnApply, on_custom_apply, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lblApply = lv_label_create(btnApply);
  lv_obj_set_style_text_font(lblApply, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblApply, "START");
  lv_obj_center(lblApply);

  tmr_custom_live = lv_timer_create(on_custom_live_tick, 100, NULL);
}

// Tabview VALUE_CHANGED — HOME compression trigger removed (U3a/D4). Kept as a
// no-op stub so the lv_obj_add_event_cb wiring still compiles.
static void on_tabview_changed(lv_event_t* e) {
  LV_UNUSED(e);
}

// HOME-tab density (inline, Decision 12a).
static constexpr int HOME_GAP    = 3;
static constexpr int HOME_BTN    = 37;
static constexpr int HOME_RADIUS = 10;
static constexpr int HOME_ARC    = 18;

static void build_home_tab(lv_obj_t* page) {
  lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_pad_all(page, 4, 0);

  // ---- Channel LEDs: top-right row @ (321,15) 55x26 (COMPONENT LAYOUT).
  // Three 14px LEDs spread across the 55px band, vertically centered (y=21).
  led_crank = lv_led_create(page);
  lv_obj_set_size(led_crank, 14, 14);
  lv_obj_align(led_crank, LV_ALIGN_TOP_LEFT, 321, 21);
  lv_led_set_color(led_crank, lv_color_hex(COL_ACCENT));
  lv_led_on(led_crank);

  led_cam1 = lv_led_create(page);
  lv_obj_set_size(led_cam1, 14, 14);
  lv_obj_align(led_cam1, LV_ALIGN_TOP_LEFT, 341, 21);
  lv_led_set_color(led_cam1, lv_color_hex(COL_LED_OFF));
  lv_led_set_brightness(led_cam1, 60);
  lv_led_on(led_cam1);

  led_cam2 = lv_led_create(page);
  lv_obj_set_size(led_cam2, 14, 14);
  lv_obj_align(led_cam2, LV_ALIGN_TOP_LEFT, 361, 21);
  lv_led_set_color(led_cam2, lv_color_hex(COL_LED_OFF));
  lv_led_set_brightness(led_cam2, 60);
  lv_led_on(led_cam2);

  arc_rpm = lv_arc_create(page);
  lv_obj_set_size(arc_rpm, 190, 190);
  lv_arc_set_rotation(arc_rpm, 135);
  lv_arc_set_bg_angles(arc_rpm, 0, 270);
  lv_arc_set_mode(arc_rpm, kArcReverse ? LV_ARC_MODE_REVERSE : LV_ARC_MODE_NORMAL);
  lv_arc_set_range(arc_rpm, 100, 6000);
  lv_arc_set_value(arc_rpm, 1000);
  lv_obj_add_style(arc_rpm, &style_arc_main, LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc_rpm, HOME_ARC, LV_PART_MAIN);
  lv_obj_add_style(arc_rpm, &style_arc_indic, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc_rpm, HOME_ARC, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(arc_rpm, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_set_style_border_width(arc_rpm, 0, LV_PART_KNOB);
  lv_obj_add_flag(arc_rpm, LV_OBJ_FLAG_ADV_HITTEST);
  lv_obj_align(arc_rpm, LV_ALIGN_LEFT_MID, 14, 8);
  lv_obj_add_event_cb(arc_rpm, on_arc_changed, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(arc_rpm, on_arc_changed, LV_EVENT_RELEASED, NULL);

  lbl_rpm_value = lv_label_create(page);
  lv_obj_add_style(lbl_rpm_value, &style_value, 0);
  lv_label_set_text(lbl_rpm_value, "1000");
  lv_obj_align_to(lbl_rpm_value, arc_rpm, LV_ALIGN_CENTER, 0, -6);

  lbl_rpm_caption = lv_label_create(page);
  lv_obj_add_style(lbl_rpm_caption, &style_caption, 0);
  lv_label_set_text(lbl_rpm_caption, "RPM");
  lv_obj_align_to(lbl_rpm_caption, arc_rpm, LV_ALIGN_CENTER, 0, 16);

  // ---- RIGHT: pattern + filter + actions ----
  lbl_pattern = lv_label_create(page);
  lv_obj_add_style(lbl_pattern, &style_caption, 0);
  lv_label_set_text(lbl_pattern, "FG Electronics Signal Generator");
  lv_obj_align(lbl_pattern, LV_ALIGN_TOP_LEFT, 120, 2);

  // D15: nudged down to (232,52) to clear the relocated LED row @ (321,15).
  dd_patterns = lv_dropdown_create(page);
  rebuild_pattern_dropdown_options(nullptr);
  lv_obj_align(dd_patterns, LV_ALIGN_TOP_LEFT, 232, 52);
  lv_obj_set_size(dd_patterns, 212, 34);
  lv_obj_set_style_radius(dd_patterns, HOME_RADIUS, 0);
  lv_obj_add_style(dd_patterns, &style_dropdown, LV_PART_MAIN);
  lv_obj_t* list = lv_dropdown_get_list(dd_patterns);
  if (list) {
    lv_obj_add_style(list, &style_dropdown, LV_PART_MAIN);
    // Re-tuned for the 44px bottom tab bar (no 116px keyboard overlap):
    // the dropdown opens near y~50; cap so it clears the keyboard band.
    lv_obj_set_height(list, 110);
  }
  lv_obj_add_event_cb(dd_patterns, on_pattern_changed, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(dd_patterns, on_pattern_open, LV_EVENT_CLICKED, NULL);

  // D15: nudged to (232,90), directly below the reflowed dropdown.
  ta_pattern_filter = lv_textarea_create(page);
  lv_textarea_set_one_line(ta_pattern_filter, true);
  lv_textarea_set_placeholder_text(ta_pattern_filter, "filter...");
  lv_obj_align(ta_pattern_filter, LV_ALIGN_TOP_LEFT, 232, 90);
  lv_obj_set_size(ta_pattern_filter, 212, 30);
  lv_obj_add_style(ta_pattern_filter, &style_dropdown, 0);
  lv_obj_add_event_cb(ta_pattern_filter, on_pattern_filter_changed,
                      LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_focused, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_focused, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_defocused, LV_EVENT_DEFOCUSED, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_defocused, LV_EVENT_READY, NULL);
  lv_obj_add_event_cb(ta_pattern_filter, on_ta_defocused, LV_EVENT_CANCEL, NULL);

  // INVERT @ (353,157) 95x16 + START/STOP @ (342,184) 113x30 (COMPONENT
  // LAYOUT). Both are primary actions -> inline cyan glow (D11).
  btn_invert = lv_btn_create(page);
  lv_obj_align(btn_invert, LV_ALIGN_TOP_LEFT, 353, 143);
  lv_obj_set_size(btn_invert, 95, 30);
  lv_obj_add_style(btn_invert, &style_btn, 0);
  lv_obj_set_style_radius(btn_invert, HOME_RADIUS, 0);
  lv_obj_set_style_shadow_width(btn_invert, 8, 0);
  lv_obj_set_style_shadow_color(btn_invert, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_shadow_opa(btn_invert, LV_OPA_50, 0);
  lv_obj_add_event_cb(btn_invert, on_invert_clicked, LV_EVENT_CLICKED, NULL);
  lbl_invert = lv_label_create(btn_invert);
  refresh_invert_label();
  lv_obj_center(lbl_invert);

  btn_run = lv_btn_create(page);
  lv_obj_align(btn_run, LV_ALIGN_TOP_LEFT, 342, 184);
  lv_obj_set_size(btn_run, 113, 30);
  lv_obj_add_style(btn_run, &style_btn, 0);
  lv_obj_set_style_radius(btn_run, HOME_RADIUS, 0);
  lv_obj_set_style_shadow_width(btn_run, 8, 0);
  lv_obj_set_style_shadow_color(btn_run, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_shadow_opa(btn_run, LV_OPA_50, 0);
  lv_obj_add_event_cb(btn_run, on_run_clicked, LV_EVENT_CLICKED, NULL);
  lbl_run = lv_label_create(btn_run);
  lv_label_set_text(lbl_run, s_running ? "STOP" : "START");
  lv_obj_center(lbl_run);

  // Error label — bottom-left under the arc.
  lbl_error = lv_label_create(page);
  lv_obj_add_style(lbl_error, &style_caption, 0);
  lv_label_set_text(lbl_error, "");
  lv_obj_align(lbl_error, LV_ALIGN_BOTTOM_LEFT, 2, -2);
  lv_obj_set_size(lbl_error, 220, 18);
  lv_label_set_long_mode(lbl_error, LV_LABEL_LONG_DOT);
  lv_obj_add_flag(lbl_error, LV_OBJ_FLAG_HIDDEN);

  update_rpm_label(lv_arc_get_value(arc_rpm));
}

// ADVANCED-tab density (inline, Decision 12a).
static constexpr int ADV_GAP    = 10;
static constexpr int ADV_ROW_H  = 42;
static constexpr int ADV_RADIUS = 12;

static void make_adv_row(lv_obj_t* list, const char* sym, const char* name,
                         const char* desc, lv_event_cb_t cb) {
  lv_obj_t* row = lv_obj_create(list);
  lv_obj_set_size(row, lv_pct(100), ADV_ROW_H);
  lv_obj_set_style_radius(row, ADV_RADIUS, 0);
  lv_obj_set_style_bg_color(row, lv_color_hex(COL_SURFACE), 0);
  lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(row, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_border_width(row, 1, 0);
  lv_obj_set_style_border_opa(row, LV_OPA_40, 0);
  lv_obj_set_style_pad_left(row, 8, 0);
  lv_obj_set_style_pad_right(row, 8, 0);
  lv_obj_set_style_pad_top(row, 2, 0);
  lv_obj_set_style_pad_bottom(row, 2, 0);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 8, 0);
  lv_obj_add_event_cb(row, cb, LV_EVENT_CLICKED, NULL);

  // Icon (force montserrat_16 so LV_SYMBOL_* glyphs render).
  lv_obj_t* icon = lv_label_create(row);
  lv_label_set_text(icon, sym);
  lv_obj_set_style_text_font(icon, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(icon, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_width(icon, 24);

  // name over desc, flex-grown so the whole row is tappable.
  lv_obj_t* col = lv_obj_create(row);
  lv_obj_set_height(col, lv_pct(100));
  lv_obj_set_flex_grow(col, 1);
  lv_obj_set_style_bg_opa(col, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(col, 0, 0);
  lv_obj_set_style_pad_all(col, 0, 0);
  lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(col, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

  lv_obj_t* lname = lv_label_create(col);
  lv_label_set_text(lname, name);
  lv_obj_set_style_text_font(lname, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lname, lv_color_hex(COL_TEXT), 0);

  lv_obj_t* ldesc = lv_label_create(col);
  lv_label_set_text(ldesc, desc);
  lv_obj_set_style_text_font(ldesc, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(ldesc, lv_color_hex(COL_MUTED), 0);
}

static void build_advanced_tab(lv_obj_t* page) {
  // Vertical scrollable 5-row list (5x~52px overflows the ~228px content).
  lv_obj_set_style_pad_all(page, ADV_GAP, 0);
  lv_obj_set_style_pad_row(page, ADV_GAP, 0);
  lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
  // SCROLLABLE kept (do NOT clear it — Decision 4); tabview content swipe
  // is on the parent content container, not this page.

  make_adv_row(page, LV_SYMBOL_REFRESH, "SWEEP",          "RPM ramp generator",        on_adv_row_sweep);
  make_adv_row(page, LV_SYMBOL_DOWNLOAD, "COMPRESSION",   "Cylinder compression dips", on_adv_row_comp);
  make_adv_row(page, LV_SYMBOL_EDIT,     "DSL EDITOR",    "Write & compile patterns",  on_adv_row_dsl);
  make_adv_row(page, LV_SYMBOL_IMAGE,    "WAVEFORM",      "Live 3-lane scope",         on_adv_row_wave);
  make_adv_row(page, LV_SYMBOL_SETTINGS, "CUSTOM PATTERN","Build a custom wheel",      on_adv_row_custom);
}

static void create_main_screen() {
  screen_main = lv_screen_active();
  lv_obj_remove_style_all(screen_main);
  lv_obj_add_style(screen_main, &style_bg, 0);
  lv_obj_clear_flag(screen_main, LV_OBJ_FLAG_SCROLLABLE);

  // 2-tab navigation (Decision 1). Order matters: position THEN size so the
  // dpi/2 default reset inside set_tab_bar_position can never win.
  tabview = lv_tabview_create(screen_main);
  lv_tabview_set_tab_bar_position(tabview, LV_DIR_BOTTOM);
  lv_tabview_set_tab_bar_size(tabview, 44);
  lv_obj_add_style(tabview, &style_bg, 0);

  tab_home = lv_tabview_add_tab(tabview, "HOME");      // index 0, default active
  tab_adv  = lv_tabview_add_tab(tabview, "ADVANCED");  // index 1

  // Style the tab bar: COL_SURFACE bg + 1px COL_ACCENT-opa TOP border.
  lv_obj_t* bar = lv_tabview_get_tab_bar(tabview);
  if (bar) {
    lv_obj_set_style_bg_color(bar, lv_color_hex(COL_SURFACE), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(COL_ACCENT), 0);
    lv_obj_set_style_border_opa(bar, LV_OPA_40, 0);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_TOP, 0);
    // Tab buttons: inactive COL_MUTED, LV_STATE_CHECKED -> COL_ACCENT.
    uint32_t nb = lv_obj_get_child_count(bar);
    for (uint32_t i = 0; i < nb; ++i) {
      lv_obj_t* tb = lv_obj_get_child(bar, i);
      if (!tb) continue;
      lv_obj_set_style_text_color(tb, lv_color_hex(COL_MUTED), LV_PART_MAIN);
      lv_obj_set_style_text_color(tb, lv_color_hex(COL_ACCENT),
                                  LV_PART_MAIN | LV_STATE_CHECKED);
      lv_obj_set_style_bg_opa(tb, LV_OPA_TRANSP, LV_PART_MAIN);
    }
  }

  lv_obj_add_event_cb(tabview, on_tabview_changed, LV_EVENT_VALUE_CHANGED, NULL);

  build_home_tab(tab_home);
  build_advanced_tab(tab_adv);
}


// =====================================================
// E-core-6 — Back-stops-output helper + shared page chrome
// =====================================================

// Idempotent: stops output without touching s_pending_*. Wired into every
// page Back handler and the Sweep/Comp STOP buttons (Decision 3).
static void ui_force_output_off() {
  if (!s_running) return;
  s_running = false;
  refresh_run_label();
  if (s_on_run) s_on_run(false);
}

// Symmetric idempotent START primitive (D1). Re-tapping a page START while
// already running re-sends config (legit mid-run reconfigure) but never
// double-fires MSG_START because this early-returns when already running (D3).
static void ui_force_output_on() {
  if (s_running) return;
  s_running = true;
  refresh_run_label();
  if (s_on_run) s_on_run(true);
}

// Comp arc dual-mode (D5) — EDGE-TRIGGERED. `on` (stopped) arms the arc for
// user RPM drag; `off` (running) makes it a read-only output visual. The
// s_comp_arc_interactive guard prevents stacking event descriptors / pool
// growth on repeated STOP taps. Reuses on_arc_changed (50ms throttle ->
// s_on_rpm). arc_comp_live is non-null only while the Comp page is open.
static void comp_arc_set_interactive(bool on) {
  if (s_comp_arc_interactive == on) return;
  s_comp_arc_interactive = on;
  if (!arc_comp_live) return;
  if (on) {
    lv_obj_add_flag(arc_comp_live, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(arc_comp_live, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_add_event_cb(arc_comp_live, on_arc_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(arc_comp_live, on_arc_changed, LV_EVENT_RELEASED, NULL);
    lv_obj_set_style_arc_opa(arc_comp_live, LV_OPA_COVER, LV_PART_INDICATOR);
  } else {
    lv_obj_remove_event_cb(arc_comp_live, on_arc_changed);
    lv_obj_remove_flag(arc_comp_live, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(arc_comp_live, LV_OBJ_FLAG_ADV_HITTEST);
  }
}

// ON/OFF status pill content (D7). Pill chrome (bg/border/font) is set once at
// creation; this only flips the text + color + opa. Driven from each page's
// 100ms tick (heartbeat) AND immediately in each START/STOP handler.
static void update_status_pill(lv_obj_t* lbl) {
  if (!lbl) return;
  if (s_running) {
    lv_label_set_text(lbl, "ON");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_ACCENT), 0);
    lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, 0);
  } else {
    lv_label_set_text(lbl, "OFF");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_MUTED), 0);
    lv_obj_set_style_text_opa(lbl, LV_OPA_70, 0);
  }
}

// Pill chrome factory (D7): a small rounded COL_SUNKEN label with a 1px accent
// border + montserrat_14 centered text, aligned at (x,y) on `parent`. The
// caller stores the returned label and drives it via update_status_pill().
static lv_obj_t* make_status_pill(lv_obj_t* parent, int x, int y) {
  lv_obj_t* lbl = lv_label_create(parent);
  lv_obj_set_size(lbl, 64, 24);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_set_style_bg_color(lbl, lv_color_hex(COL_SUNKEN), 0);
  lv_obj_set_style_bg_opa(lbl, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(lbl, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_border_width(lbl, 1, 0);
  lv_obj_set_style_border_opa(lbl, LV_OPA_40, 0);
  lv_obj_set_style_radius(lbl, 12, 0);
  lv_obj_set_style_pad_all(lbl, 3, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
  update_status_pill(lbl);
  return lbl;
}

// Full-screen LV_OPA_COVER overlay on screen_main, above the tabview so it
// hides the 44px tab bar (Decision 2). SCROLLABLE cleared on the overlay.
static lv_obj_t* make_page_overlay(lv_obj_t** slot, uint32_t bg_opa) {
  lv_obj_t* ov = lv_obj_create(screen_main);
  *slot = ov;
  lv_obj_set_size(ov, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(ov, lv_color_hex(COL_BG), 0);
  lv_obj_set_style_bg_opa(ov, bg_opa, 0);
  lv_obj_set_style_border_width(ov, 0, 0);
  lv_obj_set_style_pad_all(ov, 8, 0);
  lv_obj_clear_flag(ov, LV_OBJ_FLAG_SCROLLABLE);
  return ov;
}

// Top-left '< Back' header (>=40px hit, Decision 13) + a centered title.
// Returns nothing useful; caller wires back_cb.
static lv_obj_t* make_back_header(lv_obj_t* parent, const char* title, lv_event_cb_t back_cb) {
  lv_obj_t* btnBack = lv_btn_create(parent);
  lv_obj_set_size(btnBack, 72, 40);
  lv_obj_align(btnBack, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_add_style(btnBack, &style_btn, 0);
  lv_obj_add_event_cb(btnBack, back_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lblBack = lv_label_create(btnBack);
  lv_label_set_text(lblBack, LV_SYMBOL_LEFT " Back");
  lv_obj_set_style_text_font(lblBack, &lv_font_montserrat_14, 0);
  lv_obj_center(lblBack);

  lv_obj_t* lblTitle = lv_label_create(parent);
  lv_label_set_text(lblTitle, title);
  lv_obj_add_style(lblTitle, &style_title, 0);
  lv_obj_align(lblTitle, LV_ALIGN_TOP_MID, 0, 8);
  return btnBack;
}

// Lazy page-local live RPM arc (Decision 14). NOT arc_rpm. Read-only
// indicator: ADV_HITTEST off, no value event.
static lv_obj_t* make_page_live_arc(lv_obj_t* parent) {
  lv_obj_t* a = lv_arc_create(parent);
  lv_obj_set_size(a, 160, 160);
  lv_arc_set_rotation(a, 135);
  lv_arc_set_bg_angles(a, 0, 270);
  lv_arc_set_mode(a, kArcReverse ? LV_ARC_MODE_REVERSE : LV_ARC_MODE_NORMAL);
  lv_arc_set_range(a, 100, 6000);
  lv_arc_set_value(a, 100);
  lv_obj_add_style(a, &style_arc_main, LV_PART_MAIN);
  lv_obj_set_style_arc_width(a, 12, LV_PART_MAIN);
  lv_obj_add_style(a, &style_arc_indic, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(a, 12, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(a, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_set_style_border_width(a, 0, LV_PART_KNOB);
  lv_obj_remove_flag(a, LV_OBJ_FLAG_CLICKABLE);  // read-only indicator
  return a;
}

// Gate-dim the indicator / show 0 when output is stopped (Decision 14) so a
// stopped output never displays a misleading non-zero RPM. D6: the Comp arc is
// dual-mode — when STOPPED it owns its (dragged) value, so we hold COVER opa
// and do NOT set_value (would erase the drag); all other arcs floor to 100 @
// OPA_30. RPM is constrained to the arc range [100,6000] before display.
static void update_live_arc(lv_obj_t* arc) {
  if (!arc) return;
  if (s_running) {
    lv_arc_set_value(arc, (int32_t)constrain(sweepCurrentRpm(), 100u, 6000u));
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);
  } else if (arc == arc_comp_live) {
    // Stopped + draggable: preserve the user's dragged value, full opacity.
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);
  } else {
    lv_arc_set_value(arc, 100);  // floor of range == visually empty
    lv_obj_set_style_arc_opa(arc, LV_OPA_30, LV_PART_INDICATOR);
  }
}

// ---- Advanced-row trampolines (route to the existing open_*_panel) ----
static void on_adv_row_sweep(lv_event_t* e)  { open_sweep_panel(e); }
static void on_adv_row_comp(lv_event_t* e)   { open_comp_panel(e); }
static void on_adv_row_dsl(lv_event_t* e)    { open_dsl_panel(e); }
static void on_adv_row_wave(lv_event_t* e)   { open_wave_panel(e); }
static void on_adv_row_custom(lv_event_t* e) { LV_UNUSED(e); open_custom_panel(); }


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
      // Greyed-out — channel unused by current pattern (Decision 10).
      lv_led_set_color(rows[i].led, lv_color_hex(COL_LED_OFF));
      lv_led_set_brightness(rows[i].led, 60);
    } else {
      lv_led_set_color(rows[i].led,
                       inverted ? lv_color_hex(COL_WARN)
                                : lv_color_hex(COL_ACCENT));
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
      lv_obj_set_style_text_color(lbl_rpm_value, lv_color_hex(COL_WARN), 0);
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
    lv_obj_set_style_text_color(lbl_rpm_value, lv_color_hex(COL_ACCENT), 0);
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
  // New HUD palette (D11): COL_SURFACE base + accent border; FILLED action
  // keys (LV_PART_ITEMS) — dark-on-cyan, NO per-key shadow.
  lv_obj_set_style_bg_color(kb_filter, lv_color_hex(COL_SURFACE), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(kb_filter, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(kb_filter, lv_color_hex(COL_ACCENT), LV_PART_MAIN);
  lv_obj_set_style_border_opa(kb_filter, LV_OPA_40, LV_PART_MAIN);
  lv_obj_set_style_border_width(kb_filter, 1, LV_PART_MAIN);
  lv_obj_set_style_bg_color(kb_filter, lv_color_hex(COL_ACCENT), LV_PART_ITEMS);
  lv_obj_set_style_bg_opa(kb_filter, LV_OPA_COVER, LV_PART_ITEMS);
  lv_obj_set_style_text_color(kb_filter, lv_color_hex(COL_BG), LV_PART_ITEMS);
  lv_obj_set_style_border_color(kb_filter, lv_color_hex(COL_ACCENT), LV_PART_ITEMS);
  lv_obj_set_style_border_width(kb_filter, 1, LV_PART_ITEMS);
  lv_obj_set_style_shadow_width(kb_filter, 0, LV_PART_ITEMS);
  lv_obj_set_style_radius(kb_filter, 6, LV_PART_ITEMS);
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
// Modal numeric keypad (D8) — full-screen overlay on lv_layer_top(),
// replaces the spinbox steppers. Single-modal: kb_hide() on open.
// =====================================================

// Repaint the live value label from s_kp.buf ("0" when empty).
static void keypad_update_display() {
  if (!lbl_keypad_value) return;
  lv_label_set_text(lbl_keypad_value, s_kp.len ? s_kp.buf : "0");
}

// commit: clamp buf to [min,max] and write into the target spinbox; discard
// otherwise. Always tears down the overlay (D8).
static void close_numeric_keypad(bool commit) {
  if (commit && s_kp.target) {
    long v = strtol(s_kp.buf, NULL, 10);
    if (v < s_kp.min) v = s_kp.min;
    if (v > s_kp.max) v = s_kp.max;
    lv_spinbox_set_value(s_kp.target, (int32_t)v);
  }
  if (overlay_keypad) { lv_obj_del(overlay_keypad); overlay_keypad = nullptr; }
  lbl_keypad_value = nullptr;
  s_kp.target = nullptr;
}

// VALUE_CHANGED dispatch (D8): get_selected_button -> get_button_text -> strcmp.
static void on_keypad_button_click(lv_event_t* e) {
  lv_obj_t* bm = lv_event_get_target_obj(e);
  uint32_t id = lv_buttonmatrix_get_selected_button(bm);
  if (id == LV_BUTTONMATRIX_BUTTON_NONE) return;
  const char* t = lv_buttonmatrix_get_button_text(bm, id);
  if (!t) return;

  if (strcmp(t, "OK") == 0) { close_numeric_keypad(true); return; }
  if (strcmp(t, "Back") == 0) { close_numeric_keypad(false); return; }
  if (strcmp(t, "Clear") == 0) { s_kp.len = 0; s_kp.buf[0] = '\0'; }
  else if (strcmp(t, LV_SYMBOL_BACKSPACE) == 0) {
    if (s_kp.len) s_kp.buf[--s_kp.len] = '\0';
  } else if (t[0] >= '0' && t[0] <= '9' && t[1] == '\0') {
    if (s_kp.len < 8) { s_kp.buf[s_kp.len++] = t[0]; s_kp.buf[s_kp.len] = '\0'; }
  }
  keypad_update_display();
}

// Open the modal over `target_spin`, seeded empty, header "name (min-max)".
static void open_numeric_keypad(lv_obj_t* target_spin, const char* field_name, int32_t min, int32_t max) {
  if (overlay_keypad) return;            // single-modal
  kb_hide();                             // hide on-screen keyboard first

  s_kp.target = target_spin;
  s_kp.min = min;
  s_kp.max = max;
  s_kp.len = 0;
  s_kp.buf[0] = '\0';

  overlay_keypad = lv_obj_create(lv_layer_top());
  lv_obj_set_size(overlay_keypad, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(overlay_keypad, lv_color_hex(COL_BG), 0);
  lv_obj_set_style_bg_opa(overlay_keypad, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(overlay_keypad, 0, 0);
  lv_obj_set_style_pad_all(overlay_keypad, 12, 0);
  lv_obj_clear_flag(overlay_keypad, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* hdr = lv_label_create(overlay_keypad);
  lv_obj_add_style(hdr, &style_title, 0);
  lv_label_set_text_fmt(hdr, "%s  (%ld-%ld)", field_name ? field_name : "",
                        (long)min, (long)max);
  lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 8);

  lbl_keypad_value = lv_label_create(overlay_keypad);
  lv_obj_add_style(lbl_keypad_value, &style_value, 0);
  lv_label_set_text(lbl_keypad_value, "0");
  lv_obj_align(lbl_keypad_value, LV_ALIGN_TOP_MID, 0, 44);

  lv_obj_t* bm = lv_buttonmatrix_create(overlay_keypad);
  lv_buttonmatrix_set_map(bm, kKeypadMap);
  lv_obj_set_size(bm, 300, 216);
  lv_obj_align(bm, LV_ALIGN_BOTTOM_MID, 0, 0);
  // Force montserrat_14 so LV_SYMBOL_BACKSPACE renders (default font lacks it).
  lv_obj_set_style_text_font(bm, &lv_font_montserrat_14, 0);
  lv_obj_set_style_bg_opa(bm, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(bm, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(bm, 0, LV_PART_MAIN);
  // FILLED keys (D11): dark-on-cyan, no per-key shadow.
  lv_obj_set_style_bg_color(bm, lv_color_hex(COL_ACCENT), LV_PART_ITEMS);
  lv_obj_set_style_bg_opa(bm, LV_OPA_COVER, LV_PART_ITEMS);
  lv_obj_set_style_text_color(bm, lv_color_hex(COL_BG), LV_PART_ITEMS);
  lv_obj_set_style_border_color(bm, lv_color_hex(COL_ACCENT), LV_PART_ITEMS);
  lv_obj_set_style_border_width(bm, 1, LV_PART_ITEMS);
  lv_obj_set_style_shadow_width(bm, 0, LV_PART_ITEMS);
  lv_obj_set_style_radius(bm, 6, LV_PART_ITEMS);
  lv_obj_add_event_cb(bm, on_keypad_button_click, LV_EVENT_VALUE_CHANGED, NULL);

  // Inline cyan glow on the primary actions only — OK + Back (D11).
  lv_obj_set_style_shadow_width(bm, 8, LV_PART_ITEMS | LV_STATE_PRESSED);
  lv_obj_set_style_shadow_color(bm, lv_color_hex(COL_ACCENT), LV_PART_ITEMS | LV_STATE_PRESSED);
  lv_obj_set_style_shadow_opa(bm, LV_OPA_50, LV_PART_ITEMS | LV_STATE_PRESSED);

  keypad_update_display();
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
  arc_sweep_live = nullptr;
  lbl_sweep_status = nullptr;        // E2-5: null the pill pointer
  lbl_sweep_arc_val = lbl_sweep_arc_dir = nullptr;
}

// Back: stop output, tear down (Decisions 3/6b). E2-5: defensively del any
// orphaned keypad first (a value-box keypad could be up over this page).
static void on_sweep_back(lv_event_t* e) {
  LV_UNUSED(e);
  if (overlay_keypad) { lv_obj_del(overlay_keypad); overlay_keypad = nullptr; }
  ui_force_output_off();
  close_sweep_panel(nullptr);
}

// Sweep STOP: kill output but stay on the page (Ref3/Decision 7).
static void on_sweep_stop(lv_event_t* e) {
  LV_UNUSED(e);
  ui_force_output_off();
}

// 100ms live tick drives the page-local arc, ON/OFF pill, and the arc center
// value + direction glyph (D7/D17).
static void on_sweep_live_tick(lv_timer_t* t) {
  LV_UNUSED(t);
  update_live_arc(arc_sweep_live);
  update_status_pill(lbl_sweep_status);

  // Arc center value + direction glyph (D17). Compare current vs previous RPM.
  const uint32_t rpm = (uint32_t)constrain(sweepCurrentRpm(), 100u, 6000u);
  if (lbl_sweep_arc_val) lv_label_set_text_fmt(lbl_sweep_arc_val, "%lu", (unsigned long)rpm);
  if (lbl_sweep_arc_dir) {
    if (rpm > s_sweep_prev_rpm)      lv_label_set_text(lbl_sweep_arc_dir, LV_SYMBOL_UP);
    else if (rpm < s_sweep_prev_rpm) lv_label_set_text(lbl_sweep_arc_dir, LV_SYMBOL_DOWN);
    // equal -> hold last glyph
  }
  s_sweep_prev_rpm = rpm;
}

// START (renamed from APPLY): send the sweep config AND start output, page
// stays open (D2). ui_force_output_on() is idempotent so a re-tap mid-run
// re-sends config without double-firing MSG_START.
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
  ui_force_output_on();
  update_status_pill(lbl_sweep_status);
}

static void open_sweep_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_sweep) return;
  lv_obj_t* page = make_page_overlay(&overlay_sweep, LV_OPA_COVER);
  // D18 absolute model: pad_all(page,0) so LV_ALIGN_TOP_LEFT(x,y) lands at the
  // literal brief screen coordinate. Reset the keypad-field cursor (D9).
  lv_obj_set_style_pad_all(page, 0, 0);
  s_kp_field_n = 0;
  make_back_header(page, "SWEEP", on_sweep_back);

  // Mode label (15,87) + Mode dropdown (93,87) 86x22.
  lv_obj_t* lblMode = lv_label_create(page);
  lv_label_set_text(lblMode, "Mode");
  lv_obj_add_style(lblMode, &style_caption, 0);
  lv_obj_align(lblMode, LV_ALIGN_TOP_LEFT, 15, 57);

  dd_sweep_mode = lv_dropdown_create(page);
  lv_dropdown_set_options(dd_sweep_mode, "OFF\nLINEAR\nLOG\nWAYPOINT");
  lv_dropdown_set_selected(dd_sweep_mode, g_sweep_mode);
  lv_obj_set_size(dd_sweep_mode, 110, 30);
  lv_obj_align(dd_sweep_mode, LV_ALIGN_TOP_LEFT, 93, 50);
  lv_obj_add_style(dd_sweep_mode, &style_dropdown, LV_PART_MAIN);

  // D14 LITERAL crossed-Y (flagged as likely typo in §7): Low label@(15,151)
  // pairs with Low value@(93,117); High label@(15,117) pairs with High
  // value@(93,151). Implemented verbatim per the brief.
  
  make_value_box(page, "High RPM", &spin_sweep_high, 100, 6000, g_sweep_high_rpm,
                 15, 100, 93, 93, 75, 30);
  make_value_box(page, "Low RPM",  &spin_sweep_low,  100, 6000, g_sweep_low_rpm,
                 15, 140, 93, 133, 75, 30);
  
  make_value_box(page, "Interval us", &spin_sweep_iv, 100, 100000, (int)g_sweep_interval_us,
                 15, 170, 93, 163, 75, 30);

  // RIGHT: enlarged live RPM arc + center value + direction glyph (D17).
  arc_sweep_live = make_page_live_arc(page);
  lv_obj_align(arc_sweep_live, LV_ALIGN_RIGHT_MID, -16, -8);

  lbl_sweep_arc_val = lv_label_create(page);
  lv_obj_set_style_text_font(lbl_sweep_arc_val, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl_sweep_arc_val, lv_color_hex(COL_ACCENT), 0);
  lv_label_set_text(lbl_sweep_arc_val, "100");
  lv_obj_align_to(lbl_sweep_arc_val, arc_sweep_live, LV_ALIGN_CENTER, 0, -4);

  lbl_sweep_arc_dir = lv_label_create(page);
  lv_obj_set_style_text_font(lbl_sweep_arc_dir, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_sweep_arc_dir, lv_color_hex(COL_ACCENT), 0);
  lv_label_set_text(lbl_sweep_arc_dir, LV_SYMBOL_UP);
  lv_obj_align_to(lbl_sweep_arc_dir, arc_sweep_live, LV_ALIGN_CENTER, 0, 18);

  // ON/OFF status pill below the arc (D7).
  s_sweep_prev_rpm = 100;
  lbl_sweep_status = make_status_pill(page, 340, 188);

  // Bottom row (D10): STOP 120x40 @(0,0), START 120x40 @(164,0) = 44px gap.
  // Both primary actions -> inline cyan glow (D11). APPLY -> START.
  lv_obj_t* btnStop = lv_btn_create(page);
  lv_obj_set_size(btnStop, 120, 40);
  lv_obj_align(btnStop, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_style(btnStop, &style_btn, 0);
  lv_obj_set_style_shadow_width(btnStop, 8, 0);
  lv_obj_set_style_shadow_color(btnStop, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_shadow_opa(btnStop, LV_OPA_50, 0);
  lv_obj_add_event_cb(btnStop, on_sweep_stop, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l1 = lv_label_create(btnStop);
  lv_obj_set_style_text_font(l1, &lv_font_montserrat_14, 0);
  lv_label_set_text(l1, "STOP"); lv_obj_center(l1);

  lv_obj_t* btnStart = lv_btn_create(page);
  lv_obj_set_size(btnStart, 120, 40);
  lv_obj_align(btnStart, LV_ALIGN_BOTTOM_LEFT, 164, 0);
  lv_obj_add_style(btnStart, &style_btn, 0);
  lv_obj_set_style_shadow_width(btnStart, 8, 0);
  lv_obj_set_style_shadow_color(btnStart, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_shadow_opa(btnStart, LV_OPA_50, 0);
  lv_obj_add_event_cb(btnStart, on_sweep_apply, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l2 = lv_label_create(btnStart);
  lv_obj_set_style_text_font(l2, &lv_font_montserrat_14, 0);
  lv_label_set_text(l2, "START"); lv_obj_center(l2);

  // Reuse tmr_sweep_live for the arc / pill / glyph (Decision 7).
  tmr_sweep_live = lv_timer_create(on_sweep_live_tick, 100, NULL);
}

static void close_comp_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (tmr_comp_live) { lv_timer_del(tmr_comp_live); tmr_comp_live = nullptr; }
  if (overlay_comp) { lv_obj_del(overlay_comp); overlay_comp = nullptr; }
  sw_comp_en = sw_comp_dyn = nullptr;
  spin_comp_cyl = spin_comp_thr = spin_comp_peak = nullptr;
  arc_comp_live = nullptr;
  lbl_comp_status = nullptr;          // E2-5: null the pill pointer
  lbl_comp_arc_val = nullptr;
  s_comp_arc_interactive = false;     // E2-5: re-arm on next open (D5)
}

// Back: stop output, tear down (Decisions 3/6b). E2-5: defensively del any
// orphaned keypad first.
static void on_comp_back(lv_event_t* e) {
  LV_UNUSED(e);
  if (overlay_keypad) { lv_obj_del(overlay_keypad); overlay_keypad = nullptr; }
  ui_force_output_off();
  close_comp_panel(nullptr);
}

// Comp STOP: kill output, re-arm the arc for user drag (D5), refresh the pill.
static void on_comp_stop(lv_event_t* e) {
  LV_UNUSED(e);
  ui_force_output_off();
  comp_arc_set_interactive(true);
  update_status_pill(lbl_comp_status);
}

static void on_comp_live_tick(lv_timer_t* t) {
  LV_UNUSED(t);
  update_live_arc(arc_comp_live);
  update_status_pill(lbl_comp_status);
  if (lbl_comp_arc_val && arc_comp_live)
    lv_label_set_text_fmt(lbl_comp_arc_val, "%ld", (long)lv_arc_get_value(arc_comp_live));
}

// START (renamed from APPLY): send compression config, start output, make the
// arc read-only output visual (D5), refresh the pill; page stays open (D2).
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
  ui_force_output_on();
  comp_arc_set_interactive(false);
  update_status_pill(lbl_comp_status);
}

static void open_comp_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_comp) return;
  lv_obj_t* page = make_page_overlay(&overlay_comp, LV_OPA_COVER);
  // D18 absolute model: pad_all(page,0); reset keypad-field cursor (D9).
  lv_obj_set_style_pad_all(page, 0, 0);
  s_kp_field_n = 0;
  make_back_header(page, "COMPRESSION", on_comp_back);

  // Enabled label (14,57) 74x22 / switch (116,57) 52x22.
  lv_obj_t* lblEn = lv_label_create(page);
  lv_label_set_text(lblEn, "Enabled");
  lv_obj_add_style(lblEn, &style_caption, 0);
  lv_obj_set_size(lblEn, 74, 22);
  lv_obj_align(lblEn, LV_ALIGN_TOP_LEFT, 14, 57);
  sw_comp_en = lv_switch_create(page);
  lv_obj_set_size(sw_comp_en, 52, 22);
  lv_obj_align(sw_comp_en, LV_ALIGN_TOP_LEFT, 116, 57);
  if (g_comp_enabled) lv_obj_add_state(sw_comp_en, LV_STATE_CHECKED);

  // Cylinders / RPM Thresh / Peak value boxes (label@14, value@116, 96x22).
  make_value_box(page, "Cylinders",  &spin_comp_cyl, 1, 12, g_comp_cyl,
                 14, 89, 116, 89, 96, 22);
  make_value_box(page, "RPM Thresh", &spin_comp_thr, 100, 6000, g_comp_rpm_thresh,
                 14, 121, 116, 121, 96, 22);
  make_value_box(page, "Peak",       &spin_comp_peak, 0, 255, g_comp_peak,
                 14, 153, 116, 153, 96, 22);

  // Dynamic label (14,185) 74x22 / switch (116,185) 52x22.
  lv_obj_t* lblDyn = lv_label_create(page);
  lv_label_set_text(lblDyn, "Dynamic");
  lv_obj_add_style(lblDyn, &style_caption, 0);
  lv_obj_set_size(lblDyn, 74, 22);
  lv_obj_align(lblDyn, LV_ALIGN_TOP_LEFT, 14, 185);
  sw_comp_dyn = lv_switch_create(page);
  lv_obj_set_size(sw_comp_dyn, 52, 22);
  lv_obj_align(sw_comp_dyn, LV_ALIGN_TOP_LEFT, 116, 185);
  if (g_comp_dynamic) lv_obj_add_state(sw_comp_dyn, LV_STATE_CHECKED);

  // Live/draggable RPM arc @ TOP_LEFT(277,53) (literal brief coord) + center
  // value (D17). Dual-mode is armed below by comp_arc_set_interactive.
  arc_comp_live = make_page_live_arc(page);
  lv_obj_align(arc_comp_live, LV_ALIGN_TOP_LEFT, 277, 53);

  lbl_comp_arc_val = lv_label_create(page);
  lv_obj_set_style_text_font(lbl_comp_arc_val, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl_comp_arc_val, lv_color_hex(COL_ACCENT), 0);
  lv_label_set_text(lbl_comp_arc_val, "100");
  lv_obj_align_to(lbl_comp_arc_val, arc_comp_live, LV_ALIGN_CENTER, 0, 0);

  // ON/OFF status pill below the arc (D7).
  lbl_comp_status = make_status_pill(page, 312, 188);

  // Bottom row (D10): STOP 120x40 @(0,0), START 120x40 @(164,0) = 44px gap.
  // Both primary actions -> inline cyan glow (D11). APPLY -> START.
  lv_obj_t* btnStop = lv_btn_create(page);
  lv_obj_set_size(btnStop, 120, 40);
  lv_obj_align(btnStop, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_style(btnStop, &style_btn, 0);
  lv_obj_set_style_shadow_width(btnStop, 8, 0);
  lv_obj_set_style_shadow_color(btnStop, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_shadow_opa(btnStop, LV_OPA_50, 0);
  lv_obj_add_event_cb(btnStop, on_comp_stop, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l1 = lv_label_create(btnStop);
  lv_obj_set_style_text_font(l1, &lv_font_montserrat_14, 0);
  lv_label_set_text(l1, "STOP"); lv_obj_center(l1);

  lv_obj_t* btnStart = lv_btn_create(page);
  lv_obj_set_size(btnStart, 120, 40);
  lv_obj_align(btnStart, LV_ALIGN_BOTTOM_LEFT, 164, 0);
  lv_obj_add_style(btnStart, &style_btn, 0);
  lv_obj_set_style_shadow_width(btnStart, 8, 0);
  lv_obj_set_style_shadow_color(btnStart, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_shadow_opa(btnStart, LV_OPA_50, 0);
  lv_obj_add_event_cb(btnStart, on_comp_apply, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l2 = lv_label_create(btnStart);
  lv_obj_set_style_text_font(l2, &lv_font_montserrat_14, 0);
  lv_label_set_text(l2, "START"); lv_obj_center(l2);

  // New lazy 100ms timer for the arc / pill (Comp has none today; mirror Sweep).
  tmr_comp_live = lv_timer_create(on_comp_live_tick, 100, NULL);

  // D5: arm the dual-mode arc to its initial state (draggable iff stopped).
  comp_arc_set_interactive(!s_running);
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
  // Tear down the Help sub-page first if it is still open (defensive — Back
  // is normally under the help overlay so the user closes Help first).
  if (overlay_dsl_help) { lv_obj_del(overlay_dsl_help); overlay_dsl_help = nullptr; }
  if (tmr_dsl_err) { lv_timer_del(tmr_dsl_err); tmr_dsl_err = nullptr; }
  if (overlay_dsl) { lv_obj_del(overlay_dsl); overlay_dsl = nullptr; }
  ta_dsl_src = nullptr;
  lbl_dsl_err = nullptr;
}

static void on_dsl_back(lv_event_t* e) {
  LV_UNUSED(e);
  ui_force_output_off();
  close_dsl_panel(nullptr);
}

// Help sub-page: a scrollable overlay above the DSL page rendering
// DSL_HELP_TEXT; its Back deletes only the help overlay, returning to DSL
// (Decision 8).
static void close_dsl_help(lv_event_t* e) {
  LV_UNUSED(e);
  if (overlay_dsl_help) { lv_obj_del(overlay_dsl_help); overlay_dsl_help = nullptr; }
}

static void open_dsl_help(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_dsl_help) return;
  lv_obj_t* page = make_page_overlay(&overlay_dsl_help, LV_OPA_COVER);
  make_back_header(page, "DSL HELP", close_dsl_help);

  // E2-7/D13: TOP-aligned scrollable help viewer, fixed height so DSL_HELP_TEXT
  // overflows and drag-scrolls. KEEP CLICKABLE — the indev scroll engine only
  // scrolls the hit act_obj; a non-clickable textarea is skipped at hit-test so
  // it could not be drag-scrolled (review api#3). Do NOT bind on_ta_focused so
  // no on-screen keyboard pops (read-only viewer; a stray caret on tap is fine).
  lv_obj_t* ta = lv_textarea_create(page);
  lv_obj_set_size(ta, 452, 220);
  lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_add_style(ta, &style_dropdown, 0);
  lv_obj_set_style_text_font(ta, &lv_font_montserrat_14, 0);
  lv_obj_add_flag(ta, LV_OBJ_FLAG_SCROLLABLE);  // default on textarea; defensive
  lv_textarea_set_text(ta, DSL_HELP_TEXT);
  lv_obj_scroll_to_y(ta, 0, LV_ANIM_OFF);       // start at the top (D13)
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

// DSL page density (inline, Decision 12a): btn 32 except clamped Back (40).
static void open_dsl_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_dsl) return;
  lv_obj_t* page = make_page_overlay(&overlay_dsl, LV_OPA_COVER);
  // Back (clamped 40px via make_back_header) + a HELP button beside the title.
  make_back_header(page, "DSL EDITOR", on_dsl_back);

  lv_obj_t* btnHelp = lv_btn_create(page);
  lv_obj_set_size(btnHelp, 72, 40);
  lv_obj_align(btnHelp, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_add_style(btnHelp, &style_btn, 0);
  lv_obj_add_event_cb(btnHelp, open_dsl_help, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lh = lv_label_create(btnHelp); lv_label_set_text(lh, "HELP"); lv_obj_center(lh);

  // E2-6/D12: arc removed -> the editor textarea claims the full freed width.
  ta_dsl_src = lv_textarea_create(page);
  lv_obj_set_size(ta_dsl_src, 452, 150);
  lv_obj_align(ta_dsl_src, LV_ALIGN_TOP_LEFT, 14, 52);
  lv_obj_add_style(ta_dsl_src, &style_dropdown, 0);
  lv_textarea_set_placeholder_text(ta_dsl_src, "wheel DSL source...");
  // Reuse the shared on-screen keyboard for DSL entry.
  lv_obj_add_event_cb(ta_dsl_src, on_ta_focused, LV_EVENT_FOCUSED, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_focused, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_defocused, LV_EVENT_DEFOCUSED, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_defocused, LV_EVENT_READY, NULL);
  lv_obj_add_event_cb(ta_dsl_src, on_ta_defocused, LV_EVENT_CANCEL, NULL);

  lbl_dsl_err = lv_label_create(page);
  lv_obj_add_style(lbl_dsl_err, &style_caption, 0);
  lv_label_set_text(lbl_dsl_err, "");
  lv_obj_align(lbl_dsl_err, LV_ALIGN_BOTTOM_LEFT, 0, -42);

  // Buttons row (DSL density 32px; non-critical chrome keeps density).
  lv_obj_t* btnCompile = lv_btn_create(page);
  lv_obj_set_size(btnCompile, 90, 32);
  lv_obj_align(btnCompile, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_style(btnCompile, &style_btn, 0);
  lv_obj_add_event_cb(btnCompile, on_dsl_compile, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l1 = lv_label_create(btnCompile); lv_label_set_text(l1, "COMPILE"); lv_obj_center(l1);

  lv_obj_t* btnSave = lv_btn_create(page);
  lv_obj_set_size(btnSave, 90, 32);
  lv_obj_align(btnSave, LV_ALIGN_BOTTOM_LEFT, 95, 0);
  lv_obj_add_style(btnSave, &style_btn, 0);
  lv_obj_add_event_cb(btnSave, on_dsl_saveas, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l2 = lv_label_create(btnSave); lv_label_set_text(l2, "SAVE"); lv_obj_center(l2);

  lv_obj_t* btnLoad = lv_btn_create(page);
  lv_obj_set_size(btnLoad, 90, 32);
  lv_obj_align(btnLoad, LV_ALIGN_BOTTOM_LEFT, 190, 0);
  lv_obj_add_style(btnLoad, &style_btn, 0);
  lv_obj_add_event_cb(btnLoad, on_dsl_load, LV_EVENT_CLICKED, NULL);
  lv_obj_t* l3 = lv_label_create(btnLoad); lv_label_set_text(l3, "LOAD"); lv_obj_center(l3);

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
  // Single render path (E-wave-8): input cbs (zoom/pan/pause) set s_wave_dirty.
  // When paused the image is static, so only redraw on an explicit dirty;
  // when running the cursor advances every tick, so always redraw and clear
  // the flag. Bounded cost either way (w cols x 3 lanes, all clamped).
  if (s_wave_paused && !s_wave_dirty) return;
  s_wave_dirty = false;
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
  // Clamp slot_count to the DSL cap (4096). slot_count is uint16_t so it can
  // legally exceed the cap; the int64_t products below are sized for the cap.
  int slot_count = (int)p->slot_count;
  if (slot_count > 4096) slot_count = 4096;
  if (slot_count <= 0) { lv_obj_invalidate(canvas_wave); return; }

  // Unified column->slot map (E-wave-3). All slot-space math in 1/256-slot
  // fixed point; int64_t guards slot_count*256*256 and x*visible products
  // (ESP32 int==long==32-bit). At zoom=256 (1.0x) visible==slot_count<<8 so
  // the full wheel fills the full width for all 3 lanes (Ref5a).
  const int zoom = (s_wave_zoom_x256 >= 256 ? s_wave_zoom_x256 : 256);
  const long long full     = (long long)slot_count << 8;       // total span, x256
  long long visible        = (long long)slot_count * 256 * 256 / zoom;  // x256
  if (visible < 1) visible = 1;
  if (visible > full) visible = full;
  long long left = s_wave_panL_x256;
  const long long max_left = full - visible;
  if (left < 0) left = 0;
  if (left > max_left) left = max_left;
  if (visible >= full) left = 0;
  // Persist the clamped pan so input cbs and the cursor share one window.
  s_wave_panL_x256 = (long)left;

  for (int lane = 0; lane < 3; ++lane) {
    if (!(s_wave_lane_mask & lane_bits[lane])) continue;
    const uint16_t c = lane_c[lane];
    const int y_lo = lane * lane_h + lane_h - 4;   // logic LOW row
    const int y_hi = lane * lane_h + 4;            // logic HIGH row
    int prev_y = y_lo;

    for (int x = 0; x < w; ++x) {
      // Map this column to a slot range [s0..s1] via the unified window.
      int s0 = (int)((left + (long long)x       * visible / w) >> 8);
      int s1 = (int)((left + (long long)(x + 1) * visible / w) >> 8) - 1;
      if (s1 < s0) s1 = s0;
      if (s0 < 0) s0 = 0;
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

  // Cursor — 1 px, through the same window (E-wave-4). When paused it sticks at
  // the frozen slot; lanes are always static-from-table.
  const uint16_t cur = s_wave_paused ? s_wave_frozen_cursor : ui_get_edge_counter();
  if (cur < slot_count) {
    const long long cur_x256 = (long long)cur << 8;
    if (cur_x256 >= left && cur_x256 < left + visible) {
      int cx = (int)(((cur_x256 - left) * w) / visible);
      if (cx < 0) cx = 0;
      if (cx > w - 1) cx = w - 1;
      wave_vfill(db, cx, 0, h - 1, cursor_c);
    }
  }

  lv_obj_invalidate(canvas_wave);
}

// Container that hosts the full-width canvas (NO h-padding so the canvas
// can anchor at x=0 with width == full page content width — Decision 9).
// E-core builds this chrome; E-wave fills the renderer/zoom/pan/pause.
static lv_obj_t* wave_canvas_cont = nullptr;

// ZOOM+/ZOOM-/PAUSE + drag-pan handlers (E-wave-5/6/7). All of these ONLY
// mutate the view-model state and set s_wave_dirty; the 50 ms wave timer is the
// sole render path (E-wave-8) so cost stays bounded and allocation-free.
static lv_obj_t* btn_wave_pause_lbl = nullptr;  // PAUSE/PLAY relabel target

// Helper: current visible span (x256) for the active pattern at the current
// zoom. Returns 0 when no pattern (callers treat that as "no-op").
static long long wave_visible_x256() {
  const PatternRef* p = ui_get_active_pattern_for_wave();
  if (!p || p->slot_count == 0) return 0;
  int slot_count = (int)p->slot_count;
  if (slot_count > 4096) slot_count = 4096;
  const int zoom = (s_wave_zoom_x256 >= 256 ? s_wave_zoom_x256 : 256);
  const long long full = (long long)slot_count << 8;
  long long visible = (long long)slot_count * 256 * 256 / zoom;
  if (visible < 1) visible = 1;
  if (visible > full) visible = full;
  return visible;
}

// Center-anchored zoom (E-wave-5): keep the view center fixed, clamp pan.
static void wave_zoom_by(int new_zoom_x256) {
  const PatternRef* p = ui_get_active_pattern_for_wave();
  if (!p || p->slot_count == 0) return;
  int slot_count = (int)p->slot_count;
  if (slot_count > 4096) slot_count = 4096;
  if (new_zoom_x256 < 256)        new_zoom_x256 = 256;
  if (new_zoom_x256 > 256 * 32)   new_zoom_x256 = 256 * 32;

  const long long full = (long long)slot_count << 8;
  // Center of the CURRENT window.
  long long vis_old = (long long)slot_count * 256 * 256 / s_wave_zoom_x256;
  if (vis_old < 1) vis_old = 1;
  if (vis_old > full) vis_old = full;
  const long long center = (long long)s_wave_panL_x256 + vis_old / 2;

  // New window centered on that point.
  long long vis_new = (long long)slot_count * 256 * 256 / new_zoom_x256;
  if (vis_new < 1) vis_new = 1;
  if (vis_new > full) vis_new = full;
  long long left = center - vis_new / 2;
  const long long max_left = full - vis_new;
  if (left < 0) left = 0;
  if (left > max_left) left = max_left;
  if (vis_new >= full) left = 0;   // snapped to full-fit

  s_wave_zoom_x256 = new_zoom_x256;
  s_wave_panL_x256 = (long)left;
  s_wave_dirty = true;
}

static void on_wave_zoom_in(lv_event_t* e) {
  LV_UNUSED(e);
  wave_zoom_by(s_wave_zoom_x256 * 2);   // wave_zoom_by caps at 256*32
}
static void on_wave_zoom_out(lv_event_t* e) {
  LV_UNUSED(e);
  wave_zoom_by(s_wave_zoom_x256 / 2);   // wave_zoom_by floors at 256 (full-fit)
}

// PAUSE/PLAY (E-wave-7): freezes ONLY the cursor; lanes stay static-from-table,
// backend keeps running. Relabel the button and request a redraw.
static void on_wave_pause(lv_event_t* e) {
  LV_UNUSED(e);
  s_wave_paused = !s_wave_paused;
  if (s_wave_paused) s_wave_frozen_cursor = ui_get_edge_counter();
  if (btn_wave_pause_lbl) lv_label_set_text(btn_wave_pause_lbl, s_wave_paused ? "PLAY" : "PAUSE");
  s_wave_dirty = true;
}

// Drag-to-pan (E-wave-6): single-finger horizontal drag on the canvas. No-op at
// full-fit (visible >= full). Hard-clamp pan (no rubber-band).
static void on_wave_drag(lv_event_t* e) {
  LV_UNUSED(e);
  if (!canvas_wave) return;
  lv_draw_buf_t* db = lv_canvas_get_draw_buf(canvas_wave);
  if (!db || !db->data) return;
  const int w = (int)db->header.w;
  if (w <= 0) return;

  const PatternRef* p = ui_get_active_pattern_for_wave();
  if (!p || p->slot_count == 0) return;
  int slot_count = (int)p->slot_count;
  if (slot_count > 4096) slot_count = 4096;

  const long long full    = (long long)slot_count << 8;
  const long long visible = wave_visible_x256();
  if (visible <= 0 || visible >= full) return;   // no pan at full-fit

  lv_indev_t* d = lv_indev_active();   // lv_event_get_indev(e) is equally valid
  if (!d) return;
  lv_point_t v;
  lv_indev_get_vect(d, &v);
  if (v.x == 0) return;

  const long long delta = -(long long)v.x * visible / w;  // drag right -> view left
  long long left = (long long)s_wave_panL_x256 + delta;
  const long long max_left = full - visible;
  if (left < 0) left = 0;
  if (left > max_left) left = max_left;
  s_wave_panL_x256 = (long)left;
  s_wave_dirty = true;
}

static void on_wave_live_tick(lv_timer_t* t) {
  LV_UNUSED(t);
  update_live_arc(arc_wave_live);
}

static void close_wave_panel(lv_event_t* e) {
  LV_UNUSED(e);
  // Restore normal touch coalescing (slow-pan fix is WAVE-page-only, E-wave-6).
  s_wave_drag_coalesce_off = false;
  if (tmr_wave) { lv_timer_del(tmr_wave); tmr_wave = nullptr; }
  if (tmr_wave_live) { lv_timer_del(tmr_wave_live); tmr_wave_live = nullptr; }
  if (canvas_wave_buf) { heap_caps_free(canvas_wave_buf); canvas_wave_buf = nullptr; }
  if (overlay_wave) { lv_obj_del(overlay_wave); overlay_wave = nullptr; }
  canvas_wave = nullptr;
  wave_canvas_cont = nullptr;
  arc_wave_live = nullptr;
  btn_wave_pause_lbl = nullptr;
  s_wave_paused = false;
}

static void on_wave_back(lv_event_t* e) {
  LV_UNUSED(e);
  ui_force_output_off();
  close_wave_panel(nullptr);
}

static void open_wave_panel(lv_event_t* e) {
  LV_UNUSED(e);
  if (!screen_main || overlay_wave) return;
  lv_obj_t* page = make_page_overlay(&overlay_wave, LV_OPA_COVER);
  make_back_header(page, "WAVEFORM", on_wave_back);

  // Reset the Q24.8 view model on open (E-wave-2): full-fit, no pan, playing.
  s_wave_zoom_x256 = 256;
  s_wave_panL_x256 = 0;
  s_wave_paused = false;
  s_wave_frozen_cursor = 0;
  s_wave_dirty = true;
  // Disable my_touchpad_read's <3px/50ms move coalescing while WAVE is open so
  // slow fine drags report a real vector and pan (E-wave-6).
  s_wave_drag_coalesce_off = true;

  // Lazy live RPM arc (Decision 14) — top-right, small (chrome only).
  arc_wave_live = make_page_live_arc(page);
  lv_obj_set_size(arc_wave_live, 56, 56);
  lv_obj_align(arc_wave_live, LV_ALIGN_TOP_RIGHT, 0, 0);

  // Button row: ZOOM+ / ZOOM- / PAUSE (>=40px Back already in header).
  lv_obj_t* btnZoomIn = lv_btn_create(page);
  lv_obj_set_size(btnZoomIn, 80, 36);
  lv_obj_align(btnZoomIn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_add_style(btnZoomIn, &style_btn, 0);
  lv_obj_add_event_cb(btnZoomIn, on_wave_zoom_in, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lz1 = lv_label_create(btnZoomIn); lv_label_set_text(lz1, "ZOOM+"); lv_obj_center(lz1);

  lv_obj_t* btnZoomOut = lv_btn_create(page);
  lv_obj_set_size(btnZoomOut, 80, 36);
  lv_obj_align(btnZoomOut, LV_ALIGN_BOTTOM_LEFT, 88, 0);
  lv_obj_add_style(btnZoomOut, &style_btn, 0);
  lv_obj_add_event_cb(btnZoomOut, on_wave_zoom_out, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lz2 = lv_label_create(btnZoomOut); lv_label_set_text(lz2, "ZOOM-"); lv_obj_center(lz2);

  lv_obj_t* btnPause = lv_btn_create(page);
  lv_obj_set_size(btnPause, 80, 36);
  lv_obj_align(btnPause, LV_ALIGN_BOTTOM_LEFT, 176, 0);
  lv_obj_add_style(btnPause, &style_btn, 0);
  lv_obj_add_event_cb(btnPause, on_wave_pause, LV_EVENT_CLICKED, NULL);
  btn_wave_pause_lbl = lv_label_create(btnPause);
  lv_label_set_text(btn_wave_pause_lbl, "PAUSE"); lv_obj_center(btn_wave_pause_lbl);

  // FULL-WIDTH canvas container: NO horizontal padding so the canvas anchors
  // at x=0 with width == full page content width (Decision 9 / E-wave-2).
  // E-wave sizes the buffer from db->header.w, NOT a hardcoded constant.
  wave_canvas_cont = lv_obj_create(page);
  lv_obj_set_size(wave_canvas_cont, lv_pct(100), 144);
  lv_obj_align(wave_canvas_cont, LV_ALIGN_TOP_LEFT, 0, 58);
  lv_obj_set_style_bg_color(wave_canvas_cont, lv_color_hex(COL_SUNKEN), 0);
  lv_obj_set_style_bg_opa(wave_canvas_cont, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(wave_canvas_cont, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_border_width(wave_canvas_cont, 1, 0);
  lv_obj_set_style_border_opa(wave_canvas_cont, LV_OPA_40, 0);
  lv_obj_set_style_pad_all(wave_canvas_cont, 0, 0);
  lv_obj_set_style_radius(wave_canvas_cont, 0, 0);
  lv_obj_clear_flag(wave_canvas_cont, LV_OBJ_FLAG_SCROLLABLE);

  // Full-width canvas at x=0 (Decision 9 / E-wave-2). cw == full page content
  // width (480 minus the overlay's 2*8 pad == 464); the renderer reads
  // db->header.w so the exact pixel width drives the column->slot map. ch is
  // divisible by 3 (lane_h 48). Buffer at RGB565 (2 B/px): ~464*2 = 928 B/row
  // aligned, *144 + align ~= 135 KB in PSRAM.
  const int cw = SCREEN_W - 2 * 8;   // 464
  const int ch = 144;                // lane_h = 48
  const size_t buf_bytes = LV_CANVAS_BUF_SIZE(cw, ch, LV_COLOR_DEPTH, LV_DRAW_BUF_STRIDE_ALIGN);
  // PSRAM-first; on PSRAM-alloc failure do NOT fall back to internal heap
  // (~135 KB would risk internal OOM) — show a "PSRAM required" label instead.
  canvas_wave_buf = (lv_color_t*)heap_caps_malloc(buf_bytes,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (canvas_wave_buf) {
    canvas_wave = lv_canvas_create(wave_canvas_cont);
    lv_canvas_set_buffer(canvas_wave, canvas_wave_buf, cw, ch, LV_COLOR_FORMAT_NATIVE);
    lv_obj_set_size(canvas_wave, cw, ch);
    lv_obj_align(canvas_wave, LV_ALIGN_TOP_LEFT, 0, 0);
    // The image base class removes CLICKABLE; re-add it or PRESSING never fires
    // (E-wave-2/6). Clear SCROLLABLE on the canvas AND its actual parent
    // container (wave_canvas_cont) so drags pan instead of scrolling.
    lv_obj_add_flag(canvas_wave, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(canvas_wave, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(wave_canvas_cont, LV_OBJ_FLAG_SCROLLABLE);
    // Drag-to-pan: PRESSING fires continuously while held (E-wave-6).
    lv_obj_add_event_cb(canvas_wave, on_wave_drag, LV_EVENT_PRESSING, NULL);
  } else {
    lv_obj_t* lbl = lv_label_create(wave_canvas_cont);
    lv_label_set_text(lbl, "PSRAM required");
    lv_obj_add_style(lbl, &style_caption, 0);
    lv_obj_center(lbl);
  }

  // 50 ms (20 Hz) render timer (E-wave rewrites on_wave_tick body).
  if (canvas_wave) tmr_wave = lv_timer_create(on_wave_tick, 50, NULL);
  // Lazy live RPM arc timer (Decision 14).
  tmr_wave_live = lv_timer_create(on_wave_live_tick, 100, NULL);
}

#endif  // SIGGEN_HAS_DISPLAY

