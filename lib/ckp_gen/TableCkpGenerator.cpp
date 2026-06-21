// TableCkpGenerator — native byte-table backend (M1.1 + M2.1 implementation).
//
// Parent reference: implementation_plan.md §M1.1 / §M2.1 + §6 Agent A hard
// rules. See TableCkpGenerator.h for the full design rationale and the ISR
// 5-statement budget contract.

#include "TableCkpGenerator.h"

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_err.h>

// ESP-IDF drivers exposed by the pioarduino arduino-esp32 framework.
#include "driver/dedic_gpio.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Pin validation — JC4827W543 board on ESP32-S3-WROOM-1 N4R8
// ---------------------------------------------------------------------------
static bool isValidEsp32S3OutputPin(int pin) {
  if (pin < 0 || pin > 48) {
    Serial.printf("[gen] pin %d out of range (0..48)\n", pin);
    return false;
  }
  // GPIO 22..25 do not exist on ESP32-S3.
  if (pin >= 22 && pin <= 25) {
    Serial.printf("[gen] pin %d not present on ESP32-S3\n", pin);
    return false;
  }
  // GPIO 26..32 are reserved for SPI flash / PSRAM on N4R8 modules.
  if (pin >= 26 && pin <= 32) {
    Serial.printf("[gen] pin %d reserved for flash/PSRAM on N4R8\n", pin);
    return false;
  }
  // Strapping pins — reject to avoid boot mode / JTAG conflicts.
  if (pin == 0 || pin == 3 || pin == 45 || pin == 46) {
    Serial.printf("[gen] pin %d is a strapping pin (reject)\n", pin);
    return false;
  }
  // Board-reserved set for JC4827W543:
  //   LCD QSPI: 45, 47, 21, 48, 40, 39
  //   Backlight: 1
  //   Touch (GT911): 8, 4, 3, 38
  //   SD card: 10, 11, 12, 13
  //   I2S audio: 42, 2, 41
  switch (pin) {
  case 45:
  case 47:
  case 21:
  case 48:
  case 40:
  case 39:
    Serial.printf("[gen] pin %d reserved for LCD QSPI\n", pin);
    return false;
  case 1:
    Serial.printf("[gen] pin %d reserved for LCD backlight\n", pin);
    return false;
  case 8:
  case 4:
  case 38:
    Serial.printf("[gen] pin %d reserved for GT911 touch\n", pin);
    return false;
  case 10:
  case 11:
  case 12:
  case 13:
    Serial.printf("[gen] pin %d reserved for SD card\n", pin);
    return false;
  case 42:
  case 2:
  case 41:
    Serial.printf("[gen] pin %d reserved for I2S audio\n", pin);
    return false;
  default:
    break;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Lifetime
// ---------------------------------------------------------------------------

TableCkpGenerator::TableCkpGenerator()
    : _pin_crank(-1), _pin_cam1(-1), _pin_cam2(-1), _bundle_width(0),
      _bundle_mask(0), _table(nullptr), _slot_count(0), _table_degrees(360),
      _edge_counter(0),
      _invert_mask(0), _reverse(false), _cycle_start_us(0),
      _cycle_duration_us(0), _last_rpm(0), _timer(nullptr), _bundle(nullptr),
      _running(false), _initialized(false) {}

TableCkpGenerator::~TableCkpGenerator() {
  // Best-effort teardown. We don't attempt to re-enter the driver in
  // unusual error states; the system is expected to be alive at dtor time.
  if (_timer) {
    gptimer_stop(_timer);
    gptimer_disable(_timer);
    gptimer_del_timer(_timer);
    _timer = nullptr;
  }
  if (_bundle) {
    dedic_gpio_del_bundle(_bundle);
    _bundle = nullptr;
  }
  if (_playback_buffer) {
    heap_caps_free(_playback_buffer);
    _playback_buffer = nullptr;
  }
}

// ---------------------------------------------------------------------------
// begin() — configure dedic_gpio bundle (width 1/2/3 per pin args) + gptimer
// ---------------------------------------------------------------------------

bool TableCkpGenerator::begin(int pin_crank, int pin_cam1, int pin_cam2) {
  if (_initialized) {
    return true;
  }
  if (pin_crank < 0) {
    _last_error = GenError::GPIO_FAIL;
    return false;
  }

  // Validate every requested output pin BEFORE touching gpio_config or
  // dedic_gpio — bad pins must not leave hardware half-configured.
  if (!isValidEsp32S3OutputPin(pin_crank)) {
    Serial.printf("[gen] begin: pin %d invalid (crank)\n", pin_crank);
    _last_error = GenError::GPIO_FAIL;
    return false;
  }
  if (pin_cam1 >= 0 && !isValidEsp32S3OutputPin(pin_cam1)) {
    Serial.printf("[gen] begin: pin %d invalid (cam1)\n", pin_cam1);
    _last_error = GenError::GPIO_FAIL;
    return false;
  }
  if (pin_cam2 >= 0 && !isValidEsp32S3OutputPin(pin_cam2)) {
    Serial.printf("[gen] begin: pin %d invalid (cam2)\n", pin_cam2);
    _last_error = GenError::GPIO_FAIL;
    return false;
  }

  _pin_crank = pin_crank;
  _pin_cam1 = pin_cam1;
  _pin_cam2 = pin_cam2;

  // 1. Build the bundle pin array. Order matters — index 0 → bit0 (crank),
  //    index 1 → bit1 (cam1), index 2 → bit2 (cam2). This matches the
  //    Ardu-Stim byte-packing convention so a byte read from a PatternRef
  //    table can be written straight to the bundle.
  int bundle_pins[3] = {_pin_crank, 0, 0};
  uint8_t width = 1;
  if (pin_cam1 >= 0) {
    bundle_pins[1] = pin_cam1;
    width = 2;
    if (pin_cam2 >= 0) {
      bundle_pins[2] = pin_cam2;
      width = 3;
    }
  }
  _bundle_width = width;
  _bundle_mask = (uint8_t)((1u << width) - 1u); // 0x01 / 0x03 / 0x07

  // 2. Configure all participating GPIOs as plain OUTPUT first; dedic_gpio
  //    requires the underlying pins to already be in OUTPUT mode.
  uint64_t pin_mask = (1ULL << _pin_crank);
  if (width >= 2)
    pin_mask |= (1ULL << bundle_pins[1]);
  if (width >= 3)
    pin_mask |= (1ULL << bundle_pins[2]);

  gpio_config_t io_cfg = {};
  io_cfg.pin_bit_mask = pin_mask;
  io_cfg.mode = GPIO_MODE_OUTPUT;
  io_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  io_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_cfg.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&io_cfg) != ESP_OK) {
    _last_error = GenError::GPIO_FAIL;
    return false;
  }
  gpio_set_level((gpio_num_t)_pin_crank, 0);
  if (width >= 2)
    gpio_set_level((gpio_num_t)bundle_pins[1], 0);
  if (width >= 3)
    gpio_set_level((gpio_num_t)bundle_pins[2], 0);

  // Lower output drive strength to slow the native edge rate, reducing
  // scope-ground-lead ringing and EMI coupling on the bench, without
  // changing the logical waveform or timing.
  // NOTE: chip default is GPIO_DRIVE_CAP_2 (~20 mA — the medium/default
  // setting; CAP_3 ~40 mA is the strongest). The SDK enum names
  // (weak/stronger/medium/strongest) are misleading; numerically
  // 0<1<2<3 = ~5/10/20/40 mA. CAP_1 (~10 mA) therefore LOWERS drive
  // vs default while keeping a crisp 0/3.3 V swing into a high-Z input.
  static constexpr gpio_drive_cap_t kOutputDriveCap = GPIO_DRIVE_CAP_1;
  gpio_set_drive_capability((gpio_num_t)_pin_crank, kOutputDriveCap);
  if (width >= 2)
    gpio_set_drive_capability((gpio_num_t)bundle_pins[1], kOutputDriveCap);
  if (width >= 3)
    gpio_set_drive_capability((gpio_num_t)bundle_pins[2], kOutputDriveCap);

  // 3. Allocate the dedic_gpio bundle of the chosen width.
  dedic_gpio_bundle_config_t bundle_cfg = {};
  bundle_cfg.gpio_array = bundle_pins;
  bundle_cfg.array_size = width;
  bundle_cfg.flags.out_en = 1;
  if (dedic_gpio_new_bundle(&bundle_cfg, &_bundle) != ESP_OK) {
    _last_error = GenError::GPIO_FAIL;
    return false;
  }

  // 4. Build the gptimer with a 1 MHz resolution (= 1 µs per tick) and
  //    a non-zero default alarm. The alarm is reprogrammed by apply().
  gptimer_config_t timer_cfg = {};
  timer_cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_cfg.direction = GPTIMER_COUNT_UP;
  timer_cfg.resolution_hz = 1000000; // 1 µs tick
  if (gptimer_new_timer(&timer_cfg, &_timer) != ESP_OK) {
    _last_error = GenError::TIMER_FAIL;
    return false;
  }

  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = &TableCkpGenerator::onAlarm;
  if (gptimer_register_event_callbacks(_timer, &cbs, this) != ESP_OK) {
    _last_error = GenError::TIMER_FAIL;
    return false;
  }

  gptimer_alarm_config_t alarm_cfg = {};
  alarm_cfg.alarm_count = 1000; // 1 ms placeholder
  alarm_cfg.reload_count = 0;
  alarm_cfg.flags.auto_reload_on_alarm = true;
  if (gptimer_set_alarm_action(_timer, &alarm_cfg) != ESP_OK) {
    _last_error = GenError::TIMER_FAIL;
    return false;
  }

  if (gptimer_enable(_timer) != ESP_OK) {
    _last_error = GenError::TIMER_FAIL;
    return false;
  }

  // =====================================================================
  // ====== PATTERN PLAYBACK BUFFER ALLOCATION =============================
  // Pre-allocates a fixed-size buffer in ESP32-S3 internal DRAM. The ISR
  // reads pattern bytes from this buffer instead of flash/PSRAM, which
  // would be cache-unsafe during NVS / LittleFS writes at runtime.
  //
  // To tune buffer size: change kPlaybackBufferBytes in TableCkpGenerator.h.
  // Current allocation: 24 KB. Max slot_count enforced in apply().
  // =====================================================================
  _playback_buffer = (uint8_t *)heap_caps_malloc(
      kPlaybackBufferBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (_playback_buffer == nullptr) {
    Serial.printf("[gen] begin: failed to allocate %u-byte playback buffer\n",
                  (unsigned)kPlaybackBufferBytes);
    _last_error = GenError::NOT_INITIALIZED;
    return false;
  }
  Serial.printf("[gen] playback buffer: %u bytes at %p (internal DRAM)\n",
                (unsigned)kPlaybackBufferBytes, _playback_buffer);

  _initialized = true;
  _last_error = GenError::OK;
  return true;
}

// ---------------------------------------------------------------------------
// apply() — switch active pattern (manager-thread only; not ISR-safe)
// ---------------------------------------------------------------------------

bool TableCkpGenerator::apply(const PatternRef &ref, uint32_t rpm) {
  if (!_initialized) {
    _last_error = GenError::NOT_INITIALIZED;
    return false;
  }
  if (ref.table == nullptr) {
    _last_error = GenError::NO_TABLE;
    return false;
  }
  if (ref.slot_count == 0) {
    _last_error = GenError::BAD_SLOT_COUNT;
    return false;
  }
  if (ref.slot_count > kPlaybackBufferBytes) {
    Serial.printf("[gen] apply: pattern slot_count=%u exceeds 24KB buffer\n",
                  (unsigned)ref.slot_count);
    _last_error = GenError::BUFFER_OVERFLOW;
    return false;
  }
  if (rpm < 10) {
    _last_error = GenError::BAD_RPM;
    return false;
  }

  // Pause the timer while we swap pointer/slot_count to keep the ISR's
  // (_table, _slot_count) pair consistent.
  const bool was_running = _running;
  if (was_running) {
    gptimer_stop(_timer);
  }

  // Copy pattern bytes from .rodata/PSRAM into our pre-allocated internal-
  // DRAM buffer so the ISR is cache-safe during NVS/LittleFS writes.
  memcpy(_playback_buffer, ref.table, ref.slot_count);

  _table = _playback_buffer;
  _slot_count = ref.slot_count;
  // Persist the degree span BEFORE reprogramAlarm(rpm) below so the scaled
  // period formula never sees an uninitialized span (ordering invariant:
  // setRpm() is always preceded by an apply() that publishes _table +
  // _table_degrees together). Only 360/720 are legal — clamp anything else
  // to 360 (single-rev) per the crank-RPM scaling.
  _table_degrees = (ref.degrees == 720) ? 720 : 360;
  _edge_counter = 0;
  _cycle_start_us = (uint32_t)micros();
  _cycle_duration_us = 0;

  if (!reprogramAlarm(rpm)) {
    _last_error = GenError::TIMER_FAIL;
    return false;
  }

  if (was_running) {
    gptimer_set_raw_count(_timer, 0);
    gptimer_start(_timer);
  }
  _last_error = GenError::OK;
  return true;
}

// ---------------------------------------------------------------------------
// setRpm() — fast path. Only the alarm period changes; no buffer rebuild.
// ---------------------------------------------------------------------------

bool TableCkpGenerator::setRpm(uint32_t rpm) {
  if (!_initialized)
    return false;
  if (_table == nullptr)
    return false;
  if (_slot_count == 0)
    return false;
  if (rpm < 10)
    return false;
  return reprogramAlarm(rpm);
}

bool TableCkpGenerator::reprogramAlarm(uint32_t rpm) {
  // period_us = 60_000_000 * _table_degrees / (360 * rpm * slot_count)
  // RPM is a UNIVERSAL CRANK RPM: a 720 span (2 crank revs) carries the
  // 720/360=2 factor in the numerator → half rate (cam = ½ crank). For a
  // 360 span this reduces to 60_000_000/(rpm*slot_count).
  // (See header comment for equivalence with Ardu-Stim's OCR1A formula.)
  // uint64 overflow-safe: 60_000_000*720 fits uint64; denom 360*6000*4096 fits.
  uint64_t denom = 360ULL * (uint64_t)rpm * (uint64_t)_slot_count;
  if (denom == 0) {
    return false;
  }
  uint64_t period_us = 60000000ULL * (uint64_t)_table_degrees / denom;
  if (period_us == 0) {
    period_us = 1; // floor — clip to 1 µs to keep gptimer alarming
  }

  gptimer_alarm_config_t alarm_cfg = {};
  alarm_cfg.alarm_count = period_us;
  alarm_cfg.reload_count = 0;
  alarm_cfg.flags.auto_reload_on_alarm = true;
  if (gptimer_set_alarm_action(_timer, &alarm_cfg) != ESP_OK) {
    return false;
  }
  _last_rpm = rpm;
  return true;
}

// ---------------------------------------------------------------------------
// Inversion / start / stop / accessors
// ---------------------------------------------------------------------------

void TableCkpGenerator::setInverted(uint8_t channel_mask) {
  // Single-byte volatile store — atomic. bit0=crank, bit1=cam1, bit2=cam2.
  // Bits beyond _bundle_width are XOR'd into the byte but get dropped by
  // the bundle's _bundle_mask on write, so they are harmless.
  _invert_mask = channel_mask;
}

uint8_t TableCkpGenerator::getInverted() const { return _invert_mask; }

void TableCkpGenerator::setReverse(bool reverse) {
  _reverse = reverse; // single-byte volatile store — atomic
}

bool TableCkpGenerator::getReverse() const { return _reverse; }

bool TableCkpGenerator::start() {
  if (!_initialized) {
    _last_error = GenError::NOT_INITIALIZED;
    return false;
  }
  if (_running) {
    _last_error = GenError::OK;
    return true;
  }
  if (_table == nullptr) {
    _last_error = GenError::NO_TABLE;
    return false;
  }
  if (_slot_count == 0) {
    _last_error = GenError::BAD_SLOT_COUNT;
    return false;
  }
  gptimer_set_raw_count(_timer, 0);
  esp_err_t err = gptimer_start(_timer);
  if (err != ESP_OK) {
    Serial.printf("[gen] gptimer_start failed: %d\n", (int)err);
    _last_error = GenError::TIMER_FAIL;
    return false;
  }
  _running = true;
  _last_error = GenError::OK;
  return true;
}

bool TableCkpGenerator::stop() {
  if (!_initialized) {
    _last_error = GenError::NOT_INITIALIZED;
    return false;
  }
  if (!_running) {
    _last_error = GenError::OK;
    return true;
  }
  esp_err_t err = gptimer_stop(_timer);
  if (err != ESP_OK) {
    Serial.printf("[gen] gptimer_stop failed: %d\n", (int)err);
    _last_error = GenError::TIMER_FAIL;
    return false;
  }
  _running = false;
  // Drive every active bundle pin low so all outputs are in a known safe
  // state. _bundle_mask covers exactly the configured channels.
  if (_bundle) {
    dedic_gpio_bundle_write(_bundle, _bundle_mask, 0);
  }
  _last_error = GenError::OK;
  return true;
}

uint16_t TableCkpGenerator::getEdgeCounter() const {
  // Single naturally-aligned uint16_t read; Xtensa guarantees atomicity
  // against the ISR writer of the same word — no critical section needed.
  return _edge_counter;
}

uint32_t TableCkpGenerator::getCycleStartUs() const {
  return _cycle_start_us; // aligned 32-bit atomic read
}

uint32_t TableCkpGenerator::getCycleDurationUs() const {
  return _cycle_duration_us;
}

// ---------------------------------------------------------------------------
// ISR (≤ 5 statements of real work — per §6 Agent A)
// ---------------------------------------------------------------------------
//
// Statement budget walk (5 statements — matches §6 hard cap):
//   (1) byte load + XOR invert mask (single fused expression).
//   (2) dedic_gpio_bundle_write — atomic write of up to 3 channels in one
//       cycle. _bundle_mask selects only valid bits; extra invert bits are
//       discarded harmlessly.
//   (3) advance + wrap. One compound if/else: forward path increments and
//       wraps to 0 (publishing cycle timing on wrap); reverse path decrements
//       and wraps to slot_count-1. The compound statement counts as one per
//       the C++ statement-counting rule (a selection-statement is a single
//       statement regardless of body size).
//   (4) store advanced edge back to the volatile member.
//   (5) return false.
//
bool IRAM_ATTR TableCkpGenerator::onAlarm(
    gptimer_handle_t /*timer*/, const gptimer_alarm_event_data_t * /*edata*/,
    void *user_ctx) {
  TableCkpGenerator *self = static_cast<TableCkpGenerator *>(user_ctx);
  uint16_t e = self->_edge_counter;

  // Statement 1: load byte from .rodata table and apply inversion mask.
  uint8_t b = self->_table[e] ^ self->_invert_mask;
  // Statement 2: write all active channels atomically. _bundle_mask is
  // (1<<width)-1, so bits beyond active channels are discarded — this is
  // the per-channel slot-alignment guarantee (zero inter-channel skew).
  dedic_gpio_bundle_write(self->_bundle, self->_bundle_mask, b);
  // Statement 3: advance + wrap (forward or reverse). On forward wrap we
  // publish cycleDuration/cycleStart for Agent C's compression task.
  if (self->_reverse) {
    e = (e == 0) ? (uint16_t)(self->_slot_count - 1) : (uint16_t)(e - 1);
  } else if (++e >= self->_slot_count) {
    e = 0;
    uint32_t now = (uint32_t)esp_timer_get_time();
    self->_cycle_duration_us = now - self->_cycle_start_us;
    self->_cycle_start_us = now;
  }
  // Statement 4: commit advanced counter.
  self->_edge_counter = e;
  // Statement 5: signal no high-priority task wakeup.
  return false;
}
