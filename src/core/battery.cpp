#include "battery.h"
#include "usb_serial.h"
#include "usb_mode.h"
#include <Arduino.h>
#include <esp_system.h>
#include "battery_logic.h"

static const int PIN_BATTERY = 4;  // ADC1 ch3, behind a divider by 2

// Calibration: with a full battery and the charger finished, the raw reading was 4.08-4.09 V where the
// cell really sits at about 4.17 V, so the ADC reads about 2% low through the 200k/200k divider.
static const float CALIBRATION = 4170.0f / 4085.0f;
static battery::ChargeVoltage chargeVoltage;

#if UNIDEX_DEV
static int fakeMv;  // test build (X BATTMV <mv>): pretend the battery reads this (0 = the real reading)
#endif

int batteryMillivolts() {
#if UNIDEX_DEV
  if (fakeMv) return fakeMv;
#endif
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);  // the pin sees about 2.1 V at a full battery
  uint32_t sum = 0;
  for (int i = 0; i < 32; i++) sum += analogReadMilliVolts(PIN_BATTERY);
  return sum / 32 * 2 * CALIBRATION;
}

BatteryAdcSample batteryAdcSample() {
  const int mv = batteryMillivolts();
  constexpr int samples = 128;
  int sum = 0, lo = 4095, hi = 0;
  for (int i = 0; i < samples; ++i) {
    const int raw = analogRead(PIN_BATTERY);
    sum += raw;
    lo = min(lo, raw);
    hi = max(hi, raw);
  }
  return {mv, sum / samples, hi - lo};
}

// Kept through deep sleep and through a restart (a crash, the watchdog, an update), so the % carries on instead of
// starting again from the voltage; only a power cut loses it. Checked, as this memory isn't cleared at power-up.
RTC_NOINIT_ATTR static battery::State state;
RTC_NOINIT_ATTR static uint32_t stateCheck, topupFor;  // topupFor: ms topped up so far (millis() restarts at 0)

static uint32_t checksum() {
  uint32_t h = 2166136261u ^ topupFor;  // FNV-1a over the state
  for (size_t i = 0; i < sizeof state; i++) h = (h ^ ((const uint8_t *)&state)[i]) * 16777619u;
  return h;
}

static void keep() {
  topupFor = state.toppingUp ? millis() - state.topupAt : 0;
  stateCheck = checksum();
}

static bool kept;

static void restore() {
  static bool done;
  if (done) return;
  done = true;
  const esp_reset_reason_t why = esp_reset_reason();
  const bool power = why == ESP_RST_POWERON || why == ESP_RST_BROWNOUT || why == ESP_RST_UNKNOWN;
  kept = !power && stateCheck == checksum() && state.shown <= 100;
  if (!kept) state = battery::State();
  state.topupAt = millis() - topupFor;
  state.peakAt = millis();  // (millis() starts again: levelling off is timed afresh)
}

#if UNIDEX_DEV
bool batteryKept() {
  restore();
  return kept;
}

void batteryFake(int mv) {
  fakeMv = mv;
  chargeVoltage = battery::ChargeVoltage();
  restore();
  state = battery::State();  // a pretend reading mustn't shape the real %: start again as at power-up
  keep();
}
#endif

#if UNIDEX_DEV
// Test build: every % worked out (by the screens or the battery log), for X BATTCALLS.
struct Call {
  uint32_t ms;
  int16_t mv;
  int8_t pct;
  uint8_t plugged, charging;
};
static Call calls[300];
static uint32_t callCount;
void batteryPrintCalls() {
  for (uint32_t i = callCount > 300 ? callCount - 300 : 0; i < callCount; i++) {
    const Call &c = calls[i % 300];
    usbSerial().printf("BC %lu %d %d %d %d\n", (unsigned long)c.ms, c.mv, c.plugged, c.charging, c.pct);
  }
}
#endif

int batteryPercent() {
  const int mv = batteryMillivolts();
  const bool charging = batteryCharging();
  restore();
  const int pct = battery::update(state, mv, charging, millis());
  keep();
#if UNIDEX_DEV
  calls[callCount++ % 300] = {millis(), (int16_t)mv, (int8_t)pct, (uint8_t)usbHostConnected(), (uint8_t)charging};
#endif
  return pct;
}

const char *batteryTooLow() {
  return battery::level(batteryMillivolts(), batteryCharging()) == battery::Level::OK ? nullptr
                                                                                     : "Battery too low";
}

bool batteryEmpty() { return battery::level(batteryMillivolts(), batteryCharging()) == battery::Level::EMPTY; }

bool batteryCharging() {
  const uint32_t now = millis();
  if (chargeVoltage.due(now)) chargeVoltage.add(batteryMillivolts(), now);
#if UNIDEX_DEV
  if (fakeMv) return chargeVoltage.aboveThreshold;
#endif
  return usbHostTrafficRecent() || chargeVoltage.aboveThreshold;
}

bool batteryToppingUp() {
  restore();
  return state.toppingUp;
}
