#include "power.h"
#include "input.h"
#include "net.h"
#include "storage.h"
#include "battery.h"
#include "display.h"
#include "theme.h"
#include "usb_mode.h"
#include "../apps/notes/phone.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>

static uint32_t sleepS = 10;
static bool saver, likelyWallPower, sleepSoon;
static uint32_t powerSampleAt;
static constexpr uint32_t LAN_IDLE_MS = 5 * 60 * 1000;

static uint32_t idleTimeoutMs() { return (powerSaver() ? 5 : sleepS) * 1000; }

static void sampleWallPower() {
  likelyWallPower = batteryNearChargeVoltage(batteryMillivolts(), likelyWallPower);
  powerSampleAt = millis();
}

static const gpio_num_t PIN_LATCH = GPIO_NUM_17;      // HIGH keeps battery power on; has a pull-down
// Active LOW: the codec's supply, which also powers the I2C pull-ups. clock.cpp turns it on at start-up (the clock chip
// can't be read without it); off for deep sleep.
static const gpio_num_t PIN_AUDIO_PWR = GPIO_NUM_42;
static const gpio_num_t PIN_EPD_PWR = GPIO_NUM_6;     // active LOW
static const gpio_num_t PIN_EPD_RST = GPIO_NUM_9;
static const gpio_num_t PIN_EPD_CS = GPIO_NUM_11;
// Both buttons have external 10k pull-ups, so no internal pull-ups are needed in sleep.
static const uint64_t WAKE_MASK = (1ULL << GPIO_NUM_0) | (1ULL << GPIO_NUM_18);

static uint32_t lastActivity;
static time_t wakeAt[(int)WakeFor::Count];  // powerWakeAt, one an app: 0 = nothing to wake for
static void deepSleep(uint64_t timerUs);
static uint64_t untilWake();
static constexpr uint64_t PROBE_INTERVAL_US = 60ULL * 1000000ULL;
enum class ProbePhase : uint8_t { Idle, Baseline, AfterMinute, Complete };
RTC_DATA_ATTR static ProbePhase probePhase;
RTC_DATA_ATTR static BatteryProbeResult probeResult;
static bool probeRequested, probeFinishedAtBoot;
static volatile int holds;  // powerHold() calls not yet released (from either core)
static portMUX_TYPE holdMux = portMUX_INITIALIZER_UNLOCKED;

void powerHold() {
  portENTER_CRITICAL(&holdMux);
  holds++;
  portEXIT_CRITICAL(&holdMux);
}

void powerRelease() {
  portENTER_CRITICAL(&holdMux);
  if (holds > 0) holds--;
  portEXIT_CRITICAL(&holdMux);
  powerActivity();  // the full idle time from here, not from before the work
}

void powerInit() {
  // Set each level before releasing the hold from the last sleep. A released pin floats, and
  // the pull-ups/downs would then cut battery power or the panel's power (losing its image).
  const struct { gpio_num_t pin; int level; } held[] = {
    {PIN_LATCH, HIGH}, {PIN_EPD_PWR, LOW}, {PIN_EPD_RST, HIGH}, {PIN_EPD_CS, HIGH},
  };
  for (auto &h : held) {
    pinMode(h.pin, OUTPUT);
    digitalWrite(h.pin, h.level);
    gpio_hold_dis(h.pin);
  }
  gpio_deep_sleep_hold_dis();
  sleepS = storageGetInt("sleep_s", 10);
  saver = storageGetInt("saver", 0);

  pinMode(PIN_AUDIO_PWR, OUTPUT);
  digitalWrite(PIN_AUDIO_PWR, HIGH);

  setCpuFrequencyMhz(80);
  sampleWallPower();
  lastActivity = millis();

  if (probePhase == ProbePhase::Baseline || probePhase == ProbePhase::AfterMinute) {
    if (!powerTimerWake()) {
      probePhase = ProbePhase::Idle;  // a button interrupted the timed test
    } else if (probePhase == ProbePhase::Baseline) {
      probeResult.before = batteryAdcSample();
      probePhase = ProbePhase::AfterMinute;
      deepSleep(PROBE_INTERVAL_US);
    } else {
      probeResult.after = batteryAdcSample();
      probePhase = ProbePhase::Complete;
      probeFinishedAtBoot = true;
    }
  }
}

bool powerWokeFromSleep() {
  return esp_reset_reason() == ESP_RST_DEEPSLEEP;
}

void powerActivity() {
  sleepSoon = false;
  lastActivity = millis();
}

void powerWakeAt(WakeFor who, time_t utc) { wakeAt[(int)who] = utc; }

bool powerTimerWake() {
  return powerWokeFromSleep() && !probeFinishedAtBoot && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

void powerSleepSoon() {
  sleepSoon = true;
  lastActivity = millis() - idleTimeoutMs();
}

// Microseconds of sleep until powerWakeAt's time (0: none). The sleep timer runs on the chip's own rough oscillator,
// a few percent out, so a long wait stops at 90% of the way: the clock chip, which is accurate, sets the rest.
static uint64_t untilWake() {
  time_t soonest = 0;
  for (const time_t t : wakeAt)
    if (t && (!soonest || t < soonest)) soonest = t;
  if (!soonest) return 0;
  int64_t s = soonest - time(nullptr);
  if (s < 1) s = 1;
  if (s > 120) s = s * 9 / 10;
  return (uint64_t)s * 1000000;
}

static bool wakeDueDuringProbe() {
  const uint64_t next = untilWake();
  return next && next <= PROBE_INTERVAL_US + 5000000ULL;
}

const char *powerStartBatteryProbe() {
  if (usbHostActive()) return "Use battery or wall power";
  if (holds || phoneLanBusy()) return "Busy, try again";
  if (wakeDueDuringProbe()) return "Timer or reminder due soon";
  probePhase = ProbePhase::Idle;
  probeRequested = true;
  return nullptr;
}

bool powerBatteryProbeRunning() {
  return probeRequested || probePhase == ProbePhase::Baseline || probePhase == ProbePhase::AfterMinute;
}

const BatteryProbeResult *powerBatteryProbeResult() {
  return probePhase == ProbePhase::Complete ? &probeResult : nullptr;
}

void powerSleepIfIdle() {
  if (!usbHostConnected() && millis() - powerSampleAt >= 10000) sampleWallPower();
  // Sleeping with a button held would wake straight away, in a loop.
  if (inputAnyDown() || holds || netClaimed()) return;
  if (probeRequested) {
    probeRequested = false;
    if (wakeDueDuringProbe() || phoneLanBusy() || usbHostActive()) return;
    probePhase = ProbePhase::Baseline;
    deepSleep(1000000ULL);  // take both readings at the same point in the boot sequence
    return;
  }
  if (millis() - lastActivity < idleTimeoutMs()) return;
#if UNIDEX_DEV
  return;  // test build: always awake, so the test tools never find it asleep (USB host detection can drop out)
#endif
  // The manual saver can let a suspended USB session expire, even when the
  // controller still remembers a host. The default preserves remote wake.
  if (usbHostActive() || (!saver && (usbHostConnected() || likelyWallPower))) return;
  if (phoneLanBusy() || (phoneLanKeepAwake() && !powerLanIdleExpired())) return;
  deepSleep(untilWake());
}

// Keep the panel powered in its own deep sleep (RAM retained), so the first refresh after waking can be
// partial. RST/CS stay HIGH so it isn't woken or selected. A button wakes it (and, if timerUs, the timer).
static void deepSleep(uint64_t timerUs) {
  gpio_hold_en(PIN_LATCH);
  gpio_hold_en(PIN_EPD_PWR);
  gpio_hold_en(PIN_EPD_RST);
  gpio_hold_en(PIN_EPD_CS);
  gpio_deep_sleep_hold_en();

  phoneLanStop();  // close any files opened by the panel before unmounting the card
  netOff();
  storageEnd();  // unmount the SD card cleanly (it stays powered: no switch on its supply)
  digitalWrite(PIN_AUDIO_PWR, HIGH);  // the codec's supply off: pins are held as they are through the sleep
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);  // drop powerNap's timer and GPIO sources
  esp_sleep_enable_ext1_wakeup(WAKE_MASK, ESP_EXT1_WAKEUP_ANY_LOW);
  if (timerUs) esp_sleep_enable_timer_wakeup(timerUs);
  esp_deep_sleep_start();
}

#if UNIDEX_DEV
void powerSleepFor(uint32_t ms) {  // a reminder or alarm due sooner wakes it sooner, as in use
  const uint64_t us = (uint64_t)ms * 1000, reminder = untilWake();
  deepSleep(reminder && reminder < us ? reminder : us);
}
#endif

static uint32_t wakeWithin;  // 0: nothing asked for

void powerWakeWithin(uint32_t ms) { wakeWithin = wakeWithin ? min(wakeWithin, ms) : ms; }

void powerNap() {
  // Light sleep pauses USB, so skip it while a host is connected (the Mac sync needs the port).
  // Also skip while a button is held: its release and long-press timing need polling.
  const uint32_t idle = millis() - lastActivity;
  const uint32_t idleMs = idleTimeoutMs();
#if UNIDEX_DEV
  delay(5);  // test build: no light sleep (see powerSleepIfIdle)
  return;
#endif
  // Give USB time to enumerate before the first nap.
  if (millis() < 3000 || usbHostConnected() || phoneLanKeepAwake() || inputAnyDown() || holds) {
    delay(5);
    return;
  }
  // Past the idle time and still awake: a plain charger is holding the battery up (see powerSleepIfIdle),
  // so nap in short steps rather than spin. A button still wakes it.
  uint32_t napMs = idle >= idleMs ? 250 : idleMs - idle;  // else wake for the deep-sleep check, or sooner if an app asked
  if (wakeWithin && wakeWithin < napMs) napMs = wakeWithin;
  wakeWithin = 0;
  esp_sleep_enable_timer_wakeup((uint64_t)napMs * 1000);
  gpio_wakeup_enable(GPIO_NUM_0, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_18, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();  // (the watchdog's timer stops in light sleep too)
  powerAlive();
}

int powerSleepSeconds() { return sleepS; }

bool powerSaver() { return saver || (!usbHostConnected() && !likelyWallPower); }

bool powerSaverSetting() { return saver; }

bool powerLanIdleExpired() {
  return powerSaver() && !holds && !netClaimed() &&
         (sleepSoon || millis() - lastActivity >= LAN_IDLE_MS);
}

extern TaskHandle_t loopTaskHandle;  // Arduino's main loop task (cores/esp32/main.cpp)
static const uint32_t WATCHDOG_S = 30;

void powerWatchdogBegin() {
  esp_task_wdt_init(WATCHDOG_S, true);  // already started by the system (5 s, idle task only): this sets 30 s
  enableLoopWDT();                      // the loop feeds it once per pass
}

void powerAlive() {
  if (xTaskGetCurrentTaskHandle() == loopTaskHandle) esp_task_wdt_reset();  // only the loop task is watched
}

static void drawFlat() {
  drawEmpty("Charge me", "The battery is empty.", "Plug in USB to charge.");
}

static bool flatShown;
bool powerFlatShown() { return flatShown; }

void powerOffIfFlat() {
  static uint32_t lastCheck;
  static bool checked;
  bool &shown = flatShown;
  if (shown || (checked && millis() - lastCheck < 30000)) return;
  checked = true;
  lastCheck = millis();
  if (!batteryEmpty()) return;
  delay(1000);  // a reading can dip under load (a screen refresh): only act if it's still flat a second later
  if (!batteryEmpty()) return;
  shown = true;
  displayShow(drawFlat, true);
#if UNIDEX_DEV
  return;  // test build: stays on, so the test can see the screen
#endif
  gpio_hold_dis(PIN_LATCH);
  digitalWrite(PIN_LATCH, LOW);  // battery power off (on USB the board stays powered, but then it isn't flat)
  delay(2000);
  deepSleep(0);  // still running (PWR held down): sleep until a button
}

void powerSetSleepSeconds(int s) {
  sleepS = s;
  storagePutInt("sleep_s", s);
}

void powerSetSaver(bool on) {
  saver = on;
  storagePutInt("saver", on);
}
