#include <Arduino.h>
#include "apps/apps.h"
#include "apps/notes/phone.h"
#include "core/battery_log.h"
#include "core/clock.h"
#include "core/devtools.h"
#include "core/display.h"
#include "core/input.h"
#include "core/launcher.h"
#include "core/power.h"
#include "core/storage.h"
#include "core/usb_mode.h"
#include "core/usb_serial.h"
#include "core/usbsync.h"

#ifdef SET_LOOP_TASK_STACK_SIZE
// Third-party apps run in the loop (Lua's parser and calls recurse on the C stack): 24 KB instead of 8.
SET_LOOP_TASK_STACK_SIZE(24 * 1024);
#endif

static bool usbMode;

void setup() {
  powerInit();
  usbMode = usbModeBegin();
  if (usbMode) return;
  bool woke = powerWokeFromSleep();

#if DEBUG
  // Give a connected terminal time to open the new USB port on a cold boot.
  uint32_t t0 = millis();
  while (!woke && !usbSerial() && millis() - t0 < 3000) delay(10);
#endif

  clockBegin();  // local time for every app (one I2C read)
  inputInit();
  displayInit(!woke);
  launcherBegin(woke);
  if (usbModeTakeReturnToSettings()) launcherOpen(&settingsApp);
  powerOffIfFlat();
  powerWatchdogBegin();
}

void loop() {
  usbModePoll();
  if (usbMode) return;
  if (batteryLogBusy()) {
    batteryLogPoll();
    delay(1);
    return;
  }
  Event e = inputPoll();
  if (e != Event::None) {
    powerActivity();
    if (e == Event::WakePc) {
      const char *error = usbWakePc();
      launcherToast(error ? error : "Wake signal sent");
    } else {
      launcherHandle(e);
    }
  }
  phoneLanPoll();
  usbSyncPoll();
  if (batteryLogBusy()) return;
#if UNIDEX_DEV
  devTick();
#endif
  launcherPoll();
  powerOffIfFlat();
  powerSleepIfIdle();
  powerNap();
}
