#include "battery_log.h"

#if UNIDEX_EMU
bool batteryLogUsb(const char *) { return false; }
bool batteryLogBusy() { return false; }
bool batteryLogSending() { return false; }
void batteryLogPoll() {}
#else
#include <Arduino.h>
#include <ctype.h>
#include "battery.h"
#include "battery_logic.h"
#include "display.h"
#include "input.h"
#include "launcher.h"
#include "net.h"
#include "power.h"
#include "storage.h"
#include "theme.h"
#include "usb_mode.h"
#include "usb_serial.h"
#include "usbsync.h"
#include "../apps/apps.h"
#include "../apps/notes/job.h"
#include "../apps/notes/phone.h"

namespace {
constexpr uint32_t SAMPLE_MS = 100;
constexpr uint32_t SWITCH_TIMEOUT_MS = 20 * 60 * 1000;
constexpr uint32_t SEND_TIMEOUT_MS = 5000;
constexpr size_t MAX_ID = 24;
constexpr char CSV_HEADER[] = "source,elapsed_ms,millivolts,raw_mean,raw_span,usb_host,usb_active\n";
constexpr char SEND_END[] = "OK BATLOG END\n";

enum class Phase { Idle, Pc, WaitWall, Wall, WaitBattery, Battery, Done, Stopped };
constexpr const char *PHASE_NAMES[] = {"idle", "pc", "wait_wall", "wall", "wait_battery", "battery", "done", "stopped"};
Phase phase;
char runId[MAX_ID + 1];
uint32_t phaseAt, durationMs, nextSampleMs;
int lowSamples;
fs::File logFile, sendFile;
char pending[4096];
size_t pendingBytes, sendRemaining;
uint32_t sendLastProgress;

const char *phaseName() { return PHASE_NAMES[static_cast<size_t>(phase)]; }

void reportPhase() {
  if (usbSerial().availableForWrite() >= 32) usbSerial().printf("BATLOG STATE %s\n", phaseName());
}

void drawLog() {
  switch (phase) {
    case Phase::Pc: drawEmpty("Battery log", "Recording on computer", "Hold PWR to stop"); break;
    case Phase::WaitWall: drawEmpty("Battery log", "Connect wall charger", "Then tap PWR"); break;
    case Phase::Wall: drawEmpty("Battery log", "Recording on wall charger", "Hold PWR to stop"); break;
    case Phase::WaitBattery: drawEmpty("Battery log", "Unplug the charger", "Then tap PWR"); break;
    case Phase::Battery: drawEmpty("Battery log", "Recording on battery", "Hold PWR to stop"); break;
    default: break;
  }
}

void beginPhase(Phase next) {
  phase = next;
  displayShow(drawLog, true);
  phaseAt = millis();
  nextSampleMs = 0;
  lowSamples = 0;
  reportPhase();
}

bool savePending() {
  if (!pendingBytes) return true;
  const bool ok = logFile.write(reinterpret_cast<const uint8_t *>(pending), pendingBytes) == pendingBytes;
  pendingBytes = 0;
  logFile.flush();
  return ok;
}

bool finish(Phase next, const char *message) {
  const bool saved = savePending();
  logFile.close();
  phase = saved ? next : Phase::Stopped;
  reportPhase();
  powerActivity();
  launcherToast(saved ? message : "Battery log write failed");
  launcherOpen(&settingsApp);
  return saved;
}

void sample(uint32_t elapsed) {
  nextSampleMs = (elapsed / SAMPLE_MS + 1) * SAMPLE_MS;
  const BatteryAdcSample adc = batteryAdcSample();
  const bool host = usbHostConnected(), active = usbHostActive();
  char line[96] = "BATLOG ROW ";
  constexpr size_t prefix = sizeof "BATLOG ROW " - 1;
  const int length = snprintf(line + prefix, sizeof line - prefix, "%s,%lu,%d,%d,%d,%d,%d\n", phaseName(),
                              (unsigned long)elapsed, adc.millivolts, adc.rawMean, adc.rawSpan, host, active);
  if (length <= 0 || length >= (int)(sizeof line - prefix)) {
    finish(Phase::Stopped, "Battery log sample failed");
    return;
  }
  if (pendingBytes + length > sizeof pending && !savePending()) {
    finish(Phase::Stopped, "Battery log write failed");
    return;
  }
  memcpy(pending + pendingBytes, line + prefix, length);
  pendingBytes += length;
  if (phase == Phase::Pc && usbSerial().availableForWrite() >= (int)(prefix + length))
    usbSerial().write(reinterpret_cast<const uint8_t *>(line), prefix + length);
  const bool flat = battery::level(adc.millivolts, active) == battery::Level::EMPTY;
  lowSamples = flat ? lowSamples + 1 : 0;
  if (lowSamples >= 1000 / (int)SAMPLE_MS) finish(Phase::Stopped, "Battery too low for log");
}

void recordPoll() {
  const Event event = inputPoll();
  if (event == Event::BLong) {
    finish(Phase::Stopped, "Battery log stopped");
    return;
  }
  const uint32_t elapsed = millis() - phaseAt;
  if (phase == Phase::WaitWall || phase == Phase::WaitBattery) {
    if (event == Event::BShort && !usbHostActive())
      beginPhase(phase == Phase::WaitWall ? Phase::Wall : Phase::Battery);
    else if (elapsed >= SWITCH_TIMEOUT_MS)
      finish(Phase::Stopped, "Battery log timed out");
    return;
  }
  if (elapsed < durationMs) {
    if (elapsed >= nextSampleMs) sample(elapsed);
    return;
  }
  if (phase == Phase::Battery) {
    finish(Phase::Done, "Battery log saved");
  } else if (savePending()) {
    beginPhase(phase == Phase::Pc ? Phase::WaitWall : Phase::WaitBattery);
  } else {
    finish(Phase::Stopped, "Battery log write failed");
  }
}

bool validId(const char *id, size_t length) {
  if (!length || length > MAX_ID) return false;
  for (size_t i = 0; i < length; ++i) {
    const unsigned char c = id[i];
    if (c >= 128 || (!isalnum(c) && c != '-' && c != '_')) return false;
  }
  return true;
}

String logPath(const char *id) { return String("/battery-logs/") + id + ".csv"; }

void error(const char *reason) { usbSerial().printf("ERR BATLOG %s\n", reason); }

void start(const char *args) {
  if (logFile) return error("busy");
  const char *separator = strchr(args, ' ');
  if (!separator || !validId(args, separator - args)) return error("invalid_id");
  const char *secondsText = separator + 1;
  char *end;
  const unsigned long seconds = strtoul(secondsText, &end, 10);
  if (*secondsText < '0' || *secondsText > '9' || *end || seconds < 5 || seconds > 1800)
    return error("invalid_seconds");
  if (phoneOn() || phoneLanBusy() || netClaimed() || jobBusy() || powerBatteryProbeRunning()) return error("busy");
  char id[MAX_ID + 1];
  memcpy(id, args, separator - args);
  id[separator - args] = '\0';
  const String path = logPath(id);
  if (storageExists(path.c_str())) return error("id_exists");
  logFile = storageOpen(path.c_str(), "w");
  if (!logFile) return error("sd_open_failed");
  if (logFile.print(CSV_HEADER) != sizeof CSV_HEADER - 1) {
    logFile.close();
    return error("sd_write_failed");
  }
  logFile.flush();
  strlcpy(runId, id, sizeof runId);
  pendingBytes = 0;
  durationMs = seconds * 1000;
  phoneLanStop();
  netOff();
  usbSerial().printf("OK BATLOG START %s\n", runId);
  beginPhase(Phase::Pc);
}

void startSend(const char *id) {
  if (logFile) return error("busy");
  if (!validId(id, strlen(id))) return error("invalid_id");
  sendFile = storageOpen(logPath(id).c_str());
  if (!sendFile) return error("log_not_found");
  sendRemaining = sendFile.size();
  sendLastProgress = millis();
  usbSerial().printf("OK BATLOG READ %lu\n", (unsigned long)sendRemaining);
}

void sendPoll() {
  if (millis() - sendLastProgress >= SEND_TIMEOUT_MS) {
    sendFile.close();
    return;
  }
  const int space = usbSerial().availableForWrite();
  if (space <= 0) return;
  if (!sendRemaining) {
    if (space < (int)(sizeof SEND_END - 1)) return;
    usbSerial().write(reinterpret_cast<const uint8_t *>(SEND_END), sizeof SEND_END - 1);
    sendFile.close();
    powerActivity();
    return;
  }
  uint8_t chunk[512];
  const size_t count = min(sendRemaining, min((size_t)space, sizeof chunk));
  if (sendFile.read(chunk, count) != count || usbSerial().write(chunk, count) != count) {
    sendFile.close();
    return;
  }
  sendRemaining -= count;
  sendLastProgress = millis();
}
}

bool batteryLogBusy() { return logFile || sendFile; }
bool batteryLogSending() { return bool(sendFile); }

void batteryLogPoll() {
  if (sendFile) {
    sendPoll();
  } else if (logFile) {
    usbSyncPoll();
    if (logFile) recordPoll();
  }
  powerAlive();
}

bool batteryLogUsb(const char *line) {
  if (strncmp(line, "BATLOG ", 7) != 0) return false;
  const char *cmd = line + 7;
  if (strncmp(cmd, "START ", 6) == 0) {
    start(cmd + 6);
  } else if (strcmp(cmd, "STATUS") == 0) {
    const uint32_t elapsed = logFile ? millis() - phaseAt : 0;
    usbSerial().printf("OK BATLOG STATUS %s %s %lu\n", phaseName(), *runId ? runId : "-", (unsigned long)elapsed);
  } else if (strcmp(cmd, "STOP") == 0) {
    if (!logFile) error("no_run");
    else if (finish(Phase::Stopped, "Battery log stopped")) usbSerial().println("OK BATLOG STOP");
    else error("sd_write_failed");
  } else if (strncmp(cmd, "READ ", 5) == 0) {
    startSend(cmd + 5);
  } else {
    error("unknown_command");
  }
  return true;
}
#endif
