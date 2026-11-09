#include "battery_log.h"

#if UNIDEX_EMU
bool batteryLogUsb(const char *) { return false; }
bool batteryLogBusy() { return false; }
bool batteryLogSending() { return false; }
void batteryLogPoll() {}
bool batteryLoadActive() { return false; }
bool batteryLoadAuthorized(const char *) { return false; }
String batteryLoadStatus() { return "{}"; }
fs::File batteryLoadOpenLog() { return {}; }
#else
#include <Arduino.h>
#include <ctype.h>
#include <esp_wifi.h>
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
constexpr char LOAD_HEADER[] = "elapsed_ms,millivolts,estimated_percent,cpu_mhz,wifi_ps_none,refresh_ms\n";
constexpr char SEND_END[] = "OK BATLOG END\n";
constexpr uint32_t LOAD_SAMPLE_MS = 1000;
constexpr uint32_t LOAD_CPU_MHZ = 240;
constexpr size_t LOAD_TOKEN_LENGTH = 32;

enum class Phase { Idle, Pc, WaitWall, Wall, WaitBattery, Battery, Done, Stopped, WaitLoad, Load };
constexpr const char *PHASE_NAMES[] = {
  "idle", "pc", "wait_wall", "wall", "wait_battery", "battery", "done", "stopped", "wait_load", "load"
};
Phase phase;
char runId[MAX_ID + 1];
uint32_t phaseAt, durationMs, nextSampleMs;
int lowSamples;
fs::File logFile, sendFile;
char pending[4096];
size_t pendingBytes, sendRemaining;
uint32_t sendLastProgress;
char loadToken[LOAD_TOKEN_LENGTH + 1];
char loadSample[100];
bool loadSaved, loadConfigured, originalInvert;
uint32_t originalCpuMhz;
wifi_ps_type_t originalWifiPs;

const char *phaseName() { return PHASE_NAMES[static_cast<size_t>(phase)]; }

void reportPhase() {
  if (usbSerial().availableForWrite() >= 32) usbSerial().printf("BATLOG STATE %s\n", phaseName());
}

void drawLog() {
  switch (phase) {
    case Phase::Pc: drawEmpty("Battery log", "Recording on computer", "Hold PWR to stop"); break;
    case Phase::WaitWall: drawEmpty("Battery log", "Connect wall charger", "Then tap PWR"); break;
    case Phase::Wall: drawEmpty("Battery log", "Recording on wall charger", "Hold PWR to stop"); break;
    case Phase::WaitBattery: drawEmpty("Battery log", "Unplug USB", "Then tap PWR"); break;
    case Phase::Battery: drawEmpty("Battery log", "Recording on battery", "Hold PWR to stop"); break;
    case Phase::WaitLoad: drawEmpty("Battery load test", "Unplug USB", "Then tap PWR"); break;
    case Phase::Load: drawEmpty("Battery load test", "WiFi and screen active", "Hold PWR to stop"); break;
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
  const bool wasLoad = batteryLoadActive();
  const bool saved = savePending();
  logFile.close();
  if (wasLoad) {
    loadSaved = saved && loadSample[0];
    if (loadConfigured) {
      display.inverted = originalInvert;
      setCpuFrequencyMhz(originalCpuMhz);
      esp_wifi_set_ps(originalWifiPs);
      loadConfigured = false;
    }
    powerRelease();
  }
  phase = saved ? next : Phase::Stopped;
  reportPhase();
  powerActivity();
  launcherToast(saved ? message : "Battery log write failed");
  launcherOpen(&settingsApp);
  return saved;
}

bool appendRow(const char *line, size_t length) {
  if (pendingBytes + length > sizeof pending && !savePending()) {
    finish(Phase::Stopped, "Battery log write failed");
    return false;
  }
  memcpy(pending + pendingBytes, line, length);
  pendingBytes += length;
  return true;
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
  if (!appendRow(line + prefix, length)) return;
  if (phase == Phase::Pc && usbSerial().availableForWrite() >= (int)(prefix + length))
    usbSerial().write(reinterpret_cast<const uint8_t *>(line), prefix + length);
  const bool flat = battery::level(adc.millivolts, batteryCharging()) == battery::Level::EMPTY;
  lowSamples = flat ? lowSamples + 1 : 0;
  if (lowSamples >= 1000 / (int)SAMPLE_MS) finish(Phase::Stopped, "Battery too low for log");
}

void beginLoad() {
  originalInvert = display.inverted;
  originalCpuMhz = getCpuFrequencyMhz();
  if (esp_wifi_get_ps(&originalWifiPs) != ESP_OK) {
    finish(Phase::Stopped, "WiFi unavailable");
    return;
  }
  loadConfigured = true;
  if (!setCpuFrequencyMhz(LOAD_CPU_MHZ) || esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    finish(Phase::Stopped, "Battery load setup failed");
    return;
  }
  beginPhase(Phase::Load);
}

void sampleLoad(uint32_t elapsed) {
  nextSampleMs = (elapsed / LOAD_SAMPLE_MS + 1) * LOAD_SAMPLE_MS;
  if (getCpuFrequencyMhz() != LOAD_CPU_MHZ) setCpuFrequencyMhz(LOAD_CPU_MHZ);
  wifi_ps_type_t ps;
  bool awake = esp_wifi_get_ps(&ps) == ESP_OK;
  if (awake && ps != WIFI_PS_NONE) awake = esp_wifi_set_ps(WIFI_PS_NONE) == ESP_OK;
  display.inverted = !display.inverted;
  displayTick(drawLog);  // partial refresh with panel sleep between inversions
  const int mv = batteryMillivolts();
  const int pct = mv < battery::NO_BATTERY_MV ? -1 : battery::fromCurve(mv);
  const int length = snprintf(loadSample, sizeof loadSample, "%lu,%d,%d,%lu,%d,%lu\n",
                              (unsigned long)elapsed, mv, pct, (unsigned long)getCpuFrequencyMhz(),
                              awake, (unsigned long)displayLastRefresh().ms);
  if (length <= 0 || length >= (int)sizeof loadSample) {
    loadSample[0] = '\0';
    finish(Phase::Stopped, "Battery load sample failed");
    return;
  }
  if (!appendRow(loadSample, length)) {
    loadSaved = false;
    return;
  }
  loadSample[length - 1] = '\0';
  lowSamples = battery::level(mv, false) == battery::Level::OK ? 0 : lowSamples + 1;
  if (lowSamples >= 3) finish(Phase::Stopped, "Battery too low for load");
}

void recordPoll() {
  const Event event = inputPoll();
  if (event == Event::BLong) {
    finish(Phase::Stopped, "Battery log stopped");
    return;
  }
  const uint32_t elapsed = millis() - phaseAt;
  if (phase == Phase::WaitLoad) {
    if (event == Event::BShort) beginLoad();
    else if (elapsed >= SWITCH_TIMEOUT_MS) finish(Phase::Stopped, "Battery load timed out");
    return;
  }
  if (phase == Phase::WaitWall || phase == Phase::WaitBattery) {
    // The button confirms the source. The controller can retain its last host state after unplugging.
    if (event == Event::BShort)
      beginPhase(phase == Phase::WaitWall ? Phase::Wall : Phase::Battery);
    else if (elapsed >= SWITCH_TIMEOUT_MS)
      finish(Phase::Stopped, "Battery log timed out");
    return;
  }
  if (elapsed < durationMs) {
    if (elapsed >= nextSampleMs) {
      if (phase == Phase::Load) sampleLoad(elapsed);
      else sample(elapsed);
    }
    return;
  }
  if (phase == Phase::Battery || phase == Phase::Load) {
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

void start(const char *args, const char *command, Phase initial) {
  if (logFile) return error("busy");
  const char *separator = strchr(args, ' ');
  if (!separator || !validId(args, separator - args)) return error("invalid_id");
  const char *secondsText = separator + 1;
  char *end;
  const unsigned long seconds = strtoul(secondsText, &end, 10);
  const bool isLoad = initial == Phase::WaitLoad;
  if (*secondsText < '0' || *secondsText > '9' || (!isLoad && *end) || seconds < 5 || seconds > 1800)
    return error("invalid_seconds");
  if (isLoad) {
    if (*end != ' ' || strlen(end + 1) != LOAD_TOKEN_LENGTH) return error("invalid_token");
    for (const char *c = end + 1; *c; ++c)
      if (!isxdigit(static_cast<unsigned char>(*c))) return error("invalid_token");
    if (!*phoneLanUrl()) return error("wifi_not_connected");
  }
  if (phoneOn() || phoneLanBusy() || netClaimed() || jobBusy() || powerBatteryProbeRunning()) return error("busy");
  char id[MAX_ID + 1];
  memcpy(id, args, separator - args);
  id[separator - args] = '\0';
  const String path = logPath(id);
  if (storageExists(path.c_str())) return error("id_exists");
  logFile = storageOpen(path.c_str(), "w");
  if (!logFile) return error("sd_open_failed");
  const char *header = isLoad ? LOAD_HEADER : CSV_HEADER;
  if (logFile.print(header) != strlen(header)) {
    logFile.close();
    return error("sd_write_failed");
  }
  logFile.flush();
  strlcpy(runId, id, sizeof runId);
  pendingBytes = 0;
  durationMs = seconds * 1000;
  loadSaved = false;
  loadSample[0] = '\0';
  loadToken[0] = '\0';
  if (isLoad) {
    strlcpy(loadToken, end + 1, sizeof loadToken);
    powerHold();
  } else {
    phoneLanStop();
    netOff();
  }
  usbSerial().printf("OK BATLOG %s %s\n", command, runId);
  beginPhase(initial);
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

bool batteryLoadActive() { return phase == Phase::WaitLoad || phase == Phase::Load; }

bool batteryLoadAuthorized(const char *authorization) {
  return loadToken[0] && strncmp(authorization, "Bearer ", 7) == 0 &&
         strcmp(authorization + 7, loadToken) == 0;
}

String batteryLoadStatus() {
  if (!loadToken[0]) return "{\"state\":\"idle\"}";
  char json[240];
  snprintf(json, sizeof json, "{\"id\":\"%s\",\"state\":\"%s\",\"duration_ms\":%lu,\"sample\":[%s],\"saved\":%s}",
           runId, phaseName(), (unsigned long)durationMs, loadSample, loadSaved ? "true" : "false");
  return json;
}

fs::File batteryLoadOpenLog() {
  if (!loadToken[0] || !loadSaved || logFile) return {};
  return storageOpen(logPath(runId).c_str());
}

void batteryLogPoll() {
  if (sendFile) {
    sendPoll();
  } else if (logFile) {
    if (batteryLoadActive()) phoneLanPoll();
    usbSyncPoll();
    if (logFile) recordPoll();
  }
  powerAlive();
}

bool batteryLogUsb(const char *line) {
  if (strncmp(line, "BATLOG ", 7) != 0) return false;
  const char *cmd = line + 7;
  if (strncmp(cmd, "START ", 6) == 0) {
    start(cmd + 6, "START", Phase::Pc);
  } else if (strncmp(cmd, "ARM ", 4) == 0) {
    start(cmd + 4, "ARM", Phase::WaitWall);
  } else if (strncmp(cmd, "BATTERY ", 8) == 0) {
    start(cmd + 8, "BATTERY", Phase::WaitBattery);
  } else if (strncmp(cmd, "LOAD ", 5) == 0) {
    start(cmd + 5, "LOAD", Phase::WaitLoad);
  } else if (strcmp(cmd, "STATUS") == 0) {
    const uint32_t elapsed = logFile ? millis() - phaseAt : 0;
    usbSerial().printf("OK BATLOG STATUS %s %s %lu\n", phaseName(), *runId ? runId : "-", (unsigned long)elapsed);
  } else if (strcmp(cmd, "POWER") == 0) {
    const bool charging = batteryCharging(), frames = usbHostTrafficRecent();
    usbSerial().printf("OK BATLOG POWER charging=%d usb_frames=%d mv=%d\n", charging, frames, batteryMillivolts());
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
