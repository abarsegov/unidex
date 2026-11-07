#include "usb.h"
#include "../../core/usb_serial.h"
#include <Arduino.h>
#include <esp_rom_crc.h>
#include <math.h>
#include "cloud.h"
#include "job.h"
#include "phone.h"
#include "play.h"
#include "store.h"
#include "../../core/audio.h"
#include "../../core/credentials.h"
#include "../../core/net.h"
#include "../../core/power.h"
#include "../../core/storage.h"

static int hexValue(char c) {
  return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}

static bool fromHex(const char *hex, String &out) {
  out = "";
  const size_t n = strlen(hex);
  if (n % 2) return false;
  for (size_t i = 0; i < n; i += 2) {
    const int hi = hexValue(hex[i]), lo = hexValue(hex[i + 1]);
    if (hi < 0 || lo < 0) return false;
    out += (char)(hi << 4 | lo);
  }
  return true;
}

static void printHex(const String &s) {
  for (size_t i = 0; i < s.length(); i++) usbSerial().printf("%02x", (uint8_t)s[i]);
}

// A note id from the page: digits, letters and dashes only, so it can't reach outside /notes.
// Notes still to transcribe or push (`notes` gets how many there are). The card must be readable.
static int waitingNotes(int &notes) {
  const bool gh = credGet("gh_on") == "1";
  int waiting = 0;
  notes = 0;
  for (const NoteInfo &n : storeList()) {
    notes++;
    waiting += !n.text || (!n.pushed && gh);
  }
  return waiting;
}

static void status() {
  // Each line printed separately: credStatus writes them straight to Serial with the "NS " prefix added here.
  struct Prefixed : Print {
    bool start = true;
    size_t write(uint8_t c) override {
      if (start) usbSerial().print("NS ");
      start = c == '\n';
      return usbSerial().write(c);
    }
  } out;
  credStatus(out);
  int notes = 0;
  const bool card = storeReady();
  const int waiting = card ? waitingNotes(notes) : 0;
  usbSerial().printf("NC %d %d %d\nOK N ?\n", card, notes, waiting);
}

// The website's one-click sync: sends waiting notes in the background, as B on the Notes screen does.
static void startSync() {
  int notes = 0;
  if (jobBusy()) usbSerial().println("OK N SYNC busy");
  else if (netClaimed()) usbSerial().println("OK N SYNC fail the phone hotspot is on");  // its radio: a send would end it
  else if (!storeReady()) usbSerial().println("OK N SYNC fail no SD card");
  else if (!credWifiAny()) usbSerial().println("OK N SYNC fail WiFi not set up");
  else if (const int waiting = waitingNotes(notes)) {
    if (jobStartSweep()) usbSerial().printf("OK N SYNC started %d\n", waiting);
    else usbSerial().println("OK N SYNC busy");
  } else {
    usbSerial().println("OK N SYNC none");
  }
}

// Records 2 s and reports how loud it was. With `radio`, WiFi is connecting meanwhile (to check it adds no hum).
static void micTest(bool radio) {
  const bool ownRadio = radio && !netClaimed();  // a note sending, or the phone hotspot: the radio is on already
  if (ownRadio) netBegin();
  if (!audioBegin()) {
    usbSerial().println("OK N MIC fail codec not answering");
    return;
  }
  static int16_t buf[512];
  int peak = 0;
  double sum = 0;
  size_t total = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    const size_t n = audioRead(buf, 512);
    if (millis() - t0 < 300) continue;  // the first moments after power-up are a click
    for (size_t i = 0; i < n; i++) {
      peak = max(peak, abs((int)buf[i]));
      sum += (double)buf[i] * buf[i];
    }
    total += n;
  }
  audioEnd();
  if (ownRadio) netRelease();
  powerActivity();
  if (!total) usbSerial().println("OK N MIC fail no samples");
  else usbSerial().printf("OK N MIC %d %d\n", peak, (int)sqrt(sum / total));
}

static void list() {
  for (const NoteInfo &n : storeList()) {
    usbSerial().printf("NF %s %u %d %d ", n.id.c_str(), (unsigned)storeReadNote(n.id).length(), n.text, n.pushed);
    printHex(n.title);
    usbSerial().println();
  }
  usbSerial().println("OK N LIST");
}

static void read(const String &id) {
  if (!storeValidId(id)) {
    usbSerial().println("ERR");
    return;
  }
  const String md = storeReadNote(id);
  if (!md.length()) {
    usbSerial().println("ERR");
    return;
  }
  for (size_t i = 0; i < md.length(); i += 64) {
    usbSerial().print("ND ");
    printHex(md.substring(i, i + 64));
    usbSerial().println();
  }
  const uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)md.c_str(), md.length());
  usbSerial().printf("OK N READ %u %lu\n", (unsigned)md.length(), (unsigned long)crc);
}

static String caChunks, caFor;  // N ADD chunks of a network's CA (wifi_ca, wifi2_ca ...) waiting for its final N SET

static bool isCaName(const char *name) {
  const size_t n = strlen(name);
  return !strncmp(name, "wifi", 4) && n > 3 && !strcmp(name + n - 3, "_ca") && credKnown(name);
}

bool notesUsb(const char *l) {
  if (strncmp(l, "N ", 2) != 0) return false;
  char cmd[8] = "", arg[24] = "";
  const int n = sscanf(l + 2, "%7s %23s", cmd, arg);
  if (n < 1) return false;
  if (strcmp(cmd, "?") == 0) {
    status();
  } else if (strcmp(cmd, "SET") == 0) {
    String value;
    const char *hex = n == 2 ? strchr(l + 6, ' ') : nullptr;  // after "N SET <name>"
    const bool parsed = n == 2 && credKnown(arg) && fromHex(hex ? hex + 1 : "", value);
    if (isCaName(arg) && caFor == arg) value = caChunks + value;  // the CA is saved once, with its last chunk
    caChunks = "";
    caFor = "";
    if (parsed && credSet(arg, value))
      usbSerial().printf("OK N SET %s\n", arg);
    else
      usbSerial().println("ERR");
  } else if (strcmp(cmd, "ADD") == 0) {  // a CA chunk, kept in RAM until the N SET for the same name
    String value;
    const char *hex = n == 2 ? strchr(l + 6, ' ') : nullptr;
    if (n == 2 && isCaName(arg) && hex && fromHex(hex + 1, value) && (!caFor.length() || caFor == arg) &&
        caChunks.length() + value.length() <= 3000) {
      caFor = arg;
      caChunks += value;
      usbSerial().printf("OK N ADD %s\n", arg);
    } else {
      caChunks = "";
      caFor = "";
      usbSerial().println("ERR");
    }
  } else if (strcmp(cmd, "CLR") == 0) {
    if (jobBusy()) {  // the note being sent still needs its keys
      usbSerial().println("ERR busy");
      return true;
    }
    if (strcmp(arg, "all") == 0) credClearAll();
    else if (credKnown(arg)) credClear(arg);
    usbSerial().println("OK N CLR");
  } else if (strcmp(cmd, "TEST") == 0) {
    String detail;
    const char *err = cloudTest(arg, detail);
    if (err) usbSerial().printf("OK N TEST %s fail %s\n", arg, err);
    else if (detail.length()) usbSerial().printf("OK N TEST %s ok %s\n", arg, detail.c_str());
    else usbSerial().printf("OK N TEST %s ok\n", arg);
    powerActivity();
  } else if (strcmp(cmd, "MIC") == 0) {
    micTest(strcmp(arg, "wifi") == 0);
  } else if (strcmp(cmd, "PLAY") == 0) {
    const char *err = playNote(arg);
    if (err) usbSerial().printf("OK N PLAY fail %s\n", err);
    else usbSerial().println("OK N PLAY");
  } else if (strcmp(cmd, "LIST") == 0) {
    list();
  } else if (strcmp(cmd, "READ") == 0) {
    read(arg);
  } else if (strcmp(cmd, "DEL") == 0) {
    if (jobBusy() && strcmp(arg, jobStatus().noteId) == 0) {
      usbSerial().println("ERR busy");
      return true;
    }
    if (storeValidId(arg)) storeDelete(arg);
    usbSerial().println("OK N DEL");
#if UNIDEX_DEV
  } else if (strcmp(cmd, "UNSEND") == 0) {  // test build: the note is a waiting recording again (its .wav stays)
    if (jobBusy() || !storeValidId(arg) || !storageExists(storeWavPath(arg).c_str())) {
      usbSerial().println("ERR");
      return true;
    }
    storeUnsend(arg);
    usbSerial().println("OK N UNSEND");
#endif
  } else if (strcmp(cmd, "SYNC") == 0) {
    startSync();
  } else if (strcmp(cmd, "JOB") == 0) {
    const JobStatus js = jobStatus();
    usbSerial().printf("OK N JOB %d %d %lu ", jobBusy(), (int)js.step, (unsigned long)js.gen);
    printHex(String(js.result));
    usbSerial().print(' ');
    printHex(String(js.noteId));
    usbSerial().printf(" %u\n", (unsigned)jobStackLeft());
  } else {
    return false;
  }
  return true;
}
