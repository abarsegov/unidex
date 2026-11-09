#pragma once
#include <FS.h>

bool batteryLogUsb(const char *line);  // commands from usbSyncPoll
bool batteryLogBusy();
bool batteryLogSending();
void batteryLogPoll();
bool batteryLoadActive();
bool batteryLoadAuthorized(const char *authorization);
String batteryLoadStatus();
fs::File batteryLoadOpenLog();
