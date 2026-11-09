#pragma once

bool batteryLogUsb(const char *line);  // commands from usbSyncPoll
bool batteryLogBusy();
bool batteryLogSending();
void batteryLogPoll();
