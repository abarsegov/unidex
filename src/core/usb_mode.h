#pragma once

bool usbModeAvailable();
// Call only with no background card users. Unmounts, then requests a one-shot reader boot.
void usbReaderRequest();
// Start the normal USB interfaces or the reader. A reader boot owns the entire loop.
bool usbModeBegin();
void usbModePoll();
// Consume the one-shot return destination after the normal launcher starts.
bool usbModeTakeReturnToSettings();
bool usbHostConnected();
bool usbHostActive();  // USB is configured and the host is running the bus
// Main-loop calls. Return a message on failure, or nullptr after sending the signal.
const char *usbWakePc();
const char *usbSleepPc();
