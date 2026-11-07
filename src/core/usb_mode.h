#pragma once

enum class UsbMode { Reader, Wake };

bool usbModeAvailable();
// Call only with no background card users. Unmounts, then requests a one-shot USB boot.
void usbModeRequest(UsbMode mode);
// After powerInit, before Serial/launcher/storage initialization. True owns this entire boot.
bool usbModeBegin();
void usbModePoll();
// Consume the one-shot return destination after the normal launcher starts.
bool usbModeTakeReturnToSettings();
