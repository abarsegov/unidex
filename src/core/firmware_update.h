#pragma once
#include <stddef.h>
#include <stdint.h>

struct FirmwareUpdateInfo {
  uint32_t activeSlot, targetSlot;
  size_t maxBytes;
};

FirmwareUpdateInfo firmwareUpdateInfo();
const char *firmwareUpdateBegin(size_t bytes);
size_t firmwareUpdateRemaining();
const char *firmwareUpdateWrite(const uint8_t *data, size_t bytes);
const char *firmwareUpdateFinish(uint32_t &slot);
void firmwareUpdateAbort();
void firmwareUpdateConfirm();  // call after the LAN panel is listening on the new firmware
