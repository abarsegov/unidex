#include "firmware_update.h"

#if UNIDEX_EMU
FirmwareUpdateInfo firmwareUpdateInfo() { return {}; }
const char *firmwareUpdateBegin(size_t) { return "Firmware updates are unavailable."; }
size_t firmwareUpdateRemaining() { return 0; }
const char *firmwareUpdateWrite(const uint8_t *, size_t) { return "Firmware updates are unavailable."; }
const char *firmwareUpdateFinish(uint32_t &) { return "Firmware updates are unavailable."; }
void firmwareUpdateAbort() {}
void firmwareUpdateConfirm() {}
#else
#include <algorithm>
#include <string.h>
#include <esp_app_format.h>
#include <esp_image_format.h>
#include <esp_ota_ops.h>

static const esp_partition_t *target;
static esp_ota_handle_t handle;
static bool writing;
static size_t imageBytes, remaining, prefixUsed;
static constexpr size_t APP_DESC_OFFSET = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
static uint8_t prefix[APP_DESC_OFFSET + sizeof(uint32_t)];

static const esp_partition_t *updateSlot(const esp_partition_t *running) {
  if (!running || !esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, nullptr))
    return nullptr;
  const esp_partition_t *next = esp_ota_get_next_update_partition(running);
  return next && next->address != running->address ? next : nullptr;
}

FirmwareUpdateInfo firmwareUpdateInfo() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  const esp_partition_t *next = updateSlot(running);
  return {running ? running->address : 0, next ? next->address : 0, next ? next->size : 0};
}

const char *firmwareUpdateBegin(size_t bytes) {
  if (target) return "A firmware update is already running.";
  const esp_partition_t *next = updateSlot(esp_ota_get_running_partition());
  if (!next) return "No inactive firmware slot is available.";
  if (bytes < APP_DESC_OFFSET + sizeof(esp_app_desc_t) || bytes > next->size)
    return "Choose an ESP32-S3 application firmware.bin that fits the firmware slot.";
  target = next;
  imageBytes = remaining = bytes;
  prefixUsed = 0;
  return nullptr;
}

size_t firmwareUpdateRemaining() { return remaining; }

const char *firmwareUpdateWrite(const uint8_t *data, size_t bytes) {
  if (!target || bytes > remaining) return "Unexpected firmware length.";
  if (!bytes) return nullptr;
  remaining -= bytes;
  if (!writing) {
    const size_t count = std::min(bytes, sizeof prefix - prefixUsed);
    memcpy(prefix + prefixUsed, data, count);
    prefixUsed += count;
    if (prefixUsed < sizeof prefix) return nullptr;
    data += count;
    bytes -= count;

    esp_image_header_t image;
    uint32_t appMagic;
    memcpy(&image, prefix, sizeof image);
    memcpy(&appMagic, prefix + APP_DESC_OFFSET, sizeof appMagic);
    if (image.magic != ESP_IMAGE_HEADER_MAGIC || image.chip_id != ESP_CHIP_ID_ESP32S3 ||
        appMagic != ESP_APP_DESC_MAGIC_WORD)
      return "Choose the ESP32-S3 application firmware.bin. This file is not an application image.";

    // The exact HTTP length is enforced above. Sequential writes erase sectors as needed.
    const esp_err_t result = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (result == ESP_ERR_OTA_ROLLBACK_INVALID_STATE) return "The current firmware is still pending verification.";
    if (result != ESP_OK) return "Couldn't open the inactive firmware slot.";
    writing = true;
    if (esp_ota_write(handle, prefix, sizeof prefix) != ESP_OK) return "Couldn't write the firmware.";
  }
  return bytes && esp_ota_write(handle, data, bytes) != ESP_OK ? "Couldn't write the firmware." : nullptr;
}

const char *firmwareUpdateFinish(uint32_t &slot) {
  if (!target || !writing || remaining) return "Firmware upload is incomplete.";
  const esp_err_t result = esp_ota_end(handle);  // validates the full image and releases the handle even on error
  writing = false;
  if (result != ESP_OK) return "Firmware verification failed. Choose a complete application image.";
  // esp_ota_end checks the slot, which can still contain a tail from an older upload.
  const esp_partition_pos_t uploaded{target->address, static_cast<uint32_t>(imageBytes)};
  esp_image_metadata_t metadata;
  if (esp_image_get_metadata(&uploaded, &metadata) != ESP_OK || metadata.image_len > imageBytes)
    return "Firmware verification failed. Choose a complete application image.";
  if (esp_ota_set_boot_partition(target) != ESP_OK) return "Couldn't select the new firmware for boot.";
  slot = target->address;
  return nullptr;
}

void firmwareUpdateAbort() {
  if (writing) esp_ota_abort(handle);
  writing = false;
  target = nullptr;
  remaining = 0;
}

void firmwareUpdateConfirm() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (running && esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY)
    esp_ota_mark_app_valid_cancel_rollback();
}
#endif
