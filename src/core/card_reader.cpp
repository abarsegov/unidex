#include "card_reader.h"

#if UNIDEX_EMU
bool cardReaderAvailable() { return false; }
void cardReaderRequest() {}
bool cardReaderBegin() { return false; }
void cardReaderPoll() {}
bool cardReaderTakeReturnToSettings() { return false; }
#else
#include <Arduino.h>
#include <USB.h>
#include <atomic>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include <esp32-hal-tinyusb.h>
#include <soc/rtc_cntl_reg.h>
#include "display.h"
#include "launcher.h"
#include "storage.h"
#include "theme.h"

// Keep normal firmware's HWCDC/Web Serial. TinyUSB takes the internal PHY only
// in a separate boot, before HWCDC or any filesystem/application can start.
static constexpr uint32_t READER_MAGIC = 0x554D5343;
static constexpr uint32_t RETURN_MAGIC = 0x554D5254;
static constexpr uint32_t EXIT_HOLD_MS = 2000, EJECT_GRACE_MS = 1000;
RTC_NOINIT_ATTR static uint32_t requested;
static sdmmc_card_t card;
static DMA_ATTR uint8_t sector[512];
static std::atomic<bool> present{false}, ejected{false}, ioError{false};
static std::atomic<uint32_t> activeOperations{0}, ejectedAt{0};
static bool failed, preventRemoval, usbStarted;
static bool buttonDown, buttonRaw, exitArmed, returnToSettings;
static uint32_t buttonAt, pressedAt;

// Increment BEFORE checking the admission gate. After the loop closes it, any
// later callback can only reject the operation; earlier raw operations drain.
// Include SYNCHRONIZE CACHE's card status command in the same accounting.
struct CardOperation {
  CardOperation() { activeOperations.fetch_add(1); }
  ~CardOperation() { activeOperations.fetch_sub(1); }
};

bool cardReaderAvailable() { return true; }

bool cardReaderTakeReturnToSettings() {
  const bool returning = returnToSettings;
  returnToSettings = false;
  return returning;
}

void cardReaderRequest() {
  storageEnd();
  requested = READER_MAGIC;
  systemRestart("USB card reader");
}

static void drawReader() {
  display.fillScreen(WHITE);
  display.setTextColor(BLACK);
  drawHeader("USB card reader");
  display.setFont(FONT_SMALL);
  if (failed) {
    drawCentered("Reader unavailable", 60);
    drawCentered("Check microSD card", 86);
    drawCentered("PWR: return", 140);
  } else if (ejected.load()) {
    drawCentered("Safely ejected", 70);
    drawCentered("Returning to Settings", 115);
  } else if (exitArmed) {
    drawCentered("USB ejected/unplugged?", 65);
    drawCentered("Release PWR to return", 100);
  } else {
    drawCentered(ioError.load() ? "Card I/O error" : "microSD on computer", 55);
    drawCentered("Eject drive on PC", 85);
    drawCentered("Unplug USB, then", 110);
    drawCentered("hold PWR 2s + release", 135);
    drawCentered("to return to Settings", 155);
  }
}

static int32_t transfer(uint8_t lun, uint32_t lba, uint32_t offset,
                        void *buffer, uint32_t bytes, bool write) {
  CardOperation operation;
  if (lun || !present.load()) {
    tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0);
    return -1;
  }
  const uint64_t start = uint64_t(lba) * 512 + offset;
  const uint64_t capacity = uint64_t(card.csd.capacity) * 512;
  if (start > capacity || bytes > capacity - start || bytes > INT32_MAX) {
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x21, 0);
    return -1;
  }
  uint8_t *p = static_cast<uint8_t *>(buffer);
  uint64_t at = start;
  uint32_t left = bytes;
  // TinyUSB invokes MSC callbacks serially in its task. Only that task touches
  // raw media; the loop observes atomics and never mounts/deinitializes it.
  while (left) {
    const size_t inSector = at % 512;
    const size_t n = min<size_t>(left, 512 - inSector);
    const size_t block = at / 512;
    esp_err_t err = ESP_OK;
    if (!write || inSector || n != 512) err = sdmmc_read_sectors(&card, sector, block, 1);
    if (err == ESP_OK) {
      if (write) {
        memcpy(sector + inSector, p, n);
        err = sdmmc_write_sectors(&card, sector, block, 1);
      } else memcpy(p, sector + inSector, n);
    }
    if (err != ESP_OK) {
      ioError.store(true);
      tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, write ? 0x0c : 0x11, 0);
      return -1;
    }
    p += n;
    at += n;
    left -= n;
  }
  return bytes;
}

extern "C" {
uint8_t tud_msc_get_maxlun_cb() { return 0; }
void tud_msc_inquiry_cb(uint8_t, uint8_t vendor[8], uint8_t product[16], uint8_t revision[4]) {
  memcpy(vendor, "Unidex  ", 8);
  memcpy(product, "microSD reader  ", 16);
  memcpy(revision, "1.0 ", 4);
}
bool tud_msc_test_unit_ready_cb(uint8_t lun) {
  if (!lun && present.load()) return true;
  tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0);
  return false;
}
void tud_msc_capacity_cb(uint8_t, uint32_t *blocks, uint16_t *size) {
  *blocks = present.load() ? card.csd.capacity : 0;
  *size = 512;
}
bool tud_msc_is_writable_cb(uint8_t lun) { return !lun && present.load(); }
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buf, uint32_t size) {
  return transfer(lun, lba, offset, buf, size, false);
}
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buf, uint32_t size) {
  return transfer(lun, lba, offset, buf, size, true);
}
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t, bool start, bool eject) {
  if (lun) return false;
  if (eject && !start) {
    if (preventRemoval) {
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x53, 2);
      return false;
    }
    // All writes completed synchronously before this command. Stop accepting
    // new I/O before publishing eject to the UI; no automatic remount.
    present.store(false);
    ejectedAt.store(millis());
    ejected.store(true);
  }
  // One shot: reinsertion requires a new reader boot.
  if (start && ejected.load()) return false;
  return true;
}
int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t cmd[16], void *, uint16_t) {
  CardOperation operation;
  if (!tud_msc_test_unit_ready_cb(lun)) return -1;
  if (cmd[0] == SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL) {
    preventRemoval = (cmd[4] & 3) != 0;
    return 0;
  }
  if (cmd[0] == 0x35) {  // SYNCHRONIZE CACHE(10): no firmware write cache
    if (sdmmc_get_status(&card) == ESP_OK) return 0;
    ioError.store(true);
    tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x0c, 0);
    return -1;
  }
  tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0);
  return -1;
}
}

static uint16_t descriptor(uint8_t *dst, uint8_t *itf) {
  const uint8_t str = tinyusb_add_string_descriptor("microSD card reader");
  const uint8_t ep = tinyusb_get_free_duplex_endpoint();
  if (!ep) return 0;
  const uint8_t desc[] = {TUD_MSC_DESCRIPTOR(*itf, str, ep, uint8_t(0x80 | ep), 64)};
  ++*itf;
  memcpy(dst, desc, sizeof(desc));
  return sizeof(desc);
}

bool cardReaderBegin() {
  const bool enter = esp_reset_reason() == ESP_RST_SW && requested == READER_MAGIC;
  returnToSettings = esp_reset_reason() == ESP_RST_SW && requested == RETURN_MAGIC;
  requested = 0;  // crash/reset/power loss returns to the app, never a boot loop
  if (!enter) {
    // The RTC PHY selector can survive a software reset. Restore the same
    // hardware routing used by Arduino's usb_switch_to_cdc_jtag before Serial.
    CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG,
                       RTC_CNTL_SW_HW_USB_PHY_SEL | RTC_CNTL_SW_USB_PHY_SEL | RTC_CNTL_USB_PAD_ENABLE);
    return false;
  }
  setCpuFrequencyMhz(240);
  // Do not call inputInit: its emergency two-button restart would cut off a
  // mounted writable drive. The dedicated PWR exit below drains raw I/O first.
  pinMode(0, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  buttonRaw = buttonDown = digitalRead(18) == LOW;
  buttonAt = millis();
  pressedAt = buttonAt;
  displayInit(true);
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_1BIT;
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.clk = GPIO_NUM_39;
  slot.cmd = GPIO_NUM_41;
  slot.d0 = GPIO_NUM_40;
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
  failed = sdmmc_host_init() != ESP_OK;
  if (!failed) failed = sdmmc_host_init_slot(host.slot, &slot) != ESP_OK;
  if (!failed) failed = sdmmc_card_init(&host, &card) != ESP_OK;
  if (!failed) failed = card.csd.sector_size != 512 || !card.csd.capacity;
  if (!failed) {
    failed = tinyusb_enable_interface(USB_INTERFACE_MSC, TUD_MSC_DESC_LEN, descriptor) != ESP_OK;
    if (!failed) {
      USB.productName("Unidex microSD reader");
      present.store(true);
      usbStarted = USB.begin();
      failed = !usbStarted;
      if (failed) present.store(false);
    }
  }
  displayShow(drawReader, true);
  return true;
}

static void returnToApp() {
  present.store(false);
  // Never restart during a raw SD read/write/status operation, even on the
  // deliberate fallback. No timeout: abandoning an in-flight write is unsafe.
  while (activeOperations.load() != 0) delay(10);
  if (usbStarted) tud_disconnect();
  delay(250);
  while (digitalRead(0) == LOW) delay(10);  // release the BOOT strapping pin before restart
  requested = RETURN_MAGIC;
  ESP.restart();
}

void cardReaderPoll() {
  static bool shownEjected, shownError;
  if (shownEjected != ejected.load() || shownError != ioError.load()) {
    shownEjected = ejected.load();
    shownError = ioError.load();
    displayShow(drawReader, false);
  }
  const uint32_t now = millis();
  // Give TinyUSB time to send the successful eject CSW before disconnecting.
  // Suspend/disconnect notifications are NOT proof that the host ejected:
  // this board has no VBUS sense, so they never cause an automatic remount.
  if (ejected.load() && millis() - ejectedAt.load() >= EJECT_GRACE_MS) returnToApp();
  const bool down = digitalRead(18) == LOW;
  if (down != buttonRaw) {
    buttonRaw = down;
    buttonAt = now;
  }
  if (buttonRaw != buttonDown && now - buttonAt >= 50) {
    buttonDown = buttonRaw;
    if (buttonDown) pressedAt = now;
    else if (failed || exitArmed) returnToApp();
  }
  // A deliberate fallback for hosts which omit START STOP UNIT on eject, or
  // an already unplugged cable. Host-cached writes cannot be recovered here;
  // the display explicitly requires host eject/unplug before this gesture.
  if (!failed && buttonDown && !exitArmed && now - pressedAt >= EXIT_HOLD_MS) {
    exitArmed = true;
    displayShow(drawReader, false);
  }
  delay(10);  // no launcher, sync, background apps, or automatic sleep
}
#endif
