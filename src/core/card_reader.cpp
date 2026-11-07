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
static constexpr uint32_t READER_VBUS_MAGIC = 0x554D5344;
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

// Optional HARDWARE VBUS sense. A free ADC1 pin must be connected to USB VBUS
// through 10k, with another 10k from the pin to GND. Never connect 5V directly.
// With no wiring, the internal pulldown keeps the detector unarmed.
#ifndef UNIDEX_USB_VBUS_PIN
#define UNIDEX_USB_VBUS_PIN -1
#endif
static bool vbusArmed;
#if UNIDEX_USB_VBUS_PIN >= 0
static_assert(UNIDEX_USB_VBUS_PIN >= 1 && UNIDEX_USB_VBUS_PIN <= 10,
              "USB VBUS sense requires an unused ESP32-S3 ADC1 GPIO (1..10)");
static constexpr uint32_t VBUS_HIGH_MV = 1800, VBUS_LOW_MV = 800;
static constexpr uint32_t VBUS_ARM_MS = 300, VBUS_LOST_MS = 150, VBUS_SAMPLE_MS = 20;
static uint32_t vbusSampleAt, vbusHighAt, vbusLowAt;
static bool vbusHigh, vbusLow;

static void vbusBegin(bool inherited) {
  vbusArmed = inherited;
  vbusHigh = vbusLow = false;
  vbusSampleAt = millis() - VBUS_SAMPLE_MS;
  analogSetPinAttenuation(UNIDEX_USB_VBUS_PIN, ADC_11db);
  pinMode(UNIDEX_USB_VBUS_PIN, INPUT_PULLDOWN);
}

static bool vbusPoll() {
  if (millis() - vbusSampleAt < VBUS_SAMPLE_MS) return false;
  // Arduino 2.0.17 switches to ANALOG (clearing the pulldown) on every read.
  // Discharge an unwired input before sampling, then restore the pulldown.
  pinMode(UNIDEX_USB_VBUS_PIN, INPUT_PULLDOWN);
  delayMicroseconds(1000);
  const uint32_t mv = analogReadMilliVolts(UNIDEX_USB_VBUS_PIN);
  pinMode(UNIDEX_USB_VBUS_PIN, INPUT_PULLDOWN);
  const uint32_t now = millis();
  vbusSampleAt = now;
  if (!vbusArmed) {
    if (mv >= VBUS_HIGH_MV) {
      if (!vbusHigh) { vbusHigh = true; vbusHighAt = now; }
      if (now - vbusHighAt >= VBUS_ARM_MS) vbusArmed = true;
    } else vbusHigh = false;
    return false;
  }
  if (mv <= VBUS_LOW_MV) {
    if (!vbusLow) { vbusLow = true; vbusLowAt = now; }
    return now - vbusLowAt >= VBUS_LOST_MS;
  }
  vbusLow = false;  // a rebound or intermediate voltage cancels the low timer
  return false;
}

static bool vbusArmBeforeRestart() {
  vbusBegin(false);
  const uint32_t start = millis();
  while (!vbusArmed && millis() - start < VBUS_ARM_MS + 2 * VBUS_SAMPLE_MS) {
    vbusPoll();
    delay(5);
  }
  return vbusArmed;
}
#else
static void vbusBegin(bool) {}
static bool vbusPoll() { return false; }
static bool vbusArmBeforeRestart() { return false; }
#endif

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
  // Carry a verified high across this deliberate reboot so unplugging during
  // the restart/display refresh is still detected in the reader boot.
  requested = vbusArmBeforeRestart() ? READER_VBUS_MAGIC : READER_MAGIC;
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
    if (vbusArmed) {
      drawCentered("Unplug: auto return", 115);
      drawCentered("to Settings", 140);
    } else {
      drawCentered("Unplug USB, then", 110);
      drawCentered("hold PWR 2s + release", 135);
      drawCentered("to return to Settings", 155);
    }
#if UNIDEX_USB_VBUS_PIN >= 0
    display.setFont(FONT_TINY);
    drawCentered(vbusArmed ? "USB voltage monitored" : "USB sense: waiting", 181);
#endif
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
  const bool vbusWasArmed = esp_reset_reason() == ESP_RST_SW && requested == READER_VBUS_MAGIC;
  const bool enter = esp_reset_reason() == ESP_RST_SW && (requested == READER_MAGIC || vbusWasArmed);
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
  vbusBegin(vbusWasArmed);
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
  // This is actual VBUS voltage loss, not USB suspend or an idle bus. An
  // unwired detector never arms, so the existing manual exit remains usable.
  if (vbusPoll() && !ejected.load()) returnToApp();
  static bool shownEjected, shownError, shownVbusArmed;
  if (shownEjected != ejected.load() || shownError != ioError.load() || shownVbusArmed != vbusArmed) {
    shownEjected = ejected.load();
    shownError = ioError.load();
    shownVbusArmed = vbusArmed;
    displayShow(drawReader, false);
  }
  const uint32_t now = millis();
  // Give TinyUSB time to send the successful eject CSW before disconnecting.
  // Suspend/disconnect notifications are NOT proof that the host ejected:
  // only the optional physical VBUS detector above handles cable power loss.
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
