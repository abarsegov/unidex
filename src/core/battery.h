#pragma once

int batteryMillivolts();  // battery voltage in mV (GPIO4 through the board's divider by 2)
struct BatteryAdcSample {
  int millivolts, rawMean, rawSpan;
};
struct BatteryProbeResult {
  BatteryAdcSample before, after;
};
BatteryAdcSample batteryAdcSample();  // calibrated VBAT and a burst of raw GPIO4 readings
int batteryPercent();     // 0-100 estimated from voltage, or -1 if no battery is detected
bool batteryCharging();   // recent USB frames or filtered VBAT above the wall adapter threshold
// nullptr when there's charge for heavy work (WiFi, the radio, recording), else the reason to show ("Battery too
// low"). Always fine on USB or with no battery fitted.
const char *batteryTooLow();
bool batteryEmpty();  // flat: time to switch off (power.cpp does)
#if UNIDEX_DEV
void batteryFake(int mv);  // test build: pretend the battery reads mv (0: the real reading); the % starts again
bool batteryKept();        // test build: the % carried on from before the last restart (or sleep)
void batteryPrintCalls();  // test build: "BC <ms> <mv> <plugged> <charging> <pct>" per % worked out
#endif
