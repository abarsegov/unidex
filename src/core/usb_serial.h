#pragma once

#if UNIDEX_EMU
#include <Arduino.h>
inline decltype(Serial) &usbSerial() { return Serial; }
#else
#include <USBCDC.h>
USBCDC &usbSerial();
#endif
