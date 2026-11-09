#pragma once

struct ClimateReading {
  float temperatureC;
  float humidityRH;
};

// Reads the board's SHTC3 (about 20 ms). Returns false if it doesn't answer or a CRC is wrong.
// The temperature includes a fixed offset for board heating, which varies with charging and WiFi activity.
bool climateRead(ClimateReading &out);
