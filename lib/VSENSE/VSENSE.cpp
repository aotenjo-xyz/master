#include "VSENSE.h"
#include <Arduino.h>

float readVoltage() {
  int rawValue = analogRead(VCC_MONITOR_PIN);

  // Use 2.9 in the calculation instead of 3.3
  float voltageAtPin = (rawValue * V_REF) / 4095.0;

  // Calculate actual battery/VCC voltage
  float actualVCC = voltageAtPin * ((R1 + R2) / R2);
  return actualVCC;
}
