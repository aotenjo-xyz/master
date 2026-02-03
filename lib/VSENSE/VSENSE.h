// read VCC voltage via voltage divider
#define VCC_MONITOR_PIN PC4
const float R1 = 54000.0;
const float R2 = 5100.0;
const float V_REF = 2.9;

float readVoltage();