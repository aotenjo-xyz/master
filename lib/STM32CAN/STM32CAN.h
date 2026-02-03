#include <Arduino.h>
#include <assert.h>

#define ANGLE_COMMAND_OFFSET 0x20
#define POS_COMMAND_OFFSET 0x30
#define ESTOP 0xff

void packAngleIntoCanMessage(uint8_t *message, float angle);
float unpackAngleFromCanMessage(const uint8_t *data);
uint32_t GetFDCANDataLengthCode(uint8_t bytes);
uint8_t GetBytesFromFDCANDataLength(uint32_t dlc);