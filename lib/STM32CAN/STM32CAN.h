#include <Arduino.h>
#include <assert.h>

#define ANGLE_COMMAND_OFFSET 0x20
#define POS_COMMAND_OFFSET 0x30
#define ESTOP 0xff

typedef enum {
  ANG_CONTROL = 0,
  REQ_DATA,
  EMERGENCY_STOP,
  INVALID_COMMAND
} PARSE_RESULT;

void packAngleIntoCanMessage(uint8_t *message, float angle);
float unpackAngleFromCanMessage(const uint8_t *data);