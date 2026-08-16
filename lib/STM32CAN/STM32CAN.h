#include <Arduino.h>
#include <assert.h>

#define ANGLE_COMMAND_OFFSET 0x20
#define POS_COMMAND_OFFSET 0x30
#define VSENSE_COMMAND_OFFSET 0x40
#define ESTOP 0xff
#define PID_CONFIG_CMD_OFFSET 0x50
#define PID_CONFIG_REQUEST_CMD_OFFSET 0x60
#define PID_CONFIG_CMD (MOTOR_ID + PID_CONFIG_CMD_OFFSET)
#define PID_CONFIG_REQUEST_CMD (MOTOR_ID + PID_CONFIG_REQUEST_CMD_OFFSET)

void packAngleIntoCanMessage(uint8_t *message, float angle);
float unpackFloatFromCanMessage(const uint8_t *data);
uint32_t GetFDCANDataLengthCode(uint8_t bytes);
uint8_t GetBytesFromFDCANDataLength(uint32_t dlc);