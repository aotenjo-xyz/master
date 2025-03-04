#include "STM32CAN.h"

// Function to pack float angle into CAN message
void packAngleIntoCanMessage(CAN_msg_t *message, float angle) {
  // Set message properties (assuming standard format and data frame)
  message->format = STANDARD_FORMAT;
  message->type = DATA_FRAME;
  message->len = 4; // Length set to 4 bytes for the float

  // Convert float to byte array using memcpy (be mindful of endianness)
  memcpy(message->data, &angle, sizeof(float));
}

// Function to unpack float angle from CAN message
float unpackAngleFromCanMessage(const uint8_t *data) {
  float angle;
  memcpy(&angle, data, sizeof(float));
  return angle;
}
