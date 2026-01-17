#include "STM32CAN.h"

// Function to pack float angle into CAN message
void packAngleIntoCanMessage(uint8_t *message, float angle) {
  // Convert float to byte array using memcpy (be mindful of endianness)
  memcpy(message, &angle, sizeof(float));
}

// Function to unpack float angle from CAN message
float unpackAngleFromCanMessage(const uint8_t *data) {
  float angle;
  memcpy(&angle, data, sizeof(float));
  return angle;
}
