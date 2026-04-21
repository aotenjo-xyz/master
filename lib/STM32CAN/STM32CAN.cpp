#include "STM32CAN.h"

// Function to pack float angle into CAN message
void packAngleIntoCanMessage(uint8_t *message, float angle) {
  // Convert float to byte array using memcpy (be mindful of endianness)
  memcpy(message, &angle, sizeof(float));
}

// Function to unpack float angle from CAN message
float unpackFloatFromCanMessage(const uint8_t *data) {
  float value;
  memcpy(&value, data, sizeof(float));
  return value;
}

/**
 * @brief Convert byte length to FDCAN data length code
 */
uint32_t GetFDCANDataLengthCode(uint8_t bytes) {
  if (bytes <= 8) {
    switch (bytes) {
    case 0:
      return FDCAN_DLC_BYTES_0;
    case 1:
      return FDCAN_DLC_BYTES_1;
    case 2:
      return FDCAN_DLC_BYTES_2;
    case 3:
      return FDCAN_DLC_BYTES_3;
    case 4:
      return FDCAN_DLC_BYTES_4;
    case 5:
      return FDCAN_DLC_BYTES_5;
    case 6:
      return FDCAN_DLC_BYTES_6;
    case 7:
      return FDCAN_DLC_BYTES_7;
    default:
      return FDCAN_DLC_BYTES_8;
    }
  }
  if (bytes <= 12)
    return FDCAN_DLC_BYTES_12;
  if (bytes <= 16)
    return FDCAN_DLC_BYTES_16;
  if (bytes <= 20)
    return FDCAN_DLC_BYTES_20;
  if (bytes <= 24)
    return FDCAN_DLC_BYTES_24;
  if (bytes <= 32)
    return FDCAN_DLC_BYTES_32;
  if (bytes <= 48)
    return FDCAN_DLC_BYTES_48;
  return FDCAN_DLC_BYTES_64;
}

/**
 * @brief Convert FDCAN data length code to byte count
 */
uint8_t GetBytesFromFDCANDataLength(uint32_t dlc) {
  switch (dlc) {
  case FDCAN_DLC_BYTES_0:
    return 0;
  case FDCAN_DLC_BYTES_1:
    return 1;
  case FDCAN_DLC_BYTES_2:
    return 2;
  case FDCAN_DLC_BYTES_3:
    return 3;
  case FDCAN_DLC_BYTES_4:
    return 4;
  case FDCAN_DLC_BYTES_5:
    return 5;
  case FDCAN_DLC_BYTES_6:
    return 6;
  case FDCAN_DLC_BYTES_7:
    return 7;
  case FDCAN_DLC_BYTES_8:
    return 8;
  case FDCAN_DLC_BYTES_12:
    return 12;
  case FDCAN_DLC_BYTES_16:
    return 16;
  case FDCAN_DLC_BYTES_20:
    return 20;
  case FDCAN_DLC_BYTES_24:
    return 24;
  case FDCAN_DLC_BYTES_32:
    return 32;
  case FDCAN_DLC_BYTES_48:
    return 48;
  case FDCAN_DLC_BYTES_64:
    return 64;
  default:
    return FDCAN_DLC_BYTES_8; // Fallback to 8 bytes if unknown code
  }
}