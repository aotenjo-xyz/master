#include "VSENSE.h"

#include "BoardInfo.h"
#include <Arduino.h>

namespace {

constexpr float R1_MASTER_LEGACY = 54000.0f;
constexpr float R1_MASTER_V2_3 = 75000.0f;
constexpr float R2 = 5100.0f;
constexpr float V_REF = 2.9f;

float getR1ForBoard(const BoardInfo &board) {
  if (board.valid && board.product == AotenjoProduct::Master &&
      isHardwareAtLeast(board, 2, 3)) {
    return R1_MASTER_V2_3;
  }

  // Blank OTP means one of the existing Master v2.1 boards.
  return R1_MASTER_LEGACY;
}

} // namespace

float readVoltage() {
  const BoardInfo board = readBoardInfo();
  const float r1 = getR1ForBoard(board);

  const int rawValue = analogRead(VCC_MONITOR_PIN);
  const float voltageAtPin = static_cast<float>(rawValue) * V_REF / 4095.0f;

  return voltageAtPin * ((r1 + R2) / R2);
}