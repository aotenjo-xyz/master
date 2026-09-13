#include "BoardInfo.h"

namespace {

constexpr uintptr_t BOARD_INFO_OTP_ADDRESS = 0x1FFF7000UL;
constexpr uint32_t ERASED_OTP_WORD = 0xFFFFFFFFUL;

bool isKnownProduct(uint32_t product) {
  return product == static_cast<uint32_t>(AotenjoProduct::One) ||
         product == static_cast<uint32_t>(AotenjoProduct::Master);
}

} // namespace

BoardInfo readBoardInfo() {
  const auto *otp =
      reinterpret_cast<volatile const uint32_t *>(BOARD_INFO_OTP_ADDRESS);

  const uint32_t rawProduct = otp[0];
  const uint32_t rawVersion = otp[1];

  BoardInfo board{};

  board.programmed =
      rawProduct != ERASED_OTP_WORD || rawVersion != ERASED_OTP_WORD;

  if (!board.programmed) {
    board.product = AotenjoProduct::Unknown;
    board.major = 0;
    board.minor = 0;
    board.valid = false;
    return board;
  }

  board.product = static_cast<AotenjoProduct>(rawProduct);
  board.major = static_cast<uint16_t>(rawVersion & 0xFFFFU);
  board.minor = static_cast<uint16_t>((rawVersion >> 16) & 0xFFFFU);
  board.valid = isKnownProduct(rawProduct);

  return board;
}

bool isHardwareAtLeast(const BoardInfo &board, uint16_t major, uint16_t minor) {
  if (board.major != major) {
    return board.major > major;
  }

  return board.minor >= minor;
}
