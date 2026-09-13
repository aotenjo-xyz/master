#pragma once

#include <stdint.h>

enum class AotenjoProduct : uint32_t {
  Unknown = 0,
  One = 0x454E4F41,    // "AONE"
  Master = 0x54534D41, // "AMST"
};

struct BoardInfo {
  AotenjoProduct product;
  uint16_t major;
  uint16_t minor;
  bool programmed;
  bool valid;
};

BoardInfo readBoardInfo();

bool isHardwareAtLeast(const BoardInfo &board, uint16_t major, uint16_t minor);