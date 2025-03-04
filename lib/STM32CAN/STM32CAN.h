#include <Arduino.h>
#include <assert.h>

/* Symbolic names for bit rate of CAN message                                */
typedef enum {
  CAN_50KBPS,
  CAN_100KBPS,
  CAN_125KBPS,
  CAN_250KBPS,
  CAN_500KBPS,
  CAN_1000KBPS
} BITRATE;

/* Symbolic names for formats of CAN message                                 */
typedef enum { STANDARD_FORMAT = 0, EXTENDED_FORMAT } CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {
  DATA_FRAME = 0,
  REMOTE_FRAME,
  ERROR_FRAME,
  OVERLOAD_FRAME
} CAN_FRAME;

typedef enum {
  ANG_CONTROL = 0,
  REQ_DATA,
  EMERGENCY_STOP,
  INVALID_COMMAND
} PARSE_RESULT;

typedef struct {
  uint32_t id;     /* 29 bit identifier                               */
  uint8_t data[8]; /* Data field                                      */
  uint8_t len;     /* Length of data field in bytes                   */
  uint8_t ch;      /* Object channel(Not use)                         */
  uint8_t format;  /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
  uint8_t type;    /* 0 - DATA FRAME, 1 - REMOTE FRAME, 2 - ERROR FRAME, 3 -
                      OVERLOAD FRAME */
} CAN_msg_t;

typedef struct {
  uint16_t baud_rate_prescaler;         /// [1 to 1024]
  uint8_t time_segment_1;               /// [1 to 16]
  uint8_t time_segment_2;               /// [1 to 8]
  uint8_t resynchronization_jump_width; /// [1 to 4] (recommended value is 1)
} CAN_bit_timing_config_t;

#define CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE 1000
#define CAN_STM32_ERROR_MSR_INAK_NOT_SET 1001
#define CAN_STM32_ERROR_MSR_INAK_NOT_CLEARED 1002
#define CAN_STM32_ERROR_UNSUPPORTED_FRAME_FORMAT 1003

#define STM32_CAN_TIR_TXRQ (1U << 0U) // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CAN_STD_ID_MASK 0x000007FFU

void packAngleIntoCanMessage(CAN_msg_t *message, float angle);
float unpackAngleFromCanMessage(const uint8_t *data);