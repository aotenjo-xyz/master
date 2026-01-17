// clang-format off
#include <Arduino.h>
#include <math.h>
#include "STM32CAN.h"
// clang-format on

int CAN_LED_PIN = PC6;
int ESTOP_LED_PIN = PC15;

// CAN FD Configuration
FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t txData[64]; // CAN FD can support up to 64 bytes
uint8_t rxData[64];

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
void MX_FDCAN1_Init(void) {
  hfdcan1.Instance = FDCAN1;

  // Basic configuration
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS; // CAN FD with Bit Rate Switch
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

  // Nominal Bit Timing (Classic CAN part): 500 kbit/s
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;

  // Data Bit Timing (CAN FD part): 2 Mbit/s
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 2;

  // Message RAM configuration
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Serial.println("FDCAN initialization failed!");
    Error_Handler();
  }

  // Configure global filter to accept all messages
  if (HAL_FDCAN_ConfigGlobalFilter(
          &hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
          FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    Serial.println("FDCAN global filter configuration failed!");
    Error_Handler();
  }

  // Start FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Serial.println("FDCAN start failed!");
    Error_Handler();
  }

  Serial.println("FDCAN initialized successfully!");
}

/**
 * @brief GPIO Initialization Function for FDCAN1
 * @param None
 * @retval None
 */
void MX_FDCAN1_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIOB clock
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure GPIO pins for FDCAN1
  // PB8: FDCAN1_RX
  // PB9: FDCAN1_TX
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief Send a CAN FD message
 * @param id: Message ID
 * @param data: Pointer to data buffer
 * @param length: Data length (up to 64 bytes for CAN FD)
 * @retval HAL status
 */
/**
 * @brief Convert byte length to FDCAN data length code
 */
uint32_t GetFDCANDataLengthCode(uint8_t bytes) {
  if (bytes <= 8)
    return bytes << 16;
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
    return dlc >> 16; // Fallback for standard lengths
  }
}

/**
 * @brief Sends a CAN FD message with specified ID and data payload
 *
 * @param id The CAN message identifier (11-bit or 29-bit depending on
 * configuration)
 * @param data Pointer to the data buffer containing the message payload
 * @param length Number of bytes in the data payload (0-64 bytes for CAN FD)
 *
 * @return HAL_StatusTypeDef Returns HAL_OK on success, or appropriate error
 * code on failure
 */
HAL_StatusTypeDef CANFD_SendMessage(uint32_t id, uint8_t *data,
                                    uint8_t length) {
  // Configure transmission header
  txHeader.Identifier = id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = GetFDCANDataLengthCode(length);
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_ON; // Enable bit rate switching for CAN FD
  txHeader.FDFormat = FDCAN_FD_CAN;      // CAN FD format
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  // Send message
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

void ReceivedMotorPosition(const uint8_t *data, const uint32_t motor_id) {
  float angle = unpackAngleFromCanMessage(data);
  //   print M<id>P<angle>
  Serial.printf("M%dP%.2f\n", motor_id, angle);
};

/**
 * @brief Check for received CAN FD messages
 * @param None
 * @retval None
 */
void CANFD_CheckReceived(void) {
  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) ==
      HAL_OK) {

    uint8_t dataLength = GetBytesFromFDCANDataLength(rxHeader.DataLength);

    Serial.print("ID: 0x");
    Serial.print(rxHeader.Identifier, HEX);
    Serial.print(" (");
    Serial.print(dataLength);
    Serial.print(" bytes)");

    // Check if this is CAN FD format
    if (rxHeader.FDFormat == FDCAN_FD_CAN) {
      Serial.print(" [CAN FD]");
      if (rxHeader.BitRateSwitch == FDCAN_BRS_ON) {
        Serial.print(" [BRS]");
      }
    } else {
      Serial.print(" [Classic CAN]");
    }
    Serial.println();

    // Display message content based on size
    if (dataLength <= 8) {
      uint32_t motor_id = rxHeader.Identifier - POS_COMMAND_OFFSET;
      Serial.print("  Motor ID: ");
      Serial.println(motor_id);

      ReceivedMotorPosition(rxData, motor_id);
    }

    // Toggle LED when message received
    digitalWrite(CAN_LED_PIN, !digitalRead(CAN_LED_PIN));
  }
}

void setup() {
  delay(100);

  Serial.begin(115200);
  pinMode(CAN_LED_PIN, OUTPUT);

  delay(5000);
  // while (!Serial);

  // Initialize FDCAN GPIO
  MX_FDCAN1_GPIO_Init();

  // Enable FDCAN1 clock
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // Initialize FDCAN1
  MX_FDCAN1_Init();

  pinMode(ESTOP_LED_PIN, OUTPUT);
  digitalWrite(ESTOP_LED_PIN, LOW);

  delay(1000);
  Serial.println("Start!");
}

void handleMotorAngleCommand(int _motor_id, float targetAngle) {
  Serial.printf("M%dA%.2f\n", _motor_id, targetAngle);
  packAngleIntoCanMessage(txData, targetAngle);
  HAL_StatusTypeDef status =
      CANFD_SendMessage(_motor_id + ANGLE_COMMAND_OFFSET, txData, 4);
  if (status != HAL_OK) {
    Serial.printf("Failed to send angle command to motor %d, error code: %d\n",
                  _motor_id, status);
  }
}

void handleMotorPositionCommand(int _motor_id) {
  Serial.printf(F("Requesting motor %d position\n"), _motor_id);
  HAL_StatusTypeDef status =
      CANFD_SendMessage(_motor_id + POS_COMMAND_OFFSET, nullptr, 0);
  if (status != HAL_OK) {
    Serial.printf("Failed to request position from motor %d, error code: %d\n",
                  _motor_id, status);
  }
}

PARSE_RESULT parseCommand(String command) {
  if (command.startsWith("M")) {
    int sepIdx = command.indexOf("A");
    int posIdx = command.indexOf("P");
    int motorId = -1;

    if (sepIdx != -1) { // Angle control: MxAangle
      motorId = command.substring(1, sepIdx).toInt();
      float targetAngle = command.substring(sepIdx + 1).toFloat();
      handleMotorAngleCommand(motorId, targetAngle);
      return ANG_CONTROL;
    } else if (posIdx != -1) { // Position request: MxP
      motorId = command.substring(1, posIdx).toInt();
      handleMotorPositionCommand(motorId);
      return REQ_DATA;
    }
  } else if (command.startsWith("ESTOP")) {
    Serial.println("Emergency stop command");
    digitalWrite(ESTOP_LED_PIN, HIGH);
    HAL_StatusTypeDef status = CANFD_SendMessage(ESTOP, nullptr, 0);
    if (status != HAL_OK) {
      Serial.printf("Failed to send emergency stop command, error code: %d\n",
                    status);
    }
    return EMERGENCY_STOP;
  }
  Serial.println("Invalid command");
  return INVALID_COMMAND;
}

const int BUFFER_SIZE = 50;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void loop() {
  // Check for serial input
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      memset(txData, 0, sizeof(txData)); // Clear txData before use

      inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
      parseCommand(inputBuffer);
      bufferIndex = 0; // Reset buffer index for the next command
    } else {
      if (bufferIndex < BUFFER_SIZE - 1) { // Prevent buffer overflow
        inputBuffer[bufferIndex++] = receivedChar;
      }
    }
  }

  // Check for received messages
  CANFD_CheckReceived();

  // Small delay to prevent overwhelming the system
  delay(10);
}