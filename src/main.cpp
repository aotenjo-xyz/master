// clang-format off
#include <Arduino.h>
#include <math.h>
#include "SimpleCAN.h"
#include "CANProfile.h"
#include "STM32CAN.h"
// clang-format on

int CAN_LED_PIN = PC6;
int ESTOP_LED_PIN = PC15;

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

class RxFromCAN : public PingPongNotificationsFromCAN {
public:
  RxFromCAN(){};

  void ReceivedMotorPosition(const uint8_t *data, const uint32_t motor_id) {
    float angle = unpackAngleFromCanMessage(data);
    //   print M<id>P<angle>
    Serial.printf("M%dP%.2f\n", motor_id, angle);
  };
};

RxFromCAN CANBroker;

CANPingPong CANDevice(CreateCanLib(PB9, PB8), &CANBroker);

void setup() {
  delay(100);

  Serial.begin(115200);
  delay(5000);
  // while (!Serial);
  Serial.println("Started");

  CANDevice.Init();
  CANDevice.Can1->EnableBlinkOnActivity(CAN_LED_PIN);

  pinMode(ESTOP_LED_PIN, OUTPUT);
  digitalWrite(ESTOP_LED_PIN, LOW);

  delay(1000);
  Serial.println("Start!");
}

void handleMotorAngleCommand(int _motor_id, uint32_t _can_id, float targetAngle,
                             CAN_msg_t *message) {
  Serial.printf("M%dA%.2f\n", _motor_id, targetAngle);
  message->id = _can_id;
  packAngleIntoCanMessage(message, targetAngle);
}

void handleMotorPositionCommand(int _motor_id, uint32_t _can_id,
                                CAN_msg_t *message) {
  Serial.printf(F("Requesting motor %d position\n"), _motor_id);
  message->id = _can_id;
}

PARSE_RESULT parseCommand(String command, CAN_msg_t *message) {
  if (command.startsWith("M")) {
    int sepIdx = command.indexOf("A");
    int posIdx = command.indexOf("P");
    int motorId = -1;

    if (sepIdx != -1) { // Angle control: MxAangle
      motorId = command.substring(1, sepIdx).toInt();
      float targetAngle = command.substring(sepIdx + 1).toFloat();
      handleMotorAngleCommand(motorId, motorId + ANGLE_COMMAND_OFFSET,
                              targetAngle, message);
      return ANG_CONTROL;
    } else if (posIdx != -1) { // Position request: MxP
      motorId = command.substring(1, posIdx).toInt();
      handleMotorPositionCommand(motorId, motorId + POS_COMMAND_OFFSET,
                                 message);
      return REQ_DATA;
    }
  } else if (command.startsWith("ESTOP")) {
    message->id = ESTOP;
    Serial.println("Emergency stop command");
    return EMERGENCY_STOP;
  }
  return INVALID_COMMAND;
}

const int BUFFER_SIZE = 50;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void loop() {
  CAN_msg_t CAN_TX_msg;

  // Check for serial input
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
      PARSE_RESULT result = parseCommand(inputBuffer, &CAN_TX_msg);
      switch (result) {
      case ANG_CONTROL:
        CANDevice.CANSendByte(CAN_TX_msg.data, CAN_TX_msg.id);
        break;
      case REQ_DATA:
        CANDevice.CANRequestCommand(CAN_TX_msg.id);
        break;
      case EMERGENCY_STOP:
        CANDevice.CANRequestCommand(CAN_TX_msg.id);
        digitalWrite(ESTOP_LED_PIN, HIGH);
        break;
      case INVALID_COMMAND:
        // invalid command
        Serial.println("Invalid command");
        break;
      }
      bufferIndex = 0; // Reset buffer index for the next command
    } else {
      if (bufferIndex < BUFFER_SIZE - 1) { // Prevent buffer overflow
        inputBuffer[bufferIndex++] = receivedChar;
      }
    }
  }

  // Update message queues.
  CANDevice.Can1->Loop();
}
