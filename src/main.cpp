// clang-format off
#include <Arduino.h>
#include <math.h>
#include "SimpleCAN.h"
#include "CANProfile.h"
#include "STM32CAN.h"
// clang-format on

int CAN_LED_PIN = PC15;
int ESTOP_LED_PIN = PC4;

class RxFromCAN : public PingPongNotificationsFromCAN {
public:
  RxFromCAN(){};

  void ReceivedMotorPosition(const uint8_t *data, const MOTOR_ID motor_id) {
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

void handleMotorAngleCommand(int _motor_id, CAN_ID _can_id, float targetAngle,
                             CAN_msg_t *message) {
  Serial.printf("M%dA%.2f\n", _motor_id, targetAngle);
  message->id = _can_id;
  packAngleIntoCanMessage(message, targetAngle);
}

void handleMotorPositionCommand(int _motor_id, CAN_ID _can_id,
                                CAN_msg_t *message) {
  Serial.printf(F("Requesting motor %d position\n"), _motor_id);
  message->id = _can_id;
}

PARSE_RESULT parseCommand(String command, CAN_msg_t *message) {
  PARSE_RESULT result = ANG_CONTROL;
  // Example command: M1A6.28
  if (command.startsWith("M") && command.indexOf("A") != -1) {
    int motorId = command.substring(1, command.indexOf("A")).toInt();
    float targetAngle = command.substring(command.indexOf("A") + 1).toFloat();

    switch (motorId) {
    case MOTOR_0_ID:
      handleMotorAngleCommand(motorId, M0_ANGLE_CNTL, targetAngle, message);
      break;

    case MOTOR_1_ID:
      handleMotorAngleCommand(motorId, M1_ANGLE_CNTL, targetAngle, message);
      break;

    case MOTOR_2_ID:
      handleMotorAngleCommand(motorId, M2_ANGLE_CNTL, targetAngle, message);
      break;

    case MOTOR_3_ID:
      handleMotorAngleCommand(motorId, M3_ANGLE_CNTL, targetAngle, message);
      break;

    default:
      result = INVALID_COMMAND;
      break;
    }

  } else if (command.startsWith("M") && command.indexOf("P") != -1) {
    // M0P: send a remote frame to get the motor0 position
    int motorId = command.substring(1, command.indexOf("P")).toInt();
    result = REQ_DATA;
    switch (motorId) {
    case MOTOR_0_ID:
      handleMotorPositionCommand(motorId, M0_POS, message);
      break;
    case MOTOR_1_ID:
      handleMotorPositionCommand(motorId, M1_POS, message);
      break;
    case MOTOR_2_ID:
      handleMotorPositionCommand(motorId, M2_POS, message);
      break;
    case MOTOR_3_ID:
      handleMotorPositionCommand(motorId, M3_POS, message);
      break;
    default:
      result = INVALID_COMMAND;
      break;
    }
  } else if (command.startsWith("ESTOP")) {
    // Emergency stop command
    message->id = ESTOP;
    result = EMERGENCY_STOP;
    Serial.println("Emergency stop command");
  }
  return result;
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
