/* Symbolic names for ID of CAN message                                      */
typedef enum {
  M0_ANGLE_CNTL = 0x20,
  M1_ANGLE_CNTL,
  M2_ANGLE_CNTL,
  M3_ANGLE_CNTL,
  M0_POS,
  M1_POS,
  M2_POS,
  M3_POS,
  ESTOP
} CAN_ID;

/* Symbolic names for ID of Motor                                            */
typedef enum { MOTOR_0_ID = 0, MOTOR_1_ID, MOTOR_2_ID, MOTOR_3_ID } MOTOR_ID;

class PingPongNotificationsFromCAN {
public:
  virtual void ReceivedMotorPosition(const uint8_t *data,
                                     const MOTOR_ID motor_id) = 0;
};

class CANPingPong : public SimpleCANProfile {
public:
  CANPingPong(SimpleCan *pCan, PingPongNotificationsFromCAN *_pRxCommands)
      : SimpleCANProfile(pCan) {
    pRxCommands = _pRxCommands;
  }

  void CANRequestCommand(uint8_t CanId) { Can1->RequestMessage(0, CanId); }

  void HandleCanMessage(const SimpleCanRxHeader rxHeader,
                        const uint8_t *rxData) {
    // Serial.println("@");

    switch (rxHeader.Identifier) {
    case M0_POS:
      pRxCommands->ReceivedMotorPosition(rxData, MOTOR_0_ID);
      break;
    case M1_POS:
      pRxCommands->ReceivedMotorPosition(rxData, MOTOR_1_ID);
      break;
    case M2_POS:
      pRxCommands->ReceivedMotorPosition(rxData, MOTOR_2_ID);
      break;
    case M3_POS:
      pRxCommands->ReceivedMotorPosition(rxData, MOTOR_3_ID);
      break;
    default:
      Serial.printf("y:0x%x DLC=0x%x ", rxHeader.Identifier,
                    rxHeader.DataLength);
      Serial.println();
    }
  }

private:
  PingPongNotificationsFromCAN *pRxCommands;
};
