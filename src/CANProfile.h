#define ANGLE_COMMAND_OFFSET 0x20
#define POS_COMMAND_OFFSET 0x30
#define ESTOP 0xff

class PingPongNotificationsFromCAN {
public:
  virtual void ReceivedMotorPosition(const uint8_t *data,
                                     const uint32_t motor_id) = 0;
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
    pRxCommands->ReceivedMotorPosition(rxData, rxHeader.Identifier -
                                                   POS_COMMAND_OFFSET);
  }

private:
  PingPongNotificationsFromCAN *pRxCommands;
};
