# master
This is Aotenjo Master board firmware repository. It controls the motors with the CAN protocol.


Features
- STM32G431CB (128KB Flash, 32KB RAM, 170MHz)
- CAN (up to 1Mbps)
- USB type C


## Install

Install this repo
```bash
git clone https://github.com/aotenjo-xyz/master.git 
```

Install dependencies(SimpleCanLib)
```bash
mkdir Libraries
cd Libraries
git clone https://github.com/yuichiroaoki/SimpleCanLib.git
git checkout CANSendByte
```


Directory structure
```bash
├── Libraries
│   └── SimpleCanLib
└── master
    ├── include
    ├── lib
    ├── LICENSE
    ├── platformio.ini
    ├── README.md
    ├── src
    └── test
```

## Usage

### Send target command to the motor
```
M<motor<id>A<target>
```

e.g. Set motor 1 position to 6.28 rad
```
M0A6.28
```

### Get motor position
```
M<motor_id>P
```

e.g. Get motor 1 position
```
M0P
```

### Emergency stop
```
ESTOP
```

### Development

Format
```bash
make format
```


## References

- [Igitigit2/SimpleCanLib: CAN bus library for ESP32 and STM32 G431](https://github.com/Igitigit2/SimpleCanLib)
- [mackelec/meFDCAN: Arduino FDCAN library for stm32G4 microcontrollers](https://github.com/mackelec/meFDCAN)
