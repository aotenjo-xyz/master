# Aotenjo Master
This is Aotenjo Master board firmware repository. It controls the motors with the FDCAN protocol.

[Docs](https://aotenjo.xyz/docs/category/aotenjo-master) | [Shop](https://shop.aotenjo.xyz/products/aotenjo-master-v2-1)

<img src="/.github/images/master-v2.1.png" alt="Aotenjo Master v2.1" width="500"/>


Features
- STM32G431CB (128KB Flash, 32KB RAM, 170MHz)
- FDCAN (CAN with Flexible Data-Rate)
- USB type C
- VCC voltage sensing

![Aotenjo Master Diagram](/.github/images/master-diagram.png)

## Install

Install this repo
```bash
git clone https://github.com/aotenjo-xyz/master.git 
```

## Compatibility
This firmware is compatible with Aotenjo Master v2.0 and later.

> [!WARNING]
> This firmware is not compatible with Aotenjo Master v1.x.


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

### Read VCC voltage
```
VSENSE
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