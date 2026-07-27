# Ashless visor firmware
This repository contains firmware and 3d printable files for an orientation-aware automatic-closing ashtray. An ESP32C3 computes spatial orientation using Data from a Bosch BMI160 IMU and controls two Servos. The mechanical design is inspired by helmet visor mechanisms.


## Mechanical Design
#### Sectional View
![Sectional View 1](/docs/3d_mechanism_1.gif)
![Sectional View 2](/docs/3d_mechanism_2.gif)

The following files need to be printed from `3d/stl`:
- visor_v1_0_bottom.stl
- visor_v1_0_cup_insert.stl
- visor_v1_0_shell.stl
- visor_v1_0_Visor_inner.stl
- visor_v1_0_Visor_outer.stl

Additional Parts:
- 4x servo linkage rod (can be bent from wire)
- 2x MG90S Servo
- 4x M2 Nut
- 4x M2x16 Screw
- 4x 1x3x1mm Bearing
- 1x 2,54mm Prototype Board
- 1x ESP32C3 Supermini
- 1x TP4056 Module
- 1x 5V Step-Up Converter

## Electronics

#### Hardware components
![hardware components](/docs/hardware.drawio.svg)


#### Breadboard PCB

![electronics](/docs/electronics.png)


#### Pinout
| Signal | ESP32-C3 |
| --- | --- |
| BMI160 SDA | GPIO2 |
| BMI160 SCL | GPIO3 |
| BMI160 INT1 | GPIO4 |
| SERVO1_PIN | GPIO22|
| SERVO2_PIN | GPIO21|