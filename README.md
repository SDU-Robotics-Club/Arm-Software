# Robotic Arm Software Implementation
This workspace contains the software implementation for the Robotc Arm project of the SDU Sønderborg Robotics Club.

Current technical specifications:
- Arduino Nano ESP32
- PCA9685 Motor control board
- RDS3235 & MG996R Servo motors
- Stepper Nema23 with TMC2208 driver

## Current Implementations
- Single-servo control using Arduino board
- Connectivity to Servo motors via PCA9685 motor control board
- Servo angle control via numerical input
- Controls stepper via l1 r1 buttons on controller via interrupts

## WIP
- Single-servo calibrations
- Multi-servo connection

## Future Goals
- Multi-servo control via controller
- Control mapping
- Arm control via computer vision
