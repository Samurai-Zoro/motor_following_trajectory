# Motor Control STM32

Motor control project for STM32 Nucleo-F401RE using Arduino framework.

## Features
- ADC current sensing using ACS712
- Voltage sensing with divider + RC filter
- Limit switch handling
- Encoder reading
- PID motor control
- Binary/CSV streaming support

## Hardware
- STM32 Nucleo-F401RE
- ACS712 current sensor
- Voltage divider for motor voltage sensing
- Limit switch
- Motor drivers
- Encoders

## Required files
- motor-control-stm32.ino
- target_positions.h
- quantized_rotation_deg.h
