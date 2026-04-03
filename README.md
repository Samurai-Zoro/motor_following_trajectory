# Motor Control with ADC Current & Voltage Sensing

Arduino/STM32 motor control project for the STM32 Nucleo-F401RE.

## Project Purpose
This code is designed for a **two-motor trajectory-following system** used for **input shaping** at a chosen shaping frequency.

The two motors have different motion roles:
- **One motor performs rotational motion**
- **One motor performs linear motion**

Both motors are commanded to follow a predefined path/trajectory.  
The trajectory is shaped for a specific frequency so that vibration, oscillation, and residual motion can be reduced during operation.

## System Behavior
- 2-motor coordinated path following
- One **rotational axis**
- One **linear axis**
- Trajectory tracking with **input-shaped commands**
- Shaping tuned to a selected frequency
- Can also be used in **single-motor mode**

## Single-Motor Use
If only one motor is needed, the unused motor can be disabled by setting its corresponding values in the header file to **zero**.

For example, if one axis is not required, you can simply zero the associated trajectory/header values so the code effectively runs only the motor you want to use.

## Features
- Two-motor path following
- Coordinated rotational and linear motion
- Input-shaped motion commands
- Frequency-based trajectory shaping
- Motor control with PWM
- ADC current sensing using ACS712
- ADC voltage sensing using resistor divider
- Limit switch support
- Encoder feedback
- PID-based position control
- Streaming telemetry in binary or CSV format

## Hardware
- STM32 Nucleo-F401RE
- 2 motors:
  - 1 rotational motor
  - 1 linear motor
- ACS712 current sensor
- Motor driver
- Limit switch
- Encoders
- Piezo sensor

## Pin Summary
- Current sense: A2
- Voltage sense: A4
- Piezo: A3
- Limit switch: D13

## Required files
- motor-control-stm32.ino
- target_positions.h
- quantized_rotation_deg.h

## Notes
This project uses the Arduino framework for STM32 and depends on board-specific timer support such as `HardwareTimer.h`.

The motion profile is intended for trajectory tracking under an input-shaping scheme tuned to a specific frequency to help suppress resonance and reduce vibration.

If only one motor axis is required, the second axis can be disabled by setting the corresponding header trajectory values to zero.

## License
Add your preferred license here.
