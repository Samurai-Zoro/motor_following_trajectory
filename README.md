# Motor Control with ADC Current & Voltage Sensing

Arduino/STM32 motor control project for the STM32 Nucleo-F401RE.

## Project Purpose
This repository contains:

- **STM32 firmware** for a **2-motor trajectory-following system**
- **Python code** for reading streamed telemetry data and saving it to CSV

The system is designed for **input-shaped trajectory tracking** at a chosen shaping frequency to reduce vibration and residual oscillation.

## System Overview
This is a **two-motor path-following system**:

- **Motor A**: linear motion
- **Motor B**: rotational motion

Both motors follow a predefined trajectory/path.

The trajectory is shaped for a selected frequency so the motion is smoother and unwanted vibration is reduced.

If only one motor is needed, the unused axis can be disabled by setting the corresponding values in the header file to **zero**.

## Firmware Features
- Two-motor trajectory following
- Linear + rotational coordinated motion
- Input-shaped reference trajectory
- Frequency-based motion shaping
- PID-based tracking control
- PWM motor control
- Encoder feedback
- Limit switch support
- Piezo sensor reading
- ADC current sensing using ACS712
- ADC voltage sensing using resistor divider
- Binary or CSV telemetry streaming over serial

## Python Code
This repository also includes a Python script for reading the telemetry stream sent by the STM32 firmware and saving it as CSV.

The Python script:

- opens the serial port
- reads the firmware binary stream
- detects and validates frames
- checks CRC16 for frame integrity
- converts raw values into engineering units
- writes the data into a CSV file
- prints capture statistics such as average and instantaneous frame rate

This is useful for logging experiments and analyzing motor tracking performance.

## Repository Files
- `motor-control-stm32.ino`
- `target_positions.h`
- `quantized_rotation_deg.h`
- `Reader_2.py`
- `README.md`

## Required Firmware Files
- `motor-control-stm32.ino`
- `target_positions.h`
- `quantized_rotation_deg.h`

## Hardware
- STM32 Nucleo-F401RE
- 2 motors
  - 1 linear axis
  - 1 rotational axis
- ACS712 current sensor
- Voltage divider for motor voltage sensing
- Limit switch
- Encoders
- Piezo sensor
- Motor driver

## Pin Summary
- Current sense: `A2`
- Voltage sense: `A4`
- Piezo: `A3`
- Limit switch: `D13`

## Firmware Notes
This firmware uses the Arduino framework on STM32 and depends on board-specific timer support such as `HardwareTimer.h`.

The control system tracks a predefined trajectory using two axes:
- one linear
- one rotational

The target trajectory is intended for **input shaping** at a specified frequency to reduce resonance and vibration.

If only one axis is required, the other axis can be disabled by setting its corresponding trajectory/header values to zero.

## Telemetry Streaming
The firmware can stream telemetry data over serial.

The telemetry includes values such as:
- time
- actual and target positions
- PWM commands
- piezo response
- measured current
- measured voltage

This data can be read directly by the included Python script.

## Python Requirements
- Python 3
- `pyserial`

Install dependency with:

```bash
pip install pyserial
