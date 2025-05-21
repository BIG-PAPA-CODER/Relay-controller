# Battery-Powered Relay Switching Device (STM32L031 Project)

## Overview

This project implements a **battery-powered, low-power relay switching system** using an **STM32L031K6 microcontroller**. It is designed for controlling high-voltage AC/DC loads in energy-sensitive environments, such as remote systems or portable applications. The system supports efficient battery usage, visual status indication, adjustable relay toggle frequency, and low-battery monitoring, all managed via a single-button interface.

---

## Features

- **Relay control** for AC/DC high-current switching (up to 10A at 250VAC/30VDC)
- **Battery monitoring** using internal ADC and voltage divider
- **Single-button operation** for:
  - Short press: toggle standby (deep sleep) mode
  - Long press: change relay toggle frequency
- **Status LEDs** for power and relay indication
- **Low-battery warning** (3.1V threshold with visual feedback)
- **Ultra-low-power standby mode** (deep sleep)
- **Compact circuit powered by a 3.7V Li-ion battery**
- **Thorough code documentation** in C using STM32 HAL

---

## Hardware Overview

### Components

- **MCU**: STM32L031K6 (Cortex-M0+, ultra-low-power)
- **Relay module**: SRD-05VDC-SL-C (5V, single-channel)
- **Voltage regulators**:
  - TPS63060 (buck-boost converter for 3.3V system supply)
  - MT3608 (step-up converter to 5V for relay control)
- **Battery**: 3.7V 18650 Li-ion
- **Voltage divider**: for scaling battery voltage to ADC input
- **Push-button**: controls system state and relay behavior

### Electrical Schematic

The full schematic of the circuit, including power converters, MCU connections, and signal routing, is available here:
[`scheme.pdf`](docs/scheme.pdf)

---

## Firmware Details

- **Platform**: STM32CubeIDE (with `.ioc` project config)
- **Language**: C using STM32 HAL drivers
- **Main file**: `main.c`

### Key Functions

| Function         | Description |
|------------------|-------------|
| `battery_measure()` | Reads ADC value and calculates battery voltage |
| `enterDeepSleep()`  | Puts device into ultra-low-power standby |
| `main()`            | Core loop: button control, relay switching, LED control |

The firmware is structured with clear comments and separation between hardware initialization and main logic.

---

## Build & Flash Instructions

1. Open the project in **STM32CubeIDE**.
2. Connect your STM32L031K6 board via USB or ST-Link.
3. Build the project.
4. Flash the firmware to the MCU.
5. Power via 3.7V battery for real-world testing.

---

## Usage

- **Short press** the button → enter/exit deep sleep mode.
- **Long press** → cycle through preset relay switching frequencies.
- When **battery voltage < 3.1V**, the green onboard LED lights up as a warning.

---

## Documentation

For full technical details, including hardware overview, firmware functions, and schematics, refer to the complete project documentation:

[Documentation_relay.pdf](docs/Documentation_relay.pdf)
