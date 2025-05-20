# ESP32 Bluetooth Mouse – Lab 4

## Overview

This project implements a Bluetooth HID (Human Interface Device) mouse using the ESP32-C3 development board. The system reads orientation data from the onboard I2C motion sensor to determine tilt direction and magnitude, which is then translated into mouse cursor movements over Bluetooth. The final implementation includes direction detection, mouse movement, speed control based on tilt, and acceleration over time.

---

## Lab Structure

### 📁 `lab4_1/` – Direction Detection

- Detects board tilt in real-time and prints corresponding directions to the terminal using `ESP_LOGI`.
- Recognizes `UP`, `DOWN`, `LEFT`, `RIGHT` directions, with support for diagonal combinations (e.g., `UP LEFT`).
- Utilizes I2C communication to poll motion data at a consistent rate.

### 📁 `lab4_2/` – Basic Bluetooth Mouse Control

- Modifies the ESP-IDF BLE HID example to move the mouse cursor left and right.
- Pauses 5 seconds after movement to demonstrate stable and repeatable behavior.
- Verified BLE HID pairing with a Raspberry Pi host system.

### 📁 `lab4_3/` – Full Integration with Acceleration

- Combines tilt detection from `lab4_1` with BLE mouse control from `lab4_2`.
- Adds two levels of tilt sensitivity (`A_BIT_LEFT/RIGHT` and `A_LOT_LEFT/RIGHT`) for speed control.
- Implements acceleration logic based on time held in one direction (e.g., a = 1, 2, 3...).
- Enables smooth and dynamic mouse control, allowing precise GUI interaction such as closing a window within 4 seconds.

---

## Features

- ✅ Real-time orientation-based mouse control
- ✅ Multi-directional tilt detection
- ✅ I2C integration for motion data
- ✅ BLE HID mouse emulation using ESP-IDF
- ✅ Time-based acceleration and speed levels
- ✅ Debug output via `ESP_LOGI`

---

## Usage

### ⚙️ Build and Flash

Each subdirectory (e.g., `lab4_1`) contains its own `main/` and `CMakeLists.txt`.

To build and flash:
```bash
cd lab4_3
idf.py build
idf.py flash monitor
