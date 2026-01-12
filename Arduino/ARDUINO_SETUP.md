# Karura Arm - Arduino Motor Control Setup Guide

This guide covers the 6-joint setup for the Karura robotic arm, standardized on two Raspberry Pi Picos. (Joint 7 / Gripper is removed).

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Hardware Requirements](#hardware-requirements)
3. [Arduino IDE Setup](#arduino-ide-setup)
4. [Flashing the Picos](#flashing-the-picos)
5. [Serial Protocol Documentation](#serial-protocol-documentation)
6. [Windows Development Setup (Docker + USB)](#windows-development-setup)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 / MoveIt                          │
│                                                             │
│   ┌──────────────────┐    ┌─────────────────────────────┐  │
│   │    MoveIt 2      │───>│  send_target_joint_degrees  │  │
│   │  (Motion Plan)   │    │    (rad → deg converter)    │  │
│   └──────────────────┘    └─────────────┬───────────────┘  │
│                                         │                   │
│                           /arm_targets  │ Float64MultiArray │
│                                         ▼                   │
│                           ┌─────────────────────────────┐  │
│                           │       Arm_serial.py         │  │
│                           │    (Serial Bridge Node)     │  │
│                           └─────────┬───────────────────┘  │
└─────────────────────────────────────┼───────────────────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    │                 │                 │
              USB Serial         USB Serial         
            /dev/ttyACM0       /dev/ttyACM1
                    │                 │
                    ▼                 ▼
           ┌──────────────┐  ┌──────────────┐
           │   Pico #1    │  │   Pico #2    │
           │ pico_j123.ino│  │ pico_j456.ino│
           │  Joints 1-3  │  │  Joints 4-6  │
           └──────┬───────┘  └──────┬───────┘
                  │                 │
              CAN Bus           CAN Bus
                  │                 │
                  ▼                 ▼
             Motors 1-3        Motors 4-6
```

---

## Hardware Requirements

| Component | Quantity | Notes |
|-----------|----------|-------|
| Raspberry Pi Pico | 2 | 1 for J1-3, 1 for J4-6 |
| MCP2515 CAN Module | 2 | SPI Interface |
| Robomaster Motors | 6 | M3508 or M2006 |

---

## Arduino IDE Setup

1. **Install Arduino IDE**
2. **Add Pico Support**: Add `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json` to board manager URLs.
3. **Install Libraries**: Copy `Arduino/libraries` to your local Arduino libraries folder.

---

## Flashing the Picos

### Pico #1: Joints 1-3
1. Open `Arduino/pico_j123/pico_j123.ino`
2. Connect Pico #1
3. Select Board: **Raspberry Pi Pico**
4. Upload

### Pico #2: Joints 4-6
1. Open `Arduino/pico_j456/pico_j456.ino`
2. Connect Pico #2
3. Upload

---

## Serial Protocol Documentation

Both Picos use a **7-byte** protocol at **115200 baud**.

### Message Format
```
Byte 0:     0xFF (header)
Bytes 1-2:  Joint A Data
Bytes 3-4:  Joint B Data
Bytes 5-6:  Joint C Data
```
*   **Pico 1**: A=Joint1, B=Joint2, C=Joint3
*   **Pico 2**: A=Joint4, B=Joint5, C=Joint6

### Per-Joint Encoding (2 bytes)
```
Byte 1: [ MotorIndex(2b) | Sign(1b) | ValueHigh(5b) ]
Byte 2: [ ValueLow(8b) ]
```
> **Note**: This logic is handled by `encode()`/`decode()` functions in the firmware and `Arm_serial.py`.

---

## Windows Development Setup (Docker + USB)

1. **Install usbipd-win**: `winget install usbipd`
2. **Bind Picos**: 
   ```powershell
   usbipd list
   usbipd bind --busid <ID>
   usbipd attach --wsl --busid <ID>
   ```
3. **Run Docker**:
   ```bash
   cd Arm2026/
   docker-compose up -d
   docker exec -it karura_dev bash
   ```
4. **Launch**:
   ```bash
   # Inside container
   serial  # Starts Arm_serial node
   ```

To verify joint control without hardware, use MoveIt:
```bash
moveit
```
