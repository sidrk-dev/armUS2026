# Karura Arm (Arm2026) Project



## Quick Start Guide

This guide will take you from zero to moving a motor using Docker and a Raspberry Pi Pico.

### Prerequisites

#### Hardware
*   **Robot Arm**: Karura Arm (or at least one Robomaster motor for testing).
*   **Microcontrollers**: 2x Raspberry Pi Pico.
*   **Communication**: 2x MCP2515 CAN Bus modules.
*   **Power**: 24V Power Supply (for motors).
*   **PC**: Windows (with WSL2) or Linux.

#### Software
*   **Docker Desktop** installed and running.
*   **VS Code** (recommended).
*   **Arduino IDE** (for flashing firmware).

---

### Step 1: Hardware Setup

1.  **Wiring**: Connect your MCP2515 to the Raspberry Pi Pico as follows:
    *   **VCC** -> VBUS (5V)
    *   **GND** -> GND
    *   **CS**  -> GP5
    *   **SCK** -> GP6
    *   **SI**  -> GP7 (TX)
    *   **SO**  -> GP4 (RX)
    *   **INT** -> (Not used)
2.  **CAN Bus**: Connect the CAN H and CAN L lines from the MCP2515 to your Robomaster motors.
3.  **Power**: Ensure motors are powered (24V).

---

### Step 2: Firmware Setup (Arduino)

1.  **Install Arduino IDE**.
2.  **Add Pico Support**:
    *   Open `File > Preferences`.
    *   Add this URL to "Additional Boards Manager URLs":
        `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
    *   Go to `Tools > Board > Boards Manager`, search for "Pico", and install **Raspberry Pi Pico/RP2040**.
3.  **Install Libraries**:
    *   Copy the entire folder `Arduino/libraries` from this repository into your local Arduino libraries folder (usually `Documents/Arduino/libraries`).
4.  **Flash the Picos**:
    *   **Pico #1 (Joints 1-3)**: Open `Arduino/pico_j123/pico_j123.ino`, select "Raspberry Pi Pico" as the board, and upload.
    *   **Pico #2 (Joints 4-6)**: Open `Arduino/pico_j456/pico_j456.ino`, select "Raspberry Pi Pico" as the board, and upload.

---

### Step 3: Software Setup (Docker)

We use Docker to avoid complex ROS 2 installation on your host machine.

1.  **Open Terminal** (Powershell or Bash) in this folder.
2.  **Start the Container**:
    ```bash
    docker-compose up -d
    ```
    *This works on Windows (WSL2) and Linux. It sets up X11 forwarding for GUI apps like RViz.*

3.  **Enter the Container**:
    ```bash
    docker exec -it karura_dev bash
    ```
    *You are now inside the ROS 2 environment. The workspace is at `/root/ws_karura`.*

4.  **Build the Project**:
    Inside the container, run the alias:
    ```bash
    cb
    ```
    *(Short for `colcon build --symlink-install`)*

---

### Step 4: Moving a Motor

Now that everything is running, let's move a motor.

#### 1. Connect Hardware
Plug your flashed Picos into your computer via USB.
*   **Windows Users**: You need to attach the USB devices to WSL2.
    1.  Install `usbipd-win` (`winget install usbipd`).
    2.  Open a new PowerShell as Admin.
    3.  Run `usbipd list` to find your Pico devices.
    4.  Run `usbipd bind --busid <BUSID>` for both Picos.
    5.  Run `usbipd attach --wsl --busid <BUSID>` to attach them to the Docker/WSL instance.

#### 2. Start the Serial Bridge
Inside the Docker container:
```bash
serial
```
*(This is an alias for `ros2 run serial_comm Arm_serial`).*

You should see output like:
```text
[INFO] [Arm_serial]: Serial bridge node started
[INFO] [Arm_serial]: Opened serial port: /dev/ttyACM0
```

#### 3. Send Commands
Open a **second terminal**, enter the container (`docker exec -it karura_dev bash`), and run:

**Option A: Using the CLI Tool**
```bash
ros2 run send_target_joint_degrees send_target_joint_degrees
```
Follow the on-screen prompts to enter angles for joints 1-6.

**Option B: Manually Publishing (for testing)**
To move Joint 1 to 45 degrees:
```bash
ros2 topic pub /arm_targets std_msgs/msg/Float64MultiArray "{data: [45.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

---

## Repository Structure

*   **Arduino/**: Firmware for Raspberry Pi Pico.
*   **arm_controller/**: ROS 2 C++ node for kinematic control.
*   **karura_arm_description/**: URDF, meshes, and visual assets.
*   **karura_arm_moveit_config/**: MoveIt 2 configuration package.
*   **serial_comm/**: Python node bridging ROS 2 to Serial (Pico).
*   **send_target_joint_degrees/**: Simple CLI tool for testing joint movements.

## Troubleshooting

*   **"Serial port not found"**:
    *   Make sure you attached the USB devices using `usbipd` (Windows) or have correct permissions (Linux).
    *   Check `/dev/ttyACM0` exists inside the container.
*   **GUI not showing (RViz)**:
    *   **Windows**: Ensure **VcXsrv** is running with "Disable access control" checked.
    *   Check your `DISPLAY` environment variable.

## Maintainers

*   **Yujiro_Onishi** (Original Author)
