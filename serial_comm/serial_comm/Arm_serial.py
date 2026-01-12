"""
Arm Serial Bridge Node

This ROS2 node bridges communication between MoveIt (via arm_targets topic) and
the Raspberry Pi Pico microcontrollers that control the Robomaster motors via CAN bus.

Architecture:
    MoveIt → send_target_joint_degrees → [arm_targets] → Arm_serial → Pico(s) → Motors

Two Picos are used:
    - Pico 1 (ser1): Controls joints 1-3 via 7-byte protocol
    - Pico 2 (ser2): Controls joints 4-6 via 7-byte protocol
    (Gripper/Joint 7 has been removed)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time


class SerialJoints(Node):
    """ROS2 node for serial communication with Pico motor controllers."""
    
    # Serial protocol constants
    HEADER_BYTE = 0xFF
    BAUD_RATE = 115200
    
    # Joint mapping (Total 6 joints)
    NUM_JOINTS_PICO1 = 3  # Joints 1-3
    NUM_JOINTS_PICO2 = 3  # Joints 4-6
    
    # Protocol sizes (1 header + N*2 bytes)
    PROTOCOL_SIZE_PICO1 = 7  # 1+3*2
    PROTOCOL_SIZE_PICO2 = 7  # 1+3*2

    def __init__(self):
        super().__init__('Arm_serial')
        
        # Declare ROS2 parameters for serial ports
        self.declare_parameter('port_pico1', '/dev/ttyACM0')
        self.declare_parameter('port_pico2', '/dev/ttyACM1')
        self.declare_parameter('enable_pico2', True)
        
        # Get parameter values
        port1 = self.get_parameter('port_pico1').get_parameter_value().string_value
        port2 = self.get_parameter('port_pico2').get_parameter_value().string_value
        self.enable_pico2 = self.get_parameter('enable_pico2').get_parameter_value().bool_value
        
        # Initialize serial connections
        self.ser1 = None
        self.ser2 = None
        self._init_serial(port1, port2)
        
        # Publishers and subscribers
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm_current', 10)
        self.arm_sub = self.create_subscription(
            Float64MultiArray, 'arm_targets', self.target_callback, 10
        )
        
        # State tracking (6 joints total)
        self.target_degrees = [0.0] * 6
        self.current_degrees = [0.0] * 6
        
        self.get_logger().info('Serial bridge node started (6-joint config)')
        self.get_logger().info(f'  Pico 1 (J1-3): {port1} - {"Connected" if self.ser1 else "NOT CONNECTED"}')
        if self.enable_pico2:
            self.get_logger().info(f'  Pico 2 (J4-6): {port2} - {"Connected" if self.ser2 else "NOT CONNECTED"}')

    def _init_serial(self, port1: str, port2: str):
        """Initialize serial connections with error handling."""
        # Initialize Pico 1 (joints 1-3)
        try:
            self.ser1 = serial.Serial(port1, self.BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Opened serial port: {port1}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open Pico 1 port {port1}: {e}')
            self.ser1 = None
        
        # Initialize Pico 2 (joints 4-6)
        if self.enable_pico2:
            try:
                self.ser2 = serial.Serial(port2, self.BAUD_RATE, timeout=0.1)
                self.get_logger().info(f'Opened serial port: {port2}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open Pico 2 port {port2}: {e}')
                self.ser2 = None

    def target_callback(self, msg: Float64MultiArray):
        """Handle incoming target joint angles from MoveIt."""
        # Store target angles (expecting 6 values)
        for i in range(min(6, len(msg.data))):
            self.target_degrees[i] = msg.data[i]
        
        self.get_logger().debug(f'Received targets: {self.target_degrees}')
        self._send_to_picos()

    def _send_to_picos(self):
        """Send target angles to both Picos."""
        # Send to Pico 1 (joints 1-3)
        if self.ser1:
            data_pico1 = self._encode_joints(self.target_degrees[0:3], 3)
            try:
                self.ser1.write(data_pico1)
            except serial.SerialException as e:
                self.get_logger().error(f'Pico 1 write error: {e}')
        
        # Send to Pico 2 (joints 4-6)
        if self.ser2 and self.enable_pico2:
            data_pico2 = self._encode_joints(self.target_degrees[3:6], 3)
            try:
                self.ser2.write(data_pico2)
            except serial.SerialException as e:
                self.get_logger().error(f'Pico 2 write error: {e}')
        
        # Wait briefly then read feedback
        time.sleep(0.05)
        self._receive_from_picos()

    def _encode_joints(self, joint_values: list, num_joints: int) -> bytes:
        """
        Encode joint angles into the serial protocol format.
        Protocol: Byte 1: [motor_num(2b)][sign(2b)][value_high(4b)], Byte 2: [value_low(8b)]
        Value = angle * 5
        """
        size = 1 + num_joints * 2
        encoded = bytearray(size)
        encoded[0] = self.HEADER_BYTE
        
        for i in range(num_joints):
            angle = joint_values[i] if i < len(joint_values) else 0.0
            data = int(angle * 5)
            sign = 1 if data >= 0 else 0
            value = abs(data)
            
            encoded[i * 2 + 1] = (i << 6) | (sign << 4) | ((value >> 8) & 0x0F)
            encoded[i * 2 + 2] = value & 0xFF
        
        return bytes(encoded)

    def _receive_from_picos(self):
        """Read feedback from both Picos and publish current joint states."""
        # Pico 1 (Joints 1-3)
        if self.ser1 and self.ser1.in_waiting >= self.PROTOCOL_SIZE_PICO1:
            self._decode_feedback(self.ser1, 0, 3)
        
        # Pico 2 (Joints 4-6)
        if self.ser2 and self.ser2.in_waiting >= self.PROTOCOL_SIZE_PICO2:
            self._decode_feedback(self.ser2, 3, 3)
        
        # Publish combined feedback
        msg = Float64MultiArray()
        msg.data = self.current_degrees
        self.arm_pub.publish(msg)

    def _decode_feedback(self, ser: serial.Serial, offset: int, num_joints: int):
        """Decode feedback from a Pico."""
        size = 1 + num_joints * 2
        try:
            data = ser.read(size)
            if len(data) >= size and data[0] == self.HEADER_BYTE:
                for i in range(num_joints):
                    motor_num = (data[i * 2 + 1] & 0xC0) >> 6
                    if motor_num == i:
                        sign = 1 if (data[i * 2 + 1] & 0x30) else -1
                        value = ((data[i * 2 + 1] & 0x0F) << 8) | data[i * 2 + 2]
                        self.current_degrees[offset + i] = sign * float(value) / 5.0
            
            while ser.in_waiting > 0:
                ser.read()
                
        except serial.SerialException as e:
            self.get_logger().error(f'Feedback read error: {e}')

    def destroy_node(self):
        if self.ser1: self.ser1.close()
        if self.ser2: self.ser2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialJoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
