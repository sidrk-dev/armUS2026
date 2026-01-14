#!/usr/bin/env python3
"""
Pico Blink Test Sender

This script sends test serial packets to the Raspberry Pi Pico
to verify the pico_blink_test.ino sketch is working.

Usage:
    python pico_test_sender.py COM3        # Send single packet
    python pico_test_sender.py COM3 --loop  # Send packets continuously
"""

from serial import Serial, SerialException
import time
import sys
import argparse


def encode_joint(joint_index: int, angle_degrees: float) -> bytes:
    """
    Encode a joint angle into 2 bytes using the protocol:
    Byte 1: [motor_num(2b)][sign(2b)][value_high(4b)]
    Byte 2: [value_low(8b)]
    Value = angle * 5
    """
    value = int(angle_degrees * 5)
    sign = 1 if value >= 0 else 0
    value = abs(value)
    
    byte1 = (joint_index << 6) | (sign << 4) | ((value >> 8) & 0x0F)
    byte2 = value & 0xFF
    
    return bytes([byte1, byte2])


def create_packet(j1: float = 0, j2: float = 0, j3: float = 0) -> bytes:
    """
    Create a 7-byte packet for the Pico.
    Format: [0xFF, J1_H, J1_L, J2_H, J2_L, J3_H, J3_L]
    """
    packet = bytearray([0xFF])  # Header
    packet.extend(encode_joint(0, j1))
    packet.extend(encode_joint(1, j2))
    packet.extend(encode_joint(2, j3))
    return bytes(packet)


def send_packet(ser: Serial, j1: float, j2: float, j3: float):
    """Send a single packet and print debug info."""
    packet = create_packet(j1, j2, j3)
    print(f"Sending: {packet.hex(' ').upper()} (J1={j1}°, J2={j2}°, J3={j3}°)")
    ser.write(packet)
    
    # Wait for response
    time.sleep(0.1)
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"Response: {response.hex(' ').upper()}")


def main():
    parser = argparse.ArgumentParser(description="Send test packets to Pico")
    parser.add_argument("port", help="Serial port (e.g., COM3 or /dev/ttyACM0)")
    parser.add_argument("--loop", action="store_true", help="Send packets continuously")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()
    
    print(f"Opening {args.port} at {args.baud} baud...")
    
    try:
        ser = Serial(args.port, args.baud, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        print("Connected! Sending test packets...\n")
        
        count = 0
        while True:
            # Alternate between different angles to make the LED blink
            angle = 10 if count % 2 == 0 else -10
            send_packet(ser, angle, 0, 0)
            count += 1
            
            if not args.loop:
                # Send a few more packets to trigger the blink
                for i in range(10):
                    send_packet(ser, angle, 0, 0)
                    time.sleep(0.1)
                print("\n[Done] Check if the Pico LED blinked!")
                break
            
            time.sleep(0.5)
            
    except SerialException as e:
        print(f"Error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check that the Pico is connected")
        print("  2. Find the correct COM port in Device Manager")
        print("  3. Close any other programs using the port (Arduino IDE, etc.)")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n[Stopped]")
    finally:
        if 'ser' in locals():
            ser.close()


if __name__ == "__main__":
    main()
