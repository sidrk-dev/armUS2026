#include "motorControl.h"

// Constructor
motorControl::motorControl() : motors(nullptr), motorCount(0), pidIntervals(nullptr), 
                               lastUpdateTime(0), commandCounter(0) {}

// Initialize motor array
void motorControl::setMotor(motor_t* motorArray, uint8_t count) {
    motors = motorArray;
    motorCount = count;
}

// Set PID intervals (kept for compatibility but unused)
void motorControl::setPidInterval(pidInterval_t* intervals) {
    pidIntervals = intervals;
}

// Initialize controller
void motorControl::init() {
    lastUpdateTime = micros();
    commandCounter = 0;
}

// Simple proportional scaling from speed to current
// Tune this factor based on your motor/load
// For M2006: ~30-50 works well (current = speed * SCALE)
#define SPEED_TO_CURRENT_SCALE 35.0f

// Parse incoming motor feedback from CAN frame
void motorControl::parseMotorFeedback(struct can_frame* readMsg) {
    uint8_t motorId = readMsg->can_id - 0x201; // Feedback ID = 0x200 + motor_id + 1
    
    if (motorId >= motorCount) return;
    
    // Extract data from C620 feedback frame
    // Byte 0-1: Rotor angle (0-8191 per revolution)
    uint16_t rawAngle = (readMsg->data[0] << 8) | readMsg->data[1];
    motors[motorId].actualAngle = (float)rawAngle * 360.0 / 8192.0;
    
    // Byte 2-3: Rotor speed (RPM)
    int16_t rawSpeed = (readMsg->data[2] << 8) | readMsg->data[3];
    motors[motorId].actualSpeed = (float)rawSpeed;
    
    // Byte 4-5: Torque current (mA)
    int16_t rawCurrent = (readMsg->data[4] << 8) | readMsg->data[5];
    motors[motorId].actualCurrent = (float)rawCurrent / 1000.0; // Convert to A
    
    // Byte 6: Temperature (°C)
    motors[motorId].temperature = readMsg->data[6];
}

// Update motor command in CAN frame - DIRECT SPEED CONTROL
void motorControl::updateMotorCommand(struct can_frame* sendMsg, uint8_t motorIndex) {
    if (motorIndex >= motorCount) return;
    
    // Determine which CAN frame (0x200 or 0x1FF)
    uint8_t frameIndex = (motors[motorIndex].id - 1) / 4; // 0 for IDs 1-4, 1 for IDs 5-8
    uint8_t motorOffset = (motors[motorIndex].id - 1) % 4; // 0-3 within frame
    
    float currentCmd = 0;
    
    // Simple direct speed control
    if (motors[motorIndex].mode == MODE::SPEED) {
        // Direct proportional mapping: current = speed * scale
        currentCmd = motors[motorIndex].target.speed * SPEED_TO_CURRENT_SCALE;
    }
    // If in sleep mode, currentCmd stays 0
    
    // Apply direction
    if (motors[motorIndex].direction == DIRECTION::REV) {
        currentCmd = -currentCmd;
    }
    
    // Convert to C620 format (-16384 to 16384 for -20A to 20A)
    int16_t currentValue = (int16_t)(currentCmd * 819.2); // 16384/20 = 819.2
    
    // Clamp to valid range
    if (currentValue > 16384) currentValue = 16384;
    if (currentValue < -16384) currentValue = -16384;
    
    // Pack into CAN frame (2 bytes per motor)
    sendMsg[frameIndex].data[motorOffset * 2] = (currentValue >> 8) & 0xFF;
    sendMsg[frameIndex].data[motorOffset * 2 + 1] = currentValue & 0xFF;
}

// Main refresh function
void motorControl::refresh(uint32_t currentTime, struct can_frame* sendMsg, struct can_frame* readMsg) {
    // Parse incoming feedback if available
    if (readMsg && readMsg->can_id >= 0x201 && readMsg->can_id <= 0x208) {
        parseMotorFeedback(readMsg);
    }
    
    // Clear command frames
    for(uint8_t i = 0; i < 8; i++) {
        sendMsg[0].data[i] = 0;
        sendMsg[1].data[i] = 0;
    }
    
    // Update all motor commands
    for (uint8_t i = 0; i < motorCount; i++) {
        if (motors[i].mode != MODE::SLEEP) {
            updateMotorCommand(sendMsg, i);
        }
    }
}

// Wake all motors
void motorControl::wakeAll() {
    for (uint8_t i = 0; i < motorCount; i++) {
        motors[i].mode = MODE::SPEED;
    }
}

// Sleep all motors
void motorControl::sleepAll() {
    for (uint8_t i = 0; i < motorCount; i++) {
        motors[i].mode = MODE::SLEEP;
        motors[i].target.speed = 0;
    }
}

// Check connection status
bool motorControl::isConnected() {
    return true; // Placeholder
}
