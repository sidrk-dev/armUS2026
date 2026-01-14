#ifndef C620_H
#define C620_H

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>
#include "CalPID.h"
//#include <MsTimer2.h>

#define GEAR_RATIO 38 // Motor reduction ratio 19 * Bevel reduction ratio 2
#define STOP_THRESHOLD 5 // Considered stopped if under 5 RPM. Used for start-up control.
#define START_CURRENT 0.5 // Apply 0.5A (500mA) in the opposite direction at startup.

class C620
{
private:
    MCP2515 *mcp2515_;
    CalPID *pid_;
    
    float angle_;
    int rpm_;
    float current_;
    float target_rpm_;
    int id_;
    int start_counter_;
    bool data_receive_error_;
    bool motor_stopping_;
    int connectBytes(uint8_t upper, uint8_t lower);
    void setCurrent(can_frame *frame, int motor_id, float current);
    void divideBytes(int16_t val, uint8_t *data_array_ptr);
    float angle(can_frame *frame);
    int RPM(can_frame *frame);
    float current(can_frame *frame);
public:
    struct can_frame readMsg_;
    struct can_frame* sendMsg_;
    C620(MCP2515* mcp2515, CalPID* pid, can_frame* sendMsg, int id);
    C620();
    void setMCP2515(MCP2515* mcp2515);
    void setCalPID(CalPID* pid);
    void setCan(can_frame* sendMsg);
    void setId(int id);
    int readRPM();
    float readRad_s();
    float readAngle();
    float readCurrent();

    // Execute one of the following PID calculation functions at the cycle specified when creating the CalPID instance.
    float updatePID(int target_rpm);
    void updatePID_rad(float target_rad_s);
    
    void transfer();
    void recoverError();
    void stopMotor();
    void setCANData(can_frame* frame);

    // When updating only the sensor reading data without outputting to the motor.
    void update();
};

#endif