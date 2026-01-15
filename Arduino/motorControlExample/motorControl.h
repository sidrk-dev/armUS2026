#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <mcp2515.h>

// C620 feedback frequency (125Hz as per datasheet)
#define C620_FEEDBACK_125HZ 125.0f

// CAN message IDs for commanding motors
#define SENDMESSAGE_1_TO_4_ID 0x200
#define SENDMESSAGE_5_TO_8_ID 0x1FF

// Control modes
enum class MODE : uint8_t {
    SPEED = 0,
    ANGLE = 1,
    CURRENT = 2,
    SLEEP = 3
};

// Motor direction
enum class DIRECTION : uint8_t {
    FWD = 0,
    REV = 1
};

// PID parameters structure - RENAMED to avoid conflict
typedef struct {
    float kp;
    float ki;
    float kd;
    float limit;
} pid_params_t;

// Motor target values
typedef struct {
    float speed;
    float angle;
    float current;
} target_t;

// Motor full structure
typedef struct {
    uint8_t id;
    MODE mode;
    float gearRatio;
    DIRECTION direction;
    pid_params_t anglePid;    // Updated type name
    pid_params_t speedPid;    // Updated type name
    pid_params_t currentPid;  // Updated type name
    target_t target;
    float actualSpeed;
    float actualAngle;
    float actualCurrent;
    int16_t temperature;
} motor_t;

// PID calculation intervals
typedef struct {
    uint8_t angleInterval;
    uint8_t speedInterval;
    uint8_t currentInterval;
} pidInterval_t;

class motorControl {
private:
    motor_t* motors;
    uint8_t motorCount;
    pidInterval_t* pidIntervals;
    uint32_t lastUpdateTime;
    uint16_t commandCounter;
    
    // Internal PID calculation functions
    float calculatePID(pid_params_t* pid, float error, float dt);
    void updateMotorCommand(struct can_frame* sendMsg, uint8_t motorIndex);
    void parseMotorFeedback(struct can_frame* readMsg);

public:
    motorControl();
    
    // Initialize with motor array and count
    void setMotor(motor_t* motorArray, uint8_t count);
    
    // Set PID update intervals
    void setPidInterval(pidInterval_t* intervals);
    
    // Initialize the controller
    void init();
    
    // Main refresh function - processes feedback and updates commands
    void refresh(uint32_t currentTime, struct can_frame* sendMsg, struct can_frame* readMsg);
    
    // Wake up/sleep all motors
    void wakeAll();
    void sleepAll();
    
    // Check if all motors are responding
    bool isConnected();
};

#endif // MOTOR_CONTROL_H
