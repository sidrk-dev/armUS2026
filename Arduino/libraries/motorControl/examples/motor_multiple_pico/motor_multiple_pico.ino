#include <SPI.h>
#include "mcp2515.h"
#include "motorControl.h"
#include "RPi_Pico_TimerInterrupt.h"

#define MOTOR1 0
#define MOTOR2 1

#define RX_PIN 4
#define CS_PIN 5
#define SCK_PIN 6
#define TX_PIN 7

#define DT 0.0002 //You shouldn't change.

MCP2515 mcp(CS_PIN, 8000000, &SPI);
motorControl mc;
RPI_PICO_Timer timer(0);

M3508::motor_t motor[2]; //Set the number of array elements to match the number of motors used.
M3508::pidInterval_t pidInterval = { 8, 4, 2 };  //You shouldn't change.

struct can_frame sendMsg[2] = {}; //ID1~4, ID5~8
struct can_frame readMsg = {}; //The arguments are the motorInfo array and its number of elements.

void SPIinit();
void MCPinit();
void setMotorParam();
bool refresh(struct repeating_timer *t); //You must use arguments of this form.

void setup() {
    Serial.begin(115200);
    SPIinit();
    MCPinit();
    setMotorParam();
    mc.setMotor(motor, 2); //The arguments are the motorInfo array and the number of motors.
    mc.setPidInterval(pidInterval);
    mc.init();
    timer.attachInterruptInterval((uint32_t)(DT * 1000000), refresh); //The interval time argument is in integer (us).
    //You mustn't change here;
    sendMsg[0].can_id = SENDMESSAGE_1_TO_4_ID;
    sendMsg[0].can_dlc = 8;
    sendMsg[1].can_id = SENDMESSAGE_5_TO_8_ID;
    sendMsg[1].can_dlc = 8;
}

void loop() {
    motor[MOTOR1].target.speed = 60; //The motor rod rotate in 60 rpm.
    double time = (double)millis() / 1000.0;
    motor[MOTOR2].target.angle = 180 * sin(time); //The motor rod rotate in -180~180 deg in 6.28 sec.
    delay(100);
}

void SPIinit() {
  SPI.setRX(RX_PIN);
  SPI.setCS(CS_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setTX(TX_PIN);
  SPI.begin();
}

void MCPinit() {
  mcp.reset();
  mcp.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp.setNormalMode();
}

// You can change here.
void setMotorParam() {
    motor[MOTOR1].id = 1;
    motor[MOTOR1].mode = M3508::SPEED;
    motor[MOTOR1].gearRatio = 19.0;
    motor[MOTOR1].direction = M3508::FWD;
    motor[MOTOR1].anglePid = { 1.0, 0.5, 0.0, 200.0 };
    motor[MOTOR1].speedPid = { 0.08, 0.2, 0.0005, 10.0 };
    motor[MOTOR1].currentPid = { 1.2, 1.5, 0.0001, 10.0 };

    motor[MOTOR2].id = 5;
    motor[MOTOR2].mode = M3508::ANGLE;
    motor[MOTOR2].gearRatio = 19.0;
    motor[MOTOR2].direction = M3508::FWD;
    motor[MOTOR2].anglePid = { 1.0, 0.5, 0.0, 200.0 };
    motor[MOTOR2].speedPid = { 0.08, 0.2, 0.0005, 10.0 };
    motor[MOTOR2].currentPid = { 1.2, 1.5, 0.0001, 10.0 };
}

bool refresh(struct repeating_timer *t) {
    uint8_t status = mcp.readMessage(&readMsg);
    if (status == 0) mc.refresh(micros(), sendMsg, &readMsg);
    if (mc.needToSend[0]) mcp.sendMessage(&sendMsg[0]);
    if (mc.needToSend[1]) mcp.sendMessage(&sendMsg[1]);
    return true;
}