#include <SPI.h>
#include "mcp2515.h"
#include "motorControl.h"
#include "RPi_Pico_TimerInterrupt.h"

//pressing the boot on the pico should make the motor spin.
// MAKE SURE THERE IS A COMMON GROUND BETWEEN PICO GROUND, MAIN POWER GROUND, AND MCP2515 TJA1050 thingy
#define RX_PIN 4 //gpio4 
#define CS_PIN 5 //gpio5 CS
#define SCK_PIN 6 //gpio6
#define TX_PIN 7 //gpio7

#define M2006_GEAR_RATIO 36.0
#define PLANETARY_GEAR_RATIO 3.5
#define WORM_GEAR_RATIO 20.0

#define MOTOR_NUM 2
#define WHEEL 0
#define STEER 1

#define DT_RECV_DATA 1.0f / (MOTOR_NUM * C620_FEEDBACK_125HZ)
#define DT_SEND_DATA 1.0f / C620_FEEDBACK_125HZ

MCP2515 mcp(CS_PIN, 8000000, &SPI);
motorControl mc;
RPI_PICO_Timer recvTimer(0);
RPI_PICO_Timer sendTimer(1);

struct {
  bool isWakedUp;
  float wheel, steer;
  bool needInitData;
} sendData;

struct {
  uint8_t host, driver;
} disconnectedCnt;

struct {
  bool recvMotor = false;
  bool sendMotor = false;
  bool manualMode = false;
} flag;

motor_t motor[MOTOR_NUM];
pidInterval_t pidInterval = { 8, 4, 2 };

struct can_frame sendMsg[2] = {};
struct can_frame readMsg = {};

void SPIinit();
void MCPinit();
void setMotorParam();
void wakeUp(bool isResetTarget);
void sleep();
void recvMotor();
void sendMotor();
bool recvMotorFlag(struct repeating_timer *t);
bool sendMotorFlag(struct repeating_timer *t);

void setup() {
  Serial.begin(115200);
  SPIinit();
  MCPinit();
  setMotorParam();
  mc.setMotor(motor, MOTOR_NUM);
  mc.setPidInterval(&pidInterval);
  mc.init();
  recvTimer.attachInterruptInterval((uint32_t)(DT_RECV_DATA * 1000000), recvMotorFlag);
  sendTimer.attachInterruptInterval((uint32_t)(DT_SEND_DATA * 1000000), sendMotorFlag);
  sendMsg[0].can_id = SENDMESSAGE_1_TO_4_ID;
  sendMsg[0].can_dlc = 8;
  sendMsg[1].can_id = SENDMESSAGE_5_TO_8_ID;
  sendMsg[1].can_dlc = 8;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(22, INPUT_PULLUP);
  wakeUp(true);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  if (!digitalRead(22)) {
    motor[WHEEL].mode = MODE::SPEED;
    motor[WHEEL].target.speed = 10.0;

  }
  else if (BOOTSEL) {
    motor[WHEEL].mode = MODE::SPEED;
    motor[WHEEL].target.speed = -10.0;
  }
  else {
    motor[WHEEL].target.speed = 0.0;
  }
  if (flag.recvMotor) recvMotor();
  if (flag.sendMotor) sendMotor();
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

void setMotorParam() {
  motor[WHEEL].id = 1;
  motor[WHEEL].mode = MODE::SPEED;
  motor[WHEEL].gearRatio = M2006_GEAR_RATIO;
  motor[WHEEL].direction = DIRECTION::REV;
  motor[WHEEL].anglePid = { 0.0, 0.0, 0.0, 0.0 };
  motor[WHEEL].speedPid = { 0.2, 1.0, 0.000, 10.0 };
  motor[WHEEL].currentPid = { 1.0, 1.0, 0.00, 10.0 };
}

void wakeUp(bool isResetTarget) {
  sendData.isWakedUp = true;
  motor[WHEEL].mode = MODE::SPEED;
  if (isResetTarget) {
    motor[WHEEL].target = { 0.0, 0.0, 0.0 };
  }
}

void sleep() {
  sendData.isWakedUp = false;
  motor[WHEEL].mode = MODE::SLEEP;
  motor[WHEEL].target = { 0.0, 0.0, 0.0 };
}

void recvMotor() {
  uint8_t status = mcp.readMessage(&readMsg);
  if (status == 0) {
    disconnectedCnt.driver = 0;
    mc.refresh(micros(), sendMsg, &readMsg);
  }
  else {
    disconnectedCnt.driver++;
    if (disconnectedCnt.driver > 100) sleep();
  }
  flag.recvMotor = false;
}

void sendMotor() {
  mcp.sendMessage(&sendMsg[0]);
  mcp.sendMessage(&sendMsg[1]);
  flag.sendMotor = false;
}

bool recvMotorFlag(struct repeating_timer *t) {
  flag.recvMotor = true;
  return true;
}

bool sendMotorFlag(struct repeating_timer *t) {
  flag.sendMotor = true;
  return true;
}