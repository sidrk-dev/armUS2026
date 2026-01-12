#include <SPI.h>
#include "mcp2515.h"
#include "M3508.h"
#include "RPi_Pico_TimerInterrupt.h"

#define RX_PIN 4
#define CS_PIN 5
#define SCK_PIN 6
#define TX_PIN 7

#define DT 0.0002 //You shouldn't change.

MCP2515 mcp(CS_PIN, 8000000, &SPI);
M3508 m3508;
RPI_PICO_Timer timer(0);

M3508::motor_t motor;
M3508::pidInterval_t pidInterval = { 8, 4, 2 }; //You shouldn't change.

struct can_frame sendMsg = {};
struct can_frame readMsg = {};

void SPIinit();
void MCPinit();
void setMotorParam();
bool refresh(struct repeating_timer *t); //You must use arguments of this form.

void setup() {
  Serial.begin(115200);
  SPIinit();
  MCPinit();
  setMotorParam();
  m3508.setMotor(&motor);
  m3508.setPidInterval(pidInterval);
  m3508.init();
  timer.attachInterruptInterval((uint32_t)(DT * 1000000), refresh); //The interval time argument is in integer (us).
  //You mustn't change here;
  sendMsg.can_id = 0x200;
  sendMsg.can_dlc = 8;
}

void loop() {
  double time = (double)millis() / 1000.0;
  motor.target.angle = 180 * sin(time); //The motor rod rotate in -180~180 deg in 6.28 sec.

  Serial.print(motor.target.angle);
  Serial.print(", ");
  Serial.print(m3508.readAngle());
  Serial.println("");
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
  motor.id = 1;
  motor.mode = M3508::ANGLE;
  motor.gearRatio = 19.0;
  motor.direction = M3508::FWD;
  motor.anglePid = { 1.0, 0.5, 0.0, 200.0 };
  motor.speedPid = { 0.08, 0.1, 0.0005, 10.0 };
  motor.currentPid = { 1.6, 2.0, 0.0005, 10.0 };
  motor.target.angle = 30.0;
}

bool refresh(struct repeating_timer *t) {
  uint8_t status = mcp.readMessage(&readMsg);
  if (status == 0) m3508.refresh(micros(), &sendMsg, &readMsg);
  if (m3508.needToSend) mcp.sendMessage(&sendMsg);
  return true;
}