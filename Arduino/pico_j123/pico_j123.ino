/**
 * Karura Arm - Joints 1-3 Motor Controller
 * 
 * Controls joints 1-3 of the Karura robotic arm.
 * Hardware: Raspberry Pi Pico + MCP2515 CAN + M3508 Motors (x3)
 * 
 * Serial Protocol (7 bytes):
 *   Byte 0: 0xFF (header)
 *   Bytes 1-2: Joint 1 [motor_num(2b)|sign(2b)|value_high(4b)][value_low(8b)]
 *   Bytes 3-4: Joint 2
 *   Bytes 5-6: Joint 3
 */

#include <SPI.h>
#include "mcp2515.h"
#include "motorControl.h"
#include "RPi_Pico_TimerInterrupt.h"

// =============================================================================
// PIN DEFINITIONS
// =============================================================================
#define RX_PIN  4
#define CS_PIN  5
#define SCK_PIN 6
#define TX_PIN  7

// =============================================================================
// MOTOR GEAR RATIOS
// =============================================================================
#define M3508_INTERNAL_RATIO  36.0

// External ratios (Joint = Motor * Ratio)
#define JOINT1_RATIO  (M3508_INTERNAL_RATIO * 1.0)
#define JOINT2_RATIO  (M3508_INTERNAL_RATIO * 1.0)
#define JOINT3_RATIO  (M3508_INTERNAL_RATIO * 1.0)

// =============================================================================
// CONFIGURATION
// =============================================================================
#define MOTOR_NUM 3
#define JOINT1 0
#define JOINT2 1
#define JOINT3 2

#define DT_RECV_DATA (1.0f / (MOTOR_NUM * C620_FEEDBACK_125HZ))
#define DT_SEND_DATA (1.0f / C620_FEEDBACK_125HZ)

#define SERIAL_HEADER      0xFF
#define SERIAL_BUFFER_SIZE 7
#define ANGLE_RESOLUTION   5.0  // 0.2 degree resolution

// =============================================================================
// GLOBALS
// =============================================================================
float targets[3] = {0, 0, 0};
float current_degrees[3] = {0, 0, 0};
uint8_t send_buffer[SERIAL_BUFFER_SIZE];

MCP2515 mcp(CS_PIN, 8000000, &SPI);
motorControl mc;
RPI_PICO_Timer recvTimer(0);
RPI_PICO_Timer sendTimer(1);

struct { bool isWakedUp = false; } sendData;
struct { uint8_t driver = 0; } disconnectedCnt;
struct { bool recvMotor = false; bool sendMotor = false; } flag;

motor_t motor[MOTOR_NUM];
pidInterval_t pidInterval = { 8, 4, 2 };

struct can_frame sendMsg[2] = {};
struct can_frame readMsg = {};

// =============================================================================
// FUNCTIONS
// =============================================================================
void SPIinit() { SPI.setRX(RX_PIN); SPI.setCS(CS_PIN); SPI.setSCK(SCK_PIN); SPI.setTX(TX_PIN); SPI.begin(); }
void MCPinit() { mcp.reset(); mcp.setBitrate(CAN_1000KBPS, MCP_8MHZ); mcp.setNormalMode(); }
bool recvMotorFlag(struct repeating_timer *t) { flag.recvMotor = true; return true; }
bool sendMotorFlag(struct repeating_timer *t) { flag.sendMotor = true; return true; }

void setMotorParam() {
  motor[JOINT1].id = 1; motor[JOINT1].gearRatio = JOINT1_RATIO;
  motor[JOINT2].id = 2; motor[JOINT2].gearRatio = JOINT2_RATIO;
  motor[JOINT3].id = 3; motor[JOINT3].gearRatio = JOINT3_RATIO;
  
  for(int i=0; i<3; i++) {
    motor[i].mode = MODE::ANGLE;
    motor[i].direction = DIRECTION::FWD;
    motor[i].anglePid = { 1.0, 0.0, 0.0, 100.0 };
    motor[i].speedPid = { 0.2, 1.0, 0.0, 10.0 };
    motor[i].currentPid = { 1.0, 1.0, 0.0, 10.0 };
  }
}

void wakeUp() {
  sendData.isWakedUp = true;
  for(int i=0; i<MOTOR_NUM; i++) {
    motor[i].mode = MODE::ANGLE;
    motor[i].target = { 0.0, 0.0, 0.0 };
  }
}

void recvMotor() {
  if (mcp.readMessage(&readMsg) == 0) {
    disconnectedCnt.driver = 0;
    mc.refresh(micros(), sendMsg, &readMsg);
  } else if (++disconnectedCnt.driver > 100) {
    for(int i=0; i<MOTOR_NUM; i++) motor[i].mode = MODE::SLEEP;
  }
  flag.recvMotor = false;
}

void sendMotor() {
  mcp.sendMessage(&sendMsg[0]);
  flag.sendMotor = false;
}

void decode(uint8_t* encoded, float* data) {
  if (encoded[0] != SERIAL_HEADER) return;
  for (int i = 0; i < 3; i++) {
    if (((encoded[i*2+1] & 0xC0) >> 6) == i) {
      int sign = (encoded[i*2+1] & 0x30) ? 1 : -1;
      int value = ((encoded[i*2+1] & 0x0F) << 8) | encoded[i*2+2];
      data[i] = sign * float(value) / ANGLE_RESOLUTION;
    }
  }
}

void encode(float* data, uint8_t* encoded) {
  encoded[0] = SERIAL_HEADER;
  for (int i = 0; i < 3; i++) {
    int val = int(data[i] * ANGLE_RESOLUTION);
    encoded[i*2+1] = (i<<6) | ((val>=0?1:0)<<5) | ((abs(val)>>8)&0x1F);
    encoded[i*2+2] = abs(val) & 0xFF;
  }
}

void setup() {
  Serial.begin(115200);
  SPIinit(); MCPinit(); setMotorParam();
  mc.setMotor(motor, MOTOR_NUM);
  mc.setPidInterval(&pidInterval);
  mc.init();
  recvTimer.attachInterruptInterval((uint32_t)(DT_RECV_DATA * 1000000), recvMotorFlag);
  sendTimer.attachInterruptInterval((uint32_t)(DT_SEND_DATA * 1000000), sendMotorFlag);
  sendMsg[0].can_id = SENDMESSAGE_1_TO_4_ID; sendMsg[0].can_dlc = 8;
  pinMode(LED_BUILTIN, OUTPUT);
  wakeUp();
}

void loop() {
  if (flag.recvMotor) recvMotor();
  if (flag.sendMotor) sendMotor();
  
  for(int i=0; i<3; i++) current_degrees[i] = mc.readAngle(i);
  
  if (Serial.available() > 0) {
    uint8_t buf[SERIAL_BUFFER_SIZE];
    int idx = 0;
    delayMicroseconds(1000);
    while (Serial.available() && idx < SERIAL_BUFFER_SIZE) buf[idx++] = Serial.read();
    
    encode(current_degrees, send_buffer);
    Serial.write(send_buffer, SERIAL_BUFFER_SIZE);
    
    decode(buf, targets);
    motor[JOINT1].target.angle = targets[0];
    motor[JOINT2].target.angle = targets[1];
    motor[JOINT3].target.angle = targets[2];
  }
}
