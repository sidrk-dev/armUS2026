/**
 * Karura Arm - Joints 4-6 Motor Controller
 * 
 * Controls joints 4-6 of the Karura robotic arm.
 * Hardware: Raspberry Pi Pico + MCP2515 CAN + M3508/M2006 Motors
 * 
 * Serial Protocol (7 bytes):
 *   Byte 0: 0xFF (header)
 *   Bytes 1-2: Joint 4
 *   Bytes 3-4: Joint 5
 *   Bytes 5-6: Joint 6
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
// M3508/M2006 motor internal planetary gearbox ratio
#define M3508_INTERNAL_RATIO  36.0
#define M2006_INTERNAL_RATIO  36.0

// Joint 4: Standard M3508 joint
#define JOINT4_RATIO   (M3508_INTERNAL_RATIO * 1.0)

// Joint 5: Differential Wrist Motor A (approx -950 serial factor)
#define JOINT5_SERIAL_FACTOR  ((-20.0) * 35.0 * 19.0 / (5.0 * 14.0))

// Joint 6: Differential Wrist Motor B (approx 307 serial factor)
#define JOINT6_SERIAL_FACTOR  (35.0 * 30.0 * 19.0 / (13.0 * 5.0))

// Motor controller PID gear ratios (simplified)
#define JOINT4_MOTOR_RATIO  36.0
#define JOINT5_MOTOR_RATIO  36.0
#define JOINT6_MOTOR_RATIO  36.0

// =============================================================================
// CONFIGURATION
// =============================================================================
#define MOTOR_NUM 3
#define JOINT4 0
#define JOINT5 1
#define JOINT6 2

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
  motor[JOINT4].id = 1; motor[JOINT4].gearRatio = JOINT4_MOTOR_RATIO;
  motor[JOINT5].id = 2; motor[JOINT5].gearRatio = JOINT5_MOTOR_RATIO;
  motor[JOINT6].id = 3; motor[JOINT6].gearRatio = JOINT6_MOTOR_RATIO;
  
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
  
  const float factors[3] = { JOINT4_RATIO, JOINT5_SERIAL_FACTOR, JOINT6_SERIAL_FACTOR };
  
  for (int i = 0; i < 3; i++) {
    if (((encoded[i*2+1] & 0xC0) >> 6) == i) {
      int sign = (encoded[i*2+1] & 0x30) ? 1 : -1;
      int value = ((encoded[i*2+1] & 0x0F) << 8) | encoded[i*2+2];
      data[i] = sign * float(value) * (i==0 ? 1.0 : factors[i]); // J4 is direct, J5/6 use factors
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
    motor[JOINT4].target.angle = targets[0];
    motor[JOINT5].target.angle = targets[1];
    motor[JOINT6].target.angle = targets[2];
  }
}
