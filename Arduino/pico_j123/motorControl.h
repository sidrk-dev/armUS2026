#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include <SPI.h>
#include <mcp2515.h>
#include "CalPID.h"
#include "C620.h"

#ifndef PI
#define PI 3.14159265353238
#endif

class motorControl {
public:
  enum MOTOR_NUM {
    MOTOR1 = 0,
    MOTOR2 = 1,
    MOTOR3 = 2,
    MOTOR4 = 3
  };
  enum DIRECTION {
    FWD = 1,
    REV = -1
  };
  enum SPI_PIN {
    RX = 0,
    CS = 1,
    SCK = 2,
    TX = 3
  };
  enum UNIT {
    RPM = 0,
    RPS = 1,
    RAD_S = 2,
    DEG = 3,
    ROTATE = 4,
    RAD = 5
  };
  enum MODE {
    UNUSED = 0,
    SLEEP = 1,
    SPEED = 2,
    ANGLE = 3
  };
  enum PID_PARAM {
    KP = 0,
    KI = 1,
    KD = 2,
    MAX = 3
  };

  MODE mode[4] = {UNUSED, UNUSED, UNUSED, UNUSED};
  DIRECTION direction[4];
  uint8_t id[4];
  double pidParam[4][4];
  double pidParamAngle[4][4];
  double dt;
  uint8_t pidInterval = 25;
  double gearRatio[4] = {1, 1, 1, 1};
  double degErrorRange = 1.5;
  double targetRpm[4];
  double targetDeg[4];
  double rpm[4];
  double rps[4];
  double rad_s[4];
  double deg[4];
  double rotate[4];
  double rad[4];
  double current[4];
  double target_current[4];

  motorControl(const uint8_t CS_, const uint32_t SPI_CLOCK_, SPIClass * SPI_);
  void setMode(MOTOR_NUM motorNum, MODE mode_);
  void setMode(MODE motor1, MODE motor2, MODE motor3, MODE motor4);
  void setId(MOTOR_NUM motorNum, uint8_t value);
  void setId(uint8_t motor1, uint8_t motor2, uint8_t motor3, uint8_t motor4);
  void setDirection(MOTOR_NUM motorNum, DIRECTION value);
  void setDirection(DIRECTION motor1, DIRECTION motor2, DIRECTION motor3, DIRECTION motor4);
  void setPIDParameter(MOTOR_NUM motorNum, double kp_, double ki_, double kd_);
  void setPIDParameter(double kp_, double ki_, double kd_);
  void setPIDParameterAngle(MOTOR_NUM motorNum, double kp_, double ki_, double kd_);
  void setPIDParameterAngle(double kp_, double ki_, double kd_);
  void setMaxValue(MOTOR_NUM motorNum, double value);
  void setMaxValue(double motor1, double motor2, double motor3, double motor4);
  void setMaxValue(double value);
  void setMaxValueAngle(MOTOR_NUM motorNum, double value);
  void setMaxValueAngle(double motor1, double motor2, double motor3, double motor4);
  void setMaxValueAngle(double value);
  void setDELTA_T(double value);
  void updateMicros(uint32_t micros_);
  void setGearRatio(MOTOR_NUM motorNum, double value);
  void setGearRatio(double motor1, double motor2, double motor3, double motor4);
  void setGearRatio(double value);
  void setTarget(MOTOR_NUM motorNum, UNIT unit, double value);
  void setTarget(UNIT unit, double motor1, double motor2, double motor3, double motor4);
  void setTarget(UNIT unit, double value);
  void init();
  void read();
  uint8_t updateDriver();
  uint8_t updatePID();
  void update();
//private:
  struct can_frame readMsg_;
  struct can_frame sendMsg_;
  MCP2515 mcp2515_;
  CalPID calpid_[4] = { CalPID(), CalPID(), CalPID(), CalPID() };
  CalPID calpidAngle_[4] = { CalPID(), CalPID(), CalPID(), CalPID() };
  C620 c620_[4] = { C620(), C620(), C620(), C620() };
  const uint16_t can_id = 0x200;

  double _targetRpm[4];
  double _targetDeg[4];
  double _rpm[4];
  double _rps[4];
  double _rad_s[4];
  double _deg[4];
  double _rotate[4];
  double _rad[4];

  double degRaw[4];
  int64_t rotateCnt[4];
  double degRawPre[4];
  double degPre[4];

  uint64_t exeMicrosOverFlow = 0;
  uint64_t exeMicros;
  uint64_t exeMicrosPre = 0;
  
  uint8_t pidCnt = 0;

  void calAngle(MOTOR_NUM motorNum);
  double calAnglePID(MOTOR_NUM motorNum, double deg_);
};

#endif