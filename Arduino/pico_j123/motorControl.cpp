#include "motorControl.h"

motorControl::motorControl(const uint8_t CS_, const uint32_t SPI_CLOCK_, SPIClass * SPI_) 
  : mcp2515_(CS_, SPI_CLOCK_, SPI_) 
{
  sendMsg_.can_id = can_id;
  sendMsg_.can_dlc = 8;
  for (int cnt = 0; cnt < 8; cnt++) {
    sendMsg_.data[cnt] = 0;
  }
}

void motorControl::setMode(motorControl::MOTOR_NUM motorNum, motorControl::MODE mode_) {
  mode[motorNum] = mode_;
}

void motorControl::setMode(motorControl::MODE motor1, motorControl::MODE motor2, motorControl::MODE motor3, motorControl::MODE motor4) {
  setMode(MOTOR1, motor1);
  setMode(MOTOR2, motor2);
  setMode(MOTOR3, motor3);
  setMode(MOTOR4, motor4);
}

void motorControl::setId(motorControl::MOTOR_NUM motorNum, uint8_t value) {
  if (mode[motorNum] != UNUSED) id[motorNum] = value;
}

void motorControl::setId(uint8_t motor1, uint8_t motor2, uint8_t motor3, uint8_t motor4) {
  setId(MOTOR1, motor1);
  setId(MOTOR2, motor2);
  setId(MOTOR3, motor3);
  setId(MOTOR4, motor4);
}

void motorControl::setDirection(motorControl::MOTOR_NUM motorNum, motorControl::DIRECTION direction_) {
  if (mode[motorNum] != UNUSED) direction[motorNum] = direction_;
}

void motorControl::setDirection(motorControl::DIRECTION motor1, motorControl::DIRECTION motor2, motorControl::DIRECTION motor3, motorControl::DIRECTION motor4) {
  setDirection(MOTOR1, motor1);
  setDirection(MOTOR2, motor2);
  setDirection(MOTOR3, motor3);
  setDirection(MOTOR4, motor4);
}

void motorControl::setPIDParameter(motorControl::MOTOR_NUM motorNum, double kp_, double ki_, double kd_) {
  if ((mode[motorNum] == SPEED) | (mode[motorNum] == ANGLE)) {
    pidParam[KP][motorNum] = kp_;
    pidParam[KI][motorNum] = ki_;
    pidParam[KD][motorNum] = kd_;
  }
}

void motorControl::setPIDParameter(double kp_, double ki_, double kd_) {
  setPIDParameter(MOTOR1, kp_, ki_, kd_);
  setPIDParameter(MOTOR2, kp_, ki_, kd_);
  setPIDParameter(MOTOR3, kp_, ki_, kd_);
  setPIDParameter(MOTOR4, kp_, ki_, kd_);
}

void motorControl::setPIDParameterAngle(motorControl::MOTOR_NUM motorNum, double kp_, double ki_, double kd_) {
  if (mode[motorNum] == ANGLE) {
    pidParamAngle[KP][motorNum] = kp_;
    pidParamAngle[KI][motorNum] = ki_;
    pidParamAngle[KD][motorNum] = kd_;
  }
}

void motorControl::setPIDParameterAngle(double kp_, double ki_, double kd_) {
  setPIDParameterAngle(MOTOR1, kp_, ki_, kd_);
  setPIDParameterAngle(MOTOR2, kp_, ki_, kd_);
  setPIDParameterAngle(MOTOR3, kp_, ki_, kd_);
  setPIDParameterAngle(MOTOR4, kp_, ki_, kd_);
}

void motorControl::setMaxValue(MOTOR_NUM motorNum, double value) {
  if ((mode[motorNum] == SPEED) | (mode[motorNum] == ANGLE)) pidParam[MAX][motorNum] = value;
}

void motorControl::setMaxValue(double motor1, double motor2, double motor3, double motor4) {
  setMaxValue(MOTOR1, motor1);
  setMaxValue(MOTOR2, motor2);
  setMaxValue(MOTOR3, motor3);
  setMaxValue(MOTOR4, motor4);
}

void motorControl::setMaxValue(double value) {
  setMaxValue(MOTOR1, value);
  setMaxValue(MOTOR2, value);
  setMaxValue(MOTOR3, value);
  setMaxValue(MOTOR4, value);
}

void motorControl::setMaxValueAngle(MOTOR_NUM motorNum, double value) {
  if (mode[motorNum] == ANGLE) pidParamAngle[MAX][motorNum] = value;
}

void motorControl::setMaxValueAngle(double motor1, double motor2, double motor3, double motor4) {
  setMaxValueAngle(MOTOR1, motor1);
  setMaxValueAngle(MOTOR2, motor2);
  setMaxValueAngle(MOTOR3, motor3);
  setMaxValueAngle(MOTOR4, motor4);
}

void motorControl::setMaxValueAngle(double value) {
  setMaxValueAngle(MOTOR1, value);
  setMaxValueAngle(MOTOR2, value);
  setMaxValueAngle(MOTOR3, value);
  setMaxValueAngle(MOTOR4, value);
}

void motorControl::setGearRatio(MOTOR_NUM motorNum, double value) {
  if (mode[motorNum] != UNUSED) gearRatio[motorNum] = value;
}

void motorControl::setDELTA_T(double value) {
  dt = value;
}

void motorControl::updateMicros(uint32_t micros_) {
  if (micros_ < exeMicros) {
    exeMicrosOverFlow += 1;
  }
  exeMicros = 0x00 | exeMicrosOverFlow << 32 | micros_;
  dt = double(double(exeMicros - exeMicrosPre) / 1000000.0);
  for (uint8_t cnt = 0; cnt < 4; cnt++) {
    if ((mode[cnt] == SPEED) | (mode[cnt] == ANGLE)) {
      calpid_[cnt].setDELTA_T(dt * pidInterval);
    }
    if (mode[cnt] == ANGLE) {
      calpidAngle_[cnt].setDELTA_T(dt * pidInterval);
    }
  }
  exeMicrosPre = exeMicros;
}

void motorControl::setGearRatio(double motor1, double motor2, double motor3, double motor4) {
  setGearRatio(MOTOR1, motor1);
  setGearRatio(MOTOR2, motor2);
  setGearRatio(MOTOR3, motor3);
  setGearRatio(MOTOR4, motor4);
}

void motorControl::setGearRatio(double value) {
  setGearRatio(MOTOR1, value);
  setGearRatio(MOTOR2, value);
  setGearRatio(MOTOR3, value);
  setGearRatio(MOTOR4, value);
}

void motorControl::setTarget(MOTOR_NUM motorNum, UNIT unit, double value) {
  if (unit == RPM && mode[motorNum] == SPEED) {
    targetRpm[motorNum] = direction[motorNum] * value;
    _targetRpm[motorNum] = targetRpm[motorNum] * gearRatio[motorNum];
  }
  else if (unit == RPS && mode[motorNum] == SPEED) {
    targetRpm[motorNum] = direction[motorNum] * value * 60.0;
    _targetRpm[motorNum] = targetRpm[motorNum] * gearRatio[motorNum];
  }
  else if (unit == RAD_S && mode[motorNum] == SPEED) {
    targetRpm[motorNum] = direction[motorNum] * value * 30.0 / PI;
    _targetRpm[motorNum] = targetRpm[motorNum] * gearRatio[motorNum];
  }
  else if (unit == DEG && mode[motorNum] == ANGLE) {
    targetDeg[motorNum] = direction[motorNum] * value;
    _targetRpm[motorNum] = targetRpm[motorNum] * gearRatio[motorNum];
  }
  else if (unit == RAD && mode[motorNum] == ANGLE) {
    targetDeg[motorNum] = direction[motorNum] * value * 180.0 / PI;
    _targetRpm[motorNum] = targetRpm[motorNum] * gearRatio[motorNum];
  }
}

void motorControl::setTarget(motorControl::UNIT unit, double motor1, double motor2, double motor3, double motor4) {
  setTarget(MOTOR1, unit, motor1);
  setTarget(MOTOR2, unit, motor2);
  setTarget(MOTOR3, unit, motor3);
  setTarget(MOTOR4, unit, motor4);
}

void motorControl::setTarget(motorControl::UNIT unit, double value) {
  setTarget(MOTOR1, unit, value);
  setTarget(MOTOR2, unit, value);
  setTarget(MOTOR3, unit, value);
  setTarget(MOTOR4, unit, value);
}

void motorControl::init() {
  mcp2515_.reset();
  mcp2515_.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515_.setNormalMode();
  sendMsg_.can_id = can_id;
  sendMsg_.can_dlc = 8;
  for (int cnt = 0; cnt < 8; cnt++) {
    sendMsg_.data[cnt] = 0;
  }
  for (uint8_t cnt = 0; cnt < 4; cnt++) {
    rotateCnt[cnt] = 0;
    if ((mode[cnt] == SPEED) | (mode[cnt] == ANGLE)) {
      calpid_[cnt].setParameter(pidParam[KP][cnt], pidParam[KI][cnt], pidParam[KD][cnt]);
      calpid_[cnt].setDELTA_T(dt * 5.0);
      calpid_[cnt].setMaxValue(pidParam[MAX][cnt]);
      c620_[cnt].setMCP2515(&mcp2515_);
      c620_[cnt].setCalPID(&calpid_[cnt]);
      c620_[cnt].setId(id[cnt]);
      c620_[cnt].setCan(&sendMsg_);
    }
    if (mode[cnt] == ANGLE) {
      calpidAngle_[cnt].setParameter(pidParamAngle[KP][cnt], pidParamAngle[KI][cnt], pidParamAngle[KD][cnt]);
      calpidAngle_[cnt].setDELTA_T(dt * pidInterval);
      calpidAngle_[cnt].setMaxValue(pidParamAngle[MAX][cnt]);
    }
  }
}

void motorControl::read() {
  calAngle(MOTOR1);
  calAngle(MOTOR2);
  calAngle(MOTOR3);
  calAngle(MOTOR4);
  for (uint8_t cnt = 0; cnt < 4; cnt++) {
    _rpm[cnt] = c620_[cnt].readRPM();
    _rps[cnt] = c620_[cnt].readRPM() / 60.0;
    _rad_s[cnt] = c620_[cnt].readRad_s();
    _rotate[cnt] = _deg[cnt] / 360.0;
    _rad[cnt] = _deg[cnt] * PI / 180.0;
    rpm[cnt] = _rpm[cnt] / gearRatio[cnt] * direction[cnt];
    rps[cnt] = _rps[cnt] / gearRatio[cnt] * direction[cnt];
    rad_s[cnt] = _rad_s[cnt] / gearRatio[cnt] * direction[cnt];
    deg[cnt] = _deg[cnt] / gearRatio[cnt] * direction[cnt];
    rotate[cnt] = _rotate[cnt] / gearRatio[cnt] * direction[cnt];
    rad[cnt] = _rad[cnt] / gearRatio[cnt] * direction[cnt];
    current[cnt] = (double)c620_[cnt].readCurrent();
  }
}

uint8_t motorControl::updateDriver() {
  uint8_t status = mcp2515_.readMessage(&readMsg_);     
  read();                                                                                                                                                                                                                                                                                                                                                                                                                                                     
  if (status == MCP2515::ERROR_OK) {
    uint8_t readId = readMsg_.can_id - 0x200;
    c620_[readId - 1].setCANData(&readMsg_);
    c620_[readId - 1].update();
  }
  return status;
}

uint8_t motorControl::updatePID() {
  if (mode[MOTOR1] == ANGLE) _targetRpm[MOTOR1] = calAnglePID(MOTOR1, _deg[MOTOR1]);
  if (mode[MOTOR2] == ANGLE) _targetRpm[MOTOR2] = calAnglePID(MOTOR2, _deg[MOTOR2]);
  if (mode[MOTOR3] == ANGLE) _targetRpm[MOTOR3] = calAnglePID(MOTOR3, _deg[MOTOR3]);
  if (mode[MOTOR4] == ANGLE) _targetRpm[MOTOR4] = calAnglePID(MOTOR4, _deg[MOTOR4]);
  for (uint8_t cnt = 0; cnt < 4; cnt++) target_current[cnt] = c620_[cnt].updatePID(int(_targetRpm[cnt]));
  uint8_t status = mcp2515_.sendMessage(&sendMsg_);
  return status;
}

void motorControl::update() {
  pidCnt++;
  updateDriver();
  if (pidCnt >= pidInterval) {
    updatePID();
    pidCnt = 0;
  }
}

void motorControl::calAngle(MOTOR_NUM motorNum) {
  degRaw[motorNum] = c620_[motorNum].readAngle();
  double degDiff = degRaw[motorNum] - degRawPre[motorNum];
  double errLim = abs(_rps[motorNum] * dt * 360.0 * degErrorRange);
  if ((degDiff < 0) && (_rpm[motorNum] > 0)) {
    if ((360 - abs(degDiff)) < errLim) {
      rotateCnt[motorNum]++;
    }
  }
  if ((degDiff > 0) && (_rpm[motorNum] < 0)) {
    if ((360 - abs(degDiff)) < errLim) {
      rotateCnt[motorNum]--;
    }
  }
  _deg[motorNum] = degRaw[motorNum] + double(rotateCnt[motorNum] * 360);
  degRawPre[motorNum] = degRaw[motorNum];
}

double motorControl::calAnglePID(MOTOR_NUM motorNum, double deg_) {
  double degErr = _targetDeg[motorNum] - deg_;
  return calpidAngle_[motorNum].calPID(degErr);
}