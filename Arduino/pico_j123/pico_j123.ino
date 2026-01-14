#include <SPI.h>
#include "motorControl.h"
#include <Arduino.h>

// =============================================================================
// PIN DEFINITIONS
// =============================================================================
#define RX_PIN  4
#define CS_PIN  5
#define SCK_PIN 6
#define TX_PIN  7

// =============================================================================
// SERIAL PROTOCOL CONFIG
// =============================================================================
#define SERIAL_HEADER      0xFF
#define SERIAL_BUFFER_SIZE 7
#define ANGLE_RESOLUTION   5.0  // 0.2 degree resolution

// =============================================================================
// LED CONFIG (XIAO RP2350)
// =============================================================================
#ifndef LED_BUILTIN
  #define LED_BUILTIN 25
#endif
bool LED_INVERTED = true; // Active Low for XIAO

// =============================================================================
// GLOBALS
// =============================================================================
float targets[3] = {0, 0, 0};
uint8_t send_buffer[SERIAL_BUFFER_SIZE];

// Instantiate motorControl (it handles MCP2515 internally)
motorControl mc(CS_PIN, 8000000, &SPI);

// =============================================================================
// FUNCTIONS
// =============================================================================

void led_flash(int count) {
  for(int i=0; i<count; i++) {
     digitalWrite(LED_BUILTIN, LED_INVERTED ? LOW : HIGH); delay(100); 
     digitalWrite(LED_BUILTIN, LED_INVERTED ? HIGH : LOW); delay(100);
  }
}

void SPIinit() {
  SPI.setRX(RX_PIN);
  SPI.setCS(CS_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setTX(TX_PIN);
  SPI.begin();
}

void setup() {
  Serial.begin(115200);
  SPIinit();
  
  pinMode(LED_BUILTIN, OUTPUT);
  led_flash(3); // Startup confidence blink

  // 1. Configure Joints/Motors
  // We use ANGLE mode (Position Control)
  mc.setMode(mc.MOTOR1, mc.ANGLE);
  mc.setMode(mc.MOTOR2, mc.ANGLE);
  mc.setMode(mc.MOTOR3, mc.ANGLE);
  
  // 2. Set Gear Ratios (36.0 from original code)
  mc.setGearRatio(mc.MOTOR1, 36.0);
  mc.setGearRatio(mc.MOTOR2, 36.0);
  mc.setGearRatio(mc.MOTOR3, 36.0);

  // 3. Set PIDs
  // ANGLE loop (Outer loop)
  // KP=1.0, KI=0, KD=0, Max=100
  mc.setPIDParameterAngle(mc.MOTOR1, 1.0, 0.0, 0.0);
  mc.setMaxValueAngle(mc.MOTOR1, 100.0); 
  
  mc.setPIDParameterAngle(mc.MOTOR2, 1.0, 0.0, 0.0);
  mc.setMaxValueAngle(mc.MOTOR2, 100.0);
  
  mc.setPIDParameterAngle(mc.MOTOR3, 1.0, 0.0, 0.0);
  mc.setMaxValueAngle(mc.MOTOR3, 100.0);

  // SPEED loop (Inner loop, used by C620/CalPID internally)
  // KP=0.2, KI=1.0, KD=0.0
  // Note: setPIDParameter defaults to setting the inner loop params when mode is ANGLE
  mc.setPIDParameter(mc.MOTOR1, 0.2, 1.0, 0.0);
  mc.setPIDParameter(mc.MOTOR2, 0.2, 1.0, 0.0);
  mc.setPIDParameter(mc.MOTOR3, 0.2, 1.0, 0.0);

  // 4. Initialize Hardware
  mc.init();
}

// Decode 7-byte packet to Float Targets
void decode(uint8_t* encoded, float* data) {
  if (encoded[0] != SERIAL_HEADER) return;
  for (int i = 0; i < 3; i++) {
    // Check joint index in top bits
    if (((encoded[i*2+1] & 0xC0) >> 6) == i) {
      int sign = (encoded[i*2+1] & 0x30) ? 1 : -1;
      int value = ((encoded[i*2+1] & 0x0F) << 8) | encoded[i*2+2];
      data[i] = sign * float(value) / ANGLE_RESOLUTION;
    }
  }
}

// Encode Angles to 7-byte packet
void encode(double* data, uint8_t* encoded) {
  encoded[0] = SERIAL_HEADER;
  for (int i = 0; i < 3; i++) {
    int val = int(data[i] * ANGLE_RESOLUTION);
    // [Index(2b) | Sign(2b) | High(4b)] [Low(8b)]
    encoded[i*2+1] = (i<<6) | ((val>=0?1:0)<<5) | ((abs(val)>>8)&0x1F);
    encoded[i*2+2] = abs(val) & 0xFF;
  }
}

void loop() {
  // 1. Update Core Logic (PID, CAN read/write)
  mc.updateMicros(micros());
  mc.update(); 

  // 2. Handle Serial Communication
  if (Serial.available() > 0) {
    uint8_t buf[SERIAL_BUFFER_SIZE];
    int idx = 0;
    
    // Blocking read for simplicity (ensure full packet)
    // In a real RTOS this is bad, but for this loop it's fine
    while (Serial.available() && idx < SERIAL_BUFFER_SIZE) {
        buf[idx++] = Serial.read();
        delayMicroseconds(500); // Give time for bytes
    }
    
    if (idx == SERIAL_BUFFER_SIZE && buf[0] == SERIAL_HEADER) {
        // VALID PACKET -> Update Targets
        decode(buf, targets);
        
        mc.setTarget(mc.MOTOR1, mc.DEG, targets[0]);
        mc.setTarget(mc.MOTOR2, mc.DEG, targets[1]);
        mc.setTarget(mc.MOTOR3, mc.DEG, targets[2]);
        
        // TOGGLE LED on valid packet (makes it blink as packets arrive)
        static bool ledState = false;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? (LED_INVERTED ? LOW : HIGH) : (LED_INVERTED ? HIGH : LOW));
    }

    // 3. Send Feedback
    // We send back the CURRENT positions
    double current_angles[3];
    current_angles[0] = mc.deg[mc.MOTOR1];
    current_angles[1] = mc.deg[mc.MOTOR2];
    current_angles[2] = mc.deg[mc.MOTOR3];
    
    encode(current_angles, send_buffer);
    Serial.write(send_buffer, SERIAL_BUFFER_SIZE);
  }
}
