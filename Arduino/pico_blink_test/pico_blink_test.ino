#include <Arduino.h>

// Use LED_BUILTIN if defined, otherwise default to 25 (Standard Pico)
#ifndef LED_BUILTIN
  #define LED_BUILTIN 25
#endif

// Set to true for Xiao RP2350 (Active Low LED)
bool LED_INVERTED = true; 

#define SERIAL_HEADER      0xFF
#define SERIAL_BUFFER_SIZE 7

void led_on() {
  digitalWrite(LED_BUILTIN, LED_INVERTED ? LOW : HIGH);
}

void led_off() {
  digitalWrite(LED_BUILTIN, LED_INVERTED ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // STARTUP SEQUENCE: Blink 3 times rapidly
  // This confirms the board is running and the LED pin is correct
  for(int i=0; i<3; i++) {
    led_on(); delay(100);
    led_off(); delay(100);
  }
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming byte
    uint8_t incomingByte = Serial.read();

    // Debug: Echo back what we received in Hex
    Serial.print("Received: 0x");
    if(incomingByte < 0x10) Serial.print("0");
    Serial.println(incomingByte, HEX);

    // Check for Header 0xFF OR debug command 't'
    if (incomingByte == SERIAL_HEADER || incomingByte == 't') {
      // Packet detected! Wait for the rest (if it's a real packet)
      // If it's just 't', we'll likely timeout on the buffer fill, but we can just blink!
      
      if(incomingByte == 't') {
         Serial.println("Debug command 't' received! Blinking...");
         led_on();
         delay(500);
         led_off();
         return; // Skip the rest of the buffer logic for 't'
      }

      delay(5); // Small delay to let bytes arrive
      
      // We need at least SERIAL_BUFFER_SIZE - 1 more bytes
      // But for a simple test, we just wait for enough bytes or timeout
      int waittime = 0;
      while(Serial.available() < SERIAL_BUFFER_SIZE - 1 && waittime < 10) {
        delay(1);
        waittime++;
      }

      if (Serial.available() >= SERIAL_BUFFER_SIZE - 1) {
        // Drain the rest of the buffer (dummy read for this test)
        for(int i=0; i<SERIAL_BUFFER_SIZE-1; i++) {
          Serial.read();
        }
        
        // SUCCESS: Valid Header found + bytes available -> Blink Long
        Serial.println("Packet Valid! Blinking...");
        led_on();
        delay(500);
        led_off();
      }
    }
  }
}
