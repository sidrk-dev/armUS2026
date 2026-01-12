#include "dataEncoder.h"
#include "dataDecoder.h"

dataEncoder enc(0);
dataDecoder dec;

bool a = true;
uint8_t b = 0;
int16_t c = 0;
float d = 3.14159265353;
double e = 2.71828182846;

bool A;
uint8_t B;
int16_t C;
float D;
double E;

void setup() {
  while (!Serial);
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);

  enc.append<bool>(0, &a);  
  enc.append<uint8_t>(1, &b);
  enc.append<int16_t>(2, &c, 9);
  enc.append<float>(3, &d);
  enc.append<double>(4, &e);

  Serial.print("initializing data: ");
  for (uint8_t cnt = 0; cnt < enc.init(); cnt++) {
    Serial1.write(enc.data[cnt]);
    Serial.printf("%02x", enc.data[cnt]);
  }

  Serial.println("setup done");
  delay(1000);
}

void loop() {
  a = !a;
  b++;
  c--;
  d *= 1.01;
  e /= 1.01;
  Serial.printf("\npre encoding data: (%d, %d, %d, %f, %f)\n", a, b, c, d, e);
  Serial.print("sending data: ");
  for (uint8_t cnt = 0; cnt < enc.encode(); cnt++) {
    Serial1.write(enc.data[cnt]);
    Serial.printf("%02x", enc.data[cnt]);
  }
  Serial.println("");
  delay(1000);
}

void setup1() {
  while (!Serial);
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200);
  Serial.println("setup1 done");
  delay(500);
}

void loop1() {
  uint8_t dataNum = Serial2.available();
  if (dataNum != 0) {
    Serial.print("received data:");
    while (dec.status != dataDecoder::WAITING);
    dec.status = dataDecoder::EDITING;
    for (uint8_t cnt = 0; cnt < dataNum; cnt++) {
      dec.data[cnt] = Serial2.read();
      Serial.printf("%02x", dec.data[cnt]);
    }
    dec.status = dataDecoder::WAITING;
    Serial.println("");
    dec.decode();
    A = dec.decodedData<bool>(0, 0);
    B = dec.decodedData<uint8_t>(0, 1);
    C = dec.decodedData<int16_t>(0, 2);
    D = dec.decodedData<float>(0, 3);
    E = dec.decodedData<double>(0, 4);
    Serial.printf("decoded data: (%d, %d, %d, %f, %f)\n", A, B, C, D, E);
  }
  delay(100);
}
