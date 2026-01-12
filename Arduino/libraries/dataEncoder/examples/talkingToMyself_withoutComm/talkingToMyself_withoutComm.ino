/*
  Sample program to convert arbitrary data to binary and back.
  Please see serialEncoder.h and serialDecoder.h for detailed specifications.
  Note: 64-bit types (uint64_t, int64_t, double) are cast to 32-bit types (uint32_t, int32_t, float) internally for transmission.
*/

#include "dataEncoder.h"
#include "dataDecoder.h"

dataEncoder enc(0);  // Argument is protocol ID. serialDecoder.h allows up to 10 protocols.
dataDecoder dec;

bool a = true;
uint8_t b = 0;
int16_t c = 0;
float d = 3.14159265353;
double e = 2.71828182846;

char printBuf[128];

void setup() {
  Serial.begin(115200);
  while (!Serial);
  // Determine type with <type>. Arg 1: data order (smaller number first), Arg 2: data pointer, Arg 3: bits to transmit (defaults to type size if omitted).
  enc.append<bool>(0, &a);  
  enc.append<uint8_t>(1, &b);
  enc.append<int16_t>(2, &c, 9);
  enc.append<float>(3, &d);
  enc.append<double>(4, &e);
  //enc.init();
  sprintf(printBuf, "initializing data: ");
  // Prevent modification during decoding.
  while (dec.status != dataDecoder::WAITING);
  dec.status = dataDecoder::EDITING;
  // init() sends the transmission data protocol to the decoder. init() returns bytes to send.
  for (uint8_t cnt = 0; cnt < enc.init(); cnt++) {
    dec.data[cnt] = enc.data[cnt];
    sprintf(printBuf, "%s%02x", printBuf, dec.data[cnt]);
  }
  dec.status = dataDecoder::WAITING;
  Serial.println(printBuf);
  delay(1000);
  dec.decode();
  Serial.println("setup done");
}

void loop() {
  a = !a;
  b++;
  c--;
  d *= 1.01;
  e /= 1.01;
  sprintf(printBuf, "\npre encode data: (%d, %d, %d, ", a, b, c);
  Serial.print(printBuf);
  Serial.print(d);
  Serial.print(", ");
  Serial.print(e);
  Serial.println(")");
  //enc.encode();
  while (dec.status != dataDecoder::WAITING)
    ;
  dec.status = dataDecoder::EDITING;
  sprintf(printBuf, "encoded binary data: ");
  // Send/receive data is stored in data[]. encode() returns bytes to send.
  for (uint8_t cnt = 0; cnt < enc.encode(); cnt++) {
    dec.data[cnt] = enc.data[cnt];
    sprintf(printBuf, "%s%02x", printBuf, dec.data[cnt]);
  }
  dec.status = dataDecoder::WAITING;
  Serial.println(printBuf);
  dec.decode();
  // Specify type to retrieve with <type>. Arg 1: protocol ID, Arg 2: data index.
  bool A = dec.decodedData<bool>(0, 0);
  uint8_t B = dec.decodedData<uint8_t>(0, 1);
  int16_t C = dec.decodedData<int16_t>(0, 2);
  float D = dec.decodedData<float>(0, 3);
  double E = dec.decodedData<double>(0, 4);
  sprintf(printBuf, "decoded data: (%d, %d, %d, ", A, B, C);
  Serial.print(printBuf);
  Serial.print(D);
  Serial.print(", ");
  Serial.print(E);
  Serial.println(")");
  delay(1000);
}