/*
  任意のデータをバイナリに変換し，元に戻すサンプルプログラム．
  詳しい仕様はserialEncoder.h，serialDecoder.hの中身を見てください．
  注)64bit型(uint64_t, int64_t, double)は内部で32bit型(uint32_t, int32_t, float)にキャストされて送信されます．
*/

#include "dataEncoder.h"
#include "dataDecoder.h"

dataEncoder enc(0);  //引数はプロトコルのid．serialDecoder.hでは最大10個のプロトコルを使い分けることができる．
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
  //<type>で型を決定し，第1引数でデータの並び順(数字が小さいほど前にセットされる)，第2引数でデータのポインタ，第3引数で送信するビット数(省略すると型のビット数になる)を渡す．
  enc.append<bool>(0, &a);  
  enc.append<uint8_t>(1, &b);
  enc.append<int16_t>(2, &c, 9);
  enc.append<float>(3, &d);
  enc.append<double>(4, &e);
  //enc.init();
  sprintf(printBuf, "initializing data: ");
  //デコード中に書き換えが起こらないようにする．
  while (dec.status != dataDecoder::WAITING);
  dec.status = dataDecoder::EDITING;
  //init()でデコード側に送信データのプロトコルを送信する．init()の戻り値が送信すべきバイト数．
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
  //送信データ，受信データはdata[]に格納されている．encode()の戻り値が送信すべきバイト数．
  for (uint8_t cnt = 0; cnt < enc.encode(); cnt++) {
    dec.data[cnt] = enc.data[cnt];
    sprintf(printBuf, "%s%02x", printBuf, dec.data[cnt]);
  }
  dec.status = dataDecoder::WAITING;
  Serial.println(printBuf);
  dec.decode();
  //<type>で取り出すデータの型を指定．第1引数はプロトコルid,第2引数はデータの並び番号．
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