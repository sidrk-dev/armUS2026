#ifndef _DATA_DECODER_H_
#define _DATA_DECODER_H_

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef MAX_DATA_NUM
#define MAX_DATA_NUM 10
#define MAX_BIT MAX_DATA_NUM * 32
#define MAX_BYTE MAX_BIT / 7 + 3
#endif

#define MAX_PORT_NUM 10

class dataDecoder {
public:
  enum MODE {
    WAITING = 0,
    INITING = 1,
    READING = 2,
    EDITING = 3
  };
  MODE status;
  uint8_t length[MAX_PORT_NUM];
  uint8_t data[MAX_BYTE];
  dataDecoder();
  bool decode();
  template <typename T>
  T decodedData(uint8_t id_, uint8_t ord_);
private:
  enum TYPE {
    NONE = 0,
    BOOL = 1,
    UINT = 2,
    INT = 3,
    FLOAT = 4
  };
  bool binaryData[MAX_PORT_NUM][MAX_BIT];
  uint32_t data32[MAX_PORT_NUM][MAX_DATA_NUM];
  TYPE type[MAX_PORT_NUM][MAX_DATA_NUM];
  uint8_t size[MAX_PORT_NUM][MAX_DATA_NUM];
  bool regId(uint8_t id_);
  bool read(uint8_t id_);
  void divide(uint8_t id_);
  template <typename T>
  T cast(uint8_t id_, uint8_t ord_);
};

#endif