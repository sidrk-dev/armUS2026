#ifndef _DATA_ENCODER_H_
#define _DATA_ENCODER_H_

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef MAX_DATA_NUM
#define MAX_DATA_NUM 10
#define MAX_BIT MAX_DATA_NUM * 32
#define MAX_BYTE MAX_BIT / 7 + 3
#endif

class dataEncoder {
public:
  uint8_t id;
  uint8_t length = 0;
  uint8_t data[MAX_BYTE];
  dataEncoder(uint8_t id_);
  template <typename T>
  void append(uint8_t ord_, T *dataPtr_, uint8_t size_ = 0);
  uint8_t init();
  uint8_t encode();
private:
  enum TYPE {
    NONE = 00,
    BOOL = 10,
    UINT8_T = 20,
    UINT16_T = 21,
    UINT32_T = 22,
    UINT64_T = 23,
    INT8_T = 30,
    INT16_T = 31,
    INT32_T = 32,
    INT64_T = 33,
    FLOAT = 40,
    DOUBLE = 41
  };
  bool isEditable = true;
  TYPE type[MAX_DATA_NUM];
  void *ptr[MAX_DATA_NUM];
  uint8_t iSize[MAX_DATA_NUM];
  uint8_t oSize[MAX_DATA_NUM];
  uint8_t byteLength;
  bool binaryData[MAX_BIT];
  template <typename T>
  TYPE identifyType();
  template <typename T>
  uint8_t decideSize(TYPE type_, uint8_t size_);
  template <typename T>
  uint32_t getRawData(uint8_t ord_);
  uint32_t getData(uint8_t ord_);
  void generateBinary();
};

#endif