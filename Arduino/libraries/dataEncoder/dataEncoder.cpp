#include "dataEncoder.h"

dataEncoder::dataEncoder(uint8_t id_){
  //if (id_ >= 10) 
  id = id_;
  for (uint8_t cnt = 0; cnt < MAX_DATA_NUM; cnt++) {
    type[cnt] = NONE;
  }
}

template <typename T>
void dataEncoder::append(uint8_t ord_, T *dataPtr_, uint8_t size_) {
  if (isEditable) {
    type[ord_] = identifyType<T>();
    ptr[ord_] = (void*)dataPtr_;
    iSize[ord_] = sizeof(T);
    oSize[ord_] = decideSize<T>(type[ord_], size_);
  }
}

template void dataEncoder::append<bool>(uint8_t ord_, bool *dataPtr_, uint8_t size_);
template void dataEncoder::append<uint8_t>(uint8_t ord_, uint8_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<uint16_t>(uint8_t ord_, uint16_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<uint32_t>(uint8_t ord_, uint32_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<uint64_t>(uint8_t ord_, uint64_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<int8_t>(uint8_t ord_, int8_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<int16_t>(uint8_t ord_, int16_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<int32_t>(uint8_t ord_, int32_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<int64_t>(uint8_t ord_, int64_t *dataPtr_, uint8_t size_);
template void dataEncoder::append<float>(uint8_t ord_, float *dataPtr_, uint8_t size_);
template void dataEncoder::append<double>(uint8_t ord_, double *dataPtr_, uint8_t size_);

uint8_t dataEncoder::init() {
  isEditable = false;
  length = 0;
  for (uint8_t cnt = 0; cnt < MAX_DATA_NUM; cnt++) {
    length += oSize[cnt];
  }
  uint8_t num = 0;
  data[num] = 0b11000000 | id;
  num++;
  for (uint8_t cnt = 0; cnt < MAX_DATA_NUM; cnt++) {
    data[num] = (type[cnt] / 10);
    num++;
    data[num] = oSize[cnt];
    num++;
  }
  data[num] = 0b10000000 | (num + 1);
  return num + 1;
}

uint8_t dataEncoder::encode() {
  for (uint8_t cnt = 0; cnt < MAX_BYTE; cnt++) {
    data[cnt] = 0;
  }
  generateBinary();
  uint8_t num = 0;
  data[num] = 0b10000000 | id;
  num++;
  for (uint16_t cnt = 0; cnt < length; cnt++) {
    data[num] |= binaryData[cnt] << (6 - (cnt % 7));
    if ((cnt % 7) == 6) num++;
  }
  if ((length % 7) != 0) num++;
  data[num] = 0b10000000 | (num + 1);
  byteLength = num + 1;
  return byteLength;
}

template <typename T>
dataEncoder::TYPE dataEncoder::identifyType() {return NONE;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<bool>() {return BOOL;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<uint8_t>() {return UINT8_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<uint16_t>() {return UINT16_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<uint32_t>() {return UINT32_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<uint64_t>() {return UINT64_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<int8_t>() {return INT8_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<int16_t>() {return INT16_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<int32_t>() {return INT32_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<int64_t>() {return INT64_T;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<float>() {return FLOAT;}
template <>
dataEncoder::TYPE dataEncoder::identifyType<double>() {return DOUBLE;}

template <typename T>
uint8_t dataEncoder::decideSize(dataEncoder::TYPE type_, uint8_t size_) {
  //if (size >= 32)
  uint8_t size;
  if (type_ == BOOL) size = 1;
  else if (type_ == UINT64_T) size = 32;
  else if (type_ == INT64_T) size = 32;
  else if (type_ == FLOAT) size = 32;
  else if (type_ == DOUBLE) size = 32;
  else if (size_ == 0) size = sizeof(T) * 8;
  else size = size_;
  return size;
}

template uint8_t dataEncoder::decideSize<bool>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<uint8_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<uint16_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<uint32_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<uint64_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<int8_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<int16_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<int32_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<int64_t>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<float>(dataEncoder::TYPE type_, uint8_t size_);
template uint8_t dataEncoder::decideSize<double>(dataEncoder::TYPE type_, uint8_t size_);

template <typename T>
uint32_t dataEncoder::getRawData(uint8_t ord_) {
  T data32 = 0;
  if (type[ord_] == BOOL) data32 = (T)*static_cast<bool*>(ptr[ord_]);
  if (type[ord_] == UINT8_T) data32 = (T)*static_cast<uint8_t*>(ptr[ord_]);
  if (type[ord_] == UINT16_T) data32 = (T)*static_cast<uint16_t*>(ptr[ord_]);
  if (type[ord_] == UINT32_T) data32 = (T)*static_cast<uint32_t*>(ptr[ord_]);
  if (type[ord_] == UINT64_T) data32 = (T)*static_cast<uint64_t*>(ptr[ord_]);
  if (type[ord_] == INT8_T) data32 = (T)*static_cast<int8_t*>(ptr[ord_]);
  if (type[ord_] == INT16_T) data32 = (T)*static_cast<int16_t*>(ptr[ord_]);
  if (type[ord_] == INT32_T) data32 = (T)*static_cast<int32_t*>(ptr[ord_]);
  if (type[ord_] == INT64_T) data32 = (T)*static_cast<int64_t*>(ptr[ord_]);
  if (type[ord_] == FLOAT) data32 = (T)*static_cast<float*>(ptr[ord_]);
  if (type[ord_] == DOUBLE) data32 = (T)*static_cast<double*>(ptr[ord_]);
  union {T typedData; uint32_t untypedData;} data_;
  data_.typedData = data32;
  return data_.untypedData;
}

template uint32_t dataEncoder::getRawData<uint32_t>(uint8_t ord_);
template uint32_t dataEncoder::getRawData<int32_t>(uint8_t ord_);
template uint32_t dataEncoder::getRawData<float>(uint8_t ord_);

uint32_t dataEncoder::getData(uint8_t ord_) {
  uint32_t data32 = 0;
  if ((type[ord_] / 10) == 1) data32 = getRawData<uint32_t>(ord_);
  if ((type[ord_] / 10) == 2) data32 = getRawData<uint32_t>(ord_);
  if ((type[ord_] / 10) == 3) {
    data32 = getRawData<int32_t>(ord_);
    bool fill = bitRead(data32, 31);
    uint32_t ones = 0;
    for (uint8_t cnt = oSize[ord_]; cnt < 32; cnt++) {
      ones |= (1 << cnt);
    }
    data32 |= ones;
  }
  if ((type[ord_] / 10) == 4) data32 = getRawData<float>(ord_);
  return data32;
}

void dataEncoder::generateBinary() {
  uint16_t bitNum = 0;
  for (uint8_t cnt = 0; cnt < MAX_DATA_NUM; cnt++) {
    if (type[cnt] != NONE) {
      for (uint8_t bit_ = 0; bit_ < oSize[cnt]; bit_++) {
        binaryData[bitNum] = bitRead(getData(cnt), oSize[cnt] - bit_ - 1);
        bitNum++;
      }
    }
  }
}