#include "dataDecoder.h"

dataDecoder::dataDecoder() {
    for (uint8_t cnt = 0; cnt < MAX_PORT_NUM; cnt++) {
        for (uint8_t cnt_ = 0; cnt_ < MAX_DATA_NUM; cnt_++) {
            type[cnt][cnt_] = NONE;
            size[cnt][cnt_] = 0;
        }
        length[cnt] = 0;
    }
    status = WAITING;
}

bool dataDecoder::decode() {
    bool error;
    if (status != EDITING) {
        status = READING;
        uint8_t id = data[0] & 0x0F;
        if (((data[0] >> 6) & 0xFF) == 0b11) {
            regId(id);
            error = false;
        }
        else if (((data[0] >> 6) & 0xFF) == 0b10) {
            read(id);
            divide(id);
            error = false;
        }
        else error = true;
    }
    else error = true;
    status = WAITING;
    return error;
}

template <typename T>
T dataDecoder::decodedData(uint8_t id_, uint8_t ord_) {
    T result;
    if (type[id_][ord_] == BOOL) result = T(cast<bool>(id_, ord_));
    else if (type[id_][ord_] == UINT) result = T(cast<uint32_t>(id_, ord_));
    else if (type[id_][ord_] == INT) result = T(cast<int32_t>(id_, ord_));
    else if (type[id_][ord_] == FLOAT) result = T(cast<float>(id_, ord_));
    return result;
}

template bool dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template uint8_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template uint16_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template uint32_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template uint64_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template int8_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template int16_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template int32_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template int64_t dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template float dataDecoder::decodedData(uint8_t id_, uint8_t ord_);
template double dataDecoder::decodedData(uint8_t id_, uint8_t ord_);


bool dataDecoder::regId(uint8_t id_) {
    bool error = false;
    uint8_t num = 1;
    while ((data[num] >> 7) != 0b1) {
        if ((num % 2) == 1) type[id_][(num - 1) / 2] = TYPE(data[num]);
        else size[id_][(num - 2) / 2] = data[num];
        num++;
        if(num > (MAX_DATA_NUM * 2 + 2)) {
            error = true;
            break;
        }
    }
    length[id_] = 0;
    for (uint8_t cnt = 0; cnt < MAX_DATA_NUM; cnt++) {
        length[id_] += size[id_][cnt];
    }
    if ((data[num] & 0x0F) != num) error = true;
    return error;
}

bool dataDecoder::read(uint8_t id_) {
    bool error = false;
    uint8_t num = 1;
    for (uint16_t cnt = 0; cnt < length[id_]; cnt++) {
        binaryData[id_][cnt] = ((data[num] >> (6 - (cnt % 7))) & 0b00000001);
        if ((cnt % 7) == 6) num++;
    }
    if ((data[num] >> 7) != 0b1) num++;
    if ((data[num] & 0x0F) != num) error = true;
    return error;
}

void dataDecoder::divide(uint8_t id_) {
    uint16_t num = 0;
    for (uint8_t cnt = 0; cnt < MAX_DATA_NUM; cnt++) {
        data32[id_][cnt] = 0;
        uint32_t data32_ = 0;
        for (uint8_t cnt_ = 0; cnt_ < size[id_][cnt]; cnt_++) {
            delayMicroseconds(1);
            data32_ = ((data32_ << 1) | binaryData[id_][num]);
            num++;
        }
        if (type[id_][cnt] == INT) {
            uint32_t ones = 0;
            bool fill = bitRead(data32_, size[id_][cnt] - 1);
            for (uint8_t cnt_ = size[id_][cnt]; cnt_ < 32; cnt_++) {
                ones |= (fill << cnt_);
            }
            data32_ |= ones;
        }
        data32[id_][cnt] = data32_;
    }
}

template <typename T>
T dataDecoder::cast(uint8_t id_, uint8_t ord_) {
    union {uint32_t data32_; T outputData;} data_;
    data_.data32_ = data32[id_][ord_];
    return data_.outputData;
}

template bool dataDecoder::cast(uint8_t id_, uint8_t ord_);
template uint32_t dataDecoder::cast(uint8_t id_, uint8_t ord_);
template int32_t dataDecoder::cast(uint8_t id_, uint8_t ord_);
template float dataDecoder::cast(uint8_t id_, uint8_t ord_);