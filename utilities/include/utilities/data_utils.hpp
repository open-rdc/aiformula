#pragma once

#include <cstdint>

/*INT*/
union Int_bytes {
    int int_value;
    uint8_t bytes[4];
};
inline int bytes_to_int(uint8_t *value) {
    Int_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    tmp.bytes[2]=value[2];
    tmp.bytes[3]=value[3];
    return tmp.int_value;
}
inline void int_to_bytes(uint8_t *value,int int_value){
    Int_bytes tmp;
    tmp.int_value=int_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
    value[2]=tmp.bytes[2];
    value[3]=tmp.bytes[3];
}

/*USHORT*/
union UShort_bytes{
    unsigned short short_value;
    uint8_t bytes[2];
};
inline unsigned short bytes_to_ushort(uint8_t *value) {
    UShort_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    return tmp.short_value;
}
inline void ushort_to_bytes(uint8_t *value,unsigned short short_value) {
    UShort_bytes tmp;
    tmp.short_value=short_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
}

/*SHORT*/
union Short_bytes{
    signed short short_value;
    uint8_t bytes[2];
};
inline short bytes_to_short(uint8_t *value){
    Short_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    return tmp.short_value;
}
inline void short_to_bytes(uint8_t *value,short short_value){
    Short_bytes tmp;
    tmp.short_value=short_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
}

/*FLOAT*/
union Float_bytes {
    float float_value;
    uint8_t bytes[4];
};
inline float bytes_to_float(uint8_t *value) {
    Float_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    tmp.bytes[2]=value[2];
    tmp.bytes[3]=value[3];
    return tmp.float_value;
}
inline void float_to_bytes(uint8_t *value,float float_value) {
    Float_bytes tmp;
    tmp.float_value=float_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
    value[2]=tmp.bytes[2];
    value[3]=tmp.bytes[3];
}
