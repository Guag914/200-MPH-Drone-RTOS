//
// Created by Akshay Gillett on 7/16/26.
//

#ifndef DRONE_RTOS_HELPERS_H
#define DRONE_RTOS_HELPERS_H


//hide from c compiler but public
#ifdef __cplusplus //this section is completely HIDDEN from the C compiler (main.c)
#include <cstdint>
#include <sys/types.h>
#include "./flight_control.h"

struct DShotFrame { uint32_t bits[17]; };

extern void estimateAngles();
extern void pt1Filter();
extern void complementaryFilter();
extern DShotFrame generateDShotFrame(float throttleInput);
extern void writeOSDString(uint8_t row, uint8_t col, char* str);
extern void sendMSPFrame(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen);

extern uint32_t dshotBuffer1[17];
extern uint32_t dshotBuffer2[17];
extern uint32_t dshotBuffer3[17];
extern uint32_t dshotBuffer4[17];
extern uint8_t formatCRSFFrame(uint8_t frameType, const uint8_t* payload, uint8_t payloadLen, uint8_t* outBuffer);
extern float applyActualRates(float stick, const ActualRateConfig& config);
extern uint8_t crc8_dvb_s2(const uint8_t* data, uint8_t len);

extern int32_t parseNmeaCoord(const char* str, char dir);
extern uint32_t getUnusedStackWords(const uint8_t taskIdx);

struct BaroTrim {
    //temperature calibration
    uint16_t T1;
    uint16_t T2;
    uint8_t T3;

    //pressure calibration
    int16_t P1;
    int16_t P2;
    int8_t P3;
    int8_t P4;
    uint16_t P5;
    uint16_t P6;
    int8_t P7;
    int8_t P8;
    int16_t P9;
    int8_t P10;
    int8_t P11;
};

extern BaroTrim initBarometer();
#endif // __cplusplus

#endif //DRONE_RTOS_HELPERS_H
