//
// Created by Akshay Gillett on 7/16/26.
//

#ifndef DRONE_RTOS_HELPERS_H
#define DRONE_RTOS_HELPERS_H


//hide from c compiler but public
#ifdef __cplusplus //this section is completely HIDDEN from the C compiler (main.c)
#include <cstdint>

struct DShotFrame { uint32_t bits[17]; };

extern void estimateAngles();
extern void pt1Filter();
extern void complementaryFilter();
extern DShotFrame generateDShotFrame(float throttleInput);

extern uint32_t dshotBuffer1[17];
extern uint32_t dshotBuffer2[17];
extern uint32_t dshotBuffer3[17];
extern uint32_t dshotBuffer4[17];

extern uint8_t formatCRSFFrame(uint8_t frameType, const uint8_t* payload, uint8_t payloadLen, uint8_t* outBuffer);
extern uint8_t crc8_dvb_s2(const uint8_t* data, uint8_t len);

extern int32_t parseNmeaCoord(const char* str, char dir);
#endif // __cplusplus

#endif //DRONE_RTOS_HELPERS_H
