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

#endif // __cplusplus

#endif //DRONE_RTOS_HELPERS_H
