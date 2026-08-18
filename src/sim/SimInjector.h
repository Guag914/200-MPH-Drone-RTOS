//
// Created by Akshay Gillett on 7/18/26.
//

#ifndef DRONE_RTOS_SIMINJECTOR_H
#define DRONE_RTOS_SIMINJECTOR_H

#include "../flight/BufferPopulation.h"
#include "../flight/Helpers.h"

extern void injectMockIMUDMA();
extern void injectMockCRSFDMA();

extern BaroTrim initMockBarometer();
extern void injectMockBaroDMA();

extern void injectMockADCDMA();
void injectMockGPSDMA();

extern volatile uint32_t mock_LISR;
extern volatile uint32_t mock_HISR;

#endif //DRONE_RTOS_SIMINJECTOR_H
