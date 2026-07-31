//
// Created by Akshay Gillett on 7/18/26.
//

#ifndef DRONE_RTOS_SIMINJECTOR_H
#define DRONE_RTOS_SIMINJECTOR_H

#include "../flight/BufferPopulation.h"
#include "../flight/Helpers.h"

extern IMURawPacket populateIMUMockBuffer();
extern CRSFPacket populateCRSFMockBuffer();

extern void updateMockBaroBuffer(float targetPressurePa, float targetTempC);
extern BaroTrim initMockBarometer();

extern ADCPacket injectMockBatteryADCBuffer();

#endif //DRONE_RTOS_SIMINJECTOR_H
