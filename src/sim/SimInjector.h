//
// Created by Akshay Gillett on 7/18/26.
//

#ifndef DRONE_RTOS_SIMINJECTOR_H
#define DRONE_RTOS_SIMINJECTOR_H

#include "../flight/BufferPopulation.h"

extern IMURawPacket populateIMUMockBuffer();
extern CRSFPacket populateCRSFMockBuffer();

#endif //DRONE_RTOS_SIMINJECTOR_H
