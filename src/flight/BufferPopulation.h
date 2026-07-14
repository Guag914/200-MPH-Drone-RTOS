//
// Created by Akshay Gillett on 7/13/26.
//

#ifndef BUFFER_POPULATION_H
#define BUFFER_POPULATION_H

#include <stdint.h>

//use structs to hold returns so that cpp does not destroy local function variables
struct IMURawPacket { uint8_t bytes[12]; };
struct CRSFPacket { uint8_t bytes[26]; };

// Global function declarations
IMURawPacket populateIMUBuffer();
CRSFPacket populateCRSFBuffer();

#endif // BUFFER_POPULATION_H