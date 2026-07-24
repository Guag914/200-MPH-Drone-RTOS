//
// Created by Akshay Gillett on 7/13/26.
//

#ifndef BUFFER_POPULATION_H
#define BUFFER_POPULATION_H

#define HW_UART_BUFFER_SIZE 64

#include <stdint.h>

//use structs to hold returns so that cpp does not destroy local function variables
struct IMURawPacket { uint8_t bytes[12]; };
struct CRSFPacket { uint8_t bytes[26]; };

// Global function declarations
IMURawPacket populateIMUBuffer();
CRSFPacket populateCRSFBuffer();

extern void startIMU_DMARead();
extern void startCRSF_DMARead();

extern void startMotor1_DMATransfer();
extern void startMotor2_DMATransfer();
extern void startMotor3_DMATransfer();
extern void startMotor4_DMATransfer();

extern void startBatteryADC_DMA();

extern uint8_t imuRawBuffer[12];
extern uint8_t crsfRingBuffer[HW_UART_BUFFER_SIZE];


#endif // BUFFER_POPULATION_H