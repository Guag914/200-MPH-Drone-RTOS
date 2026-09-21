//
// BufferPopulation.h by Akshay Gillett on 7/13/26.
//

#ifndef BUFFER_POPULATION_H
#define BUFFER_POPULATION_H

#define CRSF_RING_BUFFER_SIZE 52
#define IMU_RING_BUFFER_SIZE 52
#define BATTERY_ADC_BUFFER_SIZE 2
#define BARO_BUFFER_SIZE 6
#define GPS_RING_BUFFER_SIZE 192
#define OSD_RING_BUFFER_SIZE 256

#include <stdint.h>

#include "stm32f756xx.h"

//use structs to hold returns so that cpp does not destroy local function variables
struct IMURawPacket { uint8_t bytes[12]; };
struct CRSFPacket { uint8_t bytes[26]; };
struct ADCPacket { float bytes[2]; };
struct BaroRawPacket { uint8_t bytes[8]; };
struct GPSPacket {char bytes[96]; };
struct OSDPacket { uint8_t bytes[256]; uint8_t length;};

// Global function declarations
IMURawPacket populateIMUBuffer();
CRSFPacket populateCRSFBuffer();

extern void startIMU_DMARead();
extern void startCRSF_DMARead();

extern void startMotor1_DMATransfer();
extern void startMotor2_DMATransfer();
extern void startMotor3_DMATransfer();
extern void startMotor4_DMATransfer();

extern ADCPacket populateBatteryADCBuffer();

extern void startBatteryADC_DMA();

extern void initBaro_DMATransfer();
extern void burstBaro_DMATransfer();
extern BaroRawPacket populateBarometerBuffer();

extern void startGPS_DMARead();
extern GPSPacket populateGPSBuffer();

extern void initOSD_DMATransfer();
extern void initTTX_DMATransfer();

extern void startOSD_DMARead();
extern OSDPacket populateOSDBuffer();

// extern uint8_t imuRawBuffer[12];
extern uint8_t imuRingBuffer[IMU_RING_BUFFER_SIZE];
extern uint8_t crsfRingBuffer[CRSF_RING_BUFFER_SIZE];
extern  uint16_t batteryADCRingBuffer[BATTERY_ADC_BUFFER_SIZE];
extern uint8_t baroBuffer[BARO_BUFFER_SIZE];
extern char gpsRingBuffer[GPS_RING_BUFFER_SIZE];
extern uint8_t osdRingBuffer[OSD_RING_BUFFER_SIZE];

extern void dmaBlockingTransfer(DMA_TypeDef* dma, DMA_Stream_TypeDef* stream, uint32_t memAddr, uint16_t length);

#endif // BUFFER_POPULATION_H