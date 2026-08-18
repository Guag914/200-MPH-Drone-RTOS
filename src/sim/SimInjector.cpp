//
// Created by Akshay Gillett on 7/18/26.
//

#include "SimInjector.h"
#include "../flight/BufferPopulation.h"
#include <stdlib.h>
#include "../flight/flight_control.h"
#include "../flight/Helpers.h"
#include "../rtos/logging/Logger.h"

//sole purpose is to generate random numbers for the sim
//not meant to be used ouside anywhere

volatile uint32_t mock_LISR = 0;
volatile uint32_t mock_HISR = 0;

void injectMockIMUDMA() {
    static uint16_t head = 0;

    int16_t aX = (rand() % 600) - 300, aY = (rand() % 600) - 300, aZ = 2048 + ((rand() % 200) - 100);
    int16_t gX = (rand() % 200) - 100, gY = (rand() % 200) - 100, gZ = (rand() % 200) - 100;

    uint8_t frame[12] = {
        static_cast<uint8_t>(aX & 0xFF), static_cast<uint8_t>((aX >> 8) & 0xFF),
        static_cast<uint8_t>(aY & 0xFF), static_cast<uint8_t>((aY >> 8) & 0xFF),
        static_cast<uint8_t>(aZ & 0xFF), static_cast<uint8_t>((aZ >> 8) & 0xFF),
        static_cast<uint8_t>(gX & 0xFF), static_cast<uint8_t>((gX >> 8) & 0xFF),
        static_cast<uint8_t>(gY & 0xFF), static_cast<uint8_t>((gY >> 8) & 0xFF),
        static_cast<uint8_t>(gZ & 0xFF), static_cast<uint8_t>((gZ >> 8) & 0xFF)
    };

    //circular write into RAM
    for (int i = 0; i < 12; i++) {
        imuRingBuffer[head] = frame[i];
        head = (head + 1) % IMU_RING_BUFFER_SIZE;
    }

    //hardware NDTR counts DOWN from max capacity
    currentBoardConfig.imu_dma_stream->NDTR = IMU_RING_BUFFER_SIZE - head;
}

void injectMockCRSFDMA() {
    static uint16_t head = 0;

    uint16_t ch1 = 992 + ((rand() % 160) - 80);
    uint16_t ch2 = 992 + ((rand() % 160) - 80);
    uint16_t ch3 = 992 + ((rand() % 160) - 80);
    uint16_t ch4 = 350 + (rand() % 100);
    uint16_t ch5 = 1500;

    uint8_t frame[26] = {0};
    frame[0] = 0xC8; frame[1] = 0x18; frame[2] = 0x16;
    frame[3] = ch1 & 0xFF;
    frame[4] = ((ch1 >> 8) | (ch2 << 3)) & 0xFF;
    frame[5] = ((ch2 >> 5) | (ch3 << 6)) & 0xFF;
    frame[6] = (ch3 >> 2) & 0xFF;
    frame[7] = ((ch3 >> 10) | (ch4 << 1)) & 0xFF;
    frame[8] = ((ch4 >> 7) | (ch5 << 4)) & 0xFF;
    frame[9] = (ch5 >> 4) & 0xFF;

    for (int i = 0; i < 26; i++) {
        crsfRingBuffer[head] = frame[i];
        head = (head + 1) % CRSF_RING_BUFFER_SIZE;
    }

    currentBoardConfig.crsf_dma_stream->NDTR = CRSF_RING_BUFFER_SIZE - head;
}

BaroTrim initMockBarometer() {
    BaroTrim mockTrim;

    //temperature trim
    mockTrim.T1 = 27850;
    mockTrim.T2 = 19200;
    mockTrim.T3 = -5;

    //pressure trims for 101.3 kPa (sea level)
    mockTrim.P1 = 16550;
    mockTrim.P2 = 14000;
    mockTrim.P3 = -3;
    mockTrim.P4 = 1;
    mockTrim.P5 = 12532;
    mockTrim.P6 = 15000;
    mockTrim.P7 = 8;
    mockTrim.P8 = -2;
    mockTrim.P9 = 3000;
    mockTrim.P10 = 1;
    mockTrim.P11 = -1;

    // Raw sensor baseline
    baroRingBuffer[0] = 0x66; baroRingBuffer[1] = 0x66; baroRingBuffer[2] = 0x66; //raw pressure ~6,710,886
    baroRingBuffer[3] = 0x64; baroRingBuffer[4] = 0x00; baroRingBuffer[5] = 0x00; //raw temperature  ~6,553,600

    return mockTrim;
}

void injectMockBaroDMA() {
    static uint16_t head = 0;
    static uint8_t frameBuf[6];
    static uint8_t byteIdx = 6; //forces a fresh frame to be generated on first call

    if (byteIdx >= 6) {
        //generate a new 6-byte frame, same math as before
        static int32_t rawPressure = 0x666666;
        static int32_t rawTemp = 0x640000;
        int32_t currentPress = rawPressure + ((std::rand() % 2048) - 1024);
        int32_t currentTemp  = rawTemp + ((std::rand() % 256) - 128);
        frameBuf[0] = (currentPress >> 16) & 0xFF;
        frameBuf[1] = (currentPress >> 8) & 0xFF;
        frameBuf[2] = currentPress & 0xFF;
        frameBuf[3] = (currentTemp >> 16) & 0xFF;
        frameBuf[4] = (currentTemp >> 8) & 0xFF;
        frameBuf[5] = currentTemp & 0xFF;
        byteIdx = 0;
    }

    baroRingBuffer[head] = frameBuf[byteIdx++];
    head = (head + 1) % BARO_RING_BUFFER_SIZE;
    currentBoardConfig.baro_dma_stream->NDTR = BARO_RING_BUFFER_SIZE - head;
}

void injectMockADCDMA() {
    static uint16_t head = 0;

    const uint16_t rawVoltage = static_cast<uint16_t>(1670 + ((rand() % 40) - 20));
    const uint16_t rawCurrent = static_cast<uint16_t>(3100 + ((rand() % 60) - 30));

    uint16_t frame[2] = { rawVoltage, rawCurrent };

    for (int i = 0; i < 2; i++) {
        batteryADCRingBuffer[head] = frame[i];
        head = (head + 1) % BATTERY_ADC_BUFFER_SIZE;
    }

    // Update NDTR so injectBatteryADCBuffer() calculates the correct headIndex
    currentBoardConfig.battery_dma_stream->NDTR = BATTERY_ADC_BUFFER_SIZE - head;
}

void injectMockGPSDMA() {
    static uint16_t head = 0;

    // Sample NMEA string with simulated position (e.g., 37.7749 N, 122.4194 W, 10 Sats, Fix Valid)
    const char* nmeaSentence = "$GNGGA,123519,3746.494,N,12225.164,W,1,10,0.9,50.0,M,0.0,M,,*47\r\n";
    const uint16_t len = strlen(nmeaSentence);

    for (uint16_t i = 0; i < len; i++) {
        gpsRingBuffer[head] = nmeaSentence[i];
        head = (head + 1) % GPS_RING_BUFFER_SIZE;
    }

    currentBoardConfig.gps_dma_stream->NDTR = GPS_RING_BUFFER_SIZE - head;
}