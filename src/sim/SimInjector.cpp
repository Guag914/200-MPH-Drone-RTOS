//
// Created by Akshay Gillett on 7/18/26.
//

#include "SimInjector.h"
#include "../flight/BufferPopulation.h"
#include <stdlib.h>
#include "../rtos/rtos.h"
#include "../flight/flight_control.h"
#include "../flight/Helpers.h"

#include "cmsis_gcc.h"

//sole purpose is to generate random numbers for the sim
//not meant to be used ouside anywhere


IMURawPacket populateIMUMockBuffer() {
    //triggers and yields exactly the same way as other methods
    yieldCurrentTask();

    //pull CS high
    currentBoardConfig.imu_cs_port->BSRR = currentBoardConfig.imu_cs_pin;

    IMURawPacket rawPacket;

    //generate the random binary formatting directly into the packet structure
    int16_t mockAccelX = (rand() % 600) - 300;
    int16_t mockAccelY = (rand() % 600) - 300;
    int16_t mockAccelZ = 2048 + ((rand() % 200) - 100);
    int16_t mockGyroX = (rand() % 200) - 100;
    int16_t mockGyroY = (rand() % 200) - 100;
    int16_t mockGyroZ = (rand() % 200) - 100;

    rawPacket.bytes[0] = mockAccelX & 0xFF; rawPacket.bytes[1] = (mockAccelX >> 8) & 0xFF;
    rawPacket.bytes[2] = mockAccelY & 0xFF; rawPacket.bytes[3] = (mockAccelY >> 8) & 0xFF;
    rawPacket.bytes[4] = mockAccelZ & 0xFF; rawPacket.bytes[5] = (mockAccelZ >> 8) & 0xFF;
    rawPacket.bytes[6] = mockGyroX  & 0xFF; rawPacket.bytes[7] = (mockGyroX  >> 8) & 0xFF;
    rawPacket.bytes[8] = mockGyroY  & 0xFF; rawPacket.bytes[9] = (mockGyroY  >> 8) & 0xFF;
    rawPacket.bytes[10] = mockGyroZ  & 0xFF; rawPacket.bytes[11] = (mockGyroZ  >> 8) & 0xFF;

    return rawPacket;
}

CRSFPacket populateCRSFMockBuffer() {
    CRSFPacket localContainer = {0};
    static uint16_t mockReadIndex = 0;
    uint16_t bytesCopied = 0;
    static uint16_t mockHardwareHead = 0;

    //simulate incoming bytes over time
    //if they are in sync, advance the stream head
    mockHardwareHead = (mockHardwareHead + 26) % HW_UART_BUFFER_SIZE;

    //if the read pointer caught up to the write head, return empty container
    if (mockReadIndex == mockHardwareHead) {
        return localContainer;
    }

    //generate valid packet values
    uint8_t tempFrame[26];
    uint16_t ch1_roll = 992 + ((rand() % 160) - 80);
    uint16_t ch2_pitch = 992 + ((rand() % 160) - 80);
    uint16_t ch3_yaw = 992 + ((rand() % 160) - 80);
    uint16_t ch4_throttle = 350 + (rand() % 100);
    uint16_t ch5_arm = 1500; // > 1000 means ARMED

    tempFrame[0] = 0xC8; // Valid Sync Header
    tempFrame[1] = 0x18;
    tempFrame[2] = 0x16;
    tempFrame[3] = ch1_roll & 0xFF;
    tempFrame[4] = ((ch1_roll >> 8) | (ch2_pitch << 3)) & 0xFF;
    tempFrame[5] = ((ch2_pitch >> 5) | (ch3_yaw << 6)) & 0xFF;
    tempFrame[6] = (ch3_yaw >> 2) & 0xFF;
    tempFrame[7] = ((ch3_yaw >> 10) | (ch4_throttle << 1)) & 0xFF;
    tempFrame[8] = ((ch4_throttle >> 7) | (ch5_arm << 4)) & 0xFF;
    tempFrame[9] = (ch5_arm >> 4) & 0xFF;
    tempFrame[25] = 0x00;

    // Safely copy out the simulated hardware frame
    while (mockReadIndex != mockHardwareHead && bytesCopied < 26) {
        __disable_irq();
        localContainer.bytes[bytesCopied] = tempFrame[bytesCopied];
        bytesCopied++;

        mockReadIndex = (mockReadIndex + 1) % HW_UART_BUFFER_SIZE;
        __enable_irq();
    }

    return localContainer;
}

void updateMockBaroBuffer(float targetPressurePa, float targetTempC) {
    // Approximate raw conversions for simulation
    uint32_t mockRawPress = 6710886 + static_cast<uint32_t>((101325.0f - targetPressurePa) * 10.0f);
    uint32_t mockRawTemp  = 6553600 + static_cast<uint32_t>(targetTempC * 100.0f);

    // Add tiny random jitter (+/- 20 counts) to simulate physical noise
    mockRawPress += (rand() % 41) - 20;

    // Pack Pressure into bytes 0, 1, 2
    baroBuffer[0] = static_cast<uint8_t>((mockRawPress >> 16) & 0xFF);
    baroBuffer[1] = static_cast<uint8_t>((mockRawPress >> 8) & 0xFF);
    baroBuffer[2] = static_cast<uint8_t>(mockRawPress & 0xFF);

    // Pack Temperature into bytes 3, 4, 5
    baroBuffer[3] = static_cast<uint8_t>((mockRawTemp >> 16) & 0xFF);
    baroBuffer[4] = static_cast<uint8_t>((mockRawTemp >> 8) & 0xFF);
    baroBuffer[5] = static_cast<uint8_t>(mockRawTemp & 0xFF);
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
    baroBuffer[0] = 0x66; baroBuffer[1] = 0x66; baroBuffer[2] = 0x66; //raw pressure ~6,710,886
    baroBuffer[3] = 0x64; baroBuffer[4] = 0x00; baroBuffer[5] = 0x00; //raw temperature  ~6,553,600

    return mockTrim;
}

ADCPacket injectMockBatteryADCBuffer() {
    ADCPacket localADC = {0};
    const float rawVoltage = (1670.0f + ((rand() % 40) - 20)) * (3.3f / 4095.0f);
    const float rawCurrentVolts = (3100.0f + ((rand() % 60) - 30)) * (3.3f / 4095.0f);

    localADC.bytes[0] = rawVoltage;
    localADC.bytes[1] = rawCurrentVolts;

    return localADC;
}