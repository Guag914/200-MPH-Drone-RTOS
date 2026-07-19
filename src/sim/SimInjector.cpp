//
// Created by Akshay Gillett on 7/18/26.
//

#include "SimInjector.h"
#include "../flight/BufferPopulation.h"
#include <stdlib.h>
#include "../rtos/rtos.h"
#include "../flight/flight_control.h"

#include "cmsis_gcc.h"

//sole purpose is to generate random numbers for the sim
//not meant to be used ouside anywhere


IMURawPacket populateIMUMockBuffer() {
    //triggers and yields exactly the same way as other methods
    yieldCurrentTask();

    //oull CS high
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