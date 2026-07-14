//
// Created by Akshay Gillett on 7/12/26.
//

#include "BufferPopulation.h"
#include "flight_control.h" //gives access to boardconfig

#define HW_UART_BUFFER_SIZE 64

IMURawPacket populateIMUBuffer() {
    IMURawPacket localContainer = {0};
    constexpr uint8_t startRegister = 0x0C; //BMI270 starting address

    currentBoardConfig.imu_cs_port->BSRR = static_cast<uint32_t>(currentBoardConfig.imu_cs_pin) << 16;

    //transmit starting address with read flag applied
    currentBoardConfig.imu_spi->DR = startRegister | 0x80; //write dummy values to update the register
    while (!(currentBoardConfig.imu_spi->SR & SPI_SR_TXE)) {}
    while (currentBoardConfig.imu_spi->SR & SPI_SR_BSY) {}
    (void)currentBoardConfig.imu_spi->DR; //clear out corrupted values

    //populate locally
    for (unsigned char & byte : localContainer.bytes) {
        currentBoardConfig.imu_spi->DR = 0x00; //write to update register
        while (!(currentBoardConfig.imu_spi->SR & SPI_SR_RXNE)) {} //wait for it to populate
        byte = currentBoardConfig.imu_spi->DR; //get the actual bit
    }

    currentBoardConfig.imu_cs_port->BSRR = currentBoardConfig.imu_cs_pin;

    return localContainer;
}

CRSFPacket populateCRSFBuffer() {
    CRSFPacket localContainer = {0};
    static uint16_t readIndex = 0;
    uint16_t bytesCopied = 0;

    //ndtr reads how much buffer space is left e.g. 64 of space, 10 bits arrive ndtr = 54
    //use the inverse to poll the write index of the hardware buffer
    const uint16_t headIndex = HW_UART_BUFFER_SIZE - currentBoardConfig.crsf_dma_stream->NDTR;

    while (readIndex != headIndex && bytesCopied < 26) { //if 26 bits (1 elrs packet) has been reached, AND we are reading at the point where the hardware is writing
        extern uint8_t crsfRingBuffer[HW_UART_BUFFER_SIZE];
        localContainer.bytes[bytesCopied] = crsfRingBuffer[readIndex];
        bytesCopied++;

        readIndex++;
        if (readIndex >= HW_UART_BUFFER_SIZE) {
            readIndex = 0;
        }
    }

    return localContainer;
}