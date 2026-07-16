//
// Created by Akshay Gillett on 7/12/26.
//

#include "BufferPopulation.h"
#include "flight_control.h" //gives access to boardconfig
#include "../rtos/rtos.h"

#define HW_UART_BUFFER_SIZE 64

static uint8_t imuRawBuffer[12];

void startIMU_DMARead() {
    currentBoardConfig.imu_cs_port->BSRR = static_cast<uint32_t>(currentBoardConfig.imu_cs_pin) << 16; //pull pin low to start read

    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0; //clear all other flags

    DMA2_Stream0->NDTR = 12;
    DMA2_Stream0->M0AR = reinterpret_cast<uint32_t>(imuRawBuffer); //where the buffer starts

    DMA2_Stream0->CR |= DMA_SxCR_TCIE | DMA_SxCR_EN;
    currentBoardConfig.imu_spi->CR2 |= SPI_CR2_RXDMAEN; //points SPI to trigger through DMA
}

IMURawPacket populateIMUBuffer() {
    startIMU_DMARead(); //start dma read
    yieldCurrentTask(); //yield so there is no wait

    currentBoardConfig.imu_cs_port->BSRR = currentBoardConfig.imu_cs_pin; //pull high to end the SPI transaction

    IMURawPacket rawPacket;
    for (int i = 0; i < 12; i++) {
        rawPacket.bytes[i] = imuRawBuffer[i];
    }

    return rawPacket; //return packet to reader
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
        __disable_irq();
        localContainer.bytes[bytesCopied] = crsfRingBuffer[readIndex];
        bytesCopied++;

        readIndex++;
        if (readIndex >= HW_UART_BUFFER_SIZE) {
            readIndex = 0;
        }
        __enable_irq();
    }



    return localContainer;
}