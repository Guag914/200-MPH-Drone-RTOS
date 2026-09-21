//
// BufferPopulation.cpp by Akshay Gillett on 7/12/26.
//


#include "BufferPopulation.h"
#include "./flight_control.h" //gives access to boardconfig
#include "../rtos/rtos.h"
#include "../sim/SimInjector.h"
#include "Helpers.h"

// uint8_t imuRawBuffer[12] = {0};
uint8_t imuRingBuffer[IMU_RING_BUFFER_SIZE] = {0};
uint8_t crsfRingBuffer[CRSF_RING_BUFFER_SIZE] = {0};
uint16_t batteryADCRingBuffer[BATTERY_ADC_BUFFER_SIZE] = {0};
uint8_t baroBuffer[BARO_BUFFER_SIZE]; //24 slots for each temp and pressure
char gpsRingBuffer[GPS_RING_BUFFER_SIZE];
uint8_t osdRingBuffer[OSD_RING_BUFFER_SIZE];

void startIMU_DMARead() {
    currentBoardConfig.imu_cs_port->BSRR = static_cast<uint32_t>(currentBoardConfig.imu_cs_pin) << 16;

    currentBoardConfig.imu_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.imu_dma_stream->CR & DMA_SxCR_EN) {}

    if (reinterpret_cast<uint32_t>(currentBoardConfig.imu_dma_stream) < DMA1_Stream4_BASE) {
        DMA1->LIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.imu_dma_stream) - DMA1_Stream0_BASE) / 0x18 * 6);
    } else {
        DMA1->HIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.imu_dma_stream) - DMA1_Stream4_BASE) / 0x18 * 6);
    }

    currentBoardConfig.imu_dma_stream->PAR  = reinterpret_cast<uint32_t>(&currentBoardConfig.imu_spi->DR);
    currentBoardConfig.imu_dma_stream->M0AR = reinterpret_cast<uint32_t>(imuRingBuffer);
    currentBoardConfig.imu_dma_stream->NDTR = IMU_RING_BUFFER_SIZE;

    currentBoardConfig.imu_dma_stream->CR = (currentBoardConfig.imu_spi_dma_channel << DMA_SxCR_CHSEL_Pos)
                                       | DMA_SxCR_MINC
                                       | DMA_SxCR_CIRC
                                       | DMA_SxCR_HTIE  //half transfer irq call
                                       | DMA_SxCR_TCIE  //full transfer irq call
                                       | DMA_SxCR_EN;

    currentBoardConfig.imu_spi->CR2 |= SPI_CR2_RXDMAEN;
}

void startCRSF_DMARead() {
    currentBoardConfig.crsf_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.crsf_dma_stream->CR & DMA_SxCR_EN) {}

    if (reinterpret_cast<uint32_t>(currentBoardConfig.crsf_dma_stream) < DMA1_Stream4_BASE) {
        DMA1->LIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.crsf_dma_stream) - DMA1_Stream0_BASE) / 0x18 * 6);
    } else {
        DMA1->HIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.crsf_dma_stream) - DMA1_Stream4_BASE) / 0x18 * 6);
    }

    currentBoardConfig.crsf_dma_stream->PAR  = reinterpret_cast<uint32_t>(&currentBoardConfig.crsf_uart->RDR);
    currentBoardConfig.crsf_dma_stream->M0AR = reinterpret_cast<uint32_t>(crsfRingBuffer);
    currentBoardConfig.crsf_dma_stream->NDTR = CRSF_RING_BUFFER_SIZE;

    currentBoardConfig.crsf_dma_stream->CR = (currentBoardConfig.crsf_rx_dma_channel << DMA_SxCR_CHSEL_Pos)
                                       | DMA_SxCR_MINC
                                       | DMA_SxCR_CIRC
                                       | DMA_SxCR_HTIE //half transfer irq
                                       | DMA_SxCR_TCIE //full transfer irq
                                       | DMA_SxCR_EN;

    currentBoardConfig.crsf_uart->CR3 |= USART_CR3_DMAR;
}

//converted to ring buffer
IMURawPacket populateIMUBuffer() {
    IMURawPacket localContainer = {0};
    static uint16_t readIndex = 0;
    uint16_t bytesCopied = 0;

    __disable_irq();
    const uint16_t headIndex = IMU_RING_BUFFER_SIZE - currentBoardConfig.imu_dma_stream->NDTR;


    while (readIndex != headIndex && bytesCopied < 12) {
        localContainer.bytes[bytesCopied] = imuRingBuffer[readIndex];
        bytesCopied++;

        readIndex++;
        if (readIndex >= IMU_RING_BUFFER_SIZE) {
            readIndex = 0;
        }
    }
    __enable_irq();
    return localContainer;
}

CRSFPacket populateCRSFBuffer() {
    CRSFPacket localContainer = {0};
    static uint16_t readIndex = 0;
    uint16_t bytesCopied = 0;

    //ndtr reads how much buffer space is left e.g. 64 of space, 10 bits arrive ndtr = 54
    //use the inverse to poll the write index of the hardware buffer
    __disable_irq();
    const uint16_t headIndex = CRSF_RING_BUFFER_SIZE - currentBoardConfig.crsf_dma_stream->NDTR;

    while (readIndex != headIndex && bytesCopied < 26) { //if 26 bits (1 elrs packet) has been reached, AND we are reading at the point where the hardware is writing
        localContainer.bytes[bytesCopied] = crsfRingBuffer[readIndex];
        bytesCopied++;

        readIndex++;
        if (readIndex >= CRSF_RING_BUFFER_SIZE) {
            readIndex = 0;
        }
    }
    __enable_irq();

    return localContainer;
}

//dshotgeneration helpers

//motor 1: TIM1_CH1 -> DMA2, stream 1
void startMotor1_DMATransfer() {
    currentBoardConfig.motor1_dma_stream->CR &= ~DMA_SxCR_EN;
    currentBoardConfig.motor1_dma_stream->PAR = reinterpret_cast<uint32_t>(&(currentBoardConfig.alt_motor_timer->CCR1));
    currentBoardConfig.motor1_dma_stream->M0AR = reinterpret_cast<uint32_t>(dshotBuffer1);
    currentBoardConfig.motor1_dma_stream->NDTR = 17;
    DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1;
    currentBoardConfig.motor1_dma_stream->CR |= DMA_SxCR_TCIE | DMA_SxCR_EN;
}

//motor 2: TIM1_CH2 -> DMA2, stream 2
void startMotor2_DMATransfer() {
    currentBoardConfig.motor2_dma_stream->CR &= ~DMA_SxCR_EN;
    currentBoardConfig.motor2_dma_stream->PAR = reinterpret_cast<uint32_t>(&(currentBoardConfig.motor_timer->CCR2));
    currentBoardConfig.motor2_dma_stream->M0AR = reinterpret_cast<uint32_t>(dshotBuffer2);
    currentBoardConfig.motor2_dma_stream->NDTR = 17;
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2;
    currentBoardConfig.motor2_dma_stream->CR |= DMA_SxCR_TCIE | DMA_SxCR_EN;
}

//motor 3: TIM3_CH3 -> DMA1, stream 7
void startMotor3_DMATransfer() {
    currentBoardConfig.motor3_dma_stream->CR &= ~DMA_SxCR_EN;
    currentBoardConfig.motor3_dma_stream->PAR = reinterpret_cast<uint32_t>(&(currentBoardConfig.alt_motor_timer->CCR3));
    currentBoardConfig.motor3_dma_stream->M0AR = reinterpret_cast<uint32_t>(dshotBuffer3);
    currentBoardConfig.motor3_dma_stream->NDTR = 17;
    DMA1->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7;
    currentBoardConfig.motor3_dma_stream->CR |= DMA_SxCR_TCIE | DMA_SxCR_EN;
}

//motor 4: TIM1_CH4 -> DMA2, stream 4
void startMotor4_DMATransfer() {
    currentBoardConfig.motor4_dma_stream->CR &= ~DMA_SxCR_EN;
    currentBoardConfig.motor4_dma_stream->PAR = reinterpret_cast<uint32_t>(&(currentBoardConfig.motor_timer->CCR4));
    currentBoardConfig.motor4_dma_stream->M0AR = reinterpret_cast<uint32_t>(dshotBuffer4);
    currentBoardConfig.motor4_dma_stream->NDTR = 17;
    DMA2->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4;
    currentBoardConfig.motor4_dma_stream->CR |= DMA_SxCR_TCIE | DMA_SxCR_EN;
}

//battery dma (voltage + current)
void startBatteryADC_DMA() {
    currentBoardConfig.battery_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.battery_dma_stream->CR & DMA_SxCR_EN) {}

    if (reinterpret_cast<uint32_t>(currentBoardConfig.battery_dma_stream) < DMA2_Stream4_BASE) {
        DMA2->LIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.battery_dma_stream) - DMA2_Stream0_BASE) / 0x18 * 6);
    } else {
        DMA2->HIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.battery_dma_stream) - DMA2_Stream4_BASE) / 0x18 * 6);
    }

    currentBoardConfig.battery_dma_stream->PAR  = reinterpret_cast<uint32_t>(&(currentBoardConfig.battery_adc->DR));
    currentBoardConfig.battery_dma_stream->M0AR = reinterpret_cast<uint32_t>(batteryADCRingBuffer);
    currentBoardConfig.battery_dma_stream->NDTR = BATTERY_ADC_BUFFER_SIZE;

    currentBoardConfig.battery_dma_stream->CR = DMA_SxCR_CIRC
                                               | DMA_SxCR_MINC
                                               | DMA_SxCR_MSIZE_0
                                               | DMA_SxCR_PSIZE_0
                                               | DMA_SxCR_EN;

    currentBoardConfig.battery_adc->CR1 |= ADC_CR1_SCAN;
    currentBoardConfig.battery_adc->CR2 |= ADC_CR2_CONT;

    currentBoardConfig.battery_adc->SQR3 = (currentBoardConfig.battery_voltage_channel) | (currentBoardConfig.battery_current_channel << 5);
    currentBoardConfig.battery_adc->SQR1 |= (1 << 20);

    currentBoardConfig.battery_adc->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    currentBoardConfig.battery_adc->CR2 |= ADC_CR2_ADON;
    currentBoardConfig.battery_adc->CR2 |= ADC_CR2_SWSTART;
}

//battery methods
ADCPacket populateBatteryADCBuffer() {
    ADCPacket localContainer = {0};
    localContainer.bytes[0] = batteryADCRingBuffer[0];
    localContainer.bytes[1] = batteryADCRingBuffer[1];

    return localContainer;
}

//barometer logic:
void initBaro_DMATransfer() {
    currentBoardConfig.baro_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.baro_dma_stream->CR & DMA_SxCR_EN) {}

    currentBoardConfig.baro_dma_stream->PAR  = reinterpret_cast<uint32_t>(&currentBoardConfig.baro_spi->DR);
    currentBoardConfig.baro_dma_stream->M0AR = reinterpret_cast<uint32_t>(baroBuffer);

    currentBoardConfig.baro_dma_stream->CR = (currentBoardConfig.baro_dma_channel << DMA_SxCR_CHSEL_Pos)
                                           | DMA_SxCR_MINC
                                           | DMA_SxCR_TCIE;

    currentBoardConfig.baro_spi->CR2 |= SPI_CR2_RXDMAEN;
}

//converted to use generic method for efficiency and less redundant code
void burstBaro_DMATransfer() {
    currentBoardConfig.baro_cs_port->BSRR = static_cast<uint32_t>(currentBoardConfig.baro_cs_pin) << 16; //cs low

#ifdef SIMULATION
    injectMockBaroDMA(); //no real hardware to trigger a fresh sample
#else
    dmaBlockingTransfer(DMA1, currentBoardConfig.baro_dma_stream, reinterpret_cast<uint32_t>(baroBuffer), 6);
#endif

    currentBoardConfig.baro_cs_port->BSRR = currentBoardConfig.baro_cs_pin; //cs high
}



BaroRawPacket populateBarometerBuffer() {
    BaroRawPacket localContainer = {0};

    //burst read instead of circular
    for (uint8_t i = 0; i < 6; i++) {
        localContainer.bytes[i] = baroBuffer[i];
    }

    return localContainer;
}

void startGPS_DMARead() {
    currentBoardConfig.gps_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.gps_dma_stream->CR & DMA_SxCR_EN) {}

    if (reinterpret_cast<uint32_t>(currentBoardConfig.gps_dma_stream) < DMA1_Stream4_BASE) {
        DMA1->LIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.gps_dma_stream) - DMA1_Stream0_BASE) / 0x18 * 6);
    } else {
        DMA1->HIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.gps_dma_stream) - DMA1_Stream4_BASE) / 0x18 * 6);
    }

    currentBoardConfig.gps_dma_stream->PAR  = reinterpret_cast<uint32_t>(&currentBoardConfig.gps_uart->RDR);
    currentBoardConfig.gps_dma_stream->M0AR = reinterpret_cast<uint32_t>(gpsRingBuffer);
    currentBoardConfig.gps_dma_stream->NDTR = GPS_RING_BUFFER_SIZE;

    currentBoardConfig.gps_dma_stream->CR = (currentBoardConfig.gps_dma_channel << DMA_SxCR_CHSEL_Pos)
                                           | DMA_SxCR_MINC
                                           | DMA_SxCR_CIRC
                                           | DMA_SxCR_EN;

    currentBoardConfig.gps_uart->CR3 |= USART_CR3_DMAR;
}

GPSPacket populateGPSBuffer() {
    GPSPacket localContainer = {0};
    static uint16_t readIndex = 0;
    uint16_t bytesCopied = 0;

    __disable_irq();
    const uint16_t headIndex = GPS_RING_BUFFER_SIZE - currentBoardConfig.gps_dma_stream->NDTR;

    while (readIndex != headIndex && bytesCopied < sizeof(localContainer.bytes)) { //was GPS_RING_BUFFER_SIZE, overflowed the 96-byte struct
        localContainer.bytes[bytesCopied] = gpsRingBuffer[readIndex];
        bytesCopied++;

        readIndex++;
        if (readIndex >= GPS_RING_BUFFER_SIZE) {
            readIndex = 0;
        }
    }
    __enable_irq();

    return localContainer;
}

void initOSD_DMATransfer() { //used for respond to vtx
    currentBoardConfig.osd_tx_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.osd_tx_dma_stream->CR & DMA_SxCR_EN) {}
    currentBoardConfig.osd_tx_dma_stream->PAR = reinterpret_cast<uint32_t>(&currentBoardConfig.osd_uart->TDR);
    currentBoardConfig.osd_tx_dma_stream->CR = (currentBoardConfig.osd_tx_dma_channel << DMA_SxCR_CHSEL_Pos)
                                          | DMA_SxCR_MINC
                                          | DMA_SxCR_DIR_0  //memory to peripheral
                                          | DMA_SxCR_TCIE;  //transfer complete interrupt enabled

    currentBoardConfig.osd_uart->CR3 |= USART_CR3_DMAT;
}

void startOSD_DMARead() { //used for actual osd parsing task
    currentBoardConfig.osd_rx_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.osd_rx_dma_stream->CR & DMA_SxCR_EN) {}

    if (reinterpret_cast<uint32_t>(currentBoardConfig.osd_rx_dma_stream) < DMA2_Stream4_BASE) {
        DMA2->LIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.osd_rx_dma_stream) - DMA2_Stream0_BASE) / 0x18 * 6);
    } else {
        DMA2->HIFCR = 0x3D << ((reinterpret_cast<uint32_t>(currentBoardConfig.osd_rx_dma_stream) - DMA2_Stream4_BASE) / 0x18 * 6);
    }

    currentBoardConfig.osd_rx_dma_stream->PAR  = reinterpret_cast<uint32_t>(&currentBoardConfig.osd_uart->RDR);
    currentBoardConfig.osd_rx_dma_stream->M0AR = reinterpret_cast<uint32_t>(osdRingBuffer);
    currentBoardConfig.osd_rx_dma_stream->NDTR = OSD_RING_BUFFER_SIZE;

    currentBoardConfig.osd_rx_dma_stream->CR = (currentBoardConfig.osd_rx_dma_channel << DMA_SxCR_CHSEL_Pos)
                                           | DMA_SxCR_MINC
                                           | DMA_SxCR_CIRC
                                           | DMA_SxCR_EN;

    currentBoardConfig.osd_uart->CR3 |= USART_CR3_DMAR;
}

OSDPacket populateOSDBuffer() {
    OSDPacket localContainer = {0};
    static uint16_t readIndex = 0;
    uint16_t bytesCopied = 0;

    __disable_irq();
    const uint16_t headIndex = OSD_RING_BUFFER_SIZE - currentBoardConfig.osd_rx_dma_stream->NDTR;

    while (readIndex != headIndex && bytesCopied < OSD_RING_BUFFER_SIZE) {
        localContainer.bytes[bytesCopied] = osdRingBuffer[readIndex];
        bytesCopied++;

        readIndex++;
        if (readIndex >= OSD_RING_BUFFER_SIZE) {
            readIndex = 0;
        }
    }

    localContainer.length = bytesCopied;
    __enable_irq();

    return localContainer;
}

void initTTX_DMATransfer() {
    currentBoardConfig.crsf_tx_dma_stream->CR &= ~DMA_SxCR_EN;
    while (currentBoardConfig.crsf_tx_dma_stream->CR & DMA_SxCR_EN) {}
    currentBoardConfig.crsf_tx_dma_stream->PAR = reinterpret_cast<uint32_t>(&currentBoardConfig.crsf_uart->TDR);
    currentBoardConfig.crsf_tx_dma_stream->CR = (currentBoardConfig.crsf_tx_dma_channel << DMA_SxCR_CHSEL_Pos)
                                          | DMA_SxCR_MINC
                                          | DMA_SxCR_DIR_0  //memory to peripheral
                                          | DMA_SxCR_TCIE;  //transfer complete interrupt enabled

    currentBoardConfig.crsf_uart->CR3 |= USART_CR3_DMAT;
}

static void clearDMAStreamFlags(DMA_TypeDef* dma, DMA_Stream_TypeDef* stream) {
    const uint32_t streamAddr = reinterpret_cast<uint32_t>(stream);
    const uint32_t base0 = (dma == DMA1) ? DMA1_Stream0_BASE : DMA2_Stream0_BASE;
    const uint32_t base4 = (dma == DMA1) ? DMA1_Stream4_BASE : DMA2_Stream4_BASE;

    if (streamAddr < base4) {
        dma->LIFCR = 0x3D << ((streamAddr - base0) / 0x18 * 6);
    } else {
        dma->HIFCR = 0x3D << ((streamAddr - base4) / 0x18 * 6);
    }
}

void dmaBlockingTransfer(DMA_TypeDef* dma, DMA_Stream_TypeDef* stream, uint32_t memAddr, uint16_t length) {
    stream->CR &= ~DMA_SxCR_EN;
    while (stream->CR & DMA_SxCR_EN) {}

    clearDMAStreamFlags(dma, stream);

    stream->M0AR = memAddr;
    stream->NDTR = length;
    stream->CR |= DMA_SxCR_EN;

#ifdef SIMULATION //no real hardware to auto-clear EN, so fake it here
    stream->CR &= ~DMA_SxCR_EN;
#endif

    while (stream->CR & DMA_SxCR_EN) {} //hardware auto-clears EN when NDTR hits 0 (non-circular only)
}