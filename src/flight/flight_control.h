#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <cstring>
#include <stdint.h>
#include "../rtos/rtos.h"
#include <string>
#include <string_view>
#include "stm32f7xx.h"

#ifdef __cplusplus //this section is completely HIDDEN from the C compiler (main.c)
#define TOTAL_BATTERY_MAH 1600 //add to cli
#define EVENT_MSG_LEN 64

enum class FlightState : uint8_t {
    DISARMED, //calculations running, DShot motor output forced to 0/Idle-disabled
    ARMED, //full control active, PID outputs mixed into DShot commands
    WARN, //active warning blocked from arming, displaying OSD text
    FAILSAFE, //critical lock, motors killed instantly, requires manual reset
    ERROR //error thrown by code, logs + instantly kills motors
};

inline std::string to_string(volatile FlightState state) {
    switch (state) {
            case FlightState::DISARMED: return "DISARMED";
            case FlightState::ARMED: return "ARMED";
            case FlightState::WARN: return "WARN";
            case FlightState::FAILSAFE: return "FAILSAFE";
            case FlightState::ERROR: return "ERROR";
            default: return "";
    }
}

inline void setEventMessage(char* dest, std::string_view src) {
    std::size_t len = src.size() < EVENT_MSG_LEN - 1 ? src.size() : EVENT_MSG_LEN - 1;
    std::memcpy(dest, src.data(), len);
    dest[len] = '\0';
}

struct WARN { //add warnings and their signal to the pilot via the OSD
    static constexpr std::string_view LOW_BATTERY = "LOW BATTERY DETECTED: LAND NOW!\n";
    static constexpr std::string_view ARMED_ANGLE = "PLACE DRONE ON A FLAT SURFACE TO ARM!\n";
    static constexpr std::string_view ARMED_THROTTLE = "PUSH THROTTLE TO 0 TO ARM!\n";
    static constexpr std::string_view BARO_NO_RESPONSE = "BAROMETER NOT RESPONDING: ALTITUDE INVALID!\n";
};

struct FAILSAFE { //add failsafe messages
    static constexpr std::string_view CRSF_LOST = "CRSF RADIO LINK LOST: FORCE DISARM\n";
};

struct ERROR {
    static constexpr std::string_view INIT_HEAP_MEM = "OUT OF HEAP MEMORY ALLOCATING TASK STACK\n";
    static constexpr std::string_view STACK_OVERFLOW = "STACK OVERFLOW DETECTED!\n";
    static constexpr std::string_view PREEMPT_OVERFLOW = "PREEMPT STACK OVERFLOW!\n";
    static constexpr std::string_view GUARD_BREACH = "TOP GUARD BREACH!\n";
    static constexpr std::string_view CRSF_CORRUPT_STREAM = "CORRUPTED DATA INPUT STREAM!\n";
};

struct DroneState {
    //rc inputs (mapped from crsf)
    float rollStick; //-1.0 to 1.0
    float pitchStick; //-1.0 to 1.0
    float yawStick; //-1.0 to 1.0
    float throttleStick; //0.0 to 1.0
    bool  armed;

    //sensor inputs (Raw & Calibrated)
    float gyroRaw[3]; //xyz
    float gyroCalibrated[3]; //after bias subtraction
    float gyroFiltered[3]; //after pt1 filter

    float motorOutput[4];

    float accelRaw[3]; //xyz

    bool calibrated;

    //orientation estimates
    float estimatedRoll;
    float estimatedPitch;
    float estimatedYaw;

    float attitudeRoll;
    float attitudePitch;
    float attitudeYaw;

    //protection safes
    char warnMSG[EVENT_MSG_LEN]; //used for warning the pilot about drone stats - needs attention - uses same log and osd message
    char failsafeMSG[EVENT_MSG_LEN]; //used to force disarm the drone completely - drone state critical - prints simplified message on OSD + logs drone stats
    char errorMSG[EVENT_MSG_LEN]; //also used to force disarm the drone - code threw exeption/error occured during runtime - prints "error + id" + logs all details

    //system health
    float batteryVoltage;
    float batteryCurrent;
    uint32_t batteryCapacityUsed;
    uint8_t batteryPerecent;

    uint32_t lastValidRCFrameTime;
    FlightState currentSystemState;

    //gps logic
    int32_t latitude;
    int32_t longitude;
    uint16_t altitude;
    uint16_t groundSpeed;
    uint16_t heading;
    uint8_t satellites;
};

struct HardwarePinMap {
    USART_TypeDef* cli_uart;

    USART_TypeDef* crsf_uart;
    DMA_Stream_TypeDef* crsf_dma_stream;
    uint32_t crsf_dma_channel;

    DMA_Stream_TypeDef* imu_dma_stream;
    SPI_TypeDef* imu_spi;
    uint32_t imu_spi_dma_channel;
    GPIO_TypeDef* imu_cs_port;
    uint16_t imu_cs_pin;

    //baro logic
    SPI_TypeDef* baro_spi;
    DMA_Stream_TypeDef* baro_dma_stream;
    uint32_t baro_dma_channel;
    GPIO_TypeDef* baro_cs_port;
    uint16_t baro_cs_pin;
    uint32_t baro_dma_stream_num;

    //batery logic
    ADC_TypeDef* battery_adc;
    DMA_Stream_TypeDef* battery_dma_stream;
    uint32_t battery_voltage_channel;
    uint32_t battery_current_channel;

    TIM_TypeDef* motor_timer;

    DMA_Stream_TypeDef* motor1_dma_stream;
    DMA_Stream_TypeDef* motor2_dma_stream;
    DMA_Stream_TypeDef* motor3_dma_stream;
    DMA_Stream_TypeDef* motor4_dma_stream;

    USART_TypeDef* osd_uart;

    //peripheral GPIOB pins to hardware map (led, buzzer etc.)
    GPIO_TypeDef* status_led_port;
    uint16_t      status_led_pin;

    GPIO_TypeDef* buzzer_port;
    uint16_t      buzzer_pin;

    //gps unit
    USART_TypeDef* gps_uart;

    USART_TypeDef* log_uart;

#if SIMULATION && DEBUG

#endif
};


inline HardwarePinMap currentBoardConfig = { //ensure to configure this properly in stmcubemx software when getting real pinmaps
    //usb cli commands
    .cli_uart = USART1,

    //crsf data streams
    .crsf_uart       = USART2,
    .crsf_dma_stream = DMA1_Stream5,
    .crsf_dma_channel = 4,

    //imu data streams
    .imu_dma_stream = DMA1_Stream0, //UPDATE STMCUBEMX
    .imu_spi = SPI3,
    .imu_spi_dma_channel = 0,
    .imu_cs_port = GPIOA,
    .imu_cs_pin = GPIO_PIN_4,

    //baro streams
    .baro_spi = SPI2,
    .baro_dma_stream = DMA1_Stream3,
    .baro_dma_channel = 3,
    .baro_cs_port = GPIOD,
    .baro_cs_pin = GPIO_PIN_2,
    .baro_dma_stream_num = 3,

    //battery data streams
    .battery_adc = ADC1, //timer
    .battery_dma_stream = DMA2_Stream0,
    .battery_voltage_channel = ADC_CHANNEL_10,
    .battery_current_channel = ADC_CHANNEL_11,

    //dshot motor timers + dma
    .motor_timer = TIM1, //map physical timer
    .motor1_dma_stream = DMA2_Stream1,
    .motor2_dma_stream = DMA2_Stream2,
    .motor3_dma_stream = DMA2_Stream6,
    .motor4_dma_stream = DMA2_Stream4,

    //osd
    .osd_uart = USART6,

    //peripheral gpiob pins (misc for power)
    .status_led_port = GPIOB,
    .status_led_pin  = GPIO_PIN_0,

    .buzzer_port = GPIOC,
    .buzzer_pin = GPIO_PIN_1,

    .gps_uart = USART3,

#if defined(SIMULATION) && defined(DEBUG)
    .log_uart = USART3
#endif
};

struct ActualRateConfig {
    float centerRate = 200.0f;
    float maxRate    = 670.0f;
    float expo       = 0.5f;
};

//make all tasks public to main.cpp
extern void sensorCalibration(); //init only

//these act as helpers for imuControlLoop
extern void readIMUDataRegisters();
extern void stateEstimation();
extern void flightLoop();

//interrupts + scheduled
[[noreturn]] extern void crsfParsing();
[[noreturn]] extern void radioLinkFailSafe();
[[noreturn]] extern void powerManagement();
[[noreturn]] extern void lowLevelFailSafe();
[[noreturn]] extern void dShotGeneration();
[[noreturn]] extern void flightStateMachine();
[[noreturn]] extern void imuControlLoop();
[[noreturn]] extern void iwdgTask();
[[noreturn]] extern void updatePeripherals();
[[noreturn]] extern void gpsParser();
[[noreturn]] extern void usbCLI();
[[noreturn]] extern void osdUpdate();
[[noreturn]] extern void readBarometerRegisters();
[[noreturn]] extern void checkStackHealth();


#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
//this section is visible to BOTH core/src/main.c (C) and src/Main.cpp (C++)

extern DroneState drone;

extern volatile uint16_t batteryADCBuffer[2];

constexpr uint8_t MSP_DP_RELEASE = 0x00;
constexpr uint8_t MSP_DP_WRITE   = 0x01;
constexpr uint8_t MSP_DP_DRAW    = 0x04;

//position buffer data
//row, col
extern uint8_t batVolPos[2];
extern uint8_t batCurPos[2];
extern uint8_t batMahPos[2];

extern uint8_t gpsLatPos[2];
extern uint8_t gpsLonPos[2];
extern uint8_t gpsAltPos[2];
extern uint8_t gpsSpdPos[2];
extern uint8_t gpsSatPos[2];

extern uint8_t drnMsgPos[2];

extern uint8_t baroBuffer[6];

#ifdef __cplusplus
}
#endif

#endif // FLIGHT_CONTROL_H