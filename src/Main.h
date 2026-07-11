//
// Created by Akshay Gillett on 7/9/26.
//

#ifndef MAIN_H
#define MAIN_H

#define SYSTICK_BASE_ADDRESS (0xE000E010UL)
#define CPU_SysTick ((volatile uint32_t*) SYSTICK_BASE_ADDRESS)

#include <cstdint>

// Forward declarations of system engine functions
bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName);
[[noreturn]] void executeTaskLoop();
extern "C" void SysTick_Handler();
// The core runtime entry point
int main();

// Interrupt Zone Functions (Event-Driven)
void readIMUDataRegisters();
void stateEstimation();
void flightLoop();
void crsfParsing();
void radioLinkFailSafe();
void dShotGeneration();

// Scheduled Zone Functions (Time-Driven)
// High Priority
[[noreturn]] void lowLevelFailSafe();

// Medium Priority
[[noreturn]] void telemetryTX();
[[noreturn]] void osdUpdate();
[[noreturn]] void gpsParser();

// Low Priority
[[noreturn]] void blackboxLogging();
[[noreturn]] void updatePeripherals();
[[noreturn]] void usbCLI();

// Independent Watchdog (Hardware Safeguard)
[[noreturn]] void iswg();

#endif // MAIN_H