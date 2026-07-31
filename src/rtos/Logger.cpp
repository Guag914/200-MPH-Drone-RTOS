//
// Created by Akshay Gillett on 7/16/26.
//

#include "Logger.h"

#include <stdint.h>
#include <string>
#include "../flight/flight_control.h"

#include "stm32f756xx.h"

//normal simulation debug prints
#if defined(SIMULATION) || defined(USER_TASKS)
    void printToUSART(char c) {
        currentBoardConfig.log_uart->CR1 |= (USART_CR1_TE | USART_CR1_UE);

        while (!(currentBoardConfig.log_uart->ISR & USART_ISR_TXE)) {}
        currentBoardConfig.log_uart->TDR = c;
    }
#else
    void printToUSART(char c) {
        //handle logging to a different peripheral / ground center / internal storage
    }
#endif

void printToUSART(const char* str) {
    if (!str) return;
    while (*str) {
        printToUSART(*str++);
    }
}

void printToUSART(int c) {
    printToUSART(static_cast<int32_t>(c));
}

void printToUSART(const std::string& c) {
    printToUSART(c.c_str());
}

void printToUSART(uint8_t val) {
    printToUSART(static_cast<uint32_t>(val));
}

void printToUSART(uint16_t val) {
    printToUSART(static_cast<uint32_t>(val));
}

void printToUSART(uint32_t val) {
    if (val == 0) {
        printToUSART('0');
        return;
    }
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    while (val > 0) {
        buf[--i] = '0' + (val % 10);
        val /= 10;
    }
    printToUSART(&buf[i]);
}

void printToUSART(int32_t val) {
    if (val < 0) {
        printToUSART('-');
        val = -val;
    }
    printToUSART(static_cast<uint32_t>(val));
}

void printToUSART(float val) {
    printToUSART(static_cast<double>(val));
}

void printToUSART(double val) {
    if (val < 0.0) {
        printToUSART('-');
        val = -val;
    }

    auto integerPart = static_cast<uint32_t>(val);
    printToUSART(integerPart);
    printToUSART('.');

    double fractionalPart = val - static_cast<double>(integerPart);
    for (int i = 0; i < 4; i++) { //print to 4 decimal places
        fractionalPart *= 10.0;
        auto digit = static_cast<uint32_t>(fractionalPart);
        printToUSART(static_cast<char>('0' + digit));
        fractionalPart -= digit;
    }
}

void printToUSART(bool b) {
    if (b) { printToUSART("true"); }
    else { printToUSART("false"); }
}

//CLI custom print function
void printToCLI(char c) {
    currentBoardConfig.cli_uart->CR1 |= (USART_CR1_TE | USART_CR1_UE);

    while (!(currentBoardConfig.cli_uart->ISR & USART_ISR_TXE)) {}
    currentBoardConfig.cli_uart->TDR = c;
}

void printToCLI(const char* str) {
    if (!str) return;
    while (*str) {
        printToCLI(*str++);
    }
}

void printToCLI(int c) {
    printToCLI(static_cast<int32_t>(c));
}

void printToCLI(uint8_t val) {
    printToCLI(static_cast<uint32_t>(val));
}

void printToCLI(uint16_t val) {
        printToCLI(static_cast<uint32_t>(val));
}

void printToCLI(uint32_t val) {
        if (val == 0) {
            printToCLI('0');
            return;
        }
        char buf[11];
        int i = 10;
        buf[i] = '\0';
        while (val > 0) {
            buf[--i] = '0' + (val % 10);
            val /= 10;
        }
        printToCLI(&buf[i]);
}

void printToCLI(int32_t val) {
    if (val < 0) {
        printToCLI('-');
        val = -val;
    }
    printToCLI(static_cast<uint32_t>(val));
}

void printToCLI(float val) {
    printToCLI(static_cast<double>(val));
}

void printToCLI(double val) {
    if (val < 0.0) {
        printToCLI('-');
        val = -val;
    }

    auto integerPart = static_cast<uint32_t>(val);
    printToCLI(integerPart);
    printToCLI('.');

    double fractionalPart = val - static_cast<double>(integerPart);
    for (int i = 0; i < 4; i++) { //print to 4 decimal places
        fractionalPart *= 10.0;
        auto digit = static_cast<uint32_t>(fractionalPart);
        printToCLI(static_cast<char>('0' + digit));
        fractionalPart -= digit;
    }
}

void printToCLI(bool b) {
    if (b) { printToCLI("true"); }
    else { printToCLI("false"); }
}