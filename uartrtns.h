#ifndef UARTRTNS_H
#define UARTRTNS_H

#include "stdint.h"

typedef enum
{
    UART_0 = 0,
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_6,
    UART_7
} uart_pin_t;

void UartEnable                 (const uart_pin_t pin);

#endif // UARTRTNS_H
