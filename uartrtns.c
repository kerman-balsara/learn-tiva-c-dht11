#include "TM4C123GH6PM.h"

#include "uartrtns.h"

void UartEnable(const uart_pin_t pin)
{
    // Disable clock gating for UART. See page 656 (Initialisation and Configuration step 1) and
    // page 344 (Register 63).
    SYSCTL->RCGCUART |= (1 << pin);

    // There must be a delay of 3 system clocks after a peripheral module clock is enabled in the UART register
    // before any module registers are accessed. See page 227 (System Control). We check PRUART; see page 410.
    while (1)
    {
        if (SYSCTL->PRUART & (1 << pin))
            break;
    }
}
