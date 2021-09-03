//10.3 Initialization and Configuration
//The GPIO modules may be accessed via two different memory apertures. The legacy aperture, the
//Advanced Peripheral Bus (APB), is backwards-compatible with previous devices. The other aperture,
//the Advanced High-Performance Bus (AHB), offers the same register map but provides better
//back-to-back access performance than the APB bus. These apertures are mutually exclusive. The
//aperture enabled for a given GPIO port is controlled by the appropriate bit in the GPIOHBCTL
//register (see page 258). Note that GPIO can only be accessed through the AHB aperture.
//To configure the GPIO pins of a particular port, follow these steps:
//1. Enable the clock to the port by setting the appropriate bits in the RCGCGPIO register (see
//page 340). In addition, the SCGCGPIO and DCGCGPIO registers can be programmed in the
//same manner to enable clocking in Sleep and Deep-Sleep modes.
//2. Set the direction of the GPIO port pins by programming the GPIODIR register. A write of a 1
//indicates output and a write of a 0 indicates input.
//656 June 12, 2014
//Texas Instruments-Production Data
//General-Purpose Input/Outputs (GPIOs)3. Configure the GPIOAFSEL register to program each bit as a GPIO or alternate pin. If an alternate
//pin is chosen for a bit, then the PMCx field must be programmed in the GPIOPCTL register for
//the specific peripheral required. There are also two registers, GPIOADCCTL and GPIODMACTL,
//which can be used to program a GPIO pin as a ADC or µDMA trigger, respectively.
//4. Set the drive strength for each of the pins through the GPIODR2R, GPIODR4R, and GPIODR8R
//registers.
//5. Program each pad in the port to have either pull-up, pull-down, or open drain functionality through
//the GPIOPUR, GPIOPDR, GPIOODR register. Slew rate may also be programmed, if needed,
//through the GPIOSLR register.
//6. To enable GPIO pins as digital I/Os, set the appropriate DEN bit in the GPIODEN register. To
//enable GPIO pins to their analog function (if available), set the GPIOAMSEL bit in the
//GPIOAMSEL register.
//7. Program the GPIOIS, GPIOIBE, GPIOEV, and GPIOIM registers to configure the type, event,
//and mask of the interrupts for each port.
//Note: To prevent false interrupts, the following steps should be taken when re-configuring
//GPIO edge and interrupt sense registers:
//a. Mask the corresponding port by clearing the IME field in the GPIOIM register.
//b. Configure the IS field in the GPIOIS register and the IBE field in the GPIOIBE
//register.
//c. Clear the GPIORIS register.
//d. Unmask the port by setting the IME field in the GPIOIM register.
//8. Optionally, software can lock the configurations of the NMI and JTAG/SWD pins on the GPIO
//port pins, by setting the LOCK bits in the GPIOLOCK register.
//When the internal POR signal is asserted and until otherwise configured, all GPIO pins are configured
//to be undriven (tristate): GPIOAFSEL=0, GPIODEN=0, GPIOPDR=0, and GPIOPUR=0, except for
//the pins shown in Table 10-1 on page 650. Table 10-3 on page 657 shows all possible configurations
//of the GPIO pads and the control register settings required to achieve them. Table 10-4 on page 658
//shows how a rising edge interrupt is configured for pin 2 of a GPIO port.

#include "TM4C123GH6PM.h"

#include "gpiortns.h"

void GpioEnable(const gpio_port_t port)
{
    if (SYSCTL->PRGPIO & (1 << port))
        return;
    // Disable clock gating for GPIO. See page 656 (Initialisation and Configuration step 1) and
    // page 340 (Register 60).
    SYSCTL->RCGCGPIO |= (1 << port);

    // There must be a delay of 3 system clocks after a peripheral module clock is enabled in the RCGC register
    // before any module registers are accessed. See page 227 (System Control). We check PRGPIO; see page 406.
    while (1)
    {
        if (SYSCTL->PRGPIO & (1 << port))
            break;
    }
}

