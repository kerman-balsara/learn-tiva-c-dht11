#ifndef GPIORTNS_H
#define GPIORTNS_H

typedef enum
{
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    PORT_F
} gpio_port_t;

void GpioEnable                 (const gpio_port_t port);

#endif // GPIORTNS_H
