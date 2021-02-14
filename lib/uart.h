#ifndef UART_H
#define UART_H

#include "stm8l15x.h"
#include "stdio.h"


#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#ifndef F_CPU
#warning "F_CPU not defined, using 2MHz by default"
#define F_CPU 2000000UL
#endif

/**
 * Initialize UART1.
 * Mode: 8-N-1, flow-control: none.
 *
 * PA2 -> TX
 * PA3 -> RX
 */
void uart_init();

void uart_write(uint8_t data);

uint8_t uart_read();

int putchar (int c);
int getchar (void);

#endif /* UART_H */
