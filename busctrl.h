#ifndef _BUSCTRL_H_
#define _BUSCTRL_H_

#include <avr/io.h>

#define PORTB_DATA_IN		0x01
#define PORTB_DATA_OUT	0x02
#define PORTB_CLK_IN		0x04
#define PORTB_CLK_OUT		0x08
#define PORTB_ATNA_OUT	0x10
#define PORTB_ATN_IN		0x80


typedef void (*BusCtrlIrqFunc)(uint8_t);
typedef void (*BusResetFunc)();

void busctrl_setup(BusCtrlIrqFunc irq_func,BusResetFunc reset_func);
void busctrl_reset();
void busctrl_write(uint8_t addr, uint8_t val);
uint8_t busctrl_read(uint8_t addr);
void busctrl_loop();
void busctrl_dump_ports();

#endif
