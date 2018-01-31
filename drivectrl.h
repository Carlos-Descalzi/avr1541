#ifndef _DRIVECTRL_H_
#define _DRIVECTRL_H_

#include <avr/io.h>

typedef void (*DriveCtrlIrqFunc)(uint8_t);

void drivectrl_setup(DriveCtrlIrqFunc irq_func);
void drivectrl_reset();
uint8_t drivectrl_read(uint8_t address);
void drivectrl_write(uint8_t address,uint8_t data);
void drivectrl_loop();

#endif
