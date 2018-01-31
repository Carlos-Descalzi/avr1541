#ifndef _VIA_H_
#define _VIA_H_

#include <avr/io.h>

typedef void (*ViaPortWriteFunc)(uint8_t,uint8_t,uint8_t,uint8_t);
typedef void (*ViaPortReadFunc)(uint8_t,uint8_t*);
typedef void (*ViaIRQFunc)(uint8_t);
#define VIA_PORT_A 0x01
#define VIA_PORT_B 0x02

#define ADDR_PORTB_DIR 0x02
#define ADDR_PORTA_DIR 0x03

#define REG_PB		0x00
#define	REG_PA		0x01
#define REG_DDRB	0x02
#define REG_DDRA	0x03
#define REG_T1L		0x04
#define REG_T1H		0x05
#define REG_TL1L	0x06
#define REG_TL1H	0x07
#define REG_SR		0x0A
// 0x08, 0x09, 0x0A Timer 2 and shift register unused
#define REG_ACR		0x0B
#define REG_PCR		0x0C
#define REG_IFR		0x0D		// IFR: Interrupt Flag Register
#define REG_IER		0x0E		// IER: Interrupt Enable Register


#define ACR_PA		0x01
#define ACR_PB		0x02
#define ACR_SR1		0x04
#define ACR_SR2		0x08
#define ACR_SR3		0x10
#define ACR_T2		0x20
#define ACR_T1_1	0x40
#define ACR_T1_2	0x80


#define IER_CA2		0x01
#define IER_CA1		0x02
#define IER_SR		0x04
#define IER_CB2		0x08
#define IER_CB1		0x10
#define IER_TMR2	0x20
#define IER_TMR1	0x40
#define IER_SET		0x80


#define	IFR_CA2		0x01
#define IFR_CA1		0x02
#define	IFR_SR		0x04
#define IFR_CB2		0x08
#define IFR_CB1		0x10
#define IFR_TMR2	0x20
#define IFR_TMR1	0x40
#define IFR_ANY		0x80

#define CA_CA1	0x01
#define CA_CA2	0x02
#define CB_CB1	0x04
#define CB_CB2	0x08

typedef union _ttimer ttimer;

union _ttimer {
	struct {
		unsigned char l;
		unsigned char h;
	} b;
	unsigned short w;
};

typedef struct {
	char id[4];
	uint8_t ora;
	uint8_t orb;
	uint8_t ddra;
	uint8_t cab;
	uint8_t ira;
	uint8_t ira_latch;
	uint8_t irb;
	uint8_t irb_latch;
	uint8_t ddrb;
	ttimer timer;
	ttimer timer_latch;
	uint8_t pcr;
	uint8_t ifr;
	uint8_t ier;
	uint8_t acr;
	ViaPortWriteFunc port_write_func;
	ViaPortReadFunc port_read_func;
	ViaIRQFunc irq_func;
	uint8_t irq_in_queue;
} Via;

void via_setup(Via* via,ViaPortWriteFunc port_write_func,ViaPortReadFunc port_read_func,ViaIRQFunc irq_func);
void via_reset(Via* via);
void via_loop(Via* via);
void via_write(Via* via,uint8_t addr, uint8_t val);
uint8_t via_read(Via* via,uint8_t addr);
void via_do_irq(Via* via,uint8_t mask);

#endif
