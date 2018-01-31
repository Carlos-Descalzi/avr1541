#ifndef _UP6502_H_
#define _UP6502_H_

#ifdef __AVR__
#include <avr/io.h>
#else
#include <stdio.h>
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
#endif

#define OP_BRK				0x00
#define OP_ORA_X_IND	0x01
#define OP_ORA_ZPG		0x05

#define OP_JSR_ABS		0x20

#define	OP_AND_IMM		0x29
#define OP_SEC				0x38

#define OP_RTI				0x40
#define OP_EOR_IMM		0x49
#define OP_LSR_A			0x4A
#define OP_JMP_ABS		0x4C
#define OP_LSR_ABS		0x4E
#define OP_ROL_A			0x2A
#define OP_RTS				0x60
#define OP_ADC_IMM		0x69
#define	OP_ROR_A			0x6A
#define OP_SEI				0x78
#define OP_TXA				0x8A
#define OP_STX_ABS		0x8E
#define OP_STA_ABS		0x8D
#define	OP_TYA				0x98
#define OP_TXS				0x9A
#define OP_LDY_IMM		0xA0
#define OP_LDX_IMM		0xA2
#define OP_TAY				0xA8
#define OP_LDA_IMM		0xA9
#define OP_TSX				0xBA
#define OP_CMP_IMM		0xC9

#define OP_CPX_IMM		0xE0
#define OP_INX				0xE8
#define OP_SBC_IMM		0xE9
#define OP_NOP				0xEA
#define OP_INC_ABS		0xEE

#define OP_SED				0xF8

#define OP_DEX				0xCA
#define OP_DEY				0x88
#define OP_DEC_ABS		0xCE

#define OP_CLC				0x18
#define OP_BIT_Z			0x24

#define OP_BEQ				0xF0
#define OP_BNE				0xD0
#define OP_BPL				0x10


#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_I 0x04
#define FLAG_D 0x08
#define FLAG_B 0x10
#define FLAG_V 0x40
#define FLAG_N 0x80

typedef uint8_t (*MemReadCallback)(uint16_t, uint8_t*);
typedef uint8_t (*MemWriteCallback)(uint16_t, uint8_t);

void up6502_setup(MemReadCallback read_callback, MemWriteCallback write_callback);
void up6502_reset();
void up6502_loop();
void up6502_irq();

uint8_t up6502_get_a();
uint8_t up6502_get_x();
uint8_t up6502_get_y();
uint8_t up6502_get_p();
uint8_t up6502_get_s();
uint16_t up6502_get_pc();

void up6502_set_a(uint8_t a);
void up6502_set_x(uint8_t x);
void up6502_set_y(uint8_t y);
void up6502_set_p(uint8_t p);
void up6502_set_s(uint8_t s);
void up6502_set_pc(uint16_t pc);
void up6502_set_flags(uint8_t flags);
void up6502_clear_flags(uint8_t flags);
void up6502_toggle_log();
#endif
