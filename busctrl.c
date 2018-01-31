#include "busctrl.h"
#include "via.h"
#include "debug.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <string.h>

const char DUMP_MSG_FMT[] PROGMEM = "BUS:\nATN :%c\nATNA:%c\nCLKO:%c\nCLKI:%c\nDATO:%c\nDATI:%c\nOUT:%02x\n";
static const char VIA_ID[] PROGMEM = "BUS";

#define BUS_PORT		PORTA
#define BUS_PORT_IN	PINA
#define BUS_PORTD		DDRA

#define PIN_CLK_IN		0x01
#define PIN_DATA_IN		0x02
#define PIN_ATN_IN		0x04
#define PIN_RESET			0x08
#define PIN_CLK_OUT		0x10
#define PIN_DATA_OUT	0x20

static Via via;

BusResetFunc _reset_func;
uint8_t reset_val;
static void via_port_write_callback(uint8_t port, uint8_t val, uint8_t dir_mask, uint8_t old_value);
static void via_port_read_callback(uint8_t port, uint8_t* val);
static void check_inputs();

void busctrl_setup(BusCtrlIrqFunc irq_func,BusResetFunc reset_func){
	_reset_func = reset_func;
	reset_val = 0;
	BUS_PORTD = 0x30;
	strcpy_P(via.id,VIA_ID);
	via_setup(&via,via_port_write_callback,via_port_read_callback,irq_func);
	busctrl_reset();
}

void busctrl_reset(){
	via_reset(&via);
	via_write(&via,ADDR_PORTA_DIR,0xFF);
	via_write(&via,ADDR_PORTB_DIR,0x1A);
}

void busctrl_write(uint8_t addr, uint8_t val){
	via_write(&via,addr, val);
}
uint8_t busctrl_read(uint8_t addr){
	return via_read(&via,addr);
}

static void update_clk_out(uint8_t val){
	if (val & PORTB_CLK_OUT){
		BUS_PORT |= PIN_CLK_OUT;
	} else {
		BUS_PORT &= ~PIN_CLK_OUT;
	}
}

#define handle_atna(via) (((via.orb & PORTB_ATNA_OUT) != 0) != ((via.irb & PIN_ATN_IN) !=0))

const char MSG_ATNA_H[] PROGMEM = "ATNA HIGH\n";
const char MSG_ATNA_L[] PROGMEM = "ATNA LOW\n";

static void update_data_out(uint8_t val){
	if (handle_atna(via)){
		//DEBUGP(MSG_ATNA_H);
		BUS_PORT |= PIN_DATA_OUT;
	} else {
		if (val & PORTB_DATA_OUT){
			BUS_PORT |= PIN_DATA_OUT;
		} else {
			BUS_PORT &= ~PIN_DATA_OUT;
		}
	}
} 

static void via_port_write_callback(uint8_t port, uint8_t val, uint8_t dir_mask, uint8_t old_value){
	if (port == VIA_PORT_B){
		update_clk_out(val);
		update_data_out(val);
	}
}

void busctrl_loop(){
	check_inputs();
	via_loop(&via);
}

static void via_port_read_callback(uint8_t port, uint8_t* val){
}

const char MSG_ATN_HI[] PROGMEM = "ATN HI\n";
const char MSG_ATN_LO[] PROGMEM = "ATN LO\n";

const char MSG_D1[] PROGMEM = "D 1\n";
const char MSG_C1[] PROGMEM = "C 1\n";

const char MSG_D0[] PROGMEM = "D 0\n";
const char MSG_C0[] PROGMEM = "C 0\n";

static void check_inputs(){
	uint8_t old_portbval = via.irb;

	if (!(BUS_PORT_IN & PIN_ATN_IN)){
		via.irb |= PORTB_ATN_IN;
		if (!(old_portbval & PORTB_ATN_IN)){
			via.cab |= CA_CA1;
			//BUS_PORT |= PIN_DATA_OUT;
			via_do_irq(&via,0x02);
			//DEBUGP(MSG_ATN_HI);
			//PORTD |= 0x60;
		} 
	} else {
		via.cab &= ~CA_CA1;
		via.irb &= ~PORTB_ATN_IN;
		if ((old_portbval & PORTB_ATN_IN)){
			//DEBUGP(MSG_ATN_LO);
			//PORTD &= ~0x60;
		}
	}
	
	update_data_out(via.orb);
	
	if (!(BUS_PORT_IN & PIN_DATA_IN)){
		via.irb |= PORTB_DATA_IN;
		//if (!(old_portbval & PORTB_DATA_IN)) DEBUGP(MSG_D1);
		//PORTD |= 0x20;
	} else {
		via.irb &= ~PORTB_DATA_IN;
		//if ((old_portbval & PORTB_DATA_IN)) DEBUGP(MSG_D0);
		//PORTD &= ~0x20;
	}
	
	if (!(BUS_PORT_IN & PIN_CLK_IN)){
		via.irb |= PORTB_CLK_IN;
		//if (!(old_portbval & PORTB_CLK_IN)) DEBUGP(MSG_C1);
		//PORTD |= 0x40;
	} else {
		via.irb &= ~PORTB_CLK_IN;
		//if ((old_portbval & PORTB_CLK_IN)) DEBUGP(MSG_C0);
		//PORTD &= ~0x40;
	}
	
	if (!(BUS_PORT_IN & PIN_RESET)){
		if (!reset_val){
			reset_val = 1;
			if (_reset_func){
				_reset_func();
			}
		}
	} else {
		reset_val = 0;
	}
	
}

void busctrl_dump_ports(){
	DEBUGP(DUMP_MSG_FMT,
		(via.irb & PORTB_ATN_IN ? '1':'0'),
		(via.orb & PORTB_ATNA_OUT ? '1':'0'),
		(via.orb & PORTB_CLK_OUT ? '1':'0'),
		(via.irb & PORTB_CLK_IN ? '1':'0'),
		(via.orb & PORTB_DATA_OUT ? '1':'0'),
		(via.irb & PORTB_DATA_IN ? '1':'0'),
		BUS_PORT);
}

