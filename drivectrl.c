#include "drivectrl.h"
#include "d64.h"
#include "via.h"
#include "debug.h"
#include <string.h>
#include <avr/pgmspace.h>

#define DRIVE_STATUS_HEAD1	0x01
#define DRIVE_STATUS_HEAD2	0x02
#define DRIVE_STATUS_HEADS	0x03
#define DRIVE_STATUS_MOTOR	0x04
#define DRIVE_STATUS_LED		0x08
#define DRIVE_STATUS_WP			0x10
#define DRIVE_STATUS_DD1		0x20
#define DRIVE_STATUS_DD2		0x40
#define DRIVE_STATUS_READ		0x80

#define ISR_CTRL_TIMER_ENABLE 0x40
#define TIMER_STATUS_START 		0x40
#define ISR_STATUS_TIMER 			0x40
#define ISR_CTRL_INT_SET 			0x80


#define DRIVE_PORT		PORTD
#define DRIVE_PORTD		DDRD
#define PIN_RED_LED		0x20
#define PIN_GREEN_LED	0x40

const char MSG1[] PROGMEM = "2 DRIVE WRITE %02x %02x\n";
const char MSG2[] PROGMEM = "2 -- DRIVE WRITE %02x\n";
const char MSG3[] PROGMEM = "READ BYTE\n";
static const char VIA_ID[] PROGMEM = "DRV";

static Via via;
static uint8_t head_status;
//uint8_t sync;

static void toggle_red_led(uint8_t on);
static void toggle_green_led(uint8_t on);
static void handle_head(uint8_t current, uint8_t prev);

static void via_port_write_callback(uint8_t port, uint8_t val, uint8_t dir_mask, uint8_t old_value);
static void via_port_read_callback(uint8_t port,uint8_t* val);

void drivectrl_setup(DriveCtrlIrqFunc irq_func){
	DRIVE_PORTD |= 0x60;
	via_setup(&via,via_port_write_callback,via_port_read_callback,irq_func);
	strcpy_P(via.id,VIA_ID);
	drivectrl_reset();
}

void drivectrl_reset(){
	via_reset(&via);
	via_write(&via,ADDR_PORTA_DIR,0x00);
	via_write(&via,ADDR_PORTB_DIR,0x6F);
}

uint8_t drivectrl_read(uint8_t addr){
	return via_read(&via,addr);
}

void drivectrl_write(uint8_t addr,uint8_t val){
	via_write(&via,addr, val);
}

static void via_port_write_callback(uint8_t port, uint8_t val, uint8_t dir_mask, uint8_t old_value){
	uint8_t bits_set = val & dir_mask;

	//if (val != 0x60) DEBUG("3 DRIVE WRITE %c %02x\n",port == VIA_PORT_B ? 'B' : 'A',val);

	if (port == VIA_PORT_B){
		toggle_red_led(val & DRIVE_STATUS_LED);
		toggle_green_led(val & DRIVE_STATUS_MOTOR);
		
		if (dir_mask & DRIVE_STATUS_HEADS){
			uint8_t new_head_status = val & DRIVE_STATUS_HEADS;
			handle_head(new_head_status,head_status);
			head_status = new_head_status;
		}
	} else if (port == VIA_PORT_A){
		// FIXME: write to disk
	}
}
static void via_port_read_callback(uint8_t port,uint8_t* val){
	if (port == VIA_PORT_B){
		*val &= ~DRIVE_STATUS_WP;
		/*if (sync){
			*val |= 0x80;
			sync = 0;
		} else {
			*val &= 0x7F;
		}*/
		
	} else if (port == VIA_PORT_A){
		DEBUGP(MSG3);
		d64_next_byte(val);
	}
	//DEBUG("READ PORT %c %02x\n",(port == VIA_PORT_A ? 'A' : 'B'),*val);
}

void drivectrl_loop(){
	via_loop(&via);
}

static void toggle_red_led(uint8_t on){
	if (on){
		DRIVE_PORT |= PIN_RED_LED;
	} else {
		DRIVE_PORT &= ~PIN_RED_LED;
	}
}

static void toggle_green_led(uint8_t on){
	if (on){
		DRIVE_PORT |= PIN_GREEN_LED;
	} else {
		DRIVE_PORT &= ~PIN_GREEN_LED;
	}
}

static void handle_head(uint8_t current, uint8_t prev){
	if (current > prev){
		DEBUG("MOVING HEAD\n");
		d64_next_track();
		// increase
	} else if (current < prev){
		DEBUG("MOVING HEAD\n");
		d64_prev_track();
		// decrease
	}
}
