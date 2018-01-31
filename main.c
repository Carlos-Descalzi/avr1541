#include "up6502.h"
#include "d64.h"
#include "serial.h"
#include "sdmgr.h"
#include "drivectrl.h"
#include "busctrl.h"
#include "debug.h"
#include "rom.h"


#define RAM_TOP 2048

unsigned char loram[RAM_TOP];

void do_reset();
void loop();

const char MSG_LIST[] PROGMEM = "LIST\n";
const char MSG_IMG[] PROGMEM = "SET IMAGE\n";
const char MSG_TOGGLE[] PROGMEM = "TOGGLE LOG\n";
const char MSG_DUMP[] PROGMEM = "DUMP\n";
const char MSG_UNK_CMD[] PROGMEM = "UNKNOWN COMMAND %c\n";

const char MSG_HELP[] PROGMEM = "\
l  ) LIST SD DIRECTORY\n\
0-9) SET D64 IMAGE\n\
d  ) TOGGLE DEBUG\n\
c  ) DUMP BUS STATUS\n\
r  ) RESET\n\
h  ) SHOW THIS HELP\n\
";
void check_serial(){
	if (serial_is_data()){
		char c = serial_read();
		
		if (c == 'l'){
			DEBUGP(MSG_LIST);
			sd_list_next_page(0);
		} else if (c >= '0' && c <= '9'){
			DEBUGP(MSG_IMG);
			d64_set_image(sd_get_fname(c-'0'));
		} else if (c == 'd'){
			DEBUGP(MSG_TOGGLE);
			up6502_toggle_log();
		} else if (c == 's'){
			DEBUGP(MSG_DUMP);
			busctrl_dump_ports();
		} else if (c == 'r'){
			do_reset();
		} else if (c == 'h'){
			DEBUGP(MSG_HELP);
		} else if (c == 'm'){
			PORTD |= 0x20;
		} else if (c == 'n'){
			PORTD &= ~0x20;
		} else if (c == 'j'){
			PORTD |= 0x40;
		} else if (c == 'k'){
			PORTD &= ~0x40;
		} else {
			DEBUGP(MSG_UNK_CMD,c);
		}
	}
}


uint8_t mem_read_callback(uint16_t addr, uint8_t*val){
	if (addr < 2048){
		*val = loram[addr];
		return 0;
	} else if (addr >= 0x1C00 && addr <= 0x1C0F){
		*val = drivectrl_read(addr & 0x0F);
		return 0;
	} else if (addr >= 0x1800 && addr <= 0x180F){
		*val = busctrl_read(addr & 0x0F);
		return 0;
	} else if (addr >= ROM_ADDR){
		*val = pgm_read_byte_near(ROM + (addr - ROM_ADDR));
		return 0;
	}
	
	return 1;
}

uint8_t mem_write_callback(uint16_t addr, uint8_t val){
	if (addr < 2048){
		loram[addr] = val;
		return 0;
	} else if (addr >= 0x1C00 && addr <= 0x1C0F){
		drivectrl_write(addr & 0x0F,val);
		return 0;
	} else if (addr >= 0x1800 && addr <= 0x180F){
		busctrl_write(addr & 0x0F,val);
		return 0;
	}
	return 1;
}

void via_irq(uint8_t irq){
	up6502_irq();
}

void clear_ram(){
	for (int i=0;i<RAM_TOP;loram[i++]=0);
}

const char MSG_RESET[] PROGMEM = "AVR1541 RESET\n";

void do_reset(){
	DEBUGP(MSG_RESET);
	clear_ram();
	up6502_reset();
	busctrl_reset();
	drivectrl_reset();
}

void loop(){
	up6502_loop();
	drivectrl_loop();
	busctrl_loop();
}

const char MSG_START[] PROGMEM = "AVR1541 START\n";

uint8_t reset_issued;

void reset_req(){
	reset_issued = 1;
}

int main(){
	reset_issued = 0;
	serial_init(9600);
	clear_ram();
	up6502_setup(mem_read_callback,mem_write_callback);
	sd_init();
	drivectrl_setup(via_irq);
	busctrl_setup(via_irq,reset_req);
	DEBUGP(MSG_START);
	while(1){
		check_serial();
		loop();
		
		if (reset_issued){
			reset_issued = 0;
			DEBUG("RESET !!!\n");
			do_reset();
		}
	}
}
