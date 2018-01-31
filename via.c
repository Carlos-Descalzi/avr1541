#include "via.h"
#include "util.h"
#include "debug.h"
#include <avr/pgmspace.h>

const char MSG_TIMER_IRQ[] PROGMEM = "TIMER IRQ\n";
const char MSG_PORT_IRQ[] PROGMEM = "PORT IRQ\n";

#define ACR_T1_1 		0x40

void via_setup(Via* via, ViaPortWriteFunc port_write_func,ViaPortReadFunc port_read_func,ViaIRQFunc irq_func){
	via->port_write_func = port_write_func;
	via->port_read_func = port_read_func;
	via->irq_func = irq_func;
	via_reset(via);
}

void via_reset(Via* via){
	via->irq_in_queue = 0;
	via->ora = 0;
	via->orb = 0;
	via->ira = 0;
	via->irb = 0;
	via->timer.w = 0;
	//via->timer_tick = 0;
	via->timer_latch.w = 0;
	via->acr = 0;
	via->ifr = 0;
	via->ier = 0;
	via->ddrb = 0;
	via->ddra = 0;
	via->acr = 0;
	via->cab = 0;
	via->ira_latch = 0;
	via->irb_latch = 0;
}

#define timer_continuous(v)	((v->acr & ACR_T1_1) && !(v->acr & ACR_T1_2))
#define timer_one_shot(v)		(!(v->acr & ACR_T1_1) && !(v->acr & ACR_T1_2))

void via_restart_timer(Via* via){
	if (!(via->acr & ACR_T1_1)){
		//via->timer_tick = 0;
		via->ifr &= ~IFR_TMR1;
		via->acr |= ACR_T1_1;
		via->timer = via->timer_latch;
	}
}

static void via_clear_ifr(Via* via,uint8_t flag){
	via->ifr &= ~flag;
	if (!(via->ifr & 0x7F)){
		via->ifr = 0;
	}
}

#define port_latched(v,p)	((via->cab & C##p##_C##p##1) && (via->acr & ACR_P##p))

uint8_t via_read(Via* via,uint8_t addr){
	switch (addr){
		case REG_PB:{
				uint8_t val = 0;
				uint8_t irb;
				if (via->port_read_func){
					via->port_read_func(VIA_PORT_B,&via->irb);
				} 

				if (port_latched(via,B)){
					irb = via->irb_latch;
				} else {
					irb = via->irb;
				}

				for (uint8_t pin=0;pin<=7;pin++){
					uint8_t mask = 1 << pin;
					if (via->ddrb & mask){
						val |= via->orb & mask;
					} else {
						val |= irb & mask;
					}
				}
				via_clear_ifr(via,IFR_CB1|IFR_CB2);
				return val;
			}
		case REG_PA:{
				if (port_latched(via,A)){
					return via->ira_latch;
				}
				if (via->port_read_func){
					via->port_read_func(VIA_PORT_A,&via->ira);
				} 
				via_clear_ifr(via,IFR_CA1|IFR_CA2);
				return via->ira | via->ora;
			}
		case REG_DDRB:
			return via->ddrb;
		case REG_DDRA:
			return via->ddra;
		case REG_T1L:
			// clear interrupt
			via_restart_timer(via);
			//DEBUG("%s RESTART TIMER 1\n",via->id);
			return via->timer.b.l;
		case REG_T1H:
			return via->timer.b.h;
		case REG_TL1L:
			return via->timer_latch.b.l;
		case REG_TL1H:
			return via->timer_latch.b.h;
		case REG_PCR:
			return via->acr;
		case REG_ACR:
			return via->acr;
		case REG_IFR:
			return via->ifr;
		case REG_IER:
			return via->ier;
	}
	return 0;
}


void via_write(Via* via,uint8_t addr, uint8_t val){
	switch (addr){
		case REG_PB:{
				uint8_t old_val = via->orb;
				
				set_bits(&via->orb,val,via->ddrb);
				if (via->port_write_func){
					via->port_write_func(VIA_PORT_B,via->orb,via->ddrb,old_val);
				} 
				via_clear_ifr(via,IFR_CA1|IFR_CA2);
			}
			break;
		case REG_PA:{
				uint8_t old_val = via->ora;
				
				set_bits(&via->ora,val,via->ddra);
				if (via->port_write_func){
					via->port_write_func(VIA_PORT_A,via->ora,via->ddra,old_val);
				} 
				via_clear_ifr(via,IFR_CA1|IFR_CA2);
			}
			break;
		case REG_DDRB:
			via->ddrb = val;
			break;
		case REG_DDRA:
			via->ddra = val;
			break;
		case REG_T1L:
			via->timer.b.l = val;
			break;
		case REG_T1H:
			via->timer.b.h = val;
			via->timer.b.l = via->timer_latch.b.l;
			via_restart_timer(via);
			break;
		case REG_TL1L:
			via->timer_latch.b.l = val;
			break;
		case REG_TL1H:
			via->timer_latch.b.h = val;
			break;
		case REG_PCR:
			via->pcr = val;
			break;
		case REG_IFR:
			if (via->ifr & 0x80){
				via->ifr = 0;
			} else {
				for (uint8_t i=0;i<=6;i++){
					uint8_t mask = 1 << i;
					if (val & mask){
						via->ifr &=~ mask;
					}
				}
			}
			break;
		case REG_IER:
			via->ier = val;
			if (val & 0x80){
				for (uint8_t i= 0;i<=6;i++){
					uint8_t mask = 1 << i;
					if (val & mask){
						via->ier |= mask;
					}
				}
			} else {
				for (uint8_t i= 0;i<=6;i++){
					uint8_t mask = 1 << i;
					if (val & mask){
						via->ier &= ~mask;
					}
				}
			}
			break;
		case REG_ACR:
			via->acr = val;
			if (via->acr & ACR_T1_1){
				via->timer = via->timer_latch;
			} else {
			}
			break;
	}
}


void via_loop(Via* via){
	if (via->irq_in_queue){
		if (via->irq_func){
			//DEBUG("%s HANDLING IRQ\n",via->id);
			via->irq_func(via->ifr);
		}
		via->irq_in_queue = 0;
	}
	if (via->acr & ACR_T1_1){
		//via->timer_tick++;
		//if (via->timer_tick >= 0x01){
			//via->timer_tick = 0;
			if (via->timer.w == 0){ 	//DEBUG("TIMER TICK\n");
				if (via->ier & IER_TMR1){//DEBUGP(MSG_TIMER_IRQ);
					via_do_irq(via,IFR_TMR1);
				}
				if (timer_continuous(via)){//DEBUG("%s CONTINUOUS TIMER RESTART\n", via->id);
					via->timer = via->timer_latch;
				} else {//DEBUG("%s ONE SHOT TIMER FINISH\n", via->id);
					via->timer.w--;
					via->acr &= ~ACR_T1_1;
				}
			} else {
				via->timer.w--;
			}
		//}
	}
}
void via_do_irq(Via* via,uint8_t mask){
	via->ifr |= (IFR_ANY | mask);
	via->irq_in_queue = 1;
}
