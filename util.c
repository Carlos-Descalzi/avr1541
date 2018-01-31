#include "util.h"


void set_bits(uint8_t* target, uint8_t val, uint8_t mask){
	for (uint8_t i=0;i<=7;i++){
		uint8_t bit = 1<<i;
		if (mask & bit){
			if (val & bit){
				*target |= bit;
			} else {
				*target &= ~bit;
			}
		}
	}
}

