#ifndef _SDMGR_H_
#define _SDMGR_H_

#include <avr/io.h>

void sd_init();
void sd_list_next_page(uint8_t rewind);
const char* sd_get_fname(uint8_t index);

#endif
