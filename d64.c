#include "d64.h"

#include <avr/pgmspace.h>
#include "ff.h"

#define BYTES_SECTOR          256

const uint8_t SECTORS_PER_TRACK[] PROGMEM = {
      0,     21,    21,    21,    21,
      21,    21,    21,    21,    21,
      21,    21,    21,    21,    21,
      21,    21,    21,    21,    19,
      19,    19,    19,    19,    19,
      19,    18,    18,    18,    18,
      18,    18,    17,    17,    17,
      17,    17,    17,    17,    17, 
      17  
};

const long TRACK_OFFSETS[] PROGMEM = { 
      0xFFFFF,    0x00000,    0x01500,    0x02a00,    0x03f00,
      0x05400,    0x06900,    0x07e00,    0x09300,    0x0a800,
      0x0bd00,    0x0d200,    0x0e700,    0x0fc00,    0x11100,
      0x12600,    0x13b00,    0x15000,    0x16500,    0x17800,
      0x18b00,    0x19e00,    0x1b100,    0x1c400,    0x1d700,
      0x1ea00,    0x1fc00,    0x20e00,    0x22000,    0x23200,
      0x24400,    0x25600,    0x26700,    0x27800,    0x28900,
      0x29a00,    0x2ab00,    0x2bc00,    0x2cd00,    0x2de00,
      0x2ef00
};

char sector_buffer[BYTES_SECTOR];
static FIL current_file;
UINT sector_length;
uint8_t sector_index;
uint8_t track_index;

static inline long track_offset(int trackIndex){
  return pgm_read_dword(&TRACK_OFFSETS[trackIndex]);
}
static inline uint8_t sectors_per_track(int trackIndex){
	return pgm_read_byte(&SECTORS_PER_TRACK[trackIndex]);
}

char d64_set_image(const char* image_path){
	f_open(&current_file,image_path,FA_OPEN_EXISTING|FA_READ);
	return d64_rewind();
}

char d64_rewind(){
	FRESULT result;
	f_lseek(&current_file,0);
	result = f_read(&current_file,sector_buffer,BYTES_SECTOR,&sector_length);
	if (result != FR_OK){
		return D64_ERROR;
	}
	sector_index = 0;
	track_index = 1;
	
	return D64_OK;
}
char d64_next_byte(uint8_t* data){
	if (sector_index == sector_length){
		if (sector_length < BYTES_SECTOR){
			return D64_EOF;
		} else {
			FRESULT result = f_read(&current_file,sector_buffer,BYTES_SECTOR,&sector_length);
			
			if (result != FR_OK){
				return D64_ERROR;
			}
			sector_index = 0;
		}
	}
	
	*data = sector_buffer[sector_index];
	
	sector_index++;
	
	if (sector_index >= sectors_per_track(track_index)){
		track_index++;
	}

	return D64_OK;
}

char d64_next_track(){
	track_index++;
	return D64_OK;
}
char d64_prev_track(){
	track_index++;
	return D64_OK;
}
