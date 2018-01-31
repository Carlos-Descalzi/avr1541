#ifndef _D64_H_
#define _D64_H_

#include <avr/io.h>

#define D64_OK 			 0
#define D64_ERROR 	-1
#define D64_EOF	-		-2

char d64_set_image(const char* image_path);
char d64_rewind();
char d64_next_byte(uint8_t* byte);
char d64_next_track();
char d64_prev_track();

#endif
