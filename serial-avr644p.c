#include "serial.h"
#include <avr/io.h>
#include <stddef.h>
#include <string.h>

#ifdef USE_SERIAL_STDIO
static int serial_putchar(char c, FILE* stream){
	if (c == '\n'){
		serial_write('\r');
		serial_write('\n');
	} else {
		serial_write(c);
	}
	return 0;
}
FILE serial_out = FDEV_SETUP_STREAM(serial_putchar,NULL,_FDEV_SETUP_WRITE);
#endif

void serial_init(unsigned long baud_rate){
    unsigned int bittimer = (F_CPU / ( baud_rate * 16 )) - 1;
    UBRR0H = (unsigned char) (bittimer >> 8);
    UBRR0L = (unsigned char) bittimer;
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}
char serial_read(){
	return UDR0;
}
char serial_read_wait(){
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}
void serial_write(char data){
	while ( !(UCSR0A & (1 << UDRE0)) );
	UDR0 = data;
}
char serial_is_data(){
	return	(UCSR0A & (1 << RXC0));
}

void serial_write_str(const char* str){
	for (int i=0;str[i];i++){
		serial_write(str[i]);
	}
}

