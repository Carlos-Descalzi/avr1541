#include <stdio.h>

int main(){
	for (unsigned char pin=0,i=0;pin<=7&& i< 10;pin++,i++){
		printf("%02x %d\n",1<<pin,pin <= 0x80);
	}

	return 0;
}
