#include "sdmgr.h"
#include "ff.h"
#include "serial.h"
#include "debug.h"
#include <string.h>
#include <avr/pgmspace.h>

FATFS fs;
DIR dir;
char dirbuff[10][20];

#define memset(p,v,s){for(int i=0;i<s;((char*)p)[i++]=v);}

const char MSG_MOUNT_OK[] PROGMEM = "SD MOUNT OK\n";
const char MSG_MOUNT_ERROR[] PROGMEM = "SD MOUNT ERROR %d\n";

void sd_init(){
	FRESULT result;
	memset(&fs,0,sizeof(FATFS));
	memset(&dir,0,sizeof(DIR));

	for (int i=0;i<10;i++){
		dirbuff[i][0] = 0;
	}

	result = f_mount(&fs,"",0);
	f_opendir(&dir,"/");
	switch(result){
		case FR_OK:
			DEBUGP(MSG_MOUNT_OK);
			break;
		default:
			DEBUGP(MSG_MOUNT_ERROR,result);
			break;
	}
	
}
const char ENTRY_FMT[] PROGMEM = " %d) %s\n";
void sd_list_next_page(uint8_t rewind){
	FILINFO file;
	uint8_t i = 0;
	
	if (f_readdir(&dir,&file) == FR_OK){
		for (i=0;strlen(file.fname) && i<10;i++){
			strcpy(dirbuff[i],file.fname);
			f_readdir(&dir,&file);
		}
		for (;i<10;i++){
			dirbuff[i][0] = 0;
		}
		
		for (i=0;i<10;i++){
			if (dirbuff[i][0]){
				fprintf_P(&serial_out,ENTRY_FMT,i,dirbuff[i]);
			}
		}
	} 
}

const char* sd_get_fname(uint8_t index){
	return dirbuff[index];
}

