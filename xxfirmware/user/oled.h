#ifndef _OLED_H
#define _OLED_H

#include <esp82xxutil.h>

extern int display_on;
void ICACHE_FLASH_ATTR InitOLED();
void ICACHE_FLASH_ATTR DensitronOutput( unsigned char x, unsigned char y, unsigned char * stream, unsigned char len );
void ICACHE_FLASH_ATTR DensePrint( const char * st );
void ICACHE_FLASH_ATTR DensePrintBig( const char * st );

#endif

