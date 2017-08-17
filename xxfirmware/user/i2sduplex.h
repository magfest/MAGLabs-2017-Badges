//Copyright 2016 <>< Charles Lohr, See LICENSE file.
//
//Eerrrr this is an amalgamatiaon of:
// https://github.com/cnlohr/esp8266ws2812i2s/blob/master/user/ws2812_i2s.c
//and
// https://github.com/cnlohr/esp8266duplexi2s/blob/master/user/i2sduplex.c

#ifndef _I2SDUPLEX_TEST
#define _I2SDUPLEX_TEST


//Stuff that should be for the header:

#include <c_types.h>

#define DMABUFFERDEPTH 3
#define I2SDMABUFLEN (64)
#define LINE32LEN I2SDMABUFLEN
#define RX_NUM (I2SDMABUFLEN)

extern uint32_t i2sBDRX[I2SDMABUFLEN*DMABUFFERDEPTH];
extern uint32_t i2sBDTX[I2SDMABUFLEN*DMABUFFERDEPTH];

extern int fxcycle;
extern int erx, etx;

void ICACHE_FLASH_ATTR testi2s_init();
void ICACHE_FLASH_ATTR ws2812_push( uint8_t * leds,  int ledcount);

#endif


