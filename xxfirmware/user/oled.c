#include "oled.h"
#include <esp82xxutil.h>

#define os_delay_us ets_delay_us
#define I2CSPEEDBASE 1
#define I2CNEEDGETBYTE
#define DSDA 13
#define DSCL 14
//On GPIO13/GPIO14
#include "static_i2c.h"


#define DENSITRON_ADDRESS 0b01111000
#define DPWR DENSITRON_POWER
#define DRST DENSITRON_RESET
#define CONTINUE 0x80
#define COMMAND	 0x00
#define DATA     0x40

int display_on;


//From P17 of datasheet.

#define NC1 CONTINUE | COMMAND,
#define NC2 CONTINUE | COMMAND,
const unsigned char DensitronBootMessage[] = { DENSITRON_ADDRESS, 
	CONTINUE | COMMAND, 0xAE,
	NC1 0xD5, NC2 0x80, //Clock divide ratio
	NC1 0xA8, NC2 0x3F, //[Frequency] [Multiplex Ratio]
	NC1 0x8D, NC2 0x14, //Set charge pump
	NC1 0xA1,           //Segment re-map
    NC1 0xC8,           //Set output scan direction
    NC1 0xDA, NC2 0x10, //Set COM Pins Hardware Configuration  (Was 0x02 DE-INTERLACED)  (This seems to double-density the display)

	NC1 0x20, NC2 0x00, //Addressing mode.

	NC1 0x81, NC2 0xAF, //Contrast control
    NC1 0xD9, NC2 0xF1, //Pre-charge period
	NC1 0xDB, NC2 0x20, //VCOMH Deselect Level

	NC1 0x2E, //@48 Deactivate scroll?
	NC1 0x10, //@50 Higher address for indexing pixels

	NC1 0x21, NC2 0x00, NC2 0x7f, //Column start and end addresses
	NC1 0x22, NC2 0x00, NC2 0x3f, //Page start/end addresses.

	NC1 0xD3, NC2 0x38, //Display Offset  (**** SET TO 28 FOR TOP OF SCREEN ****)
	NC1 0x40,           //Display start line

	NC1 0xA4,           //Turn on screen

	NC1 0xA6,
	NC1 0xAF,
};

#define pgm_read_byte(x) (((uint8_t*)x)[0])


void ICACHE_FLASH_ATTR InitOLED()
{
	int i;

	static const uint32_t AFMapper[16] = {
		0, PERIPHS_IO_MUX_U0TXD_U, 0, PERIPHS_IO_MUX_U0RXD_U,
		0, 0, 1, 1,
		1, 1, 1, 1,
		PERIPHS_IO_MUX_MTDI_U, PERIPHS_IO_MUX_MTCK_U, PERIPHS_IO_MUX_MTMS_U, PERIPHS_IO_MUX_MTDO_U
	};

	PIN_FUNC_SELECT( AFMapper[DSDA], 3);  //Select AF pin to be GPIO.
	PIN_FUNC_SELECT( AFMapper[DSCL], 3);  //Select AF pin to be GPIO.


	ConfigI2C();

	SendStart();
	for( i = 0; i < sizeof( DensitronBootMessage ); i++ )
		if( SendByte( pgm_read_byte( DensitronBootMessage +i ) ) ) goto fail;
	SendStop();

	display_on = 1;

	printf( "Densitron OK\n" );


	SendStart();
	SendByte( DENSITRON_ADDRESS );
	//Seek, left/right
	int x =0 ;
	int y = 0;
	for( y = 0; y < 8; y++ )
	{
		SendByte( CONTINUE | COMMAND ); SendByte( 0x00 | (x & 0x0f) );
		SendByte( CONTINUE | COMMAND ); SendByte( 0x10 | (x >> 4)   );
		SendByte( CONTINUE | COMMAND ); SendByte( 0x40 | y ); //was 0xb0

		for( i = 0; i < 128	; i++ )
		{
			SendByte( CONTINUE | DATA );SendByte( 0 );	
		}
	SendStop();	
	}
	

	return;
fail:
	printf( "Densitron failed.\n" );
	SendStop();
}

void ICACHE_FLASH_ATTR TurnOffOLED()
{
	if( !display_on ) return;
	display_on = 0;
	SendStart();
	SendByte( DENSITRON_ADDRESS );
	SendByte( CONTINUE | COMMAND ); SendByte( 0xAE );
	SendStop();
}

void ICACHE_FLASH_ATTR DensitronOutput( unsigned char x, unsigned char y, unsigned char * stream, unsigned char len )
{
	unsigned char i;

	if( !display_on ) return;

	SendStart();
	SendByte( DENSITRON_ADDRESS );
	//Seek, left/right
	SendByte( CONTINUE | COMMAND ); SendByte( 0x00 | (x & 0x0f) );
	SendByte( CONTINUE | COMMAND ); SendByte( 0x10 | (x >> 4)   );
	SendByte( CONTINUE | COMMAND ); SendByte( 0xB0 | y );

	for( i = 0; i < len; i++ )
	{
		SendByte( CONTINUE | DATA );	SendByte( stream[i] );	
	}

	SendStop();	
}





///Print functions

#include "font.h"


void ICACHE_FLASH_ATTR DensePrint( int cursorx, int cursory, const char * st )
{
	while( (*st) )
	{
		unsigned char i;
		unsigned char buff[6];
		unsigned char ch = *st;
		const unsigned char * character;

		if( ch < ' ' ) ch = ' ';
		if( ch > '~' ) ch = ' ';
		ch -= ' ';

		character = &font_5x7_data[ch*5];

		for( i = 0; i < 5; i++ )
		{
			buff[i] = pgm_read_byte( character + i );
		}

		buff[5] = 0;

		DensitronOutput( cursorx, cursory, buff, 6 );

		cursorx+=6;
		if( cursorx >= 126 || ch == '\n' )
		{
			cursorx = 0;
			cursory ++;
		}

		if( cursory == 4 )
		{
			cursory = 0;
		}

		st++;
	}
}

void ICACHE_FLASH_ATTR DensePrintBig( int cursorx, int cursory, const char * st )
{
	while( (*st) )
	{
		unsigned char i;
		unsigned char buff[12];
		unsigned char ch = *st;
		const unsigned char * character;

		if( ch < ' ' ) ch = ' ';
		if( ch > '~' ) ch = ' ';
		ch -= ' ';

		character = &font_5x7_data[ch*5];

		
		for( i = 0; i < 5; i++ )
		{
			char c = pgm_read_byte( character + i );
			char co = (c & 0x80) | ((c & 0x80)>>1) | ((c & 0x40)>>1) | ((c & 0x40)>>2) | ((c & 0x20)>>2) | ((c & 0x20)>>3) | ((c & 0x10)>>3) | ((c & 0x10)>>4);
			buff[i<<1] = co;
			buff[(i<<1)+1] = co;
		}
		buff[10] = 0;
		buff[11] = 0;
		DensitronOutput( cursorx, cursory+1, buff, 12 );

		for( i = 0; i < 5; i++ )
		{
			char c = pgm_read_byte( character + i );
			char co = ((c & 0x08)<<4) | ((c & 0x08)<<3) | ((c & 0x04)<<3) | ((c & 0x04)<<2) | ((c & 0x02)<<2) | ((c & 0x02)<<1) | ((c & 0x01)<<1) | ((c & 0x01));
			buff[i<<1] = co;
			buff[(i<<1)+1] = co;
		}
		buff[10] = 0;
		buff[11] = 0;
		DensitronOutput( cursorx, cursory, buff, 12 );

		cursorx+=12;
		if( cursorx >= 120 || ch == '\n' )
		{
			cursorx = 0;
		}

		st++;
	}

}

