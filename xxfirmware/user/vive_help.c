#include "vive_help.h"
#include <stdio.h>
#include <esp82xxutil.h>

struct vivestate VS;

void ICACHE_FLASH_ATTR init_vive()
{
	VS.enabled = 1;
}

void ICACHE_FLASH_ATTR enable_vive()
{
	VS.enabled = 1;
}

void ICACHE_FLASH_ATTR disable_vive()
{
	VS.enabled = 0;
}

void vive_data( uint32_t * data, uint32_t length )
{
	int i = 0, j = 0, mask = 1L<<31;
	struct vivestate * v = &VS;
	if( !v->enabled ) return;
	int count = v->count;

	v->rxcount++;

resct:
	if( v->in_pulse )
	{
		//Continue with pulse data
		for( ; i < length; i++ )
		{
			uint32_t dat = data[i];
			if( dat == 0xffffffff )    //In order for this to be true j must already be = 0.
			{
				count+=32;
				continue;
			}

			for( ; j < 32; j++ )
			{
				count++;
				if( dat & mask )
				{
					//Still in pulse
				}
				else
				{
					//End-of-pulse!
					vive_pulse( v, v->start, count - v->start );
					v->in_pulse = 0;
					goto searchct;
				}
				mask >>= 1;
			}
			j = 0; mask = 1L<<31;
		}
		v->count = count;

		return;
	}

searchct:
	for( ; i < length; i++ )
	{
		uint32_t dat = data[i];
		if( data[i] )
		{
			//We have a pulse
			for( ; j < 32; j++ )
			{
				if( dat & mask )
				{
					v->in_pulse = 1;
					v->start = count;
					j++;
					if( j == 32 )
					{
						i++;
						j = 0;
					}
					goto resct;				
				}
				else
				{
					mask >>= 1;
				}
				count++;
			}
			j = 0; mask = 1L<<31;
		}
		else
		{
			count += 32;
		}
	}

	v->count = count;
	return;
}

void vive_pulse( struct vivestate * v, uint32_t start, uint32_t length )
{
	v->pulses++;
//	printf( "%d %d\n", start, length );
}
