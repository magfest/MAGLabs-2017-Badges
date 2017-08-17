#ifndef _VIVE_HELP_H
#define _VIVE_HELP_H

#include <c_types.h>

extern struct vivestate VS;

struct vivestate
{
	int enabled;
	int in_pulse;
	uint32_t count;
	volatile uint32_t rxcount;
	uint32_t last;

	uint32_t start;
	uint32_t runninglength;

	uint32_t pulses;
};


void ICACHE_FLASH_ATTR init_vive();
void ICACHE_FLASH_ATTR enable_vive();
void ICACHE_FLASH_ATTR disable_vive();
void vive_data( uint32_t * data, uint32_t length );
void vive_pulse( struct vivestate * v, uint32_t start, uint32_t length );


#endif
