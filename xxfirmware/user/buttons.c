#include <esp82xxutil.h>

void ICACHE_FLASH_ATTR
gpio16_output_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC to output rtc_gpio0

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   (READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe) | (uint32)0x1);	//out enable
}

void ICACHE_FLASH_ATTR
gpio16_output_set(uint8 value)
{
    WRITE_PERI_REG(RTC_GPIO_OUT,
                   (READ_PERI_REG(RTC_GPIO_OUT) & (uint32)0xfffffffe) | (uint32)(value & 1));
}

void ICACHE_FLASH_ATTR
gpio16_input_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC and rtc_gpio0 connection

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe);	//out disable
}

uint8 ICACHE_FLASH_ATTR
gpio16_input_get(void)
{
    return (uint8)(READ_PERI_REG(RTC_GPIO_IN_DATA) & 1);
}

// NEW BADGES
// Start = GPIO 0
// Select = GPIO 2
// 4, 5, 16 = Button Readout
// 15 replaces 16 on prototype borads

// LSB to MSB:
// Right
// Down
// Left
// Up
// Select
// Start
// B
// A

uint8_t GetButtons()
{
	ETS_GPIO_INTR_DISABLE();
        uint8_t ret = 0;

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U,FUNC_GPIO0);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,FUNC_GPIO4);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);


/*
 * set 4 low
 * A = 5
 * Left = 16/15
 *
 * set 5 low
 * B = 4
 * Up = 16/15
 *
 * set 16/15 low
 * Right = 4
 * Down = 5
 */

	PIN_DIR_INPUT |= (1<<0) | (1<<2) | (1<<4) | (1<<5);

	// +++++++++++++++++++++++
	PIN_DIR_OUTPUT |= (1<<5);
	PIN_OUT_CLEAR |= (1<<5);

	
	// B
	ret |= (~PIN_IN & (1<<4))<<2;

	// Up
	ret |= (~gpio16_input_get() & 1)<<3;

	PIN_DIR_INPUT |= (1<<5);

	// +++++++++++++++++++++++
	gpio16_input_conf();

	PIN_DIR_OUTPUT |= (1<<4);
	PIN_OUT_CLEAR |= (1<<4);

	// A (1<<7)
	ret |= (~PIN_IN & (1<<5))<<2;

	// Left (1<<2)
	ret |= (~gpio16_input_get() & 1)<<2;

	PIN_DIR_INPUT |= (1<<4);

	// +++++++++++++++++++++++
	gpio16_output_conf();
	gpio16_output_set(0);

	// Right
	ret |= (~PIN_IN & (1<<4))>>4;

	// Down
	ret |= (~PIN_IN & (1<<5))>>4;

	gpio16_input_conf();

	// ========================
	// Normal Things

	// Start
	ret |= (~PIN_IN & (1<<0))<<5;

	// Select
	ret |= (~PIN_IN & (1<<2))<<2;

	ETS_GPIO_INTR_ENABLE();
        return ret;
}

