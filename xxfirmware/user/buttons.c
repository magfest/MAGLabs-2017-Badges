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


uint8_t GetButtons()
{
        uint8_t ret = 0;

	PIN_DIR_INPUT |= (1<<4) | (1<<5);
	gpio16_input_conf();

	PIN_DIR_OUTPUT |= (1<<4);
	PIN_OUT_CLEAR |= (1<<4);
	ret |= (PIN_IN & (1<<5))>>5;
	ret |= gpio16_input_get()<<1;
	PIN_DIR_INPUT |= (1<<4);

	PIN_DIR_OUTPUT |= (1<<5);
	PIN_OUT_CLEAR |= (1<<5);
	ret |= (PIN_IN & (1<<4))>>1;
	ret |= gpio16_input_get()<<4;
	PIN_DIR_INPUT |= (1<<5);

	gpio16_output_conf();
	gpio16_output_set(0);
	ret |= (PIN_IN & (1<<4))<<1;
	ret |= (PIN_IN & (1<<5))<<1;
	gpio16_input_conf();

        return ret;
}

