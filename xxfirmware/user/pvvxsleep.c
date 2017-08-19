#include <user_interface.h>
#include "pvvxsleep.h"
#include <esp82xxutil.h>
#include <uart.h>

extern UartDevice    UartDev;

void dtm_params_init(void * on_wait_func_cb, void * off_wait_func_cb);


void ICACHE_FLASH_ATTR uart_wait_tx_fifo_empty(void)
{
	while((UART_STATUS(0) >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT);
}


static void test_cb1()
{
	uart_wait_tx_fifo_empty();
	ets_delay_us(150);
}

static void ICACHE_FLASH_ATTR restore_clkfreq(void)
{
	rom_i2c_writeReg(103,4,1,136);
	rom_i2c_writeReg(103,4,2,145);
    uart_div_modify(0, UART_CLK_FREQ / (UartDev.baut_rate));
	ets_delay_us(150);
}

static void test_cb2()
{
	uint32 t = system_get_time();
	restore_clkfreq();
	uart_wait_tx_fifo_empty();
	ets_delay_us(150);
}

void ICACHE_FLASH_ATTR do_pvvx_sleep(int sleep_ms, int reboot_when_done)
{
	if( reboot_when_done )
	{
		dtm_params_init(0, 0); // назначить функцию on_wait_func_cb() до входа в sleep в ets_run() и функцию off_wait_func_cb() после выхода из sleep в ets_run()
	}
	else
	{
		dtm_params_init(test_cb1, test_cb2); // назначить функцию on_wait_func_cb() до входа в sleep в ets_run() и функцию off_wait_func_cb() после выхода из sleep в ets_run()
	}
	ets_rtc_int_register();		// установить прерывание rtc - rtc_intr_handler() для sleep
	int t1 = 5; //?????
	int t2 = (sleep_ms*2)/3; // 1 sec
//	printf("L" );
	dtm_set_params(0, t2, 0, t1, 0); // параметры sleep

	rtc_enter_sleep(); // исполнить один цикл тут
	ets_enter_sleep(); // назначить исполнение sleep в ets_run()
}


