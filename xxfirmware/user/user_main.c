//Copyright 2015 <>< Charles Lohr, see LICENSE file.

#include "mem.h"
#include "c_types.h"
#include "user_interface.h"
#include "ets_sys.h"
#include "uart.h"
#include "osapi.h"
#include "espconn.h"
#include "esp82xxutil.h"
#include "i2sduplex.h"
#include "commonservices.h"
#include "vars.h"
#include <mdns.h>
#include <vive_help.h>
#include <buttons.h>

#define procTaskPrio        0
#define procTaskQueueLen    1

uint8_t leds[12];
void ICACHE_FLASH_ATTR send_ws_leds()
{
	ws2812_push( leds, 4);
}



//static volatile os_timer_t some_timer;
static struct espconn *pUdpServer;
usr_conf_t * UsrCfg = (usr_conf_t*)(SETTINGS.UserData);

//int ICACHE_FLASH_ATTR StartMDNS();


void user_rf_pre_init(void) { /*nothing*/ }

//Tasks that happen all the time.

os_event_t    procTaskQueue[procTaskQueueLen];

static void ICACHE_FLASH_ATTR slowtick()
{
	static	int frameno;
	int i;
	frameno++;
	//frameno = 255;
	for( i = 0; i < 4; i++ )
	{
		leds[i*3+0] = frameno;
		leds[i*3+1] = frameno;
		leds[i*3+2] = frameno;
	}
	if( frameno == 2 ) frameno = 0;
	send_ws_leds();

	uint8 ret = 0;
	ret = GetButtons();
	printf("Got buttons: %d\n", ret);
//	printf( "." );
	printf( "%d %d\n", VS.count, VS.rxcount );
	//XXX TODO: Check to see if we're connected here.

	CSTick( 1 ); // Send a one to uart
}

//XXX: Tricky this actualy comes from the I2S engine.  Otherwise we can't enter light sleep.
static void ICACHE_FLASH_ATTR procTask(os_event_t *events)
{
	//This operates at ~1620Hz
	static int ticks;
	CSTick( 0 );
	ticks++;
	if( ticks == 100 )
	{
		slowtick();
		ticks = 0;
	}
}

void ICACHE_FLASH_ATTR go_deepest_sleep_we_can()
{
	printf( "DEEPSLEEP\n" );
	ets_memset( leds, 0, sizeof( leds ) ); leds[1] = 1; leds[4] = 1; leds[7] = 1; leds[10] = 1;
	send_ws_leds();

	ets_delay_us(14000);
	send_ws_leds();
	os_delay_us(14000);
	testi2s_disable();

	wifi_station_disconnect();
	wifi_set_opmode_current( 0 ); //disconnect from wifi.

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_GPIO3);

	gpio_output_set(0,0,0, 0xffff);
	ets_wdt_disable();
	int i;
	for( i = 0; i < 3; i++ )
		do_pvvx_sleep(5000,0);	//In milliseconds.
	do_pvvx_sleep(5000,1);	//In milliseconds... and reboot.

	//XXX This code SHOULD never be executed.
	ets_delay_us(10000);
	void (*my_reset)() = (void (*)())0x400000a4;
	my_reset();

}

int ICACHE_FLASH_ATTR FailedToConnect( int wifi_fail_connects )
{
	//XXX TODO: Consider seeing if we were connected, if so, reattempt connection once.  If fails again, then do deep sleep.
	go_deepest_sleep_we_can();
}




//Called when new packet comes in.
static void ICACHE_FLASH_ATTR
udpserver_recv(void *arg, char *pusrdata, unsigned short len)
{
	struct espconn *pespconn = (struct espconn *)arg;

	uart0_sendStr("X");
}

void ICACHE_FLASH_ATTR charrx( uint8_t c )
{
	//Called from UART.
}



void user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	uart0_sendStr("\r\nesp82XX Web-GUI\r\n" VERSSTR "\b");

//Uncomment this to force a system restore.
//	system_restore();

	CSSettingsLoad( 0 );
	CSPreInit();

    pUdpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
	ets_memset( pUdpServer, 0, sizeof( struct espconn ) );
	espconn_create( pUdpServer );
	pUdpServer->type = ESPCONN_UDP;
	pUdpServer->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
	pUdpServer->proto.udp->local_port = COM_PORT;
	espconn_regist_recvcb(pUdpServer, udpserver_recv);

	if( espconn_create( pUdpServer ) )
	{
		while(1) { uart0_sendStr( "\r\nFAULT\r\n" ); }
	}

	CSInit();

	SetServiceName( "labs17" );
	AddMDNSName(    "esp82xx" );
	AddMDNSName(    "labs17" );
	AddMDNSService( "_http._tcp",    "An ESP82XX Webserver", WEB_PORT );
	AddMDNSService( "_espcom._udp",  "ESP82XX Comunication", COM_PORT );
	AddMDNSService( "_esp82xx._udp", "ESP82XX Backend",      BACKEND_PORT );

	//Add a process
	system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);

	//Timer example
//	os_timer_disarm(&some_timer);
//	os_timer_setfn(&some_timer, (os_timer_func_t *)myTimer, NULL);
//	os_timer_arm(&some_timer, 100, 1);

	printf( "Boot Ok.\n" );

	init_vive(); //Prepare the Vive core for receiving data.
	testi2s_init(); //Actually start the i2s engine.
	ets_memset( leds, 0, sizeof( leds ) ); leds[0] = 1; leds[5] = 1; leds[6] = 1; leds[11] = 1;
	send_ws_leds();

/*	wifi_station_disconnect();
	wifi_set_opmode_current( 0 ); //disconnect from wifi.
	memset( leds, 0, sizeof( leds ) ); // leds[1] = 1; leds[4] = 1; leds[7] = 1; leds[10] = 1;
	send_ws_leds();
	os_delay_us(60000);
	send_ws_leds();
	os_delay_us(60000);
*/

// Forced deep sleep (TEST, DO NOT ENABLE)
//	system_deep_sleep_set_option( 0 );
//	system_deep_sleep( 10000000 );
//	ets_delay_us(1000000);

	// pvvx sleep badge total: 9 ma  (if you call deepest_sleep_we_can_do)

	//~6-18mA but the CPU goes janky. when connected.  75mA when not connected.
//	wifi_set_sleep_type(LIGHT_SLEEP_T);
//	wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

	//~20-30mA but allows continuous CPU operation. when connected to AP, but 75mA when not connected.
	wifi_set_sleep_type(MODEM_SLEEP_T);
	wifi_fpm_set_sleep_type(MODEM_SLEEP_T);

//	system_os_post(procTaskPrio, 0, 0 );
}


//There is no code in this project that will cause reboots if interrupts are disabled.
void EnterCritical() { }

void ExitCritical() { }


