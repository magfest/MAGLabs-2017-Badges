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
#include "esp_rawsend.h"
#include "commonservices.h"
#include "vars.h"
#include <mdns.h>
#include <vive_help.h>
#include <buttons.h>

#define procTaskPrio        0
#define procTaskQueueLen    1
#define REMOTE_IP_CODE 0x0a00c90a
#define UDP_TIMEOUT 500000  //0.5 seconds.
volatile uint32_t packet_tx_time;

uint8_t leds[12];
void ICACHE_FLASH_ATTR send_ws_leds()
{
	ws2812_push( leds, 4);
}

uint8_t mypacket[30+1536] = {  //256 = max size of additional payload
	0x08, //Frame type, 0x80 = beacon, Tried data, but seems to have been filtered on RX side by other ESP
	0x00, 0x00, 0x00, 
	0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,
	0x00, 0x00,  //Sequence number, cleared by espressif
	0x82, 0x66,	 //"Mysterious OLPC stuff"
	0x82, 0x66, 0x00, 0x00, //????
};
#define PACK_PAYLOAD_START 30

uint32_t last_ccount;
uint32_t scan_ccount;
int ledqtybytes = 12;
int do_led_push;

uint8_t last_leds[40*3] = {0};
uint32_t frame = 0;
uint8_t mymac[6];

int send_back_on_port;
uint32_t send_back_on_ip;

volatile int requested_update = 0;
int disable_push = 0;
int disable_raw_broadcast = 0;
int raw_broadcast_rate = 0x1a;

int soft_ap_mode = 0;
int udp_pending = 0; //If nonzero, do not attempt to send UDP packet. 
int wifi_fails = 0;
int do_deep_sleep = 0;
int disable_deep_sleep = 0;
uint32_t force_sleep_time = 0;
int need_to_do_scan = 0;

static int make_led_rssi = 0;
static int led_rssi = 0;
static int ledrssi_min;
static int ledrssi_max;
static int ledrssi_intensity;

int rainbow_run;
int rainbow_intensity;
int rainbow_speed;
int rainbow_offset;


uint8_t last_button_event_btn;
uint8_t last_button_event_dn;
static uint16_t status_update_count;
uint32_t ccount;
static int time_since_update = 0;
static int just_joined_network = 0;

//Packet length for UDP packets or -1 for a raw packet.
void ProcessData( uint8_t * data, int len )
{
	//PROTOCOL:
	//Bytes [0..5 MAC] ->
	// MAC from the server is ignored.
	//Starting at byte 6.

	//packets are sent to 10.201.0.2, UDP port 8000
	//Packets can come from anywhere.
	//Packets send packets to device on port 8001

	//Badge Status Update, sent at connect, after 10 seconds, upon request and button click.
	//  TO BADGE 0x11: Request status update.
	//  FROM BADGE 0x01: [0 reserved (could be version in future)] [RSSI] [BSSID of currently connected AP x6] [GPIO State] [Last button event, 0 if no events, 1-8 for button] [was last button event down] [Average LED power] [system 3.3v 2bytes] [status update count 2bytes] [heap free 2bytes] [sleep performance ratio] [0, reserved] [current time 4bytes]


	//Control WS2812Bs on badge.
	//    TO BADGE 0x02: [MAtch] [Mask] [Reserved, 0]   [GRB GRB GRB GRB ...]  NOTE: For raw packets, only 4 LEDs may be controlled.
	//		if (! ((mymac[5] ^ data[7])&data[8]) )

	//Glow LED to porportion of RSSI
	//    TO BADGE 0x03: [min RSSI] [max RSSI] [LED intensity]

	//Initiate SSID SCAN
	//    TO BADGE 0x04: (no parameters, ONLY UDP can control this!)
	//  FROM BADGE 0x05: [scan_timestamp (4 bytes)] [stations count in this message] [ [BSSIDx6bytes] [RSSI] [Channel] ]

	//Rainbow effect
	//    TO BADGE 0x07: [Reserved, 0] [Reserved, 0] [Reserved, 0] [Run time, MSB] [Run Time, LSB] [rainbow speed] [rainbow intensity] [rainbow offset per led]    Run time in ms.

	//Device configure
	//    TO BADGE 0x08: [requested update 1/0] [disable_push 1/0] [disable raw broadcast 1/0] [raw broadcast rate, recommended 0x1a]

	//Force deep sleep (UDP only)
	//    TO PADGE 0x09: [sleep ms MSB (4 bytes)] [0xaa, must be 0xaa]

	if( data[6] == 0x11 && len > 1 )
	{
		requested_update = 1;
	}

	//0x01 is from badge, status packet
	if( data[6] == 0x02 )
	{
		//Update LEDs
		rainbow_run = 0;
		if( len <= 0 )
		{
			len = 22;
		}
		if (! ((mymac[5] ^ data[7])&data[8]) )
		{
			ets_memcpy( last_leds, data + 10, len - 10 );
			ledqtybytes = len-10;
			do_led_push = 1;
		}
	}
	if( data[6] == 0x03 )  //RSSI the LED.
	{
		if( len == -1 )
		{
			static int rl1;
			static int rl2;
			static int rl3;
			int rl4;
			rl4 = rl3;
			rl3 = rl2;
			rl2 = rl1;
			rl1 = data[-42];
			led_rssi = (rl1+rl2+rl3+rl1)/4;
		}
		else
		{
			led_rssi = -1;
		}

		rainbow_run = 0;

		ledrssi_min = data[7];
		ledrssi_max = data[8];
		ledrssi_intensity = data[9];

		make_led_rssi = 1;
	}

	if( data[6] == 0x04 && len > 1 )  //Scan; make sure this can only be done via 
	{
		need_to_do_scan = 1;
	}

	//0x05 is from badge, browse response.

	if( data[6] == 0x07 )
	{
		rainbow_run = ((data[10]<<8) | data[11])*1000;
		rainbow_speed = data[12];
		rainbow_intensity = data[13];
		rainbow_offset = data[14];
	}
	if( data[6] == 0x08 && len > 1 )
	{
		requested_update = data[7];
		disable_push = data[8];
		disable_raw_broadcast = data[9];
		raw_broadcast_rate = data[10];
	}
	if( data[6] == 0x09 && len > 1 )
	{
		force_sleep_time = (data[7]<<24) || (data[8]<<16) || (data[9]<<8) || (data[10]);
	}
}

void  __attribute__ ((noinline)) rx_func( struct RxPacket * r, void ** v )
{
	if( r->data[24] != 0x82 || r->data[25] != 0x66 || r->data[26] != 0x82 || r->data[27] != 0x66 )
	{
		return;
	}

	ProcessData( &r->data[30], -1 );
}

void udpsendok_cb(void *arg)
{
	udp_pending = 0;
}

//Called when new packet comes in.
static void ICACHE_FLASH_ATTR
udpserver_recv(void *arg, char *pusrdata, unsigned short len)
{
	printf("Got packet\n");
	struct espconn * rc = (struct espconn *)arg;
	remot_info * ri = 0;
	espconn_get_connection_info( rc, &ri, 0);

	if( pusrdata[6] == 0x11 || pusrdata[6] == 0x04 )
	{
		send_back_on_ip = IP4_to_uint32(ri->remote_ip);
		send_back_on_port = ri->remote_port;
	}
	struct espconn *pespconn = (struct espconn *)arg;
	ProcessData( pusrdata, len );
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
	frameno = 1;
	for( i = 0; i < 4; i++ )
	{
		leds[i*3+0] = frameno;
		leds[i*3+1] = frameno;
		leds[i*3+2] = frameno;
	}
	if( frameno == 2 ) frameno = 0;
	send_ws_leds();

	uint8 ret = 0;
/*	printf("             RDLUSEBA\n");
	printf("Got buttons: %c%c%c%c%c%c%c%c\n",
		   (ret >> 0) & 1 ? 'x':'-',
		   (ret >> 1) & 1 ? 'x':'-',
		   (ret >> 2) & 1 ? 'x':'-',
		   (ret >> 3) & 1 ? 'x':'-',
		   (ret >> 4) & 1 ? 'x':'-',
		   (ret >> 5) & 1 ? 'x':'-',
		   (ret >> 6) & 1 ? 'x':'-',
		   (ret >> 7) & 1 ? 'x':'-');
	printf("Got buttons: %d\n", ret);
	printf( "%d %d\n", VS.count, VS.rxcount );
*/	//XXX TODO: Check to see if we're connected here.

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

	//Determine if we want to send a status packet.
	if( 0 )
	{
		printf("Sending status...\n");
		uint8_t * pp = &mypacket[PACK_PAYLOAD_START];
		static int sleepperformance = 0;

		#define STATUS_PACKSIZE 32

		pp+=6; //MAC is first.		

		*(pp++) = 0x01; //Message type
		*(pp++) = 0;
		*(pp++) = wifi_station_get_rssi();	//Wifi power

		{
			struct station_config stationConf;
			wifi_station_get_config(&stationConf);
			ets_memcpy( pp, stationConf.bssid, 6 );
		}
		pp+=6;

		*(pp++) = LastGPIOState;          							//Last GPIO State
		//printf( "%d %d %d\n", LastGPIOState, last_button_event_btn,last_button_event_dn);
		*(pp++) = last_button_event_btn; last_button_event_btn = 0; //Last GPIO Event (0 = none)
		*(pp++) = last_button_event_dn;  							 //Was last GPIO Event down? 
		//computer power for LED.
		int i;
		int sum;
		for( i = 0; i < 12; i++ )
		{
			sum += last_leds[i];
		}
		*(pp++) = sum / 12;						//Average power to LEDs

		uint16_t vv = system_get_vdd33();					//System 3.3v
		*(pp++) = vv>>8;
		*(pp++) = vv & 0xff;

		*(pp++) = status_update_count>>8;						//# of packets sent.
		*(pp++) = status_update_count&0xff;

		uint16_t heapfree = system_get_free_heap_size();
		*(pp++) = heapfree>>8;
		*(pp++) = heapfree & 0xff;

		//Metric discussing how effective is sleeping.
		*(pp++) = sleepperformance & 0xff;
		*(pp++) = 0;

		*(pp++) = ccount>>24;
		*(pp++) = ccount>>16;
		*(pp++) = ccount>>8;
		*(pp++) = ccount>>0;

		status_update_count++;
		time_since_update = 0;
		just_joined_network = 0;
		requested_update = 0;

		if( send_back_on_ip && send_back_on_port )
		{
			pUdpServer->proto.udp->remote_port = send_back_on_port;
			uint32_to_IP4(send_back_on_ip,pUdpServer->proto.udp->remote_ip);
			send_back_on_ip = 0; send_back_on_port = 0;
		}
		else
		{
			pUdpServer->proto.udp->remote_port = 8000;
			uint32_to_IP4(REMOTE_IP_CODE,pUdpServer->proto.udp->remote_ip);  
		}

		udp_pending = UDP_TIMEOUT;
		espconn_send( (struct espconn *)pUdpServer, &mypacket[PACK_PAYLOAD_START], STATUS_PACKSIZE );

		if( !disable_raw_broadcast )
		{
			packet_tx_time = 0;
			wifi_set_phy_mode(PHY_MODE_11N);
			wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
			wifi_send_pkt_freedom( mypacket, 30 + STATUS_PACKSIZE, true) ; 
		}
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

        struct station_config stationConf;
        wifi_station_get_config(&stationConf);
        wifi_get_macaddr(STATION_IF, mymac);

	ets_memcpy( stationConf.ssid, "BadgeFi", 7 );
	ets_memcpy( stationConf.password, "youmustconstructadditonalpylons", 31 );
        stationConf.bssid_set = 0;
        wifi_set_opmode_current( 1 );
        wifi_set_opmode( 1 );
        wifi_station_set_config(&stationConf);
        wifi_station_connect();
        wifi_station_set_config(&stationConf);  //I don't know why, doing this twice seems to make it store more reliably.
                                                                                                                                                                
    pUdpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
	ets_memset( pUdpServer, 0, sizeof( struct espconn ) );
	espconn_create( pUdpServer );
	pUdpServer->type = ESPCONN_UDP;
	pUdpServer->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
	pUdpServer->proto.udp->local_port = 8001;
	pUdpServer->proto.udp->remote_port = 8000;
	uint32_to_IP4(0x0a00c90a,pUdpServer->proto.udp->remote_ip);
	espconn_regist_recvcb(pUdpServer, udpserver_recv);
	espconn_regist_sentcb(pUdpServer, udpsendok_cb);
		

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

static uint8_t global_scan_complete = 0;
static uint8_t global_scan_elements = 0;
static uint8_t global_scan_data[2048];  //Where SSID, channel, power go.

static void ICACHE_FLASH_ATTR scandone(void *arg, STATUS status)
{
	uint8_t * gsp = global_scan_data;
	global_scan_elements = 0;
	scaninfo *c = arg;
	struct bss_info *inf;
	scan_ccount = last_ccount;
	if( !c->pbss ) {global_scan_elements = 0;  global_scan_complete = 1; return;  }
	STAILQ_FOREACH(inf, c->pbss, next) {
		global_scan_elements++;
		//not using inf->authmode;
		ets_memcpy( gsp, inf->bssid, 6 );
		gsp+=6;
		*(gsp++) = inf->rssi;
		*(gsp++) = inf->channel;
		if( gsp - global_scan_data >= 2040 ) break;
	}
	global_scan_complete = 1;
}

//There is no code in this project that will cause reboots if interrupts are disabled.
void EnterCritical() { }

void ExitCritical() { }


