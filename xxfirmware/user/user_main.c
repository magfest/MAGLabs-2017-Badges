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

#define REMOTE_IP_CODE 0x0a00c90a
//#define REMOTE_IP_CODE 0xd65b650a
#define MARK_IP        0x885c650a

#define procTaskPrio        0
#define procTaskQueueLen    1
#define STATUS_PACKSIZE 32
#define FIRMWARE_VERSION 2

os_event_t procTaskQueue[procTaskQueueLen];
uint8_t mymac[6];
static struct espconn *pUdpServer;
uint8_t udp_pending = 0;
uint8_t leds[12];
static int ticks = 0;
static int bigticks = 0;
int send_back_on_port;
uint32_t send_back_on_ip;
static int did_raw_init = 0;
uint8_t init_steps = 2;

// Status vars
uint8_t new_buttons = 0;
uint8_t last_buttons = 0;
uint8_t last_button_event_btn = 0;
uint8_t last_button_event_dn = 0;

uint8_t requested_update = 1;
uint16_t status_update_count = 0;
uint8_t just_joined_network = 1;

uint8_t rainbow_run = 0;
uint8_t rainbow_speed = 0;
uint8_t rainbow_intensity = 0;
uint8_t rainbow_offset = 0;

uint8_t ledqtybytes = 0;
uint8_t do_led_push = 0;

uint8_t ledrssi_min = 0;
uint8_t ledrssi_max = 0;
uint8_t ledrssi_intensity = 0;
uint8_t make_led_rssi = 0;
uint8_t led_rssi = 0;

uint8_t need_to_do_scan = 1;
static uint8_t global_scan_complete = 0;
static uint8_t global_scan_elements = 0;
static uint8_t global_scan_data[2048];  //Where SSID, channel, power go.

uint8_t disable_push = 0;
uint8_t disable_raw_broadcast = 0;
uint8_t raw_broadcast_rate = 
	PHY_RATE_6; //
	//0x0c;   //54 MBPS 802.11g
	//0x0a;  //12 MBPS 802.11g

uint8_t force_sleep_time = 0;
uint8_t do_deep_sleep = 0;

uint8_t mypacket[30+1536] = {  //256 = max size of additional payload
        0x08, //Frame type, 0x80 = beacon, Tried data, but seems to have be$
        0x00, 0x00, 0x00,
        0xff,0xff,0xff,0xff,0xff,0xff,
        0xff,0xff,0xff,0xff,0xff,0xff,
        0xff,0xff,0xff,0xff,0xff,0xff,
        0x00, 0x00,  //Sequence number, cleared by espressif
        0x82, 0x66,      //"Mysterious OLPC stuff"
        0x82, 0x66, 0x00, 0x00, //????
};
#define PACK_PAYLOAD_START 30

void ICACHE_FLASH_ATTR send_ws_leds() {
  ws2812_push( leds, 4);
}


static void ICACHE_FLASH_ATTR scandone(void *arg, STATUS status)
{
	//printf("WiFi scan is complete\n");
	uint8_t * gsp = global_scan_data;
	global_scan_elements = 0;
	scaninfo *c = arg;
	struct bss_info *inf;
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

void ICACHE_FLASH_ATTR ProcessData(uint8_t *data, int len) {
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
    //printf("Status update requested\n");
    requested_update = 1;
  }

  //0x01 is from badge, status packet
  if( data[6] == 0x02 )
  {
    //printf("LED Update\n");
    //Update LEDs
    rainbow_run = 0;
    if( len <= 0 )
    {
      len = 22;
    }
    if (! ((mymac[5] ^ data[7])&data[8]) )
    {
      ets_memcpy( leds, data + 10, 12 );
      ledqtybytes = len-10;
      //printf("ledqtybytes %d\n", ledqtybytes);
      do_led_push = 1;
    }
  }
  if( data[6] == 0x03 )  //RSSI the LED.
  {
    //printf("Set LED to RSSI\n");
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
    //printf("Status packet request\n");
    need_to_do_scan = 1;
  }

  //0x05 is from badge, browse response.
  if( data[6] == 0x07 )
  {
    //printf("Rainbow mode\n");
    rainbow_run = ((data[10]<<8) | data[11])*1000;
    rainbow_speed = data[12];
    rainbow_intensity = data[13];
    rainbow_offset = data[14];
  }

  if( data[6] == 0x08 && len > 1 )
  {
    //printf("Configure updates\n");
    requested_update = data[7];
    disable_push = data[8];
    disable_raw_broadcast = data[9];
    raw_broadcast_rate = data[10];
	//wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
  }

  if( data[6] == 0x09 && len > 1 )
  {
    //printf("Setting sleep mode\n");
    force_sleep_time = (data[7]<<24) || (data[8]<<16) || (data[9]<<8) || (data[10]);
    do_deep_sleep = data[11] == 0xAA; 
  }
  if( data[6] == 0x12 && len > 1 )
  {
    if( data[9] )
      DensePrintBig( data[7], data[8], data+10 ); 
    else
      DensePrint( data[7], data[8], data+10 ); 
  }
}

void ICACHE_FLASH_ATTR send_status_update() {
    //printf("Status update %d\n", status_update_count);
    requested_update = 0;
    uint8_t *pp = &mypacket[PACK_PAYLOAD_START];
    pp += 6; // MAC Address
    *(pp++) = 0x01; //Message type
    *(pp++) = FIRMWARE_VERSION;
    *(pp++) = wifi_station_get_rssi(); // WiFi Power
    struct station_config stationConf;
    wifi_station_get_config(&stationConf);
    ets_memcpy(pp, stationConf.bssid, 6);
    pp += 6;
    *(pp++) = new_buttons;
    *(pp++) = last_button_event_btn; last_button_event_btn = 0;
    *(pp++) = last_button_event_dn;
    //computer power for LED.
    int i;
    int sum;
    for( i = 0; i < 12; i++ ) {
      sum += leds[i];
    }
    *(pp++) = sum / 12;

    uint16_t vv = system_get_vdd33();					//System 3.3v
    *(pp++) = vv>>8;
    *(pp++) = vv & 0xff;

    *(pp++) = status_update_count>>8;						//# of packets sent.
    *(pp++) = status_update_count&0xff;

    uint16_t heapfree = system_get_free_heap_size();
    *(pp++) = heapfree>>8;
    *(pp++) = heapfree & 0xff;

    //Metric discussing how effective is sleeping.
    //*(pp++) = sleepperformance & 0xff;
    *(pp++) = 0;
    *(pp++) = 0;

    *(pp++) = 0;
    *(pp++) = 0;
    *(pp++) = 0;
    *(pp++) = 0;

    status_update_count++;
    just_joined_network = 0;

    if( printed_ip ) {
      if( send_back_on_ip && send_back_on_port ) {
        pUdpServer->proto.udp->remote_port = send_back_on_port;
        uint32_to_IP4(send_back_on_ip,pUdpServer->proto.udp->remote_ip);
        send_back_on_ip = 0; send_back_on_port = 0;
      } else {
        pUdpServer->proto.udp->remote_port = 8000;
        uint32_to_IP4(REMOTE_IP_CODE,pUdpServer->proto.udp->remote_ip);  
      }

      udp_pending = 1;
      espconn_send( (struct espconn *)pUdpServer, &mypacket[PACK_PAYLOAD_START], STATUS_PACKSIZE );
    }

    if( !disable_raw_broadcast )
    {
      wifi_set_phy_mode(PHY_MODE_11N);
      //wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
      wifi_send_pkt_freedom( mypacket, 30 + STATUS_PACKSIZE, true) ; 
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// This is the raw packet stuff
///////////////////////////////////////////////////////////////////////////////////////////////////


void  __attribute__ ((noinline)) rx_func( struct RxPacket * r, void ** v )
{
	if( r->data[24] != 0x82 || r->data[25] != 0x66 || r->data[26] != 0x82 || r->data[27] != 0x66 )
	{
		return;
	}

	ProcessData( &r->data[30], -1 );
}


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


static void ICACHE_FLASH_ATTR slowtick() {
  bigticks++;
  if (!(bigticks % 5)) {
    if (init_steps) {
      printf("Init: %d\n", init_steps);
      int i;
      if (init_steps == 2) {
        for (i=0;i<12;i=i+3) {
          leds[i] = 25;
        }
      } else if (init_steps == 1) {
        for (i=0;i<12;i++) {
          leds[i] = 4;
        }
      }
      send_ws_leds();
      init_steps--;
    }
  }
  if (bigticks == 100) {
    if (!udp_pending && printed_ip) {
      send_status_update();
    }
    bigticks = 0;
    udp_pending = 0;
  }
  CSTick(1);


  //This configures the monitor mode.
  if( printed_ip && !did_raw_init )
  {
      int i;
      for (i=0;i<12;i+=2) {
        leds[i] = 0;
      }
      send_ws_leds();

    wifi_set_raw_recv_cb( rx_func );
    //wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
    did_raw_init = 1;
  }
/*
  //Make everyone's badges red.
  mypacket[PACK_PAYLOAD_START+6] = 0x02;
  mypacket[PACK_PAYLOAD_START+7] = 0x00;
  mypacket[PACK_PAYLOAD_START+8] = 0x00;
  mypacket[PACK_PAYLOAD_START+9] = 0x00;

  mypacket[PACK_PAYLOAD_START+10] = 0x00;
  mypacket[PACK_PAYLOAD_START+11] = 0xff;
  mypacket[PACK_PAYLOAD_START+12] = 0x00;
  mypacket[PACK_PAYLOAD_START+13] = 0x00;
  mypacket[PACK_PAYLOAD_START+14] = 0xff;
  mypacket[PACK_PAYLOAD_START+15] = 0x00;
  mypacket[PACK_PAYLOAD_START+16] = 0x00;
  mypacket[PACK_PAYLOAD_START+17] = 0xff;
  mypacket[PACK_PAYLOAD_START+18] = 0x00;
  mypacket[PACK_PAYLOAD_START+19] = 0x00;
  mypacket[PACK_PAYLOAD_START+20] = 0xff;
  mypacket[PACK_PAYLOAD_START+21] = 0x00;
  mypacket[PACK_PAYLOAD_START+22] = 0x00;

  wifi_set_phy_mode(PHY_MODE_11N);
  //wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
  wifi_send_pkt_freedom( mypacket, 30 + STATUS_PACKSIZE, true) ; 
*/
/*
  //Wifi strength
  mypacket[PACK_PAYLOAD_START+6] = 0x03;
  mypacket[PACK_PAYLOAD_START+7] = 0x10;
  mypacket[PACK_PAYLOAD_START+8] = 0x20;
  mypacket[PACK_PAYLOAD_START+9] = 0x01;
  wifi_set_phy_mode(PHY_MODE_11N);
  //wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
  wifi_send_pkt_freedom( mypacket, 30 + STATUS_PACKSIZE, true) ; 
*/

}

static void ICACHE_FLASH_ATTR check_wifi_scan() {
	if(need_to_do_scan && printed_ip)
	{
#if 1
		//printf("Initiating WiFi Scan\n");
		int r;
		struct scan_config sc;
		sc.ssid = 0;  sc.bssid = 0;  sc.channel = 0;  sc.show_hidden = 1;
		global_scan_complete = 0;
		global_scan_elements = 0;
		r = wifi_station_scan(&sc, scandone );
		if( r )
		{
			//printf("WiFi Scan is complete.\n");
			//Scan good
		}
		else
		{
			global_scan_complete = 1;
			global_scan_elements = 0;
		}
		//send out globalscanpacket.
		need_to_do_scan = 0;
#endif
	}

	//XXX Tricky, after a scan is complete, start shifting the UDP data out.
	if( global_scan_complete && !udp_pending )
	{
		int i;
		uint8_t * pp = &mypacket[PACK_PAYLOAD_START];
		pp+=6; //MAC is first.		
		*(pp++) = 0x05; //Message type

		//do this here so we always have a "scan complete" packet sent back to host.
		if( global_scan_elements == 0 )
		{
			send_back_on_ip = 0;
			send_back_on_port = 0;
			global_scan_complete = 0;
		}

#define MAX_STATIONS_PER_PACKET 130

		int stations = global_scan_elements;
		if( stations > MAX_STATIONS_PER_PACKET ) stations = MAX_STATIONS_PER_PACKET;
		*(pp++) = 0;
		*(pp++) = 0;
		*(pp++) = 0;
		*(pp++) = 0;

		*(pp++) = stations;

		ets_memcpy( pp, global_scan_data, stations*8 );
		pp += stations*8;
		//Slide all remaining stations up.
		for( i = 0; i < global_scan_elements - stations; i++ )
		{
			ets_memcpy( global_scan_data + i*8, global_scan_data + (i+stations)*8, 8 );
		}
		global_scan_elements -= stations;


		if( send_back_on_ip && send_back_on_port)
		{
			pUdpServer->proto.udp->remote_port = send_back_on_port;
			uint32_to_IP4(send_back_on_ip,pUdpServer->proto.udp->remote_ip);  
		}
		else
		{
			pUdpServer->proto.udp->remote_port = 8000;
			uint32_to_IP4(REMOTE_IP_CODE,pUdpServer->proto.udp->remote_ip);  
		}


		udp_pending = 1;
		espconn_send( (struct espconn *)pUdpServer, &mypacket[PACK_PAYLOAD_START], pp - &mypacket[PACK_PAYLOAD_START]  );
	}
}


void ICACHE_FLASH_ATTR go_deepest_sleep_we_can() {
  printf( "DEEPSLEEP\n" );
  DensePrint( 24, 7, "No network\n" ); 
  ets_memset( leds, 0, sizeof( leds ) ); leds[1] = 4; leds[4] = 0; leds[7] = 0; leds[10] = 0;
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
  do_pvvx_sleep(5000,0);	//In milliseconds.
  TurnOffOLED();
  for( i = 0; i < 5; i++ )
    do_pvvx_sleep(5000,0);	//In milliseconds.
  do_pvvx_sleep(5000,1);	//In milliseconds... and reboot.
  //XXX This code SHOULD never be executed.
  ets_delay_us(10000);
  void (*my_reset)() = (void (*)())0x400000a4;
  my_reset();

}
static int did_print_ip;

static void ICACHE_FLASH_ATTR procTask(os_event_t *events) {
  if (do_deep_sleep) {
    go_deepest_sleep_we_can();
  }
  check_wifi_scan();
  // Called at ~1620Hz
  new_buttons = GetButtons();
  if (new_buttons != last_buttons) {
    printf("Button pressed\n");
    int i;
    for(i=0; i<8; i++) {
      int mask = 1<<i;
      if((new_buttons & mask) != (last_buttons & mask)) {
        last_button_event_btn = i+1;
        printf("Button %d was ", last_button_event_btn);
        last_button_event_dn = (new_buttons & mask)?1:0;
        if (last_button_event_dn) {
          printf("pushed\n");
        } else {
          printf("released\n");
        }
        if (printed_ip) {
          send_status_update();
        }
      }
    }
  }

  if( !did_print_ip && printed_ip )
  {
    char ctspr[100];

    struct ip_info ipi;
    wifi_get_ip_info(0, &ipi);
	#define chop_ip(x) (((x)>>0)&0xff), (((x)>>8)&0xff), (((x)>>16)&0xff), (((x)>>24)&0xff)
    ets_sprintf( ctspr, "%d.%d.%d.%d", chop_ip(ipi.ip.addr)  );
    DensePrint( 24, 7, ctspr ); 
  }
  
  if (!udp_pending && printed_ip && requested_update) {
    send_status_update();
  }

  if (do_led_push) {
    send_ws_leds();
    do_led_push = 0;
  }
  
  last_buttons = new_buttons;
  CSTick(0);
  ticks++;
  if (ticks == 100) {
    slowtick();
    ticks = 0;
  }
}

void udpsendok_cb(void *arg) {
  udp_pending = 0;
}

static void ICACHE_FLASH_ATTR udpserver_recv(void *arg, char *pusrdata, unsigned short len) {
  //printf("Got Packet!");
  struct espconn * rc = (struct espconn *)arg;
  remot_info * ri = 0;
  espconn_get_connection_info( rc, &ri, 0);

  if( pusrdata[6] == 0x11 || pusrdata[6] == 0x04 ) {
    send_back_on_ip = IP4_to_uint32(ri->remote_ip);
    send_back_on_port = ri->remote_port;
  }
  struct espconn *pespconn = (struct espconn *)arg;
  ProcessData(pusrdata, len);
}

int ICACHE_FLASH_ATTR FailedToConnect( int wifi_fail_connects ) {
  //XXX TODO: Consider seeing if we were connected, if so, reattempt connection once.  If fails again, then do deep sleep.
  printf("Failed to connect: %d\n", wifi_fail_connects);
  go_deepest_sleep_we_can();
}



void ICACHE_FLASH_ATTR charrx( uint8_t c ) {
}

void user_init(void) {
  uart_init(BIT_RATE_115200, BIT_RATE_115200);
  printf("\nesp8266 Badge for MAGFest Labs 2\n" VERSSTR "\n");
  last_buttons = GetButtons();
  printf("Initial buttons: %d\n", last_buttons);


	struct rst_info * r = system_get_rst_info();
	printf( "Reason: %p\n", r->reason );
	printf( "Exec  : %p\n", r->exccause );
	printf( "epc1  : %p\n", r->epc1 );
	printf( "epc2  : %p\n", r->epc2 );
	printf( "epc3  : %p\n", r->epc3 );
	printf( "excvaddr:%p\n", r->excvaddr );
	printf( "depc: %p\n", r->depc );


  CSSettingsLoad(0);
  CSPreInit();

  struct station_config stationConf;
  wifi_station_get_config(&stationConf);
  wifi_get_macaddr(STATION_IF, mymac);

  printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mymac[0], mymac[1], mymac[2], mymac[3], mymac[4], mymac[5]);

  ets_memcpy(stationConf.ssid, "BadgeFi", 8);
  ets_memcpy(stationConf.password, "youmustconstructadditonalpylons", 32);

  stationConf.bssid_set = 0;
  wifi_set_opmode_current(1);
  wifi_set_opmode(1);
  wifi_station_set_config(&stationConf);
  wifi_station_connect();
  wifi_station_set_config(&stationConf); // Charles has seen issues without the second call to this.

  pUdpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
  ets_memset(pUdpServer, 0, sizeof(struct espconn));
  pUdpServer->type = ESPCONN_UDP;
  pUdpServer->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
  pUdpServer->proto.udp->local_port = 8081;
  pUdpServer->proto.udp->remote_port = 8080;
  uint32_to_IP4(REMOTE_IP_CODE, pUdpServer->proto.udp->remote_ip);
  espconn_regist_recvcb(pUdpServer, udpserver_recv);
  espconn_regist_sentcb(pUdpServer, udpsendok_cb);

  if (espconn_create(pUdpServer)) {
    printf("Failed to create UDP server!!!\n");
    printf("A fatal error has occured.\n");
  }

  CSInit();

  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_set_sleep_type(MODEM_SLEEP_T);
  
  wifi_get_macaddr(STATION_IF, mypacket + 10);
  wifi_get_macaddr(STATION_IF, mypacket + PACK_PAYLOAD_START);  
  
  init_vive(); //Prepare the Vive core for receiving data.
  testi2s_init(); //Actually start the i2s engine.
  
  system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);

  InitOLED();

  DensePrintBig( 24, 0, "MAGLabs\n" ); 
  DensePrint( 24, 2, "OLED Addon Mod\n" ); 

  printf("Boot Ok.\n");
}

void EnterCritical() {}
void ExitCritical() {}
