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
#define procTaskPrio        0
#define procTaskQueueLen    1

uint8_t mymac[6];
static struct espconn *pUdpServer;
os_event_t procTaskQueue[procTaskQueueLen];
uint8_t udp_pending;
uint8_t leds[12];

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

void ProcessData(uint8_t *data, int len) {}

static void ICACHE_FLASH_ATTR slowtick() {
  CSTick(1);
}

static void ICACHE_FLASH_ATTR procTask(os_event_t *events) {
  // Called at ~1620Hz
  static int ticks;
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
  printf("Got Packet!");
}

void ICACHE_FLASH_ATTR send_ws_leds() {
  ws2812_push( leds, 4);
}

void ICACHE_FLASH_ATTR go_deepest_sleep_we_can() {
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
  uint8_t init_buttons = GetButtons();
  printf("Initial buttons: %d\n", init_buttons);

  CSSettingsLoad(0);
  CSPreInit();

  struct station_config stationConf;
  wifi_station_get_config(&stationConf);
  wifi_get_macaddr(STATION_IF, mymac);

  printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mymac[0], mymac[1], mymac[2], mymac[3], mymac[4], mymac[5]);

  ets_memcpy(stationConf.ssid, "BadgeFi", 7);
  ets_memcpy(stationConf.password, "youmustconstructadditonalpylons", 31);

  stationConf.bssid_set = 0;
  wifi_set_opmode_current(2);
  wifi_set_opmode(2);
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

  system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);

  printf("Boot Ok.\n");
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_set_sleep_type(MODEM_SLEEP_T);
}

void EnterCritical() {}
void ExitCritical() {}
