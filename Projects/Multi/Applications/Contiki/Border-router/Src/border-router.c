/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         border-router
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/slip.h"

#include "servreg-hack.h"
#include "stm32l1xx_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "ESP8266.h"
#include "hw-config.h"


/** @addtogroup Border_router
  * @{
  */

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"



//extern volatile uint8_t UART_RxBuffer[UART_RxBufferSize];
extern  UART_HandleTypeDef huart4;
static uip_ipaddr_t prefix;
static uint8_t prefix_set;
static uint8_t sender_ip[16];

extern bool new_data;
uint8_t ServerCommandFlag=0;
uint32_t Server_Command_Len=0;

uip_ipaddr_t server_ip;  // for registraion

//flash command
uint8_t readEEPROMByte(uint32_t address);
HAL_StatusTypeDef writeEEPROMByte(uint32_t address, uint8_t data);




//uint8_t DMAstr[7];
/* for receiver function */
#define UDP_PORT 1234
#define SERVICE_ID 190

#define SEND_INTERVAL		(10 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

static struct simple_udp_connection unicast_connection;
PROCESS(unicast_receiver_process, "Unicast receiver example process");
/* for receiver function */

PROCESS(border_router_process, "Border router process");

  //set router prefix as:0xaaaa  Thomas
static uip_ipaddr_t router_prefix={0xaa,0xaa,0,0,0,0,0,0};  


//
static void create_rpl_dag(uip_ipaddr_t *ipaddr);

#if WEBSERVER==0
/* No webserver */
//AUTOSTART_PROCESSES(&border_router_process);
AUTOSTART_PROCESSES(&unicast_receiver_process);
#elif WEBSERVER>1
/* Use an external webserver application */
#include "webserver-nogui.h"
//AUTOSTART_PROCESSES(&border_router_process,&webserver_nogui_process);
AUTOSTART_PROCESSES(&unicast_receiver_process);
#else
/* Use simple webserver with only one page for minimum footprint.
 * Multiple connections can result in interleaved tcp segments since
 * a single static buffer is used for all segments.
 */
#include "httpd-simple.h"
/* The internal webserver can provide additional information if
 * enough program flash is available.
 */
#define WEBSERVER_CONF_LOADTIME 0
#define WEBSERVER_CONF_FILESTATS 0
#define WEBSERVER_CONF_NEIGHBOR_STATUS 0
/* Adding links requires a larger RAM buffer. To avoid static allocation
 * the stack can be used for formatting; however tcp retransmissions
 * and multiple connections can result in garbled segments.
 * TODO:use PSOCk_GENERATOR_SEND and tcp state storage to fix this.
 */
#define WEBSERVER_CONF_ROUTE_LINKS 0
#if WEBSERVER_CONF_ROUTE_LINKS
#define BUF_USES_STACK 1
#endif
#if 0
PROCESS(webserver_nogui_process, "Web server");
PROCESS_THREAD(webserver_nogui_process, ev, data)
{
  PROCESS_BEGIN();

 // httpd_init();

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    httpd_appcall(data);
  }

  PROCESS_END();
}
#endif
//AUTOSTART_PROCESSES(&border_router_process);//,&webserver_nogui_process);
AUTOSTART_PROCESSES(&unicast_receiver_process);
static const char *TOP = "<html><head><title>ContikiRPL</title></head><body>\n";
static const char *BOTTOM = "</body></html>\n";
#if BUF_USES_STACK
static char *bufptr, *bufend;
#define ADD(...) do {                                                   \
    bufptr += snprintf(bufptr, bufend - bufptr, __VA_ARGS__);      \
  } while(0)
#else
static char buf[256];
static int blen;
#define ADD(...) do {                                                   \
    blen += snprintf(&buf[blen], sizeof(buf) - blen, __VA_ARGS__);      \
  } while(0)
#endif

#endif
 
/*---------------------------------------------------------------------------*/
static void
ipaddr_add(const uip_ipaddr_t *addr)
{
  uint16_t a;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) ADD("::");
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        ADD(":");
      }
      ADD("%x", a);
    }
  }
}
/*---------------------------------------------------------------------------*/

static
PT_THREAD(generate_routes(struct httpd_state *s))
{
  static uip_ds6_route_t *r;
  static uip_ds6_nbr_t *nbr;
#if BUF_USES_STACK
  char buf[256];
#endif
#if WEBSERVER_CONF_LOADTIME
  static clock_time_t numticks;
  numticks = clock_time();
#endif

  PSOCK_BEGIN(&s->sout);

  SEND_STRING(&s->sout, TOP);
#if BUF_USES_STACK
  bufptr = buf;bufend=bufptr+sizeof(buf);
#else
  blen = 0;
#endif
  ADD("Neighbors<pre>");

  for(nbr = nbr_table_head(ds6_neighbors);
      nbr != NULL;
      nbr = nbr_table_next(ds6_neighbors, nbr)) {

#if WEBSERVER_CONF_NEIGHBOR_STATUS
#if BUF_USES_STACK
{char* j=bufptr+25;
      ipaddr_add(&nbr->ipaddr);
      while (bufptr < j) ADD(" ");
      switch (nbr->state) {
      case NBR_INCOMPLETE: ADD(" INCOMPLETE");break;
      case NBR_REACHABLE: ADD(" REACHABLE");break;
      case NBR_STALE: ADD(" STALE");break;
      case NBR_DELAY: ADD(" DELAY");break;
      case NBR_PROBE: ADD(" NBR_PROBE");break;
      }
}
#else
{uint8_t j=blen+25;
      ipaddr_add(&nbr->ipaddr);
      while (blen < j) ADD(" ");
      switch (nbr->state) {
      case NBR_INCOMPLETE: ADD(" INCOMPLETE");break;
      case NBR_REACHABLE: ADD(" REACHABLE");break;
      case NBR_STALE: ADD(" STALE");break;
      case NBR_DELAY: ADD(" DELAY");break;
      case NBR_PROBE: ADD(" NBR_PROBE");break;
      }
}
#endif
#else
      ipaddr_add(&nbr->ipaddr);
#endif

      ADD("\n");
#if BUF_USES_STACK
      if(bufptr > bufend - 45) {
        SEND_STRING(&s->sout, buf);
        bufptr = buf; bufend = bufptr + sizeof(buf);
      }
#else
      if(blen > sizeof(buf) - 45) {
        SEND_STRING(&s->sout, buf);
        blen = 0;
      }
#endif
  }
  ADD("</pre>Routes<pre>");
  SEND_STRING(&s->sout, buf);
#if BUF_USES_STACK
  bufptr = buf; bufend = bufptr + sizeof(buf);
#else
  blen = 0;
#endif

  for(r = uip_ds6_route_head(); r != NULL; r = uip_ds6_route_next(r)) {

#if BUF_USES_STACK
#if WEBSERVER_CONF_ROUTE_LINKS
    ADD("<a href=http://[");
    ipaddr_add(&r->ipaddr);
    ADD("]/status.shtml>");
    ipaddr_add(&r->ipaddr);
    ADD("</a>");
#else
    ipaddr_add(&r->ipaddr);
#endif
#else
#if WEBSERVER_CONF_ROUTE_LINKS
    ADD("<a href=http://[");
    ipaddr_add(&r->ipaddr);
    ADD("]/status.shtml>");
    SEND_STRING(&s->sout, buf); //TODO: why tunslip6 needs an output here, wpcapslip does not
    blen = 0;
    ipaddr_add(&r->ipaddr);
    ADD("</a>");
#else
    ipaddr_add(&r->ipaddr);
#endif
#endif
    ADD("/%u (via ", r->length);
    ipaddr_add(uip_ds6_route_nexthop(r));
    if(1 || (r->state.lifetime < 600)) {
      ADD(") %lus\n", (unsigned long)r->state.lifetime);
    } else {
      ADD(")\n");
    }
    SEND_STRING(&s->sout, buf);
#if BUF_USES_STACK
    bufptr = buf; bufend = bufptr + sizeof(buf);
#else
    blen = 0;
#endif
  }
  ADD("</pre>");

#if WEBSERVER_CONF_FILESTATS
  static uint16_t numtimes;
  ADD("<br><i>This page sent %u times</i>",++numtimes);
#endif

#if WEBSERVER_CONF_LOADTIME
  numticks = clock_time() - numticks + 1;
  ADD(" <i>(%u.%02u sec)</i>",numticks/CLOCK_SECOND,(100*(numticks%CLOCK_SECOND))/CLOCK_SECOND));
#endif

  SEND_STRING(&s->sout, buf);
  SEND_STRING(&s->sout, BOTTOM);

  PSOCK_END(&s->sout);
}
/*---------------------------------------------------------------------------*/
httpd_simple_script_t
httpd_simple_get_script(const char *name)
{

  return generate_routes;
}

/* WEBSERVER */

/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTA("Server IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINTA(" ");
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTA("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
void
request_prefix(void)
{
  /* mess up uip_buf with a dirty request... */
  uip_buf[0] = '?';
  uip_buf[1] = 'P';
  uip_len = 2;
  slip_send();
  uip_clear_buf();
}
/*---------------------------------------------------------------------------*/
void
set_prefix_64(uip_ipaddr_t *prefix_64)
{
  rpl_dag_t *dag;
  uip_ipaddr_t ipaddr;
  memcpy(&prefix, prefix_64, 16);
  memcpy(&ipaddr, prefix_64, 16);
  prefix_set = 1;
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  
  //print prefix
  PRINTF(" init  Print Prefix:");
  uip_debug_ipaddr_print(&prefix);
  PRINTF("\n");
  
  PRINTF(" init  Print server:");
  uip_debug_ipaddr_print(&ipaddr);
  uip_ipaddr_copy(&server_ip,&ipaddr);
  
  PRINTF("\n");
  uip_debug_ipaddr_print(&server_ip);
  PRINTF("\n");
  
  dag = rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
  if(dag != NULL) {
    rpl_set_prefix(dag, &prefix, 64);
    PRINTF("created a new RPL dag\n");
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(border_router_process, ev, data)
{
  static struct etimer et;
 
  PROCESS_BEGIN();

/* While waiting for the prefix to be sent through the SLIP connection, the future
 * border router can join an existing DAG as a parent or child, or acquire a default
 * router that will later take precedence over the SLIP fallback interface.
 * Prevent that by turning the radio off until we are initialized as a DAG root.
 */
  prefix_set = 0;
  NETSTACK_MAC.off(0);

  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);

  PRINTF("RPL-Border router started\n");
#if 0
   /* The border router runs with a 100% duty cycle in order to ensure high
     packet reception rates.
     Note if the MAC RDC is not turned off now, aggressive power management of the
     cpu will interfere with establishing the SLIP connection */
  NETSTACK_MAC.off(1);
#endif

#if 0
  /* Request prefix until it has been received */
  while(!prefix_set) {
    etimer_set(&et, CLOCK_SECOND);
    request_prefix();
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  
#endif
  
  set_prefix_64(&router_prefix); 
  
  /* Now turn the radio on, but disable radio duty cycling.
   * Since we are the DAG root, reception delays would constrain mesh throughbut.
   */
  NETSTACK_MAC.off(1);

#if DEBUG || 1
  print_local_addresses();
#endif

  process_start(&unicast_receiver_process, NULL);  
  
  while(1) {
  
    // PROCESS_YIELD();
       PROCESS_PAUSE();

 

    if(new_data==true)
    {
        new_data=false;
       // Reg_Server_Account();
      #if 1    
       //if(Wifi.Mode==_WIFI_REG_PROJECT_CODE && Wifi.RxBuffer[12]==0x04)
       if(Wifi.RxBuffer[12]==0x04)  
       {
            Reg_Project_Check();
        
        }else if(Wifi.RxBuffer[REG_INDEX+1]==0x02)
        {
            if(Reg_Server_Account()==REGISTER_SERVER_SUCCEFULL)
            Wifi.Mode=_WIFI_SERVER_MODE;
           //send registration data to server
    
        }
       #endif 
    }
    
    if(Wifi.Mode==_WIFI_SERVER_MODE)
    {
        if(Wifi.IsAPConnected!=true)
        // connect to ap  
        Reg_Server_Registration();// parsing json data
    }
    
    if(Wifi.Mode==_WIFI_REPORT_MODE)
    {
        if(Wifi.IsAPConnected==false)
        Report_Connect_AP();// connect to ap
    }
    
    
#if 1    
    if (ev == sensors_event && data == &button_sensor) {
      // RESET EEPROM HERE
            
      PRINTF("Initiating global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
#endif
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
	  
    int i;
    uint8_t buf[17];
    memcpy(buf,sender_addr,16);
    
    //printf("Data received from ");
	  uip_debug_ipaddr_print(sender_addr);
	
 
#if 1      
      for(i=0;i<16;i++)
      {
        sender_ip[i]=((uint8_t *)sender_addr)[i];
        printf("ith: %d ip:%x \r\n",i,sender_ip[i]);
      } 
      
#endif  

     
      //ESP8266 send data
      ESP8266_SendData(sender_ip,data);
 
}
/*---------------------------------------------------------------------------*/

static uip_ipaddr_t *
set_global_address(void)
{
  static uip_ipaddr_t ipaddr;
  
  int i;
  uint8_t state;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  PRINTF("global IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
    }
}
     PRINTF("\r\n reg server address \r\n");
     uip_debug_ipaddr_print(&server_ip); 
     PRINTF("\n");
  return &ipaddr;
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(unicast_receiver_process, ev, data)
{

    
  
  int i;
  uip_ipaddr_t *ipaddr;
  
 
  
  PROCESS_BEGIN();

  servreg_hack_init();

  ipaddr = set_global_address();

  create_rpl_dag(ipaddr);

  servreg_hack_register(SERVICE_ID, ipaddr);

  simple_udp_register(&unicast_connection, UDP_PORT,
                      NULL, UDP_PORT, receiver);

  
  /* init json parser */ 
 // jsmn_init(&parser);  
  
  printf("\n unicast_receiver_process thread started\n");   
 // ESP8266_Write("AT+\n"); //disconnect with ap
 
  while(1) {
  
  
    // PROCESS_YIELD();
       PROCESS_PAUSE();
    
    //parsing json file
    
   
    if(new_data==true)
    {
        new_data=false;
       // Reg_Server_Account();
      #if 1    
       //if(Wifi.Mode==_WIFI_REG_PROJECT_CODE && Wifi.RxBuffer[12]==0x04)
       if(Wifi.RxBuffer[12]==0x04)  
       {
            Reg_Project_Check();
        
        }else if(Wifi.RxBuffer[REG_INDEX+1]==0x02)
        {
            if(Reg_Server_Account()==REGISTER_SERVER_SUCCEFULL)
            Wifi.Mode=_WIFI_SERVER_MODE;
           //send registration data to server
    
        }
       #endif 
    }
    
    if(Wifi.Mode==_WIFI_SERVER_MODE)
    {
        if(Wifi.IsAPConnected!=true)
        // connect to ap  
        Reg_Server_Registration();// parsing json data
    }
    
    if(Wifi.Mode==_WIFI_REPORT_MODE)
    {
        if(Wifi.IsAPConnected==false)
        Report_Connect_AP();// connect to ap
    }
    
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void
create_rpl_dag(uip_ipaddr_t *ipaddr)
{
  struct uip_ds6_addr *root_if;

  root_if = uip_ds6_addr_lookup(ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    uip_ipaddr_t prefix;
    
    rpl_set_root(RPL_DEFAULT_INSTANCE, ipaddr);
    dag = rpl_get_any_dag();
    uip_ip6addr(&prefix, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &prefix, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
}


/**
  * @}
  */

/*
 * An example of reading JSON from stdin and printing its content to stdout.
 * The output looks like YAML, but I'm not sure if it's really compatible.
 */
