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

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/resolv.h"
#include "lib/sensors.h"
#include "dev/leds.h"
//#include "udp-client.c"

#include <string.h>
#include <stdbool.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define SEND_INTERVAL		5 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN		40
#define CONN_PORT     8802
#define MDNS 0

#define LED_TOGGLE_REQUEST  0x79
#define LED_SET_STATE       0x7A
#define LED_GET_STATE       0x7B
#define LED_STATE           0x7C

#define OP_REQUEST          0x6E
#define OP_RESULT           0x6F

#define OP_MULTIPLY         0x22
#define OP_DIVIDE           0x23
#define OP_SUM              0x24
#define OP_SUBTRACT         0x25


struct mathopreply {
  uint8_t opResult;
  int32_t intPart;
  uint32_t fracPart;
  float fpResult;
  uint8_t crc;
}  __attribute__((packed));

struct mathopreqstr {
  uint8_t opRequest;
  int32_t op1;
  uint8_t operation;
  int32_t op2;
  float fc;
}  __attribute__((packed));

static char buf[MAX_PAYLOAD_LEN];

static struct uip_udp_conn *client_conn;

#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&resolv_process,&udp_client_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
    char *dados;
    char payload[2]={ LED_STATE, 0x00};
    struct mathopreply* resposta;
    static int i;

    payload[1] = leds_get();

    if(uip_newdata())
    {
        dados = uip_appdata;
        printf("Recebidos %d bytes\n",uip_datalen());
        switch (dados[0])
        {
            case OP_RESULT:
                resposta = (struct mathopreply*)dados;
                printf("Resultado = %d",resposta->intPart);
                break;
            case LED_GET_STATE:
                uip_udp_packet_send(client_conn, payload, 2);
                break;
            case LED_SET_STATE:
                printf("comando para os leds: %d\n",dados[1]);
                leds_off(LEDS_ALL);
                if (dados[1] & 0x01 )
                {
                    leds_on(LEDS_GREEN);
                }
                if (dados[1] & 0x02 )
                {
                    leds_on(LEDS_RED);
                }
                payload[1] = leds_get();
                uip_udp_packet_send(client_conn, payload, 2);
                break;

        }
        dados[uip_datalen()] = '\0';
        printf("Response from the server: '%s'\n", dados);
    }
}
/*---------------------------------------------------------------------------*/
static void
timeout_handler(void)
{
    char payload = LED_TOGGLE_REQUEST;

    buf[0] = payload;
    if(uip_ds6_get_global(ADDR_PREFERRED) == NULL)
    {
      PRINTF("Aguardando auto-configuracao de IP\n");
      return;
    }
    /*uip_udp_packet_send(client_conn, buf, 1);
    PRINTF("Cliente para [");
    PRINT6ADDR(&client_conn->ripaddr);
    PRINTF("]: %u\n",UIP_HTONS(client_conn->rport));*/
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
#if UIP_CONF_ROUTER
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}
#endif /* UIP_CONF_ROUTER */
/*---------------------------------------------------------------------------*/

#if MDNS

static resolv_status_t
set_connection_address(uip_ipaddr_t *ipaddr)
{
#ifndef UDP_CONNECTION_ADDR
#if RESOLV_CONF_SUPPORTS_MDNS
#define UDP_CONNECTION_ADDR       contiki-udp-server.local
#elif UIP_CONF_ROUTER
#define UDP_CONNECTION_ADDR       fd00:0:0:0:0212:7404:0004:0404
#else
#define UDP_CONNECTION_ADDR       fe80:0:0:0:6466:6666:6666:6666
#endif
#endif /* !UDP_CONNECTION_ADDR */

#define _QUOTEME(x) #x
#define QUOTEME(x) _QUOTEME(x)

    resolv_status_t status = RESOLV_STATUS_ERROR;

    if(uiplib_ipaddrconv(QUOTEME(UDP_CONNECTION_ADDR), ipaddr) == 0) {
        uip_ipaddr_t *resolved_addr = NULL;
        status = resolv_lookup(QUOTEME(UDP_CONNECTION_ADDR),&resolved_addr);
        if(status == RESOLV_STATUS_UNCACHED || status == RESOLV_STATUS_EXPIRED) {
            PRINTF("Attempting to look up %s\n",QUOTEME(UDP_CONNECTION_ADDR));
            resolv_query(QUOTEME(UDP_CONNECTION_ADDR));
            status = RESOLV_STATUS_RESOLVING;
        } else if(status == RESOLV_STATUS_CACHED && resolved_addr != NULL) {
            PRINTF("Lookup of \"%s\" succeded!\n",QUOTEME(UDP_CONNECTION_ADDR));
        } else if(status == RESOLV_STATUS_RESOLVING) {
            PRINTF("Still looking up \"%s\"...\n",QUOTEME(UDP_CONNECTION_ADDR));
        } else {
            PRINTF("Lookup of \"%s\" failed. status = %d\n",QUOTEME(UDP_CONNECTION_ADDR),status);
        }
        if(resolved_addr)
            uip_ipaddr_copy(ipaddr, resolved_addr);
    } else {
        status = RESOLV_STATUS_CACHED;
    }

    return status;
}
#endif

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer et;
  uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();
  PRINTF("UDP client process started\n");

#if UIP_CONF_ROUTER
  //set_global_address();
#endif

  etimer_set(&et, 2*CLOCK_SECOND);
  while(uip_ds6_get_global(ADDR_PREFERRED) == NULL)
  {
      PROCESS_WAIT_EVENT();
      if(etimer_expired(&et))
      {
          PRINTF("Aguardando auto-configuracao de IP\n");
          etimer_set(&et, 2*CLOCK_SECOND);
      }
  }


  print_local_addresses();

#if MDNS
  static resolv_status_t status = RESOLV_STATUS_UNCACHED;
  while(status != RESOLV_STATUS_CACHED) {
      status = set_connection_address(&ipaddr);

      if(status == RESOLV_STATUS_RESOLVING) {
          //PROCESS_WAIT_EVENT_UNTIL(ev == resolv_event_found);
          PROCESS_WAIT_EVENT();
      } else if(status != RESOLV_STATUS_CACHED) {
          PRINTF("Can't get connection address.\n");
          PROCESS_YIELD();
      }
  }
#else
  //c_onfigures the destination IPv6 address
  //uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x212, 0x4b00, 0x07b9, 0x5e8d);
  uip_ip6addr(&ipaddr, 0xfd00, 0x0000, 0x0000, 0x0000, 0x0212, 0x4b00, 0x0aff, 0x6b01); // 2018
#endif
  /* new connection with remote host */
  client_conn = udp_new(&ipaddr, UIP_HTONS(CONN_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(CONN_PORT));

  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

  etimer_set(&et, SEND_INTERVAL);
  while(1)
  {
      static unsigned char buf[14];
      static struct mathopreqstr operacao;
      static value1 = 1;
      static value2 = 2;
      static int i=0;
      static char* ptr;

    PROCESS_YIELD();
    if(etimer_expired(&et))
    {
      timeout_handler();
      etimer_restart(&et);
    } else if(ev == tcpip_event)
    {
      tcpip_handler();
    }
    else if (ev == sensors_event)
    {
        printf("apertou o botao\n");
        value1++;
        value2++;

        operacao.opRequest = OP_REQUEST;
        operacao.operation = OP_SUM;
        operacao.op1 = value1;
        operacao.op2 = value2;
        operacao.fc = 1;

        ptr = (char*)&operacao;

        /*for (i=0; i < 14; i++)
        {
            buf[i]=ptr[i];
        }*/
        //memcpy()

        if(uip_ds6_get_global(ADDR_PREFERRED) != NULL)
        {
            uip_udp_packet_send(client_conn,&operacao,sizeof(struct mathopreqstr));
        }

        printf("pacote enviado, soma de %d e %d\n",value1,value2);




    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
