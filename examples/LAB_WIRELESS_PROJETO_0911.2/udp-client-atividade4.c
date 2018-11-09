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
#include "dev/leds.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "ioc.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "lpm.h"
#include "dev/serial-line.h"
#include "dev/cc26xx-uart.h"
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

#define MUSIC_PLAY_START_PAUSE    0x7C
#define MUSIC_PLAY_STOP           0x79
//#define MUSIC_PLAY_PAUSE          0x7A

#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

// defines
#define STATE_PLAYING   0
#define STATE_PAUSED    1
#define STATE_STOPPED   2

#define TONE_PLAY_EVENT      44
#define TONE_PLAYED_ACK      45
#define TONE_STOP_EVENT      46
#define TONE_STOPPED_ACK 47


// funcs prototypes
int16_t pwminit(int32_t freq);
void tone(unsigned int frequency, unsigned int duration);
uint8_t pwm_request_max_pm(void);
void sleep_enter(void);
void sleep_leave(void);
void play_command_procedure(void);
void stop_command_procedure(void);

//globals
static struct etimer et_hello,t1;
static struct timer t;
int16_t marcha_imperial[][2] =  {{440, 500},{440, 500},{440, 500},{349, 350},{523, 150},{440,500},{349,350},    \
                                 {523,150},{440,1000},{659,500},{659,500},{659,500},{698,350},{523,150},        \
                                 {415,500},{349,350},{523,150},{440,1000},{880,500},{440,350},{440,150},        \
                                 {880,500},{803,250},{784, 250},{740, 125},{698,125},{740, 250},{0,250},        \
                                 {455, 250},{622, 500},{587, 250},{554, 250},{523,125},{466, 125},{523,250},    \
                                 {0,250},{349,250},{415,500},{349,375},{523,125},{440,500},{349,375},{261,125}, \
                                 {440,1000},{0,250}};
int8_t state_player=STATE_STOPPED;
int16_t current_play_position=0;

static char buf[MAX_PAYLOAD_LEN];
static struct uip_udp_conn *client_conn;



/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(hello_world_process2, "Hello world process");
PROCESS(music_process,"Music Player");
AUTOSTART_PROCESSES(&resolv_process,&udp_client_process,&hello_world_process2,&music_process);
LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
    char *dados;
    char payload = 0x60;
    int i=0;

    if(uip_newdata())
    {
        dados = uip_appdata;
        printf("Recebidos %d bytes, dado[0] = %d\n",uip_datalen(),dados[0]);
        switch (dados[0])
        {
            case MUSIC_PLAY_START_PAUSE:
                printf("MUSIC_PLAY_START_PAUSE received\n");
                process_post(&hello_world_process2, MUSIC_PLAY_START_PAUSE,1);
                break;
            case MUSIC_PLAY_STOP:
                printf("MUSIC_PLAY_STOP received\n");
                process_post(&hello_world_process2, MUSIC_PLAY_STOP,1);
                break;
            /*case LED_SET_STATE:
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
                break;*/

        }
        /*dados[uip_datalen()] = '\0';
        printf("Response from the server: '%s'\n", dados);*/
    }
}
/*---------------------------------------------------------------------------*/
static void
timeout_handler(void)
{
    /*char payload = LED_TOGGLE_REQUEST;

    buf[0] = payload;*/
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
#define UDP_CONNECTION_ADDR       raspbassan.local
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
  //uip_ip6addr(&ipaddr, 0x2804, 0x014c, 0x8786, 0x8166, 0x823b, 0x93e1, 0xe7af, 0xd9a6); // 2018
  uip_ip6addr(&ipaddr, 0x2804, 0x014c, 0x8786, 0x8166, 0x1993, 0x21ee, 0x5e37, 0x51e7); // 2018

  //inet6 addr: 2804:14c:8786:8166:ac22:e8c:8d5c:14ad/64 Scope:Global
  //inet6 addr: 2804:14c:8786:8166:823b:93e1:e7af:d9a6/64 Scope:Global

#endif
  /* new connection with remote host */
  client_conn = udp_new(&ipaddr, UIP_HTONS(CONN_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(CONN_PORT));

  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

  etimer_set(&et, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(etimer_expired(&et)) {
      timeout_handler();
      etimer_restart(&et);
    } else if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
//process1
PROCESS_THREAD(hello_world_process2, ev, data)
{
    static int16_t current_duty = 0;
    static int16_t loadvalue;
    static int i=0;
    static int16_t marcha_size;

    PROCESS_BEGIN();

    marcha_size = sizeof(marcha_imperial)/(2*sizeof(int16_t));
    /*for (i=0; i<marcha_size; i++)
    {
        tone(marcha_imperial[i][0],marcha_imperial[i][1]);
    }*/

    etimer_set(&et_hello, 1 * CLOCK_SECOND);


    cc26xx_uart_set_input(serial_line_input_byte);

    while (1)
    {
        PROCESS_WAIT_EVENT();

        if (ev == TONE_PLAYED_ACK && state_player == STATE_PLAYING)
        {
            current_play_position++;
            // acabou a musica
            if (current_play_position >= marcha_size)
            {
                state_player = STATE_STOPPED;
                current_play_position = 0;
                process_post(&music_process,TONE_STOP_EVENT,NULL);
            }
            else
            {
                process_post(&music_process,TONE_PLAY_EVENT,current_play_position);
            }
        }
        if(ev == serial_line_event_message)
        {
            printf("received line: %s\n", (char *)data);
        }
        if (ev == MUSIC_PLAY_START_PAUSE)
        {
            play_command_procedure();
        }
        if (ev == MUSIC_PLAY_STOP)
        {
            stop_command_procedure();
        }
        if (ev == sensors_event)
        {
            printf("Button PRessed\n");
            // pause or play
            if (data == &button_left_sensor)
            {
                play_command_procedure();
                /*if (state_player == STATE_STOPPED || state_player == STATE_PAUSED)
                {
                    state_player = STATE_PLAYING;
                    printf("#MAIN TASK# - PLAY PRESSED\n");
                    process_post(&music_process,TONE_PLAY_EVENT,current_play_position);
                }
                else if (state_player == STATE_PLAYING)
                {
                    state_player = STATE_PAUSED;
                    printf("#MAIN TASK# - PAUSE PRESSED\n");
                    process_post(&music_process, TONE_STOP_EVENT,NULL);
                }*/
            }
            // stop
            else if (data == &button_right_sensor)
            {
                stop_command_procedure();
                /*state_player = STATE_STOPPED;
                printf("#MAIN TASK# - STOP PRESSED\n");
                current_play_position = 0;
                process_post(&music_process, TONE_STOP_EVENT,NULL);*/
            }
        }
    }

    PROCESS_END();
}

PROCESS_THREAD(music_process, ev, data)
{
    PROCESS_BEGIN();
    while (1)
    {
        PROCESS_WAIT_EVENT();
        if (ev == TONE_PLAY_EVENT)
        {
            printf("#TONE TASK# - play tone[%d]\n",(int)data);
            tone(marcha_imperial[(int)data][0],marcha_imperial[(int)data][1]);
            //process_post(&hello_world_process2,TONE_PLAYED_ACK,NULL);
        }
        if (ev == TONE_STOP_EVENT)
        {
            tone(0,70);
            printf("#TONE TASK# - stop play\n");
        }
        if (ev == PROCESS_EVENT_TIMER)
        {
            process_post(&hello_world_process2,TONE_PLAYED_ACK,NULL);
        }
    }
    PROCESS_END();
}

/* funcoes */
int16_t pwminit(int32_t freq)
{
    uint32_t load=0;

    ti_lib_ioc_pin_type_gpio_output(IOID_21);
    leds_off(LEDS_RED);

    /* enable GPT0 clocks under active mode */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);

    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    lpm_register_module(&pwmdrive_module);

    /* Drive the I/O ID With gpt0 timer a */
    ti_lib_ioc_port_configure_set(IOID_21, IOC_PORT_MCU_PORT_EVENT0,IOC_STD_OUTPUT);

    /* GPT0 TImer A: PWM, Interrupt Enable */
    ti_lib_timer_configure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    /* stop timers */
    ti_lib_timer_disable(GPT0_BASE, TIMER_A);
    ti_lib_timer_disable(GPT0_BASE, TIMER_B);

    if (freq > 0)
    {
        load = ( GET_MCU_CLOCK / freq );

        ti_lib_timer_load_set(GPT0_BASE, TIMER_A, load);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, load-1);

        /* start timer a */
        ti_lib_timer_enable(GPT0_BASE, TIMER_A);
    }
    return load;
}

// frequency in Hz, duration in miliseconds
void tone(unsigned int frequency, unsigned int duration)
{
    static int16_t loadvalue,duty;
    static int16_t duration_mul7ms;
    if (frequency != 0)
    {
        loadvalue=pwminit(frequency);
        duty=loadvalue/2;
    }
    else
    {
        loadvalue=pwminit(2000);
        duty = loadvalue-1;
    }

    duration_mul7ms = duration / 7;
    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, duty);

    etimer_set(&t1,duration_mul7ms);
    /*timer_set(&t, duration_mul7ms);
    while (!timer_expired(&t))
    {
    }*/
}

uint8_t pwm_request_max_pm(void)
{
    return LPM_MODE_DEEP_SLEEP;
}

void sleep_enter(void)
{
    leds_on(LEDS_RED);
}

void sleep_leave(void)
{
    leds_off(LEDS_RED);
}

void play_command_procedure(void)
{
    if (state_player == STATE_STOPPED || state_player == STATE_PAUSED)
    {
        state_player = STATE_PLAYING;
        printf("#MAIN TASK# - PLAY PRESSED\n");
        process_post(&music_process, TONE_PLAY_EVENT, current_play_position);
    }
    else if (state_player == STATE_PLAYING)
    {
        state_player = STATE_PAUSED;
        printf("#MAIN TASK# - PAUSE PRESSED\n");
        process_post(&music_process, TONE_STOP_EVENT, NULL);
    }
}

void stop_command_procedure(void)
{
    state_player = STATE_STOPPED;
    printf("#MAIN TASK# - STOP PRESSED\n");
    current_play_position = 0;
    process_post(&music_process, TONE_STOP_EVENT, NULL);
}
