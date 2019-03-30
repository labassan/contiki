/*
  Basic MQTT-SN client library
  Copyright (C) 2013 Nicholas Humfrey

  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Modifications:
  Copyright (C) 2013 Adam Renner
*/


#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-nameserver.h"
#include "mqtt-sn.h"
#include "rpl.h"
#include "net/ip/resolv.h"
#include "net/rime/rime.h"
#include "simple-udp.h"
#include "button-sensor.h"
#include "ioc.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "lpm.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "dev/leds.h"

#define UDP_PORT 1883

#define REQUEST_RETRIES 4
#define DEFAULT_SEND_INTERVAL		(10 * CLOCK_SECOND)
#define REPLY_TIMEOUT (3 * CLOCK_SECOND)

/* music player START */
#define MUSIC_PLAY_START_PAUSE    0x7C
#define MUSIC_PLAY_STOP           0x79

#define STATE_PLAYING   0
#define STATE_PAUSED    1
#define STATE_STOPPED   2

#define TONE_PLAY_EVENT      44
#define TONE_PLAYED_ACK      45
#define TONE_STOP_EVENT      46
#define TONE_STOPPED_ACK     47

// funcs prototypes
int16_t pwminit(int32_t freq);
void tone(unsigned int frequency, unsigned int duration);
uint8_t pwm_request_max_pm(void);
void sleep_enter(void);
void sleep_leave(void);
void play_command_procedure(void);
void stop_command_procedure(void);

int16_t marcha_imperial[][2] =  {{440, 500},{440, 500},{440, 500},{349, 350},{523, 150},{440,500},{349,350},    \
                                 {523,150},{440,1000},{659,500},{659,500},{659,500},{698,350},{523,150},        \
                                 {415,500},{349,350},{523,150},{440,1000},{880,500},{440,350},{440,150},        \
                                 {880,500},{803,250},{784, 250},{740, 125},{698,125},{740, 250},{0,250},        \
                                 {455, 250},{622, 500},{587, 250},{554, 250},{523,125},{466, 125},{523,250},    \
                                 {0,250},{349,250},{415,500},{349,375},{523,125},{440,500},{349,375},{261,125}, \
                                 {440,1000},{0,250}};
int8_t state_player=STATE_STOPPED;
int16_t current_play_position=0;
static struct etimer t1;

/* music player END */

static struct mqtt_sn_connection mqtt_sn_c;
static char mqtt_client_id[17];
static char ctrl_topic[22] = "0000000000000000/ctrl\0";//of form "0011223344556677/ctrl" it is null terminated, and is 21 charactes
static char pub_topic[21] = "0000000000000000/msg\0";
static uint16_t ctrl_topic_id;
static uint16_t publisher_topic_id;
static publish_packet_t incoming_packet;
static uint16_t ctrl_topic_msg_id;
static uint16_t reg_topic_msg_id;
static uint16_t mqtt_keep_alive=10;
static int8_t qos = 1;
static uint8_t retain = FALSE;
static char device_id[17];
static clock_time_t send_interval;
static mqtt_sn_subscribe_request subreq;
static mqtt_sn_register_request regreq;
//uint8_t debug = FALSE;

static enum mqttsn_connection_status connection_state = MQTTSN_DISCONNECTED;

/*A few events for managing device state*/
static process_event_t mqttsn_connack_event;

PROCESS(example_mqttsn_process, "Configure Connection and Topic Registration");
PROCESS(publish_process, "register topic and publish data");
PROCESS(ctrl_subscription_process, "subscribe to a device control channel");
PROCESS_NAME(cetic_6lbr_client_process);
/* MUSIC PROCESSES */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process2, "Hello world process");
PROCESS(music_process,"Music Player");
LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);
/*---------------------------------------------------------------------------*/
AUTOSTART_PROCESSES(&example_mqttsn_process);

/*---------------------------------------------------------------------------*/
static void
puback_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen)
{
  printf("Puback received\n");
}
/*---------------------------------------------------------------------------*/
static void
connack_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen)
{
  uint8_t connack_return_code;
  connack_return_code = *(data + 3);
  printf("Connack received\n");
  if (connack_return_code == ACCEPTED) {
    process_post(&example_mqttsn_process, mqttsn_connack_event, NULL);
  } else {
    printf("Connack error: %s\n", mqtt_sn_return_code_string(connack_return_code));
  }
}
/*---------------------------------------------------------------------------*/
static void
regack_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen)
{
  regack_packet_t incoming_regack;
  memcpy(&incoming_regack, data, datalen);
  printf("Regack received\n");
  if (incoming_regack.message_id == reg_topic_msg_id) {
    if (incoming_regack.return_code == ACCEPTED) {
      publisher_topic_id = uip_htons(incoming_regack.topic_id);
    } else {
      printf("Regack error: %s\n", mqtt_sn_return_code_string(incoming_regack.return_code));
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
suback_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen)
{
  suback_packet_t incoming_suback;
  memcpy(&incoming_suback, data, datalen);
  printf("Suback received\n");
  if (incoming_suback.message_id == ctrl_topic_msg_id) {
    if (incoming_suback.return_code == ACCEPTED) {
      ctrl_topic_id = uip_htons(incoming_suback.topic_id);
    } else {
      printf("Suback error: %s\n", mqtt_sn_return_code_string(incoming_suback.return_code));
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
publish_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen)
{
  int id_msg;
  //publish_packet_t* pkt = (publish_packet_t*)data;
  memcpy(&incoming_packet, data, datalen);
  incoming_packet.data[datalen-7] = 0x00;
  printf("Published message received: %s\n", incoming_packet.data);
  id_msg = atoi(incoming_packet.data);
  if (id_msg == 0)
  {
      leds_off(LEDS_ALL);
  }
  else
      leds_on(LEDS_ALL);
  printf("O id messagem recebido seria %d",id_msg);
  //see if this message corresponds to ctrl channel subscription request
  if (uip_htons(incoming_packet.topic_id) == ctrl_topic_id) {
    //the new message interval will be read from the first byte of the received packet
    //send_interval = (uint8_t)incoming_packet.data[0] * CLOCK_CONF_SECOND;
      send_interval = 10 * CLOCK_CONF_SECOND;
  } else {
    printf("unknown publication received\n");
  }

}
/*---------------------------------------------------------------------------*/
static void
pingreq_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen)
{
  printf("PingReq received\n");
}
/*---------------------------------------------------------------------------*/
/*Add callbacks here if we make them*/
static const struct mqtt_sn_callbacks mqtt_sn_call = {
  publish_receiver,
  pingreq_receiver,
  NULL,
  connack_receiver,
  regack_receiver,
  puback_receiver,
  suback_receiver,
  NULL,
  NULL
  };

/*---------------------------------------------------------------------------*/
/*this process will publish data at regular intervals*/
PROCESS_THREAD(publish_process, ev, data)
{
  static uint8_t registration_tries;
  static struct etimer send_timer;
  static uint8_t buf_len;
  static uint32_t message_number;
  static char buf[20];
  static mqtt_sn_register_request *rreq = &regreq;

  PROCESS_BEGIN();
  send_interval = DEFAULT_SEND_INTERVAL;
  memcpy(pub_topic,device_id,16);
  printf("registering topic\n");
  registration_tries =0;
  while (registration_tries < REQUEST_RETRIES)
  {

    reg_topic_msg_id = mqtt_sn_register_try(rreq,&mqtt_sn_c,pub_topic,REPLY_TIMEOUT);
    //PROCESS_WAIT_EVENT_UNTIL(mqtt_sn_request_returned(rreq));
    etimer_set(&send_timer, 5*CLOCK_SECOND);
    PROCESS_WAIT_EVENT();
    if (mqtt_sn_request_success(rreq)) {
      registration_tries = 4;
      printf("registration acked\n");
    }
    else {
      registration_tries++;
      if (rreq->state == MQTTSN_REQUEST_FAILED) {
          printf("Regack error: %s\n", mqtt_sn_return_code_string(rreq->return_code));
      }
    }
  }
  if (mqtt_sn_request_success(rreq)){
    //start topic publishing to topic at regular intervals
    etimer_set(&send_timer, send_interval);
    while(1)
    {
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
      sprintf(buf, "Message %" PRIu32, message_number); //removendo o warning do GCC para o uint32_t
      printf("publishing at topic: %s -> msg: %s\n", pub_topic, buf);
      message_number++;
      buf_len = strlen(buf);
      mqtt_sn_send_publish(&mqtt_sn_c, publisher_topic_id,MQTT_SN_TOPIC_TYPE_NORMAL,buf, buf_len,qos,retain);
      /*if (ctimer_expired(&(mqtt_sn_c.receive_timer)))
      {
          process_post(&example_mqttsn_process, (process_event_t)(NULL), (process_event_t)(41));
      }*/
      etimer_set(&send_timer, send_interval);
    }
  } else {
    printf("unable to register topic\n");
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*this process will create a subscription and monitor for incoming traffic*/
PROCESS_THREAD(ctrl_subscription_process, ev, data)
{
  static uint8_t subscription_tries;
  static mqtt_sn_subscribe_request *sreq = &subreq;
  static struct etimer periodic_timer;
  PROCESS_BEGIN();
  subscription_tries = 0;
  memcpy(ctrl_topic,device_id,16);
  printf("requesting subscription\n");
  while(subscription_tries < REQUEST_RETRIES)
  {
      printf("subscribing... topic: %s\n", ctrl_topic);
      ctrl_topic_msg_id = mqtt_sn_subscribe_try(sreq,&mqtt_sn_c,ctrl_topic,0,REPLY_TIMEOUT);

      //PROCESS_WAIT_EVENT_UNTIL(mqtt_sn_request_returned(sreq));
      etimer_set(&periodic_timer, 5*CLOCK_SECOND);
      PROCESS_WAIT_EVENT();
      if (mqtt_sn_request_success(sreq)) {
          subscription_tries = 4;
          printf("subscription acked\n");
      }
      else {
          subscription_tries++;
          if (sreq->state == MQTTSN_REQUEST_FAILED) {
              printf("Suback error: %s\n", mqtt_sn_return_code_string(sreq->return_code));
          }
      }
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*this main process will create connection and register topics*/
/*---------------------------------------------------------------------------*/


static struct ctimer connection_timer;
static process_event_t connection_timeout_event;

static void connection_timer_callback(void *mqc)
{
  process_post(&example_mqttsn_process, connection_timeout_event, NULL);
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
static resolv_status_t
set_connection_address(uip_ipaddr_t *ipaddr)
{
#ifndef UDP_CONNECTION_ADDR
#if RESOLV_CONF_SUPPORTS_MDNS
#define UDP_CONNECTION_ADDR       pksr.eletrica.eng.br
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



PROCESS_THREAD(example_mqttsn_process, ev, data)
{
  static struct etimer periodic_timer;
  static struct etimer et;
  static uip_ipaddr_t broker_addr,google_dns;
  static uint8_t connection_retries = 0;
  static resolv_status_t status;
  char contiki_hostname[16];

  PROCESS_BEGIN();

#if RESOLV_CONF_SUPPORTS_MDNS
#ifdef CONTIKI_CONF_CUSTOM_HOSTNAME
  sprintf(contiki_hostname,"node%02X%02X",linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);
  resolv_set_hostname(contiki_hostname);
  PRINTF("Setting hostname to %s\n",contiki_hostname);
#endif
#endif

  mqttsn_connack_event = process_alloc_event();

  mqtt_sn_set_debug(1);
  //uip_ip6addr(&broker_addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
  //uip_ip6addr(&broker_addr, 0x2001, 0x0db8, 1, 0xffff, 0, 0, 0xc0a8, 0xd480);//192.168.212.128 with tayga
  //uip_ip6addr(&broker_addr, 0xaaaa, 0, 2, 0xeeee, 0, 0, 0xc0a8, 0xd480);//192.168.212.128 with tayga
  //uip_ip6addr(&broker_addr, 0xaaaa, 0, 2, 0xeeee, 0, 0, 0xac10, 0xdc01);//172.16.220.1 with tayga
  uip_ip6addr(&google_dns, 0x2001, 0x4860, 0x4860, 0x0, 0x0, 0x0, 0x0, 0x8888);//172.16.220.1 with tayga
#if CC26XX_WEB_DEMO_6LBR_CLIENT
  process_start(&cetic_6lbr_client_process, NULL);
#endif
  etimer_set(&periodic_timer, 2*CLOCK_SECOND);
  while(uip_ds6_get_global(ADDR_PREFERRED) == NULL)
  {
    PROCESS_WAIT_EVENT();
    if(etimer_expired(&periodic_timer))
    {
        PRINTF("Aguardando auto-configuracao de IP\n");
        etimer_set(&periodic_timer, 2*CLOCK_SECOND);
    }
  }

  print_local_addresses();




  rpl_dag_t *dag = rpl_get_any_dag();
  if(dag) {
    //uip_ipaddr_copy(sixlbr_addr, globaladdr);
    uip_nameserver_update(&google_dns, UIP_NAMESERVER_INFINITE_LIFETIME);
  }

  status = RESOLV_STATUS_UNCACHED;
  while(status != RESOLV_STATUS_CACHED) {
    status = set_connection_address(&broker_addr);

    if(status == RESOLV_STATUS_RESOLVING) {
      //PROCESS_WAIT_EVENT_UNTIL(ev == resolv_event_found);
      PROCESS_WAIT_EVENT();
    } else if(status != RESOLV_STATUS_CACHED) {
      PRINTF("Can't get connection address.\n");
      etimer_set(&periodic_timer, 2*CLOCK_SECOND);
      PROCESS_WAIT_EVENT();
    }
  }

  //uip_ip6addr(&broker_addr, 0x2804,0x7f4,0x3b80,0xcdf7,0x241b,0x1ab2,0xa46a,0x9912);//172.16.220.128 with tayga

  mqtt_sn_create_socket(&mqtt_sn_c,UDP_PORT, &broker_addr, UDP_PORT);
  (&mqtt_sn_c)->mc = &mqtt_sn_call;

  sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",linkaddr_node_addr.u8[0],
          linkaddr_node_addr.u8[1],linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
          linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],linkaddr_node_addr.u8[6],
          linkaddr_node_addr.u8[7]);

  sprintf(mqtt_client_id,"sens%02X%02X%02X%02X",linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);


  /*Request a connection and wait for connack*/
  printf("requesting connection \n ");
  connection_timeout_event = process_alloc_event();
  //testegoto:
  connection_retries = 0;
  ctimer_set( &connection_timer, REPLY_TIMEOUT, connection_timer_callback, NULL);
  mqtt_sn_send_connect(&mqtt_sn_c,mqtt_client_id,mqtt_keep_alive);
  connection_state = MQTTSN_WAITING_CONNACK;
  while (connection_retries < 15)
  {
    PROCESS_WAIT_EVENT();
    if (ev == mqttsn_connack_event) {
      //if success
      printf("connection acked\n");
      ctimer_stop(&connection_timer);
      connection_state = MQTTSN_CONNECTED;
      connection_retries = 15;//using break here may mess up switch statement of proces
    }
    if (ev == connection_timeout_event) {
      connection_state = MQTTSN_CONNECTION_FAILED;
      connection_retries++;
      printf("connection timeout\n");
      ctimer_restart(&connection_timer);
      if (connection_retries < 15) {
        mqtt_sn_send_connect(&mqtt_sn_c,mqtt_client_id,mqtt_keep_alive);
        connection_state = MQTTSN_WAITING_CONNACK;
      }
    }
  }
  ctimer_stop(&connection_timer);
  if (connection_state == MQTTSN_CONNECTED){
    process_start(&ctrl_subscription_process, 0);
    etimer_set(&periodic_timer, 3*CLOCK_SECOND);
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    while(!etimer_expired(&periodic_timer))
        PROCESS_WAIT_EVENT();
    process_start(&publish_process, 0);
    etimer_set(&et, 2*CLOCK_SECOND);
    while(1)
    {
      PROCESS_WAIT_EVENT();
      if(etimer_expired(&et)) {
        //leds_toggle(LEDS_ALL);
        etimer_restart(&et);
      }
    }
  } else {
    printf("unable to connect\n");
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

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

    //etimer_set(&et_hello, 1 * CLOCK_SECOND);


    //cc26xx_uart_set_input(serial_line_input_byte);

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
        /*if(ev == serial_line_event_message)
        {
            printf("received line: %s\n", (char *)data);
        }*/
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
