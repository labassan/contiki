/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
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
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include "dev/leds.h"
#include "button-sensor.h"

#define LED_PING_EVENT      44
#define LED_PONG_EVENT      45

#define HELLO_WORLD_PROC_ID     1
#define BLINK_PROC_ID           2
#define PROC3_ID                3
#define BUTTON_PROC_ID          4

static struct etimer et_hello,et_blink,et_10s;
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process2, "Hello world process");
PROCESS(blink_process,"Blink process");
PROCESS(proc3_process,"Process 10sec print serial");
PROCESS(pong_process,"Pong Process");
PROCESS(read_button_process, "read button process");

AUTOSTART_PROCESSES(&hello_world_process2,&blink_process,&proc3_process,&pong_process,&read_button_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process2, ev, data)
{
  PROCESS_BEGIN();
  etimer_set(&et_hello, 4*CLOCK_SECOND);

  while(1)
  {
      PROCESS_WAIT_EVENT();
      if (ev == PROCESS_EVENT_TIMER)
      {
          process_post(&pong_process, LED_PING_EVENT,HELLO_WORLD_PROC_ID);
          printf("[SND]PING: Enviado de hello_world_process\n");
          etimer_reset(&et_hello);
      }
      else if (ev == LED_PONG_EVENT)
                printf("[RCV]PONG: Recebido em hello_world\n");
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();
  etimer_set(&et_blink, 2*CLOCK_SECOND);

  while(1)
  {
      PROCESS_WAIT_EVENT();
      if (ev == PROCESS_EVENT_TIMER)
      {
          process_post(&pong_process, LED_PING_EVENT,BLINK_PROC_ID);
          printf("[SND]PING: Enviado de blink_process\n");
          leds_toggle(LEDS_GREEN);
          etimer_reset(&et_blink);
      }
      else if (ev == LED_PONG_EVENT)
          printf("[RCV]PONG: Recebido em blink\n");
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(proc3_process, ev, data)
{
  PROCESS_BEGIN();
  etimer_set(&et_10s, 10*CLOCK_SECOND);

  while(1)
  {
      PROCESS_WAIT_EVENT();
      if (ev == PROCESS_EVENT_TIMER)
      {
          process_post(&pong_process, LED_PING_EVENT,PROC3_ID);
          printf("[SND]PING: Enviado de proc3_process\n");
          printf("10sec periodic serial message\n");
          etimer_reset(&et_10s);
      }
      else if (ev == LED_PONG_EVENT)
          printf("[RCV]PONG: Recebido em proc3\n");
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pong_process, ev, data)
{
  PROCESS_BEGIN();

  while(1)
  {
      PROCESS_WAIT_EVENT();
      if (ev == LED_PING_EVENT)
      {
          switch ((int)data)
          {
              case HELLO_WORLD_PROC_ID:
                  printf("[RCV]Ping received from HELLO_WORLD\n");
                  process_post(&hello_world_process2,LED_PONG_EVENT,NULL);
                  break;
              case BLINK_PROC_ID:
                  printf("[RCV]Ping received from BLINK\n");
                  process_post(&blink_process,LED_PONG_EVENT,NULL);
                  break;
              case PROC3_ID:
                  printf("[RCV]Ping received from PROC3_ID\n");
                  process_post(&proc3_process,LED_PONG_EVENT,NULL);
                  break;
              case BUTTON_PROC_ID:
                  printf("[RCV]Ping received from BUTTON PROCESS\n");
                  process_post(&read_button_process,LED_PONG_EVENT,NULL);
                  break;
          }
      }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(read_button_process, ev, data)
{
    PROCESS_BEGIN();

    while(1)
    {
        PROCESS_YIELD();

        if(ev == sensors_event)
        {
            if(data == &button_left_sensor)
            {
                printf("Left Button!\n");
                leds_toggle(LEDS_RED);
            }
            else if(data == &button_right_sensor)
            {
                leds_toggle(LEDS_GREEN);
                printf("Right Button!\n");
            }
            process_post(&pong_process, LED_PING_EVENT,BUTTON_PROC_ID);
            printf("[SND]PING: Enviado de button_process\n");
        }
        else if (ev == LED_PONG_EVENT)
            printf("[RCV]PONG: Recebido em button process\n");
    }
    PROCESS_END();
    return 0;
}
