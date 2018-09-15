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
#include "ioc.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"


static struct etimer et_hello;

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process2, "Hello world process");

AUTOSTART_PROCESSES(&hello_world_process2);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process2, ev, data)
{
  PROCESS_BEGIN();

  static struct sensors_sensor *sensor;

  static int valor,divisor,i,result_div;
  static float valor_V;
  static float teste=1.12;
  sensor = sensors_find(ADC_SENSOR);

  etimer_set(&et_hello, 1*CLOCK_SECOND);

  while(1)
  {
      PROCESS_WAIT_EVENT();
      if (ev == PROCESS_EVENT_TIMER)
      {
          etimer_set(&et_hello, 1*CLOCK_SECOND);
          //SENSORS_ACTIVATE(*sensor);
          sensor->configure(SENSORS_ACTIVE, 1);
          sensor->configure(ADC_SENSOR_SET_CHANNEL, ADC_COMPB_IN_AUXIO0);
          valor = sensor->value(ADC_SENSOR_VALUE);

          divisor = 1000000;
          printf("Valor do ad: ");
          for (i=0; i < 2; i++)
          {
              result_div = valor/divisor;
              printf("%d",result_div);
              if (i==0)
                  printf(".");
              divisor=divisor/10;
          }
          printf(" V \n");
          SENSORS_DEACTIVATE(*sensor);
      }
  }

  PROCESS_END();
}
