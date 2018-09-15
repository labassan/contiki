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
#include "ti-lib.h"
#include "lpm.h"
#include "dev/serial-line.h"
#include "dev/cc26xx-uart.h"



int16_t pwminit(int32_t freq);

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

static struct etimer et_hello;

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process2, "Hello world process");

AUTOSTART_PROCESSES(&hello_world_process2);
LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process2, ev, data)
{
    static int16_t current_duty = 0;
    static int16_t loadvalue;

    PROCESS_BEGIN();

    etimer_set(&et_hello, 1 * CLOCK_SECOND);
    loadvalue = pwminit(5000);
    current_duty = loadvalue - 1;

    cc26xx_uart_set_input(serial_line_input_byte);

    while (1)
    {
        PROCESS_WAIT_EVENT();

        if(ev == serial_line_event_message)
        {
            printf("received line: %s\n", (char *)data);
        }
        if (ev == sensors_event)
        {
            if (data == &button_left_sensor)
            {
                current_duty =  current_duty - (loadvalue/10);
            }
            else if (data == &button_right_sensor)
            {
                current_duty =  current_duty + (loadvalue/10);
            }
            if (current_duty >= loadvalue)
            {
                current_duty = loadvalue - 1;
            }
            if (current_duty < 0)
            {
                current_duty=0;
            }
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, current_duty );
            printf("Current duty = %d\n",current_duty);
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

