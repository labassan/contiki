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

// includes
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

// defines
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

LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

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
        if (ev == sensors_event)
        {
            printf("Button PRessed\n");
            // pause or play
            if (data == &button_left_sensor)
            {
                if (state_player == STATE_STOPPED || state_player == STATE_PAUSED)
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
                }
            }
            // stop
            else if (data == &button_right_sensor)
            {
                state_player = STATE_STOPPED;
                printf("#MAIN TASK# - STOP PRESSED\n");
                current_play_position = 0;
                process_post(&music_process, TONE_STOP_EVENT,NULL);
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
