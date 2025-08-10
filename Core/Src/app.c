/*
 * App.c
 *
 *  Created on: Jan 25, 2025
 *      Author: rffernandes
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "debug_api.h"
#include "cmsis_os.h"

uint32_t firsttime=0;
uint16_t count=0;
#define	FIRSTIME_WAIT 2000

void app_init(void){

	//led_time_ms = 1000;

    DBG_LOG(1,(" App Init"));
}

uint32_t hw_time_get(void){

	uint32_t millis =  0;
	uint32_t freq = osKernelGetTickFreq ();

	millis =  osKernelGetTickCount() * 1000 / freq;

	return (millis);
}

void hw_delay_ms(uint32_t delayms){

	 // HAL_Delay(delayms);
	osDelay(delayms);

}

void app_maintask(void)
{
	uint32_t actualtime;

	firsttime = hw_time_get();

    while(1)
    {
    	actualtime = hw_time_get();

    	if ((actualtime-firsttime) > FIRSTIME_WAIT) {
       	    changevalue();
       		firsttime = actualtime;
    	}

    	hw_delay_ms(1);
	}

}



