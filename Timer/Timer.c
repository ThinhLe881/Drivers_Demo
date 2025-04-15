/*
 * Timer.c
 *
 *  Created on: Apr 15, 2025
 *      Author: thinh
 */

/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include "Timer.h"


/* **********************************************************************/
/* ***              Definition of local variables                     ***/
/* **********************************************************************/
static uint16_t TIMER_DURATIONS[N_TIMERS] = {
        10,
        70,
        100,
};

static Timer_t timers[N_TIMERS];


/* **********************************************************************/
/* ***            Definition of global functions                      ***/
/* **********************************************************************/
void timer_system_init(void)
{
	for (uint8_t i_timer = 0u; i_timer < N_TIMERS; i_timer++)
	{
		timers[i_timer].counter = 0;
		timers[i_timer].duration = TIMER_DURATIONS[i_timer];
		timers[i_timer].expired = false;
	}
}

void timer_system_step(void)
{
	for (uint8_t i_timer = 0u; i_timer < N_TIMERS; i_timer++)
	{
		timers[i_timer].counter++;

		if (!(timers[i_timer].expired) &&
			(timers[i_timer].counter >= timers[i_timer].duration))
		{
			timers[i_timer].expired = true;
			timers[i_timer].counter = 0u;
		}
	}
}

bool timer_system_reset(TIMER_ID_t timer_id)
{
	bool ret = false;

	if (timers[timer_id].expired)
	{
		timers[timer_id].expired = false;
		ret = true;
	}

	return ret;
}

