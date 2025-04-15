/*
 * Timer.h
 *
 *  Created on: Apr 15, 2025
 *      Author: thinh
 */

#ifndef TIMER_TIMER_H_
#define TIMER_TIMER_H_

/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include <stdint.h>
#include <stdbool.h>


/* **********************************************************************/
/* ***            Definition of global plain CONSTants                ***/
/* **********************************************************************/
#define N_TIMERS                        (3u)


/* **********************************************************************/
/* ***               Definition of global types                       ***/
/* **********************************************************************/
typedef enum
{
	TIMER_10ms,
	TIMER_70ms,
	TIMER_100ms,
} TIMER_ID_t;

typedef struct
{
	volatile uint16_t   counter;
	volatile uint16_t   duration;
	volatile bool      	expired;
} Timer_t;


/* **********************************************************************/
/* ***            Declaration of global functions                     ***/
/* **********************************************************************/
/**
 * @brief Initialize timer system.
 */
void timer_system_init(void);

/**
 * @brief Update timer system.
 */
void timer_system_step(void);

/**
 * @brief Reset the timer with ID `timer_id` if it has expired.
 *
 * @param timer_id                      ID of the timer to check.
 *
 * @return `true` if the timer's expired state was reset, otherwise `false`.
 */
bool timer_system_reset(TIMER_ID_t timer_id);


#endif /* TIMER_TIMER_H_ */
