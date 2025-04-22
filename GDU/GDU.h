/*
 * GDU.h
 *
 *  Created on: Mar 7, 2025
 *      Author: thinh
 */

#ifndef GDU_GDU_H_
#define GDU_GDU_H_

/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include "TI_DRV8000/DRV8000.h"


/* **********************************************************************/
/* ***               Definition of global types                       ***/
/* **********************************************************************/
typedef struct
{
    st_DRV8000_Interface_t  interface;
} st_DRV8000_t;


/* **********************************************************************/
/* ***            Definition of global plain CONSTants                ***/
/* **********************************************************************/
#define GDU_SPI_TIMEOUT                     ((uint32_t)1000000u)
#define GDU_DRV8000_MAX_PWM_PERIOD          6400u /* Max 25 kHz ~ 6400 ticks using 160 MHz clock source */
#define GDU_DELAY_TICK_1us                  160u /* 160 MHz clock */

#define GDU_DRV8000_SPI_INSTANCE            DRV8000_SPI_INST
#define GDU_DRV8000_PWM1_INSTANCE           PWM1_PWM_INST
#define GDU_DRV8000_PWM1_CHANNEL            PWM1_PWM_CH
#define GDU_DRV8000_PWM2_INSTANCE           PWM2_PWM_INST
#define GDU_DRV8000_PWM2_CHANNEL            PWM2_PWM_CH
#define GDU_DRV8000_PWM_GD_IN1_INSTANCE     GD_IN1_PWM_INST
#define GDU_DRV8000_PWM_GD_IN1_CHANNEL      GD_IN1_PWM_CH
#define GDU_DRV8000_PWM_GD_IN2_INSTANCE     GD_IN2_PWM_INST
#define GDU_DRV8000_PWM_GD_IN2_CHANNEL      GD_IN2_PWM_CH
#define GDU_DRV8000_nSLEEP_PORT             nSLEEP_PORT
#define GDU_DRV8000_DRVOFF_PORT             DRVOFF_PORT
#define GDU_DRV8000_GD_IN2_PORT             GD_IN2_PORT
#define GDU_DRV8000_nSLEEP_PIN              nSLEEP_PIN
#define GDU_DRV8000_DRVOFF_PIN              DRVOFF_PIN
#define GDU_DRV8000_GD_IN2_PIN              GD_IN2_PIN


/* **********************************************************************/
/* ***            Declaration of global functions                     ***/
/* **********************************************************************/
uint8_t gdu_drv8000_init(void);

uint8_t gdu_drv8000_watchdog_trig(void);


#endif /* GDU_GDU_H_ */
