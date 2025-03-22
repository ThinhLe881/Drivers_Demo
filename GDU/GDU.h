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
#include "../HAL/HAL.h"
#include "TI_DRV8000/DRV8000.h"


/* **********************************************************************/
/* ***               Definition of global types                       ***/
/* **********************************************************************/
typedef struct
{
    st_DRV8000_Interface_t  interface;
    uint8_t                 dev_id;
} st_DRV8000_t;

typedef enum
{
    PORT_AL,
    PORT_AH,
    PORT_BL,
    PORT_BH,
    PORT_CL,
    PORT_CH,
} en_GPIO_PORT_t;


/* **********************************************************************/
/* ***            Definition of global plain CONSTants                ***/
/* **********************************************************************/
#define GDU_SPI_TIMEOUT                     ((uint32_t)1000000u)
#define GDU_DRV8000_MAX_PWM_PERIOD          6400u /* Max 25 kHz ~ 6400 ticks using 160 MHz clock source */
#define GDU_DELAY_TICK_1us                  160u /* 160 MHz clock */

#define GDU_DRV8000_SPI_INSTANCE            SPI_0
#define GDU_DRV8000_PWM1_INSTANCE           PWM_INSTANCE_1
#define GDU_DRV8000_PWM1_CHANNEL            PWM_CHANNEL_1
#define GDU_DRV8000_PWM_GD_IN1_INSTANCE     PWM_INSTANCE_2
#define GDU_DRV8000_PWM_GD_IN1_CHANNEL      PWM_CHANNEL_2
#define GDU_DRV8000_PWM_GD_IN2_INSTANCE     PWM_INSTANCE_3
#define GDU_DRV8000_PWM_GD_IN2_CHANNEL      PWM_CHANNEL_3
#define GDU_DRV8000_nSLEEP_PORT             PORT_CH
#define GDU_DRV8000_DRVOFF_PORT             PORT_AL
#define GDU_DRV8000_GD_IN2_PORT             PORT_BH


/* **********************************************************************/
/* ***            Declaration of global functions                     ***/
/* **********************************************************************/
uint8_t gdu_drv8000_init(void);


#endif /* GDU_GDU_H_ */
