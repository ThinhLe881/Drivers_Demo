/*
 * GDU.c
 *
 *  Created on: Mar 7, 2025
 *      Author: thinh
 */

/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include "GDU.h"
#include "TI_DRV8000/DRV8000.h"


/* **********************************************************************/
/* ***            Definition of local plain CONSTants                 ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***               Definition of local types                        ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***              Definition of local variables                     ***/
/* **********************************************************************/
static st_DRV8000_t gdu_drv8000;


/* **********************************************************************/
/* ***             Declaration of local functions                     ***/
/* **********************************************************************/
static uint8_t gpio_control(uint8_t port,
                            uint8_t pin,
                            uint8_t pin_level);

static uint16_t pwm_set_dutycycle(uint8_t instance,
                                  uint8_t channel,
                                  uint16_t duty_cycle);

static void pwm_set_period(uint8_t instance,
                           uint8_t channel,
                           uint16_t period);

static uint8_t spi_transceive_drv8000(const uint8_t* tx_buffer,
                                      uint8_t* rx_buffer,
                                      const uint16_t len);

static void delay(uint16_t delay_us);


/* **********************************************************************/
/* ***              Definition of global variables                    ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***            Definition of global functions                      ***/
/* **********************************************************************/
uint8_t gdu_drv8000_init(void)
{
    uint8_t ret = 0u;

    gdu_drv8000.interface.fptr_SpiTransceive = spi_transceive_drv8000;
    gdu_drv8000.interface.fptr_PwmSetPeriod = pwm_set_period;
    gdu_drv8000.interface.fptr_PwmSetDutycycle = pwm_set_dutycycle;
    gdu_drv8000.interface.fptr_Gpio = gpio_control;
    gdu_drv8000.interface.fptr_Delay = delay;
    gdu_drv8000.interface.pwm_max_period = GDU_DRV8000_MAX_PWM_PERIOD;
    gdu_drv8000.interface.pwm_gd_in1_instance = GDU_DRV8000_PWM_GD_IN1_INSTANCE;
    gdu_drv8000.interface.pwm_gd_in1_channel = GDU_DRV8000_PWM_GD_IN1_CHANNEL;
    /* gdu_drv8000.interface.pwm_gd_in2_instance = GDU_DRV8000_PWM_GD_IN2_INSTANCE; */
    /* gdu_drv8000.interface.pwm_gd_in2_channel = GDU_DRV8000_PWM_GD_IN2_CHANNEL;   */
    gdu_drv8000.interface.pwm1_instance = GDU_DRV8000_PWM1_INSTANCE;
    gdu_drv8000.interface.pwm1_channel = GDU_DRV8000_PWM1_CHANNEL;
    gdu_drv8000.interface.nsleep_port = GDU_DRV8000_nSLEEP_PORT;
    gdu_drv8000.interface.nsleep_pin = nSLEEP_PIN;
    gdu_drv8000.interface.drvoff_port = GDU_DRV8000_DRVOFF_PORT;
    gdu_drv8000.interface.drvoff_pin = DRVOFF_PIN;
    gdu_drv8000.interface.gd_in2_port = GDU_DRV8000_GD_IN2_PORT;
    gdu_drv8000.interface.gd_in2_pin = GD_IN2_PIN;

    /* Set DRV8000 interface */
    drv8000_interface_set(&gdu_drv8000.interface);
    /* Reset DRV8000 */
    drv8000_reset();

    /* Check device ID */
    gdu_drv8000.dev_id = drv8000_read_devid();

    if (DRV8000_DEFVAL_DEVICE_ID != gdu_drv8000.dev_id)
    {
        ret = 1u;
    }
    if (0u == ret)
    {
        /* Clear fault flags */
        ret = drv8000_clear_fault();
    }
    if (0u == ret)
    {
        /* Read registers */
        ret = drv8000_read_registers();
    }
    if (0u == ret)
    {
        /* Enable PWM2 */
        ret = drv8000_ipropi_mode(IPROPI_INPUT_PWM,
                                  IPROPI_NO_OUT);
    }

    return ret;
}


/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
static uint8_t gpio_control(uint8_t port,
                            uint8_t pin,
                            uint8_t pin_level)
{
    Hal_Gpio_t * base;

    switch (port)
    {
        case PORT_AL:
            base = PORT_BASE_AL;
            break;
        case PORT_AH:
            base = PORT_BASE_AH;
            break;
        case PORT_BL:
            base = PORT_BASE_BL;
            break;
        case PORT_BH:
            base = PORT_BASE_BH;
            break;
        case PORT_CL:
            base = PORT_BASE_CL;
            break;
        case PORT_CH:
            base = PORT_BASE_CH;
            break;
        default:
            return 1u;
    }

    if (0u == pin_level)
    {
        Hal_Gpio_Clear(base, pin);
    } else if (1u == pin_level)
    {
        Hal_Gpio_Set(base, pin);
    } else 
    {
        return 1u;
    }

    return 0u;
}

static void pwm_set_period(uint8_t instance,
                           uint8_t channel,
                           uint16_t period)
{
    Hal_Pwm_SetPeriod(instance, channel, period);
}

static uint16_t pwm_set_dutycycle(uint8_t instance,
                                  uint8_t channel,
                                  uint16_t duty_cycle)
{
    return (uint16_t)(Hal_Pwm_SetDutyCycle(instance,
                                           channel,
                                           duty_cycle));
}


static uint8_t spi_transceive_drv8000(const uint8_t* tx_buffer,
                                      uint8_t* rx_buffer,
                                      const uint16_t len)
{
    return (uint8_t)(Hal_Spi_Transceive(&GDU_DRV8000_SPI_INSTANCE,
                                        tx_buffer,
                                        rx_buffer,
                                        len,
                                        GDU_SPI_TIMEOUT));
}

static void delay(uint16_t delay_us)
{
    uint16_t delay_tick = delay_us * GDU_DELAY_TICK_1us;
    volatile uint16_t tick_i;

	for (tick_i = 0u; tick_i < delay_tick; tick_i++)
    {
        __asm__("nop");
    }
}
