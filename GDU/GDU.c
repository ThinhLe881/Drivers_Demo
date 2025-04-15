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
static uint8_t gpio_control(const void* port,
                            uint8_t pin,
                            uint8_t pin_level);

static void delay(uint16_t delay_us);


/* **********************************************************************/
/* ***              Definition of global variables                    ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***            Definition of global functions                      ***/
/* **********************************************************************/
/* DRV8000 user-defined handler functions */
uint8_t drv8000_spi_transceive(const void* spi_instance, 
                                const uint8_t* tx_buffer, 
                                uint8_t* rx_buffer, 
                                const uint16_t len)
{
    return (uint8_t)(Lpspi_Ip_SyncTransmit(spi_instance,
                                            tx_buffer,
                                            rx_buffer,
                                            len,
                                            GDU_SPI_TIMEOUT));
}

uint8_t drv8000_pwm_set_dc(uint8_t instance,
                            uint8_t channel,
                            uint16_t duty_cycle)
{
    return (uint8_t)(Emios_Pwm_Ip_SetDutyCycle(instance,
                                                channel,
                                                duty_cycle));
}

uint8_t drv8000_gpio(const void* port, 
                        uint8_t pin, 
                        uint8_t pin_level)
{
    return gpio_control(port,
                        pin,
                        pin_level);
}

void drv8000_delay(uint16_t delay_us)
{
    delay(delay_us);
}

uint8_t gdu_drv8000_init(void)
{
    en_DRV8000_STST_t ret;

    gdu_drv8000.interface.spi_instance = GDU_DRV8000_SPI_INSTANCE;
    gdu_drv8000.interface.pwm_gd_in1_instance = GDU_DRV8000_PWM_GD_IN1_INSTANCE;
    gdu_drv8000.interface.pwm_gd_in1_channel = GDU_DRV8000_PWM_GD_IN1_CHANNEL;
    /* gdu_drv8000.interface.pwm_gd_in2_instance = GDU_DRV8000_PWM_GD_IN2_INSTANCE; */
    /* gdu_drv8000.interface.pwm_gd_in2_channel = GDU_DRV8000_PWM_GD_IN2_CHANNEL;   */
    gdu_drv8000.interface.pwm1_instance = GDU_DRV8000_PWM1_INSTANCE;
    gdu_drv8000.interface.pwm1_channel = GDU_DRV8000_PWM1_CHANNEL;
    gdu_drv8000.interface.nsleep_port = GDU_DRV8000_nSLEEP_PORT;
    gdu_drv8000.interface.nsleep_pin = GDU_DRV8000_nSLEEP_PIN;
    gdu_drv8000.interface.drvoff_port = GDU_DRV8000_DRVOFF_PORT;
    gdu_drv8000.interface.drvoff_pin = GDU_DRV8000_DRVOFF_PIN;
    gdu_drv8000.interface.gd_in2_port = GDU_DRV8000_GD_IN2_PORT;
    gdu_drv8000.interface.gd_in2_pin = GDU_DRV8000_GD_IN2_PIN;

    /* Init DRV8000 */
    delay(500);
    ret = drv8000_init(&gdu_drv8000.interface);

    if (STST_SUCCESS == ret)
    {
        /* Enable PWM2 */
        ret = drv8000_ipropi_mode(IPROPI_INPUT_PWM,
                                    IPROPI_NO_OUT);
    }
    if (STST_SUCCESS == ret)
    {
        /* Configure HS drivers */
        ret = drv8000_hs_driver_cnfg(HS_CNFG_DISABLED,
                                        HS_CNFG_DISABLED,
                                        HS_CNFG_DISABLED,
                                        HS_CNFG_SPI_CONTROL,
                                        HS_CNFG_SPI_CONTROL,
                                        HS_CNFG_PWM_GEN,
                                        HEATER_CNFG_PWM_PIN_CONTROL);
    }
    if (STST_SUCCESS == ret)
    {
        /* Configure electrochromic driver */
        ret = drv8000_ec_driver_cnfg(ECFB_MAX_1_2V, 
                                        EC_OLEN_DIS,     
                                        EC_ECFB_LS_NO_PWM_DISCHARGE, 
                                        EC_FLT_HiZ_EC, 
                                        EC_ECFB_UV_OV_NO_ACTION, 
                                        EC_ECFB_UV_OV_NO_ACTION, 
                                        EC_ECFB_UV_OV_DG_20us, 
                                        EC_ECFB_UV_OV_DG_20us, 
                                        EC_ECFB_UV_TH_100mV, 
                                        EC_OL_CURR_SRC_DIS);     
    }
    if (STST_SUCCESS == ret)
    {
        /* Configure HHB drivers */
        ret = drv8000_hhb_out1234_set_mode(HHB_SPI_EN, 
                                            HHB_DISABLED, 
                                            HHB_DISABLED, 
                                            HHB_DISABLED);
    }
    if (STST_SUCCESS == ret)
    {
        /* Configure freewheeling for HHB drivers */
        ret = drv8000_hhb_set_fw(HHB_PASSIVE_FW,
                                    HHB_PASSIVE_FW,
                                    HHB_PASSIVE_FW,
                                    HHB_PASSIVE_FW,
                                    HHB_ACTIVE_FW,
                                    HHB_PASSIVE_FW);
    }
    if (STST_SUCCESS == ret)
    {
        /* Configure IC settings and watchdog */
        ret = drv8000_set_ic_cnfg1(OTSD_GLOBAL_SHUTDOWN,
                                    CHARGE_PUMP_ENABLE,
                                    PVDD_OV_MODE_LATCHED_FLT,
                                    PVDD_OV_DG_1us,
                                    PVDD_OV_LVL_21_5V,
                                    VCP_UV_LVL_4_75V,
                                    CP_MODE_AUT_SWITCH,
                                    VCP_UV_MODE_LATCHED_FLT,
                                    PVDD_UV_MODE_LATCHED_FLT,
                                    WD_FLT_FAULT_DRIVER_OFF,
                                    WD_WIN_10_to_100ms,
                                    EN_SSC_ENABLE,
                                    WD_ENABLE);
    }

    return (uint8_t)ret;
}


/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
static uint8_t gpio_control(const void* port,
                            uint8_t pin,
                            uint8_t pin_level)
{
    if (0u == pin_level)
    {
        Hal_Gpio_Clear((Hal_Gpio_t*)port, pin);
    } else if (1u == pin_level)
    {
        Hal_Gpio_Set((Hal_Gpio_t*)port, pin);
    } else 
    {
        return 1u;
    }

    return 0u;
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
