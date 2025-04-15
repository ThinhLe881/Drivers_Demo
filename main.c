/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include "HAL/HAL.h"
#include "GDU/GDU.h"
#include "GDU/TI_DRV8000/DRV8000.h"
#include "Timer/Timer.h"

/* **********************************************************************/
/* ***              Definition of local variables                     ***/
/* **********************************************************************/
volatile int exit_code = 0;

/* **********************************************************************/
/* ***             Declaration of local functions                     ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
void pit0_rti_callback(uint8_t channel);


int main(void)
{
    /* HAL initialization */
    HAL_Init_Clock();
    HAL_Init_Port();
    HAL_Init_SPI();
    HAL_Init_PWM();
    HAL_Init_Pit();

    /* initialize DRV8000 */
    exit_code = (int)gdu_drv8000_init();

    if (0 == exit_code)
    {
        /* Initialize scheduler timer */
        timer_system_init();
        /* Start RTI for scheduler timer */
        Hal_Pit_StartChannel(PIT_INSTANCE_0, RTI, RTI_UPDATE_TICKS);
    } else
    {
        return exit_code;
    }

    for(;;)
    {
        if (timer_system_reset(TIMER_70ms))
        {
            exit_code = (int)gdu_drv8000_watchdog_trig();

            if(0 != exit_code)
            {
                drv8000_sleep_wake(SLEEP_MODE);
            }
        }
        if (timer_system_reset(TIMER_100ms) && 0 == exit_code)
        {
            exit_code = (int)drv8000_regs_periodic_read();
        }
    }

    return exit_code;
}


/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
void pit0_rti_callback(uint8_t channel)
{
    (void)channel;
    timer_system_step();
}

/** @} */
