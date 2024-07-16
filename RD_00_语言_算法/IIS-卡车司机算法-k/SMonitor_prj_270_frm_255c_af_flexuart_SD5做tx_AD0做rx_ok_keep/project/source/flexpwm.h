
#ifndef _FLEXPWM_H_
#define _FLEXPWM_H_
 
#include "fsl_flexio.h"
#include "flexcom.h"

extern uint32_t duty;
 
////#define DEMO_TIME_DELAY_FOR_DUTY_CYCLE_UPDATE (2000000U)
#define DEMO_FLEXIO_BASEADDR                  FLEXIO1

#define DEMO_FLEXIO_OUTPUTPIN                 (8U)  

#define DEMO_FLEXIO_TIMER_CH                  (3U)  /* Flexio timer0 used */


#define PWM_FLEXIO_CLOCK_FREQUENCY \
 (CLOCK_GetFreq(kCLOCK_Usb1PllClk) / (FLEXIO_CLOCK_PRE_DIVIDER + 1U) / (FLEXIO_CLOCK_DIVIDER + 1U))


#define DEMO_FLEXIO_FREQUENCY (48000U)
#define FLEXIO_MAX_FREQUENCY (PWM_FLEXIO_CLOCK_FREQUENCY / 2U)
#define FLEXIO_MIN_FREQUENCY (PWM_FLEXIO_CLOCK_FREQUENCY / 512U)


static void flexio_pwm_init(uint32_t freq_Hz, uint32_t duty);
static void flexio_pwm_start(void);
static void flexio_pwm_stop(void);
void flexpwm_init(void);

#endif
 
 
 
 
 
 
 
 
 