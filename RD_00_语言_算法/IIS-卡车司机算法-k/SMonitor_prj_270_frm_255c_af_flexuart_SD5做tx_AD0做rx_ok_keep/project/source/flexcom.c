 
#include "fsl_flexio.h"
#include "flexcom.h"
 
 
void flexio_clock_setting(void)
{
   /* Clock setting for Flexio */
   CLOCK_SetMux(kCLOCK_Flexio1Mux, FLEXIO_CLOCK_SELECT);
   CLOCK_SetDiv(kCLOCK_Flexio1PreDiv, FLEXIO_CLOCK_PRE_DIVIDER);
   CLOCK_SetDiv(kCLOCK_Flexio1Div, FLEXIO_CLOCK_DIVIDER);      
}
 