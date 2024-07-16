

#include "fsl_flexio_uart.h"

extern flexio_uart_config_t config;
status_t flexuart_init(uint32_t ibps);


void FLEXIO_UART_UserCallback(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle, status_t status, void *userData);
status_t uart_send_string(FLEXIO_UART_Type *base, const uint8_t *txData, size_t txSize);


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_FLEXIO_BASE  FLEXIO1

//#define FLEXIO_UART_TX_PIN 21U    //AD9
//#define FLEXIO_UART_RX_PIN 22U    //AD10

//#define FLEXIO_UART_TX_PIN 20U    //AD0
//#define FLEXIO_UART_RX_PIN 20U    //AD0

#define FLEXIO_UART_TX_PIN 11U    //SD5
#define FLEXIO_UART_RX_PIN 20U    //AD0



#define FLEXIO_CLOCK_SELECT (3U)
#define FLEXIO_CLOCK_PRE_DIVIDER (4U)
#define FLEXIO_CLOCK_DIVIDER (7U)

#define FLEXIO_CLOCK_FREQUENCY \
    (CLOCK_GetFreq(kCLOCK_Usb1PllClk) / (FLEXIO_CLOCK_PRE_DIVIDER + 1U) / (FLEXIO_CLOCK_DIVIDER + 1U))
    
#define ECHO_BUFFER_LENGTH 8




extern flexio_uart_handle_t g_uartHandle;
extern FLEXIO_UART_Type uartDev;

extern uint8_t g_tipString[7];

extern uint8_t g_txBuffer[ECHO_BUFFER_LENGTH];
extern uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH];

 
extern volatile bool rxbuffull ;
extern volatile bool txbufempty;

extern volatile bool txOnGoing ;
extern volatile bool rxOnGoing;



 