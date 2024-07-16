
#include "fsl_debug_console.h"

#include "flexuart.h"


flexio_uart_config_t config;
flexio_uart_handle_t g_uartHandle;
FLEXIO_UART_Type uartDev;

uint8_t g_tipString[] ="start\r\n";

uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = {0};

 
volatile bool rxbuffull            = 0;
volatile bool txbufempty           =1;

volatile bool txOnGoing                = false;
volatile bool rxOnGoing                = false;


status_t flexuart_init(uint32_t ibps)
{

   status_t result = kStatus_Success;
//   flexio_uart_config_t config;
   
   FLEXIO_UART_GetDefaultConfig(&config);
   config.baudRate_Bps = ibps;  //FLEXIO_UART_BAUDRATE;
   config.enableUart   = true;

   uartDev.flexioBase      = BOARD_FLEXIO_BASE;
   uartDev.TxPinIndex      = FLEXIO_UART_TX_PIN;
   uartDev.RxPinIndex      = FLEXIO_UART_RX_PIN;
   uartDev.shifterIndex[0] = 0U;
   uartDev.shifterIndex[1] = 1U;
   uartDev.timerIndex[0]   = 0U;
   uartDev.timerIndex[1]   = 1U;

   result = FLEXIO_UART_Init(&uartDev, &config, FLEXIO_CLOCK_FREQUENCY);
   if (result != kStatus_Success)
   {
     return -1;
   }    

   FLEXIO_UART_TransferCreateHandle(&uartDev, &g_uartHandle, FLEXIO_UART_UserCallback, NULL);  
   
   return result;
}

void FLEXIO_UART_UserCallback(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;
   
//PRINTF("irq--03\r\n");  

    if (kStatus_FLEXIO_UART_TxIdle == status)
    {
        //txBufferFull = false;
        txbufempty =1;
       
        txOnGoing    = false;
    }

    if (kStatus_FLEXIO_UART_RxIdle == status)
    {
        rxbuffull= 1;
        rxOnGoing     = false;
       
PRINTF((const char *)g_rxBuffer);
PRINTF(" ok\r\n");     

//flg_rx=1;
       
    }
}


status_t uart_send_string(FLEXIO_UART_Type *base, const uint8_t *txData, size_t txSize)
{
    assert(txData != NULL);
    assert(txSize != 0U);   

   ////test
   //uint32_t  istatu=0;
   //istatu=FLEXIO_GetShifterStatusFlags(base->flexioBase);  
   //PRINTF("s1 %d\r\n",istatu);   
 
    while (0U != txSize--)
    { 
        while (0U == (FLEXIO_GetShifterStatusFlags(base->flexioBase) & (1UL << base->shifterIndex[0]))) ;
 
        base->flexioBase->SHIFTBUF[base->shifterIndex[0]] = *txData++;
       
    }
    while (0U == (FLEXIO_GetShifterStatusFlags(base->flexioBase) & (1UL << base->shifterIndex[0]))) ;  //wait for tx finish
    
    return kStatus_Success;
}

