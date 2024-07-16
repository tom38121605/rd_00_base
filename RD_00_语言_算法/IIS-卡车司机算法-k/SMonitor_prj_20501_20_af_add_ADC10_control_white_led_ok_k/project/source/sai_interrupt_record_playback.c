/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_sai.h"
#include "fsl_codec_common.h"

#include "fsl_wm8960.h"
#include "fsl_codec_adapter.h"

#include "music.h"  //--add
#include "fsl_lpi2c.h"  //--add
#include "fsl_adc.h"   //--add

#include "lpm.h"         //--add
#include "fsl_gpt.h"     //--add
#include "specific.h"    //--add
#include "fsl_lpuart.h"   //--add
#include "peripherals.h"  //--add


#define WORKBOARD
//#define EVKBOARD

#define BTMUSIC
//#define ADCMUSIC

#define SAI1_TX_ONE
//#define SAI1_TX_TWO


uint8_t  flg_standby=0;
uint8_t flg_mainfirst=0;
uint8_t ibtstate=0;

gpio_pin_config_t output_config1 = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};     //-add

      
      
//adc
#define DEMO_ADC_BASE        ADC1
//--#define DEMO_ADC_USER_CHANNEL  14U
#define DEMO_ADC_USER_CHANNEL  1U
#define DEMO_ADC_CHANNEL_GROUP 0U
const uint32_t g_Adc_12bitFullRange = 4096U;

adc_config_t adcConfigStruct;
adc_channel_config_t adcChannelConfigStruct;      


static volatile bool isFinished = false;  //--add
sai_transfer_t xfer;



#define DEMO_LPUART            LPUART1
#define DEMO_LPUART_CLK_FREQ   BOARD_DebugConsoleSrcFreq()
#define DEMO_LPUART_IRQn       LPUART1_IRQn
#define DEMO_LPUART_IRQHandler LPUART1_IRQHandler

/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16

uint8_t flg_uart_rx =0;

    
/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
#define UI_ENABLE_GPIO       GPIO2        //--add
#define UI_ENABLE_GPIO_PIN   0           //--add

#define STANDBY_CTRL_GPIO       GPIO1        //--add
#define STANDBY_CTRL_GPIO_PIN   2            //--add

#define EXAMPLE_SW_GPIO         BOARD_USER_BUTTON_GPIO
#define EXAMPLE_SW_GPIO_PIN     BOARD_USER_BUTTON_GPIO_PIN
#define EXAMPLE_SW_IRQ          BOARD_USER_BUTTON_IRQ
#define EXAMPLE_GPIO_IRQHandler BOARD_USER_BUTTON_IRQ_HANDLER
#define EXAMPLE_SW_NAME         BOARD_USER_BUTTON_NAME

#define EXAMPLE_I2C_MASTER_BASE (LPI2C1_BASE)

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define LPI2C_MASTER_CLOCK_FREQUENCY LPI2C_CLOCK_FREQUENCY
//--#define WAIT_TIME                    10U


#define EXAMPLE_I2C_MASTER ((LPI2C_Type *)EXAMPLE_I2C_MASTER_BASE)

#define SLAVE_ADDR_7BIT_TCA9555 0x20U  //TCA9555
#define SLAVE_ADDR_7BIT_TAS5805_4K7 0x2CU  //TAS5805-BTL
#define SLAVE_ADDR_7BIT_TAS5805_120K 0x2FU  //TAS5805-PBTL



#define LPI2C_BAUDRATE               100000U
#define LPI2C_DATA_LENGTH            33U
 
uint8_t g_master_rxBuff[LPI2C_DATA_LENGTH];

uint8_t iexP1[10]={0};
uint8_t iexP0[10]={0};
 
/* SAI instance and clock */
//--#define DEMO_CODEC_WM8960
#define DEMO_SAI              SAI1
#define DEMO_SAI_CHANNEL      (0)
#define DEMO_SAI_IRQ          SAI1_IRQn
#define DEMO_SAITxIRQHandler  SAI1_IRQHandler
#define DEMO_SAI_TX_SYNC_MODE kSAI_ModeAsync
#define DEMO_SAI_RX_SYNC_MODE kSAI_ModeSync
#define DEMO_SAI_MCLK_OUTPUT  true
   
#define DEMO_SAI_MASTER_SLAVE kSAI_Master


#define DEMO_AUDIO_DATA_CHANNEL (2U)                    
#define DEMO_AUDIO_BIT_WIDTH    kSAI_WordWidth32bits  
#define DEMO_AUDIO_SAMPLE_RATE  (kSAI_SampleRate48KHz)
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ


/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (0U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (63U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* I2C instance and clock */
#define DEMO_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define DEMO_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (DEMO_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define BOARD_MASTER_CLOCK_CONFIG()

#define BUFFER_SIZE   (1024U)
//#define BUFFER_SIZE   (2048U)
//#define BUFFER_SIZE   (512U)

#define BUFFER_NUMBER (4U)
//#define BUFFER_NUMBER (8U)
//#define BUFFER_NUMBER (2U)

#ifndef DEMO_CODEC_VOLUME
#define DEMO_CODEC_VOLUME 100U
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
wm8960_config_t wm8960Config = {
    .i2cConfig = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .route     = kWM8960_RoutePlaybackandRecord,
    .leftInputSource  = kWM8960_InputDifferentialMicInput3,
    .rightInputSource = kWM8960_InputDifferentialMicInput2,
    .playSource       = kWM8960_PlaySourceDAC,
    .slaveAddress     = WM8960_I2C_ADDR,
    .bus              = kWM8960_BusI2S,
    .format = {.mclk_HZ = 6144000U, .sampleRate = kWM8960_AudioSampleRate16KHz, .bitWidth = kWM8960_AudioBitWidth16bit},
    .master_slave = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8960, .codecDevConfig = &wm8960Config};

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 77/100)
 *                              = 786.48 MHz
 */
////const clock_audio_pll_config_t audioPllConfig = {
////    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
////    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
////    .numerator   = 77,  /* 30 bit numerator of fractional loop divider. */
////    .denominator = 100, /* 30 bit denominator of fractional loop divider */
////};
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t Buffer[BUFFER_NUMBER * BUFFER_SIZE], 4);
sai_handle_t txHandle = {0}, rxHandle = {0};
static uint32_t tx_index = 0U, rx_index = 0U;

//--volatile uint32_t emptyBlock = BUFFER_NUMBER;
volatile int8_t emptyBlock = BUFFER_NUMBER;

extern codec_config_t boardCodecConfig;
codec_handle_t codecHandle;

//#if (defined BTMUSIC) 
sai_handle_t rxHandle3 = {0};   //--sai3
//#endif


/*******************************************************************************
 * Code
 ******************************************************************************/

void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}

static void rx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
   static uint32_t inum =0;
   
    if (kStatus_SAI_RxError == status)
    {
        /* Handle the error. */
    }
    else
    {
       if(inum++>=100)
       {
          inum=0;
          //--PRINTF("rx1 call\r\n");
          
       }
       
       //--if(emptyBlock>0)    //--add       //20230130chg--01 remove think
          emptyBlock--;
    }
        
}

static void tx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
   static uint32_t inum =0;   

   if (kStatus_SAI_TxError == status)
    {
        /* Handle the error. */
    }
    else
    {
       if(inum++>=100)
       {
          inum=0;
          //--PRINTF("tx call\r\n");          
       }
       
       if(emptyBlock<BUFFER_NUMBER)   //--add
          emptyBlock++;
    }
    
    isFinished = true;
}

 
static void rx_callback3(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
    static uint32_t inum =0;   
    if (kStatus_SAI_RxError == status)
    {
        /* Handle the error. */
    }
    else
    {
       if(inum++>=100)
       {
          inum=0;
          //--PRINTF("rx3 call\r\n");
       }
       
       //if(emptyBlock>0)    //--add
          emptyBlock--;
    }
}


volatile bool g_InputSignal = false;
void EXAMPLE_GPIO_IRQHandler(void)
{
    /* clear the interrupt status */
    GPIO_PortClearInterruptFlags(EXAMPLE_SW_GPIO, 1U << EXAMPLE_SW_GPIO_PIN);
    /* Change state of switch. */
    g_InputSignal = true;
    SDK_ISR_EXIT_BARRIER;
}

////void APP_WAKEUP_BUTTON_IRQ_HANDLER(void)
////{
////    if ((1U << APP_WAKEUP_BUTTON_GPIO_PIN) & GPIO_GetPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO))
////    {
////        /* Disable interrupt. */
////        GPIO_DisableInterrupts(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
////        GPIO_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
////        LPM_DisableWakeupSource(APP_WAKEUP_BUTTON_IRQ);
////    }
////    SDK_ISR_EXIT_BARRIER;
////}

////void APP_WAKEUP_GPT_IRQn_HANDLER(void)
////{
////    GPT_ClearStatusFlags(APP_WAKEUP_GPT_BASE, kGPT_OutputCompare1Flag);
////    GPT_StopTimer(APP_WAKEUP_GPT_BASE);
////    LPM_DisableWakeupSource(APP_WAKEUP_GPT_IRQn);
////    SDK_ISR_EXIT_BARRIER;
////}



int I2C_writedata(uint8_t idevice_addr, uint8_t isub_addr, uint8_t *sbuf_write, uint8_t ilen)
{   
    uint8_t isubaddress = isub_addr;
    size_t txCount        = 0xFFU;
    status_t reVal        = kStatus_Fail;
   
    if (kStatus_Success == LPI2C_MasterStart(EXAMPLE_I2C_MASTER, idevice_addr, kLPI2C_Write))  //send SLAW
    {       
       
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
       
        while (txCount)
            LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);        

        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)           
            return kStatus_LPI2C_Nak;                  
       
        //send sub address
        //--isubaddress=0X03;  //P1 putout value register
        reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, &isubaddress, 1);
        if (reVal != kStatus_Success)
        {
           
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
            }
            return -1;
        }   
        
        reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, sbuf_write, ilen);
        if (reVal != kStatus_Success)
        {       
           
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
            }
            return -1;
        }     

        reVal = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
        if (reVal != kStatus_Success)
        {       
           
            return -1;
        }                      
        
    }   
    return reVal;
}    

int I2C_readdata(uint8_t idevice_addr, uint8_t isub_addr, uint8_t *sbuf_read, uint8_t ilen)
{       
    uint8_t isubaddress = isub_addr;
    size_t txCount        = 0xFFU;
    status_t reVal        = kStatus_Fail;
   
    /* Receive blocking data from slave */
    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
    if (kStatus_Success == LPI2C_MasterStart(EXAMPLE_I2C_MASTER, idevice_addr, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
        while (txCount)
        {
            LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
        }
        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)
        {
            return kStatus_LPI2C_Nak;
        }

        //send SUBADDR
        //--isubaddress=0X03;  //P1 putout value register
        reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, &isubaddress, 1);  //subAddress
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
            }
            return -1;
        }

        reVal = LPI2C_MasterRepeatedStart(EXAMPLE_I2C_MASTER, idevice_addr, kLPI2C_Read);
        if (reVal != kStatus_Success)
        {
            return -1;
        }

        reVal = LPI2C_MasterReceive(EXAMPLE_I2C_MASTER, sbuf_read, 1);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
            }
            return -1;
        }

        reVal = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
        
    }
    return reVal;
}       
       
void gpio_ui_init(void)
{  
    //gpio_pin_config_t output_config1 = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};     //-add
    
    //open ui 3.3v   P2.0
    GPIO_PinInit(GPIO2, 0, &output_config1); 
    GPIO_ClearPinsOutput(GPIO2, 1u << 0); //set 0, open ui 3.3v      
    
    //standby control init   p1.2
    GPIO_PinInit(GPIO1, 2, &output_config1); 
    GPIO_SetPinsOutput(GPIO1, 1u << 2); //set 1, not standby    
    
    //GPIO0 init   p1.0
    GPIO_PinInit(GPIO1, 0, &output_config1); 
    GPIO_SetPinsOutput(GPIO1, 1u << 0); //set 1, GPIO0 High  //NORMAL   
    //GPIO_ClearPinsOutput(GPIO1, 1u << 0); //set 0, GPIO0 LOW  


//    //--testio  --timeline   p2.1   (after test , normal use to LED_UI_WHITE)
//    GPIO_PinInit(GPIO2, 1, &output_config1); 
//    GPIO_ClearPinsOutput(GPIO2, 1u << 1); //set 0    
    
}  

void button_init(void)
{       
    //button init
    gpio_pin_config_t input_config1 = { kGPIO_DigitalInput, 0, kGPIO_IntRisingEdge, };    
     /* Init input switch GPIO. */
    GPIO_PinInit(GPIO1, 13, &input_config1);    //P1.13      

    /* Enable GPIO pin interrupt */
    EnableIRQ(EXAMPLE_SW_IRQ);  //--move here
    GPIO_PortEnableInterrupts(EXAMPLE_SW_GPIO, 1U << EXAMPLE_SW_GPIO_PIN);     //P1.13       
}

void i2c_init(void)
{    
    lpi2c_master_config_t masterConfig;      

    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);


    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = LPI2C_BAUDRATE;
    LPI2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, LPI2C_MASTER_CLOCK_FREQUENCY);
}

#define I2CDELAY 50000  //1ms
void expander_io_init(void)
{
    uint8_t ibuf_exio_dir[10]={0};
   
   //init P0 low        
   iexP0[0]=0;
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x02, iexP0, 1);

   for (uint32_t i = 0; i < I2CDELAY; i++)    
        __NOP();  
    
    //i2c write,set expand p0 direction
    ibuf_exio_dir[0]=0;  //output
    I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x06, ibuf_exio_dir, 1);   //reg06.7-0=0x00

    for (uint32_t i = 0; i < I2CDELAY; i++)    
        __NOP();  
    
   
   //init P1 low    
   iexP1[0]=0;
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1);     //--add
    
    for (uint32_t i = 0; i < I2CDELAY; i++)    
        __NOP();      
    

    //i2c write,set expand p1 direction
    ibuf_exio_dir[0]=0;  //output
    I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x07, ibuf_exio_dir, 1);   //reg07.7-0=0x00

    for (uint32_t i = 0; i < I2CDELAY; i++)    
        __NOP();  
   
}
 
////void expander_io_init(void)
////{
////    uint8_t ibuf_exio_dir[10]={0};
////   
////    //i2c write,set expand p0 direction
////    ibuf_exio_dir[0]=0;  //output
////    I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x06, ibuf_exio_dir, 1);   //reg06.7-0=0x00

////    for (uint32_t i = 0; i < 20000000; i++)    
////        __NOP();  
////   

////    //i2c write,set expand p1 direction
////    ibuf_exio_dir[0]=0;  //output
////    I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x07, ibuf_exio_dir, 1);   //reg07.7-0=0x00

////    for (uint32_t i = 0; i < 20000000; i++)    
////        __NOP();  
////   
////}  

void open_enable_ADC(void)
{       
   //open enable ADC , EXP0.4=1
   iexP0[0]|=1<<4;
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x02, iexP0, 1);   
}

void close_enable_ADC(void)
{       
   //close enable ADC , EXP0.4=1
   iexP0[0]&=~(1<<4);
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x02, iexP0, 1);   
}


void Select_MCU_USB(void)
{   
    
   //open enable MCU USB , EXP0.2=1
   iexP0[0]|=1<<2;
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x02, iexP0, 1);   
}

void open_tas5805_PDN(void)
{       
   //open tas5805 , EXP1.0=1
   iexP1[0]|=1<<0;
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1);   
}

void close_tas5805_PDN(void)
{       
   //open tas5805 , EXP1.0=1
   iexP1[0]&=~(1<<0);
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1);   
}


void i2s_init_evk(void)
{    
    sai_transceiver_t saiConfig;   
       
    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);  

    /* SAI init */
    SAI_Init(DEMO_SAI);
    SAI_TransferTxCreateHandle(DEMO_SAI, &txHandle, tx_callback, NULL);
    SAI_TransferRxCreateHandle(DEMO_SAI, &rxHandle, rx_callback, NULL);   

    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1U << DEMO_SAI_CHANNEL);    
    
////    saiConfig.syncMode    = DEMO_SAI_TX_SYNC_MODE;
////    saiConfig.masterSlave = DEMO_SAI_MASTER_SLAVE;
////    SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &saiConfig);
///    saiConfig.syncMode = DEMO_SAI_RX_SYNC_MODE;    
////    SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &saiConfig);

    saiConfig.masterSlave = kSAI_Master;
    //saiConfig.masterSlave = kSAI_Slave;   
    saiConfig.syncMode    = kSAI_ModeAsync;
    SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &saiConfig);    
    saiConfig.syncMode = kSAI_ModeSync;      
    SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &saiConfig);
    

    /* set bit clock divider */
    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,
                          DEMO_AUDIO_DATA_CHANNEL);                          
    SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,   
                          DEMO_AUDIO_DATA_CHANNEL);

    /* master clock configurations */
    BOARD_MASTER_CLOCK_CONFIG();    
}


void SAI1_init(void)    
{    
    sai_transceiver_t saiConfig;   
       
    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    /*Enable MCLK clock*/
    //--BOARD_EnableSaiMclkOutput(true);  

    /* SAI init */
    SAI_Init(DEMO_SAI);
    SAI_TransferTxCreateHandle(DEMO_SAI, &txHandle, tx_callback, NULL);
    SAI_TransferRxCreateHandle(DEMO_SAI, &rxHandle, rx_callback, NULL);   

    /* I2S mode configurations */
      #if (defined SAI1_TX_ONE)  
          SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1U << DEMO_SAI_CHANNEL);  //tx d0  

          //--saiConfig.masterSlave = kSAI_Master;
          saiConfig.masterSlave = kSAI_Slave;
         
          saiConfig.syncMode    = kSAI_ModeAsync;
          SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &saiConfig);
          
          saiConfig.syncMode = kSAI_ModeSync;      
          SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &saiConfig);   

     #else      
        SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 3U << DEMO_SAI_CHANNEL);    //tx d0,d1 
        //SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1U << DEMO_SAI_CHANNEL);    //tx d0,d1 
        //SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1U << 1);    //tx d1 
        
        //saiConfig.fifo.fifoCombine=0; 
        saiConfig.fifo.fifoCombine=kSAI_FifoCombineModeEnabledOnReadWrite; //3   
       
       
       saiConfig.masterSlave = kSAI_Master;
       //--saiConfig.masterSlave = kSAI_Slave;
      
      
       //------tx-----
       saiConfig.channelMask=3;
       saiConfig.fifo.fifoCombine=1;   
      
       saiConfig.syncMode    = kSAI_ModeSync;
       SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &saiConfig);
       
       
       //------rx-----
       saiConfig.channelMask=1;
       saiConfig.fifo.fifoCombine=0;    
      
       saiConfig.syncMode =kSAI_ModeAsync ;      
       SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &saiConfig);       

      #endif      
   

////    saiConfig.masterSlave = kSAI_Master;
////    //--saiConfig.masterSlave = kSAI_Slave;
////   
////    saiConfig.syncMode    = kSAI_ModeAsync;
////    SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &saiConfig);
////    
////    saiConfig.syncMode = kSAI_ModeSync;      
////    SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &saiConfig);
    
    
//--tcr4 |= I2S_TCR4_FCOMB(config->fifoCombine);    
    

    /* set bit clock divider */
    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,
                          DEMO_AUDIO_DATA_CHANNEL); 


    SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,   
                          DEMO_AUDIO_DATA_CHANNEL);

    /* master clock configurations */
    BOARD_MASTER_CLOCK_CONFIG();    
}

//#if (defined BTMUSIC) 
void SAI3_init(void)
{    
    sai_transceiver_t saiConfig;   

    /* SAI init */
    SAI_Init(SAI3);
    SAI_TransferRxCreateHandle(SAI3, &rxHandle3, rx_callback3, NULL);   

    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1U << DEMO_SAI_CHANNEL);     
  

    saiConfig.masterSlave = kSAI_Slave;

    saiConfig.syncMode = kSAI_ModeAsync; //kSAI_ModeSync;      
    SAI_TransferRxSetConfig(SAI3, &rxHandle3, &saiConfig);

    /* set bit clock divider */
    SAI_RxSetBitClockRate(SAI3, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,   
                          DEMO_AUDIO_DATA_CHANNEL);

}
//#endif


void i2s_init(void)    //20230130chg--02
{    
    sai_transceiver_t saiConfig;   
       
    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);  

    /* SAI init */
    SAI_Init(DEMO_SAI);
    SAI_TransferTxCreateHandle(DEMO_SAI, &txHandle, tx_callback, NULL);
    SAI_TransferRxCreateHandle(DEMO_SAI, &rxHandle, rx_callback, NULL);   

    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1U << DEMO_SAI_CHANNEL);  //tx d0   
    //SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 3U << DEMO_SAI_CHANNEL);    //tx d0,d1     

    saiConfig.masterSlave = kSAI_Master;
    //--saiConfig.masterSlave = kSAI_Slave;
   
    saiConfig.syncMode    = kSAI_ModeAsync;
    SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &saiConfig);
    
    saiConfig.syncMode = kSAI_ModeSync;      
    SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &saiConfig);

    /* set bit clock divider */
    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,
                          DEMO_AUDIO_DATA_CHANNEL);                          
    SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,   
                          DEMO_AUDIO_DATA_CHANNEL);

    /* master clock configurations */
    BOARD_MASTER_CLOCK_CONFIG();    
}

void i2s3_init(void)       //20230130chg--03
{    
    sai_transceiver_t saiConfig;   

    /* SAI init */
    SAI_Init(SAI3);
    SAI_TransferRxCreateHandle(SAI3, &rxHandle3, rx_callback3, NULL);   

    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&saiConfig, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1U << DEMO_SAI_CHANNEL);     
  

    saiConfig.masterSlave = kSAI_Slave;

    saiConfig.syncMode = kSAI_ModeAsync; //kSAI_ModeSync;      
    SAI_TransferRxSetConfig(SAI3, &rxHandle3, &saiConfig);

    /* set bit clock divider */
    //SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,   
    //                      DEMO_AUDIO_DATA_CHANNEL);

    SAI_RxSetBitClockRate(SAI3, DEMO_AUDIO_MASTER_CLOCK, kSAI_SampleRate48KHz, kSAI_WordWidth32bits,   
                          DEMO_AUDIO_DATA_CHANNEL);

}
 

void tas5805_init_BTL(void)
{
      uint8_t  ibuf_BTL[10]={0};

      for (uint32_t i = 0; i < 20000000; i++)    
         __NOP(); 

      //regmap1
      for(uint32_t j=0; j< (sizeof(regmap1)/sizeof(regmap1[0])); j=j+2  )
      {
         ibuf_BTL[0]=regmap1[j+1];   
         I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, regmap1[j], ibuf_BTL, 1); 

         //PRINTF("wr reg%x  %x\r\n", regmap1[j], ibuf_BTL[0]);           

         for (uint32_t i = 0; i < 2000; i++)    
            __NOP();        
      }   
   
      //write tas5805 reg0  
      ibuf_BTL[0]=0;  //page0
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x00, ibuf_BTL, 1);         
      ibuf_BTL[0]=0;  //book0
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x7F, ibuf_BTL, 1);         
      ibuf_BTL[0]=0;  //page0
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x00, ibuf_BTL, 1);  
      
      //write tas5805 reg02.2       
      ibuf_BTL[0]=0<<2;     //BTL
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x02, ibuf_BTL, 1);  
      
      //write tas5805 reg33.1-0       
      ibuf_BTL[0]=3;        //0-16bit, 1-20bit, 2-24bit, 3-32bit
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x33, ibuf_BTL, 1);  
      
      //write tas5805 reg28.7-4 , reg28.3-0       
      //ibuf_BTL[0]=0x99;     //256FS, 48KHz
      ibuf_BTL[0]=0x059;     //64FS + 48KHz
      //--ibuf_BTL[0]=0x90;     //256FS + auto
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x28, ibuf_BTL, 1);   
       
      //write tas5805 reg4c.7-0       
      //ibuf_BTL[0]=39;  //vol1  max-19.5db=4.5db
      ibuf_BTL[0]=80;   
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x4c, ibuf_BTL, 1);  
 
      //write tas5805 reg03.1-0       
      ibuf_BTL[0]=2;  //Hiz
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x03, ibuf_BTL, 1);     
      ibuf_BTL[0]=3;  //PLAY
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x03, ibuf_BTL, 1);     

}
 
 

void tas5805_init_PBTL(void)
{
      uint8_t  ibuf_PBTL[10]={0};

      //write tas5805 reg0  
      ibuf_PBTL[0]=0;     //page0
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x00, ibuf_PBTL, 1);  
       
      ibuf_PBTL[0]=0;     //book0
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x7F, ibuf_PBTL, 1);  
       
      ibuf_PBTL[0]=0;     //page0
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x00, ibuf_PBTL, 1);  
       
      //write tas5805 reg02.2       
      ibuf_PBTL[0]=1<<2;  //PBTL
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x02, ibuf_PBTL, 1);        
      
      //write tas5805 reg33.1-0       
      ibuf_PBTL[0]=3;        //32bit
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x33, ibuf_PBTL, 1);        
      
      //write tas5805 reg28.7-4 , reg28.3-0       
      ibuf_PBTL[0]=0x059;     //64FS + 48KHz
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x28, ibuf_PBTL, 1);     
       
      //write tas5805 reg4c.7-0       
      //ibuf_PBTL[0]=39;  //vol1  max-19.5db=4.5db
      ibuf_PBTL[0]=80;
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x4c, ibuf_PBTL, 1);   
 
      //write tas5805 reg03.1-0       
      ibuf_PBTL[0]=2;  //Hiz
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x03, ibuf_PBTL, 1);     
      ibuf_PBTL[0]=3;  //PLAY
      I2C_writedata(SLAVE_ADDR_7BIT_TAS5805_120K, 0x03, ibuf_PBTL, 1);         

} 

void tas5805_readreg_test(void)
{
   
////   //read tas5805 regs
////   I2C_readdata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x28, g_master_rxBuff, 1);  
////   PRINTF("\r\n\r\nread reg28  %x\r\n", g_master_rxBuff[0]);  
////   I2C_readdata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x4c, g_master_rxBuff, 1);  
////   PRINTF("read reg4c  %x\r\n", g_master_rxBuff[0]);  
////   I2C_readdata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x03, g_master_rxBuff, 1);  
////   PRINTF("read reg03  %x\r\n", g_master_rxBuff[0]);  
////   I2C_readdata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x33, g_master_rxBuff, 1);  
////   PRINTF("read reg33  %x\r\n", g_master_rxBuff[0]);  
////   I2C_readdata(SLAVE_ADDR_7BIT_TAS5805_4K7, 0x02, g_master_rxBuff, 1);  
////   PRINTF("read reg02  %x\r\n", g_master_rxBuff[0]);  

}

void playtone(void)
{   
       
    xfer.data     = (uint8_t *)music;       
    xfer.dataSize = MUSIC_LEN;
    SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);

    while (isFinished != true) ;   
       
}
 


//-------------------------power ---------------------------------

#define CPU_NAME "iMXRT1011"

#define APP_WAKEUP_BUTTON_GPIO        GPIO1
//#define APP_WAKEUP_BUTTON_GPIO_PIN    0
#define APP_WAKEUP_BUTTON_GPIO_PIN    13
#define APP_WAKEUP_BUTTON_IRQ         GPIO1_Combined_0_15_IRQn
#define APP_WAKEUP_BUTTON_IRQ_HANDLER GPIO1_Combined_0_15_IRQHandler
#define APP_WAKEUP_BUTTON_NAME        BOARD_USER_BUTTON_NAME

#define APP_WAKEUP_GPT_BASE         GPT2
#define APP_WAKEUP_GPT_IRQn         GPT2_IRQn
//--#define APP_WAKEUP_GPT_IRQn_HANDLER GPT2_IRQHandler

#define APP_WAKEUP_SNVS_IRQ         SNVS_HP_WRAPPER_IRQn
#define APP_WAKEUP_SNVS_IRQ_HANDLER SNVS_HP_WRAPPER_IRQHandler

typedef enum _app_wakeup_source
{
    kAPP_WakeupSourceTimer, /*!< Wakeup by Timer.        */
    kAPP_WakeupSourcePin,   /*!< Wakeup by external pin. */
} app_wakeup_source_t;

static uint8_t s_wakeupTimeout;            /* Wakeup timeout. (Unit: Second) */
static app_wakeup_source_t s_wakeupSource; /* Wakeup source.                 */
static lpm_power_mode_t s_targetPowerMode;
static lpm_power_mode_t s_curRunMode = LPM_PowerModeOverRun;
static const char *s_modeNames[]     = {"Over RUN",    "Full Run",       "Low Speed Run", "Low Power Run",
                                    "System Idle", "Low Power Idle", "Suspend",       "SNVS"};

/*******************************************************************************
 * Code
 ******************************************************************************/
void SetLowPowerClockGate(void)
{
    CLOCK_ControlGate(kCLOCK_Aips_tz1, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Aips_tz2, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Mqs, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_FlexSpiExsc, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Dcp, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Lpuart3, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Trace, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Gpt2, kCLOCK_ClockNeededRun);     //Gpt2
    CLOCK_ControlGate(kCLOCK_Gpt2S, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Lpuart2, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Gpio2, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Lpspi1, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Lpspi2, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Pit, kCLOCK_ClockNotNeeded);
   
    //CLOCK_ControlGate(kCLOCK_Adc1, kCLOCK_ClockNotNeeded);     //adc1  
    CLOCK_ControlGate(kCLOCK_Adc1, kCLOCK_ClockNeededRun);     //adc1  
   
    CLOCK_ControlGate(kCLOCK_Gpt1, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Gpt1S, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Lpuart4, kCLOCK_ClockNotNeeded);   
    CLOCK_ControlGate(kCLOCK_Gpio1, kCLOCK_ClockNeededRun);    //gpio1
    CLOCK_ControlGate(kCLOCK_Csu, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Gpio5, kCLOCK_ClockNeededRun);     //gpio5
    CLOCK_ControlGate(kCLOCK_OcramExsc, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_IomuxcSnvs, kCLOCK_ClockNeededRun);
    
    //CLOCK_ControlGate(kCLOCK_Lpi2c1, kCLOCK_ClockNotNeeded);       //lpi2c1
    CLOCK_ControlGate(kCLOCK_Lpi2c1, kCLOCK_ClockNeededRun);       //lpi2c1
    
    CLOCK_ControlGate(kCLOCK_Lpi2c2, kCLOCK_ClockNotNeeded);      
    CLOCK_ControlGate(kCLOCK_Ocotp, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Xbar1, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Ewm0, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Wdog1, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_FlexRam, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_IomuxcSnvsGpr, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Sim_m7_clk_r, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Iomuxc, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_IomuxcGpr, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_SimM7, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_SimM, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_SimEms, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Pwm1, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Rom, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Flexio1, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Wdog3, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Dma, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Kpp, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Wdog2, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Spdif, kCLOCK_ClockNotNeeded);
    
    //CLOCK_ControlGate(kCLOCK_Sai1, kCLOCK_ClockNotNeeded);       //sai1
    //CLOCK_ControlGate(kCLOCK_Sai3, kCLOCK_ClockNotNeeded);       //sai3
    CLOCK_ControlGate(kCLOCK_Sai1, kCLOCK_ClockNeededRun);       //sai1
    CLOCK_ControlGate(kCLOCK_Sai3, kCLOCK_ClockNeededRun);       //sai3
    
    
    CLOCK_ControlGate(kCLOCK_Lpuart1, kCLOCK_ClockNeededRun);     //lpuart1
    CLOCK_ControlGate(kCLOCK_SnvsHp, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_SnvsLp, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_UsbOh3, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_Dcdc, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_FlexSpi, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Trng, kCLOCK_ClockNotNeeded);
    CLOCK_ControlGate(kCLOCK_SimPer, kCLOCK_ClockNeededRun);
    CLOCK_ControlGate(kCLOCK_Anadig, kCLOCK_ClockNeededRun);
}

void PowerDownUSBPHY(void)
{
    USBPHY->CTRL = 0xFFFFFFFFU;
}


void APP_WAKEUP_GPT_IRQn_HANDLER(void)
{
    GPT_ClearStatusFlags(APP_WAKEUP_GPT_BASE, kGPT_OutputCompare1Flag);
    GPT_StopTimer(APP_WAKEUP_GPT_BASE);
    LPM_DisableWakeupSource(APP_WAKEUP_GPT_IRQn);
    SDK_ISR_EXIT_BARRIER;
}


////void APP_WAKEUP_BUTTON_IRQ_HANDLER(void)
////{
////    if ((1U << APP_WAKEUP_BUTTON_GPIO_PIN) & GPIO_GetPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO))
////    {
////        /* Disable interrupt. */
////        GPIO_DisableInterrupts(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
////        GPIO_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
////        LPM_DisableWakeupSource(APP_WAKEUP_BUTTON_IRQ);
////    }
////    SDK_ISR_EXIT_BARRIER;
////}


void APP_WAKEUP_SNVS_IRQ_HANDLER(void)
{
    /* Clear SRTC alarm interrupt. */
    SNVS->LPSR |= SNVS_LPSR_LPTA_MASK;
    SDK_ISR_EXIT_BARRIER;
}


/*!
 * @brief Get input from user about wakeup timeout
 */
////static uint8_t APP_GetWakeupTimeout(void)
////{
////    uint8_t timeout;

////    while (1)
////    {
////        PRINTF("Select the wake up timeout in seconds.\r\n");
////        PRINTF("The allowed range is 1s ~ 9s.\r\n");
////        PRINTF("Eg. enter 5 to wake up in 5 seconds.\r\n");
////        PRINTF("\r\nWaiting for input timeout value...\r\n\r\n");

////        timeout = GETCHAR();
////        PRINTF("%c\r\n", timeout);
////        if ((timeout > '0') && (timeout <= '9'))
////        {
////            return timeout - '0';
////        }
////        PRINTF("Wrong value!\r\n");
////    }
////}

/* Get wakeup source by user input. */
////static app_wakeup_source_t APP_GetWakeupSource(lpm_power_mode_t targetMode)
////{
////    uint8_t ch;

////    while (1)
////    {
////        PRINTF("Select the wake up source:\r\n");
////        PRINTF("Press T for Timer\r\n");
////        PRINTF("Press S for switch/button %s. \r\n", APP_WAKEUP_BUTTON_NAME);

////        PRINTF("\r\nWaiting for key press..\r\n\r\n");

////        ch = GETCHAR();

////        if ((ch >= 'a') && (ch <= 'z'))
////        {
////            ch -= 'a' - 'A';
////        }

////        if (ch == 'T')
////        {
////            return kAPP_WakeupSourceTimer;
////        }
////        else if (ch == 'S')
////        {
////            return kAPP_WakeupSourcePin;
////        }
////        else
////        {
////            PRINTF("Wrong value!\r\n");
////        }
////    }
////}

/* Get wakeup timeout and wakeup source. */
static void APP_GetWakeupConfig(lpm_power_mode_t targetMode)
{
////#if defined(HAS_WAKEUP_PIN) && (HAS_WAKEUP_PIN == 0)
////    /* If no WAKEUP pin available on board, then timer is the only wake up source in SNVS mode. */
////    if (targetMode == LPM_PowerModeSNVS)
////    {
////        s_wakeupSource = kAPP_WakeupSourceTimer;
////    }
////    else
////#endif
////    {
////        /* Get wakeup source by user input. */
////        s_wakeupSource = APP_GetWakeupSource(targetMode);
////    }

////    if (kAPP_WakeupSourceTimer == s_wakeupSource)
////    {
////        /* Wakeup source is timer, user should input wakeup timeout value. */
////        s_wakeupTimeout = APP_GetWakeupTimeout();
////        PRINTF("Will wakeup in %d seconds.\r\n", s_wakeupTimeout);
////    }
////    else
////    {
////        PRINTF("Switch %s from off to on to wake up.\r\n", APP_WAKEUP_BUTTON_NAME);
////    }
}

static void APP_SetWakeupConfig(lpm_power_mode_t targetMode)
{
    /* Set timer timeout value. */
    if (kAPP_WakeupSourceTimer == s_wakeupSource)
    {
        /* GPT can not work in SNVS mode, so we set SRTC as the wakeup source. */
        if (targetMode == LPM_PowerModeSNVS)
        {
            /* Stop SRTC time counter */
            SNVS->LPCR &= ~SNVS_LPCR_SRTC_ENV_MASK;
            while ((SNVS->LPCR & SNVS_LPCR_SRTC_ENV_MASK))
            {
            }
            /* Disable SRTC alarm interrupt */
            SNVS->LPCR &= ~SNVS_LPCR_LPTA_EN_MASK;
            while ((SNVS->LPCR & SNVS_LPCR_LPTA_EN_MASK))
            {
            }

            SNVS->LPSRTCMR = 0x00;
            SNVS->LPSRTCLR = 0x00;
            /* Set alarm in seconds*/
            SNVS->LPTAR = s_wakeupTimeout;
            EnableIRQ(APP_WAKEUP_SNVS_IRQ);
            /* Enable SRTC time counter and alarm interrupt */
            SNVS->LPCR |= SNVS_LPCR_SRTC_ENV_MASK | SNVS_LPCR_LPTA_EN_MASK;
            while (!(SNVS->LPCR & SNVS_LPCR_LPTA_EN_MASK))
            {
            }

            LPM_EnableWakeupSource(APP_WAKEUP_SNVS_IRQ);
        }
        else
        {
            GPT_StopTimer(APP_WAKEUP_GPT_BASE);
            /* Update compare channel1 value will reset counter */
            GPT_SetOutputCompareValue(APP_WAKEUP_GPT_BASE, kGPT_OutputCompare_Channel1,
                                      (CLOCK_GetRtcFreq() * s_wakeupTimeout) - 1U);

            /* Enable GPT Output Compare1 interrupt */
            GPT_EnableInterrupts(APP_WAKEUP_GPT_BASE, kGPT_OutputCompare1InterruptEnable);
            NVIC_ClearPendingIRQ(APP_WAKEUP_GPT_IRQn);
            NVIC_EnableIRQ(APP_WAKEUP_GPT_IRQn);
            EnableIRQ(APP_WAKEUP_GPT_IRQn);

            /* Restart timer */
            GPT_StartTimer(APP_WAKEUP_GPT_BASE);

            LPM_EnableWakeupSource(APP_WAKEUP_GPT_IRQn);
        }
    }
    else
    {
        GPIO_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
        /* Enable GPIO pin interrupt */
        GPIO_EnableInterrupts(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
        NVIC_ClearPendingIRQ(APP_WAKEUP_BUTTON_IRQ);
        NVIC_EnableIRQ(APP_WAKEUP_BUTTON_IRQ);
        /* Enable the Interrupt */
        EnableIRQ(APP_WAKEUP_BUTTON_IRQ);
        /* Enable GPC interrupt */
        LPM_EnableWakeupSource(APP_WAKEUP_BUTTON_IRQ);
    }
}

lpm_power_mode_t APP_GetRunMode(void)
{
    return s_curRunMode;
}

void APP_SetRunMode(lpm_power_mode_t powerMode)
{
    s_curRunMode = powerMode;
}

static void APP_ShowPowerMode(lpm_power_mode_t powerMode)
{
    if (powerMode <= LPM_PowerModeRunEnd)
    {
        //--PRINTF("    Power mode: %s\r\n", s_modeNames[powerMode]);
        APP_PrintRunFrequency(1);
    }
    else
    {
        assert(0);
    }
}

/*
 * Check whether could switch to target power mode from current mode.
 * Return true if could switch, return false if could not switch.
 */
bool APP_CheckPowerMode(lpm_power_mode_t originPowerMode, lpm_power_mode_t targetPowerMode)
{
    bool modeValid = true;

    /* Don't need to change power mode if current mode is already the target mode. */
    if (originPowerMode == targetPowerMode)
    {
        //--PRINTF("Already in the target power mode.\r\n");
        modeValid = false;
    }

    return modeValid;
}

void APP_PowerPreSwitchHook(lpm_power_mode_t targetMode)
{
    if (targetMode == LPM_PowerModeSNVS)
    {
        ;//--PRINTF("Now shutting down the system...\r\n");
    }

    if (targetMode > LPM_PowerModeRunEnd)
    {
        /* Wait for debug console output finished. */
        while (!(kLPUART_TransmissionCompleteFlag & LPUART_GetStatusFlags((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)))
        {
        }
        DbgConsole_Deinit();

        /*
         * Set pin for current leakage.
         * Debug console RX pin: Set pinmux to GPIO input.
         * Debug console TX pin: Don't need to change.
         */
        ConfigUartRxPinToGpio();
    }
}

void APP_PowerPostSwitchHook(lpm_power_mode_t targetMode)
{
    if (targetMode > LPM_PowerModeRunEnd)
    {
        /*
         * Debug console RX pin is set to GPIO input, need to re-configure pinmux.
         * Debug console TX pin: Don't need to change.
         */
        ReConfigUartRxPin();
        BOARD_InitDebugConsole();
    }
    else
    {
        /* update current run mode */
        APP_SetRunMode(targetMode);
    }
}

void APP_PowerModeSwitch(lpm_power_mode_t targetPowerMode)
{
    lpm_power_mode_t curRunMode = APP_GetRunMode();

    switch (targetPowerMode)
    {
        case LPM_PowerModeOverRun:
            LPM_OverDriveRun(curRunMode);
            break;
        case LPM_PowerModeFullRun:
            LPM_FullSpeedRun(curRunMode);
            break;
        case LPM_PowerModeLowSpeedRun:
            LPM_LowSpeedRun(curRunMode);
            break;
        case LPM_PowerModeLowPowerRun:
            LPM_LowPowerRun(curRunMode);
            break;
        case LPM_PowerModeSysIdle:
            LPM_EnterSystemIdle(curRunMode);
            LPM_EnterSleepMode(kCLOCK_ModeWait);
            LPM_ExitSystemIdle(curRunMode);
            break;
        case LPM_PowerModeLPIdle:
            LPM_EnterLowPowerIdle(curRunMode);
            LPM_EnterSleepMode(kCLOCK_ModeWait);
            LPM_ExitLowPowerIdle(curRunMode);
            break;
        case LPM_PowerModeSuspend:
            LPM_EnterSuspend();
            LPM_EnterSleepMode(kCLOCK_ModeStop);
            break;
        case LPM_PowerModeSNVS:
            LPM_EnterSNVS();
            break;
        default:
            assert(false);
            break;
    }
}
//---------------------------------power ----end-----------------------------


void bt_poweron(void)
{
   
   //--PRINTF("bt onoff\r\n");
   
   //open BT   exp1.3   
   iexP1[0]|=1<<3;   //|=1<<3
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1);   //BT pair on/off            

   for (uint32_t i = 0; i < 40000000; i++)    //100ms   
     __NOP();                

   iexP1[0]&=~(1<<3);
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1); 

   for (uint32_t i = 0; i < I2CDELAY; i++)    
     __NOP();  
 
}

   
void adc_init(void)
{
   // adc init     
//   adc_config_t adcConfigStruct;
//   adc_channel_config_t adcChannelConfigStruct;      
 
   //IOMUXC_SetPinMux(IOMUXC_GPIO_AD_11_GPIOMUX_IO25, 0U); 
   //IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_11_GPIOMUX_IO25, 0xA0U);  
      
   ADC_GetDefaultConfig(&adcConfigStruct);
   ADC_Init(DEMO_ADC_BASE, &adcConfigStruct);
      
   //#if !(defined(FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE) && FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE)
   ADC_EnableHardwareTrigger(DEMO_ADC_BASE, false);
   //#endif

   /* Do auto hardware calibration. */
   if (kStatus_Success == ADC_DoAutoCalibration(DEMO_ADC_BASE))
   {
      ;//PRINTF("ADC_DoAutoCalibration() Done.\r\n");
   }
   else
   {
      ;//PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
   }

   /* Configure the user channel and interrupt. */
   //adcChannelConfigStruct.channelNumber                  = DEMO_ADC_USER_CHANNEL;
   adcChannelConfigStruct.channelNumber                  = 11;
   adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;

   //--PRINTF("ADC Full Range: %d\r\n", g_Adc_12bitFullRange);      
}
   


#define GPT_IRQ_ID             GPT2_IRQn
#define EXAMPLE_GPT            GPT2
//#define EXAMPLE_GPT_IRQHandler GPT2_IRQHandler

/* Get source clock for GPT driver (GPT prescaler = 0) */
#define EXAMPLE_GPT_CLK_FREQ CLOCK_GetFreq(kCLOCK_PerClk)

volatile bool gptIsrFlag = false;
volatile uint8_t flg_gotostandby =0;
volatile uint32_t itimercount =0;


//void EXAMPLE_GPT_IRQHandler(void)
void GPT2_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    GPT_ClearStatusFlags(EXAMPLE_GPT, kGPT_OutputCompare1Flag);

    gptIsrFlag = 1;
   
    if(flg_gotostandby==1)
       itimercount++;
   
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F, Cortex-M7, Cortex-M7F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
    __DSB();
#endif
}

void timer_init(void)
{
    uint32_t gptFreq;
    gpt_config_t gptConfig;
   
    GPT_GetDefaultConfig(&gptConfig);

    /* Initialize GPT module */
    GPT_Init(EXAMPLE_GPT, &gptConfig);

    /* Divide GPT clock source frequency by 3 inside GPT module */
    GPT_SetClockDivider(EXAMPLE_GPT, 3);

    /* Get GPT clock frequency */
    gptFreq = EXAMPLE_GPT_CLK_FREQ;

    /* GPT frequency is divided by 3 inside module */
    gptFreq /= 3;

    /* Set both GPT modules to 1 second duration */
    GPT_SetOutputCompareValue(EXAMPLE_GPT, kGPT_OutputCompare_Channel1, gptFreq);

    /* Enable GPT Output Compare1 interrupt */
    GPT_EnableInterrupts(EXAMPLE_GPT, kGPT_OutputCompare1InterruptEnable);

    /* Enable at the Interrupt */
    EnableIRQ(GPT_IRQ_ID);

/* Start Timer */
    //--PRINTF("\r\nStarting GPT timer ...");
    GPT_StartTimer(EXAMPLE_GPT);

}   

void standby(void)
{
   flg_mainfirst=0;
   
   //close AMP
   close_tas5805_PDN();
   
   //close ADC ENABLE
   close_enable_ADC();
   
   //GPIO0 init   p1.0
   //GPIO_ClearPinsOutput(GPIO1, 1u << 0);    //set 0, GPIO0 low      
    
   
   //close SAI1 
   SAI_TransferAbortReceive(SAI1, &rxHandle);
   SAI_TransferAbortSend(SAI1, &txHandle);
   SAI_Deinit(SAI1);
   CLOCK_ControlGate(kCLOCK_Sai1, kCLOCK_ClockNotNeeded);          //sai1
   set_SAI1_pins_gpio();

   //close SAI3 
   SAI_TransferAbortReceive(SAI3, &rxHandle3);
   SAI_Deinit(SAI3);
   CLOCK_ControlGate(kCLOCK_Sai3, kCLOCK_ClockNotNeeded);          //sai3   
   set_SAI3_pins_gpio();

   BOARD_EnableSaiMclkOutput(false);   
   CLOCK_DeinitAudioPll();
   
    
   //I2C deinit   
   LPI2C_MasterDeinit(EXAMPLE_I2C_MASTER);
   CLOCK_ControlGate(kCLOCK_Lpi2c1, kCLOCK_ClockNotNeeded);       //lpi2c1
   set_i2c_pins_gpio();    
    
   //standby control init   p1.2
   GPIO_ClearPinsOutput(GPIO1, 1u << 2);    //set 0,  standby    
   
   //close ui 3.3v   P2.0
   GPIO_SetPinsOutput(GPIO2, 1u << 0);    //set 1,  close ui 3.3v       
   
   
   
   //close leds on the main
   GPIO_PinInit(GPIO2, 1, &output_config1); 
   GPIO_ClearPinsOutput(GPIO2, 1 << 1);       //set 0, close LED_UI_WHITE  (before is timeline )

   GPIO_PinInit(GPIO2, 2, &output_config1); 
   GPIO_ClearPinsOutput(GPIO2, 1 << 2);       //set 0, close LED_PWM_RED  
   
   

////   //close GPT2
////   GPT_StopTimer(EXAMPLE_GPT);
////   NVIC_DisableIRQ(GPT_IRQ_ID);
////   GPT_DisableInterrupts(EXAMPLE_GPT, kGPT_OutputCompare1InterruptEnable);
////   //GPT_Init(EXAMPLE_GPT, &gptConfig);
////   GPT_Deinit(EXAMPLE_GPT);
////   CLOCK_ControlGate(kCLOCK_Gpt2, kCLOCK_ClockNotNeeded);  
   
   
}


void outstandby(void)
{

   BOARD_InitPins_work();   //work board
   gpio_ui_init();    
   button_init();

   CLOCK_ControlGate(kCLOCK_Lpi2c1, kCLOCK_ClockNeededRun);       //lpi2c1
   i2c_init();

   expander_io_init();
   open_enable_ADC();
   Select_MCU_USB();   
   open_tas5805_PDN();
   
   
   //open leds on the Hifi
   iexP1[0]|=(1<<2);  //BT_LED
   iexP1[0]|=(1<<5);  //LED_LAN_Y
   iexP1[0]|=(1<<6);  //LED_LAN_G
   I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1);     //--add      
   
   
   //open leds on the main
   
   GPIO_PinInit(GPIO2, 1, &output_config1); 
   GPIO_SetPinsOutput(GPIO2, 1 << 1);       //set 1, open LED_UI_WHITE  (before is timeline )

   GPIO_PinInit(GPIO2, 2, &output_config1); 
   GPIO_SetPinsOutput(GPIO2, 1 << 2);       //set 1, open LED_PWM_RED 
     
   
   
   CLOCK_InitAudioPll(&audioPllConfig);

   //BOARD_EnableSaiMclkOutput(false);   
   CLOCK_ControlGate(kCLOCK_Sai1, kCLOCK_ClockNeededRun);         //sai1
   CLOCK_ControlGate(kCLOCK_Sai3, kCLOCK_ClockNeededRun);         //sai3      
   SAI1_init(); 
   SAI3_init();  
   //PRINTF("af i2s3 init \r\n");   

   tas5805_init_BTL();
   //PRINTF("amp BTL init \r\n");    

   tas5805_init_PBTL(); 
   //PRINTF("amp PBTL init \r\n"); 
   
////   //timer init
////   CLOCK_ControlGate(kCLOCK_Gpt2, kCLOCK_ClockNeededRun);  
////   timer_init();   
   

   ////bt power on
   //bt_poweron();   
   //for (uint32_t i = 0; i < 40000000; i++)    //100ms   
   //  __NOP();       
   ////reconnect     
   //LPUART_WriteBlocking(LPUART1, (uint8_t*)"AT#CC\r\n", 7);
   
}


void BT_pair_ctrl()
{   
     //test button       
     if (g_InputSignal)
     {
        for (uint32_t i = 0; i < 2000000; i++)       
           __NOP();   
        
         if (1 == GPIO_PinRead(EXAMPLE_SW_GPIO, EXAMPLE_SW_GPIO_PIN))
         {
               if(flg_mainfirst==1)
                  //PRINTF("AT#CA\r\n");   //pair    
                  LPUART_WriteBlocking(LPUART1, (uint8_t*)"AT#CA\r\n", 7);      
               
               flg_mainfirst=1;

               for (uint32_t i = 0; i < 2000000U; i++)    
                 __NOP();                           
         }

         g_InputSignal = false;
     }  
}

void powermode_init(void)
{ 
   uint8_t ch;
   uint32_t freq;
   bool needSetWakeup; /* Need to set wakeup. */

   /* When wakeup from suspend, peripheral's doze & stop requests won't be cleared, need to clear them manually */
   IOMUXC_GPR->GPR4  = 0;
   IOMUXC_GPR->GPR7  = 0;
   IOMUXC_GPR->GPR8  = 0;
   IOMUXC_GPR->GPR12 = 0;

   //BOARD_InitBootPeripherals();
   //SetLowPowerClockGate();

   /* USBPHY is not used in this application. */
   CLOCK_DisableUsbhs0PhyPllClock();
   PowerDownUSBPHY();

   //PRINTF("\r\nCPU wakeup source 0x%x...\r\n", SRC->SRSR);

   APP_PrintRunFrequency(0);

   LPM_Init();    
}


#define CMDHEADER1 'M'
#define CMDHEADER2 'U'
#define CMDEND1 0x0D
#define CMDEND2 0x0A

volatile uint8_t flg_rxin1 =0;
volatile uint8_t flg_rxin2 =0;
volatile uint8_t irxnum =0;

#define RXDATALEN 10

volatile uint8_t irxdata[RXDATALEN] ={0};
volatile uint8_t irxdata2[RXDATALEN] ={0}; 

volatile uint8_t irxcount = 0;
volatile uint8_t irxcount2 = 0;


void DEMO_LPUART_IRQHandler(void)  //can MU
{
    uint8_t cuartbyte;

    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(DEMO_LPUART))
    {
        cuartbyte = LPUART_ReadByte(DEMO_LPUART);
       
        //flg_uart_rx=1;
          
       
        if( (flg_rxin1==0) && (cuartbyte != CMDHEADER1 ) )
        {
            goto lll;  
        }
         
        if( (flg_rxin1==0) && (cuartbyte == CMDHEADER1 ) )
        {
             flg_rxin1=1;
             irxdata[irxcount++] = cuartbyte;
           
             goto lll;  
         }    

         if ((flg_rxin1==1) && (flg_rxin2==0) )
         {
            if(cuartbyte == CMDHEADER2 )
            {
               flg_rxin2=1;     
               irxdata[irxcount++] = cuartbyte;

               goto lll;              
            }
            else
            {
               flg_rxin1=0;   
               irxcount=0;               
               goto lll;  
            }
         }    

         if (flg_rxin2==1) 
         {         
            irxdata[irxcount++] = cuartbyte;
            
            if( cuartbyte == CMDEND1 )  
            {    
               //irxcount=irxcount-1;
               
               irxcount2 = irxcount;               
               memcpy((uint8_t*)irxdata2, (uint8_t*)irxdata, irxcount);

               flg_rxin1 =0;
               flg_rxin2 =0;
               irxcount=0;               
               flg_uart_rx=1;
               
               goto lll;               
            }          
             
         } 
        
    }

lll:       
    
    SDK_ISR_EXIT_BARRIER;
}

void dowithuart(void)
{
   
    switch (irxdata2[2])
    {
      case '0': 
         ibtstate='0';

         break;

      case '1': 
         ibtstate='1';

         break;

      case '2': 
         ibtstate='2';          

         break;

      case '3': 
         ibtstate='3';
  
         break;

      case 'D':   //BT play music
         ibtstate='D';

         //out standby
         if(flg_standby==1) 
         {               
            APP_PowerModeSwitch(LPM_PowerModeOverRun);
            outstandby(); 

            LPUART_WriteBlocking(LPUART1, (uint8_t*)"outstandby\r\n", 12);               
            flg_standby=0; 
            
            //--APP_ShowPowerMode(LPM_PowerModeOverRun);
         }           

         break;
          
      default: 
         break;          
       
    }

}   


#define ADCGAP  200
#define ADCNUM  2
int main(void)      //WORK BOARD
{
   sai_transceiver_t saiConfig;   
   
   BOARD_ConfigMPU();          //--same
   
   #if (defined WORKBOARD)      
      BOARD_InitPins_work();   //work board
   #else   
      BOARD_InitPins_evk();    //evk board
   #endif      
   
   BOARD_InitBootClocks();  //--same
   CLOCK_InitAudioPll(&audioPllConfig);
   
   
   //-------- 1wire IO init ----------  
   
   //sense input
   gpio_pin_config_t input_config1 = { kGPIO_DigitalInput, 0, 0, };   
   GPIO_PinInit(GPIO1, 14, &input_config1);
   
   //fetdriver output
   GPIO_PinInit(GPIO1, 1, &output_config1); 
   //GPIO_SetPinsOutput(GPIO1, 1 << 1);    
   GPIO_ClearPinsOutput(GPIO1, 1 << 1);   
   
   //-------- 1wire IO init ---end-------   
   
   
   //uart init
   BOARD_InitDebugConsole();
   PRINTF("start...\r\n"); 
   
   /* Configure UART divider to default */
   CLOCK_SetMux(kCLOCK_UartMux, 1); /* Set UART source to OSC 24M */
   CLOCK_SetDiv(kCLOCK_UartDiv, 0); /* Set UART divider to 1 */   

    //uart init
    lpuart_config_t config;  
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;
    LPUART_Init(LPUART1, &config, DEMO_LPUART_CLK_FREQ);

    //send start
    //uint8_t sstart[] = "start......\r\n";
    //LPUART_WriteBlocking(LPUART1, sstart, sizeof(sstart) / sizeof(sstart[0]));

    /* Enable RX interrupt. */
    LPUART_EnableInterrupts(LPUART1, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(DEMO_LPUART_IRQn);
 

   #if (defined WORKBOARD)   
      //gpio ui init 
      gpio_ui_init();
      

      //open leds
      GPIO_PinInit(GPIO2, 1, &output_config1); 
      GPIO_SetPinsOutput(GPIO2, 1 << 1);       //set 1, open LED_UI_WHITE  (before is timeline )

      GPIO_PinInit(GPIO2, 2, &output_config1); 
      GPIO_SetPinsOutput(GPIO2, 1 << 2);       //set 1, open LED_PWM_RED 
    

      //button init 
      button_init();
      //PRINTF("button init   \r\n");
      
      
////      //sense input
////      gpio_pin_config_t input_config1 = { kGPIO_DigitalInput, 0, 0, };   
////      GPIO_PinInit(GPIO1, 14, &input_config1);  
////      
////      
////      //fetdriver output
////      GPIO_PinInit(GPIO1, 1, &output_config1); 
////      //GPIO_SetPinsOutput(GPIO1, 1 << 1);    
////      GPIO_ClearPinsOutput(GPIO1, 1 << 1);          
      
   #endif
   

//--------------add------------------------

   //i2c init 
   i2c_init();
   //PRINTF("i2c init   \r\n");

   #if (defined WORKBOARD)
   
      //expander io init
      expander_io_init();
      //PRINTF("expander io init \r\n");
      
      //enable adc sound
      open_enable_ADC();
      //PRINTF(" test052 --af enable ADC \r\n"); 
      
      //select mcu USB
      Select_MCU_USB();
      

      open_tas5805_PDN();
      //PRINTF("open amp PDN \r\n");
      
      
      //open leds  (on the hifi)
      iexP1[0]|=(1<<2);  //BT_LED
      iexP1[0]|=(1<<5);  //LED_LAN_Y
      iexP1[0]|=(1<<6);  //LED_LAN_G
      I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1);     //--add    


      //set LVDS high //EXP1.4=1
      iexP1[0]|=(1<<4);  //LVDS
      I2C_writedata(SLAVE_ADDR_7BIT_TCA9555, 0x03, iexP1, 1);     //--add    

      
   #endif

   //i2s init
   #if (defined WORKBOARD)   
   
      SAI1_init(); 

      //PRINTF("af sai1 init \r\n");  
      
      //COMBIN
      //SAI1->TCR4 &= ~I2S_TCR4_FCOMB_MASK;
      //SAI1->TCR4 |= I2S_TCR4_FCOMB(3);      
    
      #if (defined BTMUSIC) 
      SAI3_init();        
      //--i2s3_init();        //20230130chg--05
      
      //--PRINTF("af i2s3 init \r\n");    
      #endif      
 
   #else
      i2s_init_evk(); 
   #endif

   //amp init
   #if (defined WORKBOARD)
      tas5805_init_BTL();
      //PRINTF("amp BTL init \r\n");    

      tas5805_init_PBTL(); 
      //PRINTF("amp PBTL init \r\n"); 

      tas5805_readreg_test();
      
   #else         
       /* Use default setting to init codec */
       if (CODEC_Init(&codecHandle, &boardCodecConfig) != kStatus_Success)
       {
           assert(false);
       }
       if (CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight,
                           70) != kStatus_Success) //DEMO_CODEC_VOLUME
       {
           assert(false);
       }   
   #endif
   
    //open BT
    bt_poweron();

    
    adc_init();     //adc init
       
    //timer_init();   //gpt2 init
   
    powermode_init();  //power mode init   
    
    
////    while (1)
////    {
////   
////         for (uint32_t i = 0; i < (0.1*1000000); i++)    
////           __NOP();   
////          
////          //uart_send_string(&uartDev,(uint8_t*)"11111\r\n",7);  

////         GPIO_PortToggle(GPIO1, 1 << 1);  
////         //GPIO_SetPinsOutput(GPIO1, 1 << 1);           
////         //GPIO_PortClear(GPIO1, 1 << 1);           
////    }    

    while (1)
    {
       
      static uint32_t imaincount=0;
      uint32_t iadcval=0;
      uint32_t iadcval2=0;
      //static uint8_t  flg_standby=0;
       
      static uint32_t ibig=0;
      static uint32_t ilit=0;
       
       
      if (1 == gptIsrFlag)
      {
         //PRINTF("time in\r\n");
         gptIsrFlag = 0;
      }
        
      if(imaincount++>2000000)   
      //if(imaincount++>50000000)  //1500ms
      {
         imaincount=0;     
         
         //LPUART_WriteBlocking(LPUART1, (uint8_t*)"run\r\n", 5);
         
         if(flg_uart_rx==1)
         {
            flg_uart_rx=0;
            
            //LPUART_WriteBlocking(LPUART1, (uint8_t*)"uartin\r\n", 8);
            LPUART_WriteBlocking(LPUART1, (uint8_t*)irxdata2, irxcount2);
            LPUART_WriteBlocking(LPUART1, (uint8_t*)"\r\n", 2);
            
            dowithuart();            
         }
         
         PRINTF("rx,tx,eblock: %d %d %d\r\n ",rx_index,tx_index,emptyBlock  );   

         
         //adc10
         adcChannelConfigStruct.channelNumber                  = 10;      
         ADC_SetChannelConfig(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
         while (0U == ADC_GetChannelStatusFlags(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP));      
         iadcval2=ADC_GetChannelConversionValue(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP)*3300/4096;         
         PRINTF("S4 ADC10: %d\r\n", iadcval2  );   
         
         if(iadcval2>=1800)
         {
            GPIO_ClearPinsOutput(GPIO1, 1 << 1);  
         }
         else
         {
            GPIO_SetPinsOutput(GPIO1, 1 << 1);               
         }
         
         if(flg_standby==0) 
         {
            //adc11 AUTO OFF  (ok)
            adcChannelConfigStruct.channelNumber                  = 11;      
            ADC_SetChannelConfig(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
            while (0U == ADC_GetChannelStatusFlags(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP));      
            iadcval2=ADC_GetChannelConversionValue(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP)*3300/4096;
            //PRINTF("S6 ADC11: %d\r\n", iadcval2  );      
            //SDK_DelayAtLeastUs(100000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);    
         }
         
         if(flg_standby==1) 
         {         
            //adc1 STANDBY_XLR (wait test)
            adcChannelConfigStruct.channelNumber                  = 1;      
            ADC_SetChannelConfig(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
            while (0U == ADC_GetChannelStatusFlags(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP));
            iadcval=ADC_GetChannelConversionValue(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP)*3300/4096;         
            //--PRINTF("XLR ADC1: %d\r\n", iadcval  );      
                

            if( (iadcval+ADCGAP)<1600 )
               ilit++;            
            if( (iadcval)>1600+ADCGAP )
               ibig++; 
            
            //adc2 STANDBY_RCA (wait test)
            adcChannelConfigStruct.channelNumber                  = 2;      
            ADC_SetChannelConfig(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
            while (0U == ADC_GetChannelStatusFlags(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP));      
            iadcval=ADC_GetChannelConversionValue(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP)*3300/4096;
            //--PRINTF("RCA ADC2: %d\r\n",  iadcval );      
               

            if( (iadcval+ADCGAP)<1600 )
               ilit++;            
            if( (iadcval)>1600+ADCGAP )
               ibig++;                  
         } 
         
         //--PRINTF("\r\n");       
            
         
         if(iadcval2>1500)
         {
            if(ibtstate!='D')
            {
               if(flg_standby==0)     
               {           
                  flg_gotostandby=1;
                  
                  //if(itimercount>=3)
                  {

                     flg_gotostandby=0;
                     itimercount=0;
                     
                     flg_standby=1;            
                     //--PRINTF("standby\r\n");   
                     LPUART_WriteBlocking(LPUART1, (uint8_t*)"standby\r\n", 9);
                     standby();  
                     APP_PowerModeSwitch(LPM_PowerModeLowSpeedRun);
                     
                     //--APP_ShowPowerMode(LPM_PowerModeLowSpeedRun);
                  }
               }
               else
               {
                  flg_gotostandby=0;
                  itimercount=0;
               }  
            }               
            
         }    
         else
         { 
            flg_gotostandby=0;
            itimercount=0;
            
            //if(flg_standby==1)   
            //{               
            //   outstandby(); 
            //   //PRINTF("standbyout\r\n");   
            //   flg_standby=0;   
            //}
         }         
        
         if((ibig>=ADCNUM) || (ilit>=ADCNUM) )
         {
            ibig=0;
            ilit=0;
            
            if(flg_standby==1) 
            {               
               APP_PowerModeSwitch(LPM_PowerModeOverRun);
               outstandby(); 

               //--PRINTF("standbyout\r\n");  
               LPUART_WriteBlocking(LPUART1, (uint8_t*)"outstandby\r\n", 12);               
               flg_standby=0; 
               
               //--APP_ShowPowerMode(LPM_PowerModeOverRun);
            } 
         }           
         
         if(flg_standby==1)         
            ;//PRINTF("standby\r\n");          
         else
            ;//PRINTF("standbyout\r\n");   


         //GPIO_PortToggle(GPIO2, 1u << 1);    //--testio  --timeline
         
         //#if (defined BTMUSIC)  
         if(flg_sai3_rx>=1)
         {
            flg_sai3_rx=0;
            //--PRINTF("rx3 in\r\n");
         }             
         //#endif  
         
         if(flg_sai_rx==1)
         {
            flg_sai_rx=0;
            //--PRINTF("rx1 in\r\n");
         }         

         if(flg_sai_tx==1)
         {
            flg_sai_tx=0;
            //--PRINTF("tx1 out\r\n");
         }         

      }
        
      #if (defined WORKBOARD)       
         BT_pair_ctrl();   

         if (1 == GPIO_PinRead(GPIO1, 14))   
             GPIO_SetPinsOutput(GPIO2, 1 << 1);    //open white led
         else
             GPIO_ClearPinsOutput(GPIO2, 1 << 1);;   //close white led          
      #endif           
        

      #if (defined BTMUSIC) 
        
      //receive the sai3 data to buf
      if (emptyBlock > 0)   //have space to receiver data
      {
         xfer.data     = Buffer + rx_index * BUFFER_SIZE;
         xfer.dataSize = BUFFER_SIZE;
         if (kStatus_Success == SAI_TransferReceiveNonBlocking(SAI3, &rxHandle3, &xfer))
         {
             rx_index++;
         }
         if (rx_index == BUFFER_NUMBER)
         {
             rx_index = 0; 
         }
      }         
         
      //send the sai1 data out to slave      
      if (emptyBlock < BUFFER_NUMBER)   //have data come in
      {
         xfer.data     = Buffer + tx_index * BUFFER_SIZE;
         xfer.dataSize = BUFFER_SIZE;
         if (kStatus_Success == SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer))
         {
             tx_index++;
         }
         if (tx_index == BUFFER_NUMBER)
         {
             tx_index = 0U;
         }         
      }       
      #else  //adcmusic
      
      //receive the sai1 data to buf
      if (emptyBlock > 0)   //have space to receiver data
      {
         xfer.data     = Buffer + rx_index * BUFFER_SIZE;
         xfer.dataSize = BUFFER_SIZE;
         if (kStatus_Success == SAI_TransferReceiveNonBlocking(DEMO_SAI, &rxHandle, &xfer))
         {
             rx_index++;
         }
         if (rx_index == BUFFER_NUMBER)
         {
             rx_index = 0U; 
         }
      }    

      //send the sai1 data out to slave      
      if (emptyBlock < BUFFER_NUMBER)   //have data come in
      {
         xfer.data     = Buffer + tx_index * BUFFER_SIZE;
         xfer.dataSize = BUFFER_SIZE;
         if (kStatus_Success == SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer))
         {
             tx_index++;
         }
         if (tx_index == BUFFER_NUMBER)
         {
             tx_index = 0U;
         }
         
      }                  

      #endif  

      #if (defined WORKBOARD) 
      #else
         //for (uint32_t i = 0; i < 200000000; i++)    //100ms ?   
         //  __NOP();    

         //xfer.data     = (uint8_t *)music;       
         //xfer.dataSize = MUSIC_LEN;
         //SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);
         //while (isFinished != true) ; 

         //playtone();
      #endif
        
    }
    
    
}


