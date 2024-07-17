/*************************************************************************
 Copyright (C), 2009, forsight Technology Co., Ltd.
　Version:        1.00   
  Author:         Henry                   
  Date:           2013-8-21 04:35PM
　产品型号：　　　Wifi_RGB调光 MASTER
　Check Sum:      0x043FEE
  Chip Type:      PMC232CS20
  Contact:        邓先生 139 2833 9299
*************************************************************************/
#define BUD9600
//#define BUD4800

//#include "2_4G.H"

#define    ONLINE    PB.6     // output,联网成功LED
#define    LINK      PA.0

#define    B         PA.4 //G         PA.4   
#define    G         PC.5 //R         PC.5
#define    R         PC.4 //B         PC.4
//=========20140318 ADD
#define    C         PC.1   
#define    W         PC.0


//#define    RESETKEY  PC.0
//#define    KEYMODE   PC.1  

#define    p_TXD     PA.3     // output,RS232-TX
#define    p_RXD     PA.6     // input,RS232-RX
#define    p_READY   PA.5     // input, 命令模式下,无线网络连接状态,低电平为已连接,高电平未连接	
#define    p_MODE    PA.2     // output,上电复位时,此脚高300ms为配置模式,低电平为正常工作模式
#define    p_RESET   PA.7     // output,复位引脚,低有效

.RAMADR	0
word RamSize;
.RAMADR SYSTEM

.ROLLING  2WORD 
word  RollPoint;

word  T16Data;
byte  Timer3s;
byte  Timer400ms;
byte  Timer500ms;
byte  HalfSecond;
byte  Timer20s;
//word  KeyDelay;
//byte  NetDelay;
byte  TimerComm;
byte  Timer250ms;
word  TimerWait;
byte  SendDelay;

//word  AdMemory;  //--zz

bit   b2ms0;
bit   b2ms1;
bit   b2ms2;
bit   bFactorySet;            //出厂设置
//bit   bNetOk;
//bit   bKeyPressed;
bit   bQuick;
bit   bWorking;
bit   bTopLight;
bit   bCommOK;
//bit   bKeyMode;
//=============
bit  RGBOK;
bit  WCOK;
//=============

bit   bNeedSend;
bit   bSend; 

byte  LightLevel;
byte  Mode;
byte  White;                 //色环调色

byte  Value;

byte  Red;
byte  Green;
byte  Blue;
//=========
byte  Warm;
byte  Colour;
//=========
byte  RedValue;
byte  GreenValue;
byte  BlueValue;

//====
byte  WValue;
byte  CValue;
word  WCount;
word  CCount;
//====
word  RedCount;
word  GreenCount;
word  BlueCount;

byte  ChangeNum;
byte  Count;
word  SpeedBuff;
word  LightFlow;

byte  AddrH;
byte  AddrL;

byte  SendFlag;               //发送数据类型标志



//黄白单色
byte flg_WC_Open;
byte flg_WC_Open_Old;

#define C_OPEN 1
#define W_OPEN 2


//APP开灯标志
byte 	flg_LED_Open1;
byte 	flg_LED_Open2;


//模拟串口部分----------------------------------------------------------
enum 
{ 
	START, 
	DATA, 
	STOP 
};

#define STCNT  2
#define DTCNT  4

byte rStatus;                 //数据状态
byte TimeCnt;                 //时间计数
byte BitCount;                //数据位计数
byte DataBuff;                //数据缓冲单元
#define MAXLENGTH 23
byte DataRecord[MAXLENGTH];   //接收的数据
word Ptr;                     //指针
byte Number;                  //接收(发送)数据个数
byte DataCount;               //接收数据的总个数
byte ReceiveTimer;            //最后一次接收数据延时

bit bReceiveOk;
bit bSendEn;


#define  LIGHTSTEP  10          //亮度调节步长   --zz
byte iAdcTimes;


//--------------------------------------------------------------------

void VM_RS232_RX(void)        // 模拟232接收函数 
{ 
   switch(rStatus) 
   { 
      case START: 
        if(p_RXD) 
        { 
        	TimeCnt = 0; 
        } 
        else 
        { 
        	TimeCnt++;
          if(TimeCnt >= STCNT) 
          { 
          	TimeCnt = 0; 
          	BitCount = 0; 
          	rStatus = DATA; 
          } 
        } 
        break; 
      case DATA: 
      	TimeCnt++;
        if(TimeCnt >= DTCNT)   //--timer2触发一次26us, 触发4次104us  -->波特率9600
        { 
          TimeCnt = 0; 
          if(p_RXD) 
          {
          	DataBuff |= 0x80; 
          } 
          else 
          { 
          	DataBuff &= 0x7F; 
          } 
          
          BitCount++;
          if(BitCount < 8) 
          { 
          	DataBuff >>= 1; 
          } 
          else 
          { 
          	BitCount = 0; 
          	rStatus = STOP; 
          } 
        } 
        break; 
      case STOP: 
        if(p_RXD) 
        {
          if (!bReceiveOk)
          {
        		if(Number==0) Ptr=DataRecord;
        		*Ptr=DataBuff;
        		Number++;
        		if(Number<MAXLENGTH)
        		{
        			Ptr++;
        		}
        		ReceiveTimer=6;
          }
          rStatus = START; 
        } 
        break; 
      default: 
        TimeCnt = 0; 
        rStatus = START; 
        break; 
   } 
}

void PortInit(void)
{
	PA  = 0b00001000;           //data
	PAC = 0b10011111;           //input or  output
	PAPH= 0b01001000;           //pull up
	
	$ PADIER 0b11111101

	PB  = 0b00000000;           //data
	PBC = 0b11011110;           //input or  output
	PBPH= 0b00000001;           //pull up
	$ PBDIER 0b01111111
	
	PC  = 0b00000000;           //data
	PCC = 0b00110011;           //input or  output
	PCPH= 0b00000011;           //pull up
}

void TimerInit(void)
{
  T16M=0b00101111;            // cLk/4
  T16Data=30770;              //32768-2000   2ms
  stt16 T16Data;
  //T16M=0b00001111;          //disable cLk/4
  
  TM2CT = 0;                  
  //TM2C=0x00;                //disable 
  TM2C=0b00010000;            //system clock
#ifdef BUD4800
  TM2S=0b00100011;            //4*(3+1)/4m=4us
  TM2B = 12;                  //上限 4*13=52us
#else
  TM2S=0b00100001;            //4*(1+0)/4m=2us
  TM2B = 12;                  //26us
#endif
}

void AdcInit(void)
{
  $ ADCC  Disable,PB5;
  //=====
  $ ADCC  Enable,PB5;
  //=====
  adcm=0b10001010;           //12bit,/32
}

#ifdef CONFIGMODE
void CmdMode(void)
{                             //命令模式
	$ p_RESET	Out, Low;
	$ p_MODE	Out, High;
	p_TXD=1;
	p_MODE=1;
	p_RESET=0;
	Timer400ms=200;
	while(Timer400ms) wdreset;
	//.delay 200000
	p_RESET=1;
	//$ p_RESET	In, High;
	//$ p_MODE	In, Low;
	Timer400ms=200;
	while(Timer400ms) wdreset;
	//.delay 200000
	
}
#endif

void AutoMode(void)
{                             //自动模式
	$ p_RESET	Out, Low;
	$ p_MODE	Out, Low;
	p_TXD=1;                    //发送脚为高才能复位成功
	p_MODE=0;
	p_RESET=0;
	Timer400ms=200;
	while(Timer400ms) wdreset;
	//.delay 200000
	p_RESET=1;
	//$ p_RESET	In, High;
	//$ p_MODE	In, Low;
	Timer400ms=200;
	while(Timer400ms) wdreset;
	//.delay 200000
}

void ExitTransmitMode(void)
{                             //退出透传
	DataRecord[0]='+';
	DataRecord[1]='+';
	DataRecord[2]='+';
	DataCount=3;
	SendFlag=1;
	bSendEn=1;
}

void FactorySet(void)
{                             //恢复出厂设置
	DataRecord[0]='A';
	DataRecord[1]='T';
	DataRecord[2]='+';
	DataRecord[3]='R';
	DataRecord[4]='S';
	DataRecord[5]='T';
	DataRecord[6]='F';
	DataRecord[7]=0x0d;
	DataCount=8;
	bSendEn=1;
}

void ResetModule(void)
{                             //复位
	DataRecord[0]='A';
	DataRecord[1]='T';
	DataRecord[2]='+';
	DataRecord[3]='Z';
	DataRecord[4]=0x0d;
	DataCount=5;
	bSendEn=1;
}

void SetNetParamter(void)
{                             //采用UDP协议,通信端口50000
	DataRecord[0]='A';
	DataRecord[1]='T';
	DataRecord[2]='+';
	DataRecord[3]='A';
	DataRecord[4]='T';
	DataRecord[5]='R';
	DataRecord[6]='M';
	DataRecord[7]='=';
	DataRecord[8]='!';
	DataRecord[9]='1';
	DataRecord[10]=',';
	DataRecord[11]='1';
	DataRecord[12]=',';
	DataRecord[13]='1';
	DataRecord[14]='2';
	DataRecord[15]='0';
	DataRecord[16]=',';
	DataRecord[17]='5';
	DataRecord[18]='0';
	DataRecord[19]='0';
	DataRecord[20]='0';
	DataRecord[21]='0';
	DataRecord[22]=0x0d;
	DataCount=23;
	bSendEn=1;
}

void GetRollData(void)
{
	RollPoint  =  _SYS (ADR.ROLL);
	AddrL  =  *RollPoint$W;
	RollPoint++;
	AddrH  =  *RollPoint$W;
}

void DataInit(void)
{
	Count=0;
	SpeedBuff=0;
	bTopLight=0;
	LightFlow=0;
	ChangeNum=0;
	LightLevel=0;
}

/*void SendData(void)
{
	byte CheckSum;
	
	if(SendDelay) return;
	
	if(!bNeedSend) return;
	
	r_RegAdr = 52;	            //清空FIFO
	r_RegDH = 0x80;
	r_RegDL = 0x80;
	WriteReg();
	
	r_RegAdr = 50;	            //写FIFO
	r_RegDH = 13;               //packet length
	r_RegDL = 0x55;
	WriteReg();
	
	r_RegAdr = 50;	            //写FIFO
	r_RegDH = AddrH;    
	r_RegDL = AddrL;
	WriteReg();
	
	r_RegAdr = 50;	            //写FIFO
	r_RegDH = DataRecord[0];    
	r_RegDL = DataRecord[1];
	WriteReg();
	
	r_RegAdr = 50;	            //写FIFO
	if(bSend)
		r_RegDH = DataRecord[2];    
	else
		r_RegDH =0;
	r_RegDL = DataRecord[3];
	WriteReg();
	
	r_RegAdr = 50;	            //写FIFO
	r_RegDH = DataRecord[4];    
	r_RegDL = DataRecord[5];
	WriteReg();
	
	r_RegAdr = 50;	            //写FIFO
	r_RegDH = DataRecord[6];  
	r_RegDL = Mode;
	WriteReg();
	
	CheckSum =DataRecord[0];
	CheckSum+=DataRecord[1];
	//CheckSum+=DataRecord[2];
	CheckSum+=DataRecord[3];
	CheckSum+=DataRecord[4];
	CheckSum+=DataRecord[5];
	CheckSum+=DataRecord[6];
	CheckSum+=Mode;
	CheckSum+=White;
	
	r_RegAdr = 50;	            //写FIFO
	r_RegDH = White;  
	r_RegDL = CheckSum;
	WriteReg();
	
	r_RegAdr = 7;					      //启动发送
	r_RegDH = 1;
	r_RegDL = LT8900_Frequency;
	WriteReg();
	//while(!PKT_FLAG);				  //等待发送完成
	SendDelay=100;
	bSend=0;
}*/

void TimerCtrl(void) 
{                             //3S一次
	if(ReceiveTimer) return;    //正在接收数据
	if(bSendEn) return;         //正在发送 
	if(!bFactorySet) return;    //复位成功
	if(Timer3s && !bQuick) return;
	Timer3s=3;
	bQuick=0;
	
	if(!Timer20s) 
	{
		Timer20s=20;
		AutoMode();
		SendFlag=1;
	}
	
	switch(SendFlag)
	{
		case 1:                   //退出透传
			ExitTransmitMode();
			break;
		case 2:                   //恢复出厂设置
			FactorySet();
			break;
		case 3:                   //设置通信参数
			SetNetParamter();
			break;
		case 4:                   //复位wifi模块
			ResetModule();
			break;
		default:
			SendFlag=1;
			break;
	}
}

BYTE	mul_y1, mul_x1;
WORD	mul_t2;
void	Byte_Mul_Byte (void)
{	//	mul_t2[W]	=	mul_x1[B] * mul_y1[B]
	mul_t2$1	=	0;
	BYTE	cnt;
	cnt	=	8;

	do
	{
		mul_x1	>>=	1;
		if (CF)
		{
			mul_t2	+=	(mul_y1 << 8);
		}
		mul_t2	>>>=	1;
	} while (--cnt);
}

WORD	div_src2;
BYTE	div_val;
static	void	Word_Div_Byte (void)
{	//	div_src2[W] / div_val[B]	=	div_src2[W] * div_val[B] + div_res[B]
	BYTE	div_cnt, div_tmp, div_res;
	div_cnt	=	0;
	div_res	=	0;
	goto	div_shf;

	do
	{
		slc		div_res;
		slc		div_tmp;
		div_cnt++;
		div_res	-=	div_val;

		if (!div_tmp.0 && CF)
		{
			div_res	+=	div_val;
			CF	=	0;
		}
		else	CF	=	1;
div_shf:
		div_src2	<<<=	1;
	} while (! div_cnt.4);
}

void RedColor(void)  //--红
{
	Red=255;
	Green=0;
	Blue=0;
}
void GreenColor(void)  //--绿
{
	Red=0;
	Green=255;
	Blue=0;
}
void BlueColor(void)   //--兰
{
	Red=0;
	Green=0;
	Blue=255;
}
void YellowColor(void)  //--黄色
{
	Red=255;
	Green=55;
	Blue=0;
}
void PurpleColor(void)  //--紫色
{
	Red=255;
	Green=0;
	Blue=55;
}
void CyanColor(void)   //--蓝绿
{
	Red=0;
	Green=255;
	Blue=55;
}
void WhiteColor(void)  //--白
{
	Red=255;
	Green=255;
	Blue=255;
}

void AdConvert(void)
{
	word AdTemp;  
	word CurrenTemp;     
	BYTE iTempLight;     
	BYTE iTempValue;     
	
	if(!bWorking) return;
	if(Mode) return;	//20140319去掉
	
/*
	$ ADCC  Enable,PB5  
	AD_Start = 1;
	wait1 AD_Done
	AdTemp=(ADCRH << 8) | ADCRL;
	AdTemp>>=4;
	//AdTemp=ADCRH;
	CurrenTemp=AdTemp;


//=================================
//	White=0;
//	Mode=0;
//	WCOK=0;
//	RGBOK=0;
//	Mode=1;
//	Value=ADCRH;
//=================================


	//--AdMemory  --> old AdTemp   --算法整理: 波动大则亮, 波动小则暗
	if(AdMemory>=AdTemp)   //新的adc比旧的小
	{
		AdTemp+=(105-Value);
		if(AdMemory>AdTemp)  //新的比旧的小很多
		{

			RedValue=RED;
			GreenValue=Green;
//			BlueValue=Blue;
		}
		else   //新的比旧的略小
		{
			RedValue=0;
			GreenValue=0;
			BlueValue=0;
		}
	}

	else  //新的比旧的大
	{
		AdMemory+=(105-Value);
		if(AdTemp>=AdMemory)    //新的比旧的大很多
		{
			RedValue=RED;
			GreenValue=Green;
			BlueValue=Blue;
		}
		else  //新的比旧的略大
		{
			RedValue=0;
			GreenValue=0;
			BlueValue=0;
		}

	}
	AdMemory=CurrenTemp;
	
--*/

	//iAdcTimes++;
	//if (iAdcTimes<20) return;
	//iAdctimes=0;

	$ ADCC  Enable,PB5  
	AD_Start = 1;
	wait1 AD_Done
	AdTemp=(ADCRH << 8) | ADCRL;
	//AdTemp>>=4;
	//CurrenTemp=AdTemp;
	CurrenTemp=(ADCRH>>4);

    //--亮度
    iTempLight=ADCRH;

/*	if (iTempLight>0xc0)
	   LightLevel=100;
	else if (iTempLight>0xb0)
	   LightLevel=100;
	else if (iTempLight>0xa0)
	   LightLevel=100;
	else if (iTempLight>0x90)
	   LightLevel=100;
	else if (iTempLight>0x80)
	   LightLevel=100;
	else if (iTempLight>0x70)
	   LightLevel=100;
	else if (iTempLight>0x60)
	   LightLevel=100;
	else if (iTempLight>0x50)
	   LightLevel=100;
	else if (iTempLight>0x40)
	   LightLevel=95;
	else if (iTempLight>0x30)
	   LightLevel=10;
	else if (iTempLight>0x20)
	   LightLevel=10;
	else if (iTempLight>0x10)
	   LightLevel=10;
	else
	   LightLevel=10;
*/    

/*
	//三极管
	if (iTempLight>0x10)
	   LightLevel=100;
	else if (iTempLight>0x8)
	   LightLevel=100;
	else if (iTempLight>0x7) 
	   LightLevel=30;
	else
	   LightLevel=30;
	if (LightLevel<=30) return; 


	//运放 --5v
	if (iTempLight>0x60)
	   LightLevel=100;
	else if (iTempLight>0x50)
	   LightLevel=100;
	else if (iTempLight>0x49)  //480mV
	   LightLevel=30;
	else
	   LightLevel=30;
	   
	if (LightLevel<=30) return; 	

	//运放 --3v (高灵敏)
	if (iTempLight>0x3f)
	   LightLevel=100;
	else if (iTempLight>0x3d)  
	   LightLevel=30;
	else
	   LightLevel=30;
	   
	if (LightLevel<=30) return; 

	//运放 --3v (中灵敏)
	if (iTempLight>0x5f)
	   LightLevel=100;
	else if (iTempLight>0x5C)  
	   LightLevel=30;
	else
	   LightLevel=30;
	   
	if (LightLevel<=30) return; 

	//运放 --3v (中低灵敏)
	if (iTempLight>0x68)
	   LightLevel=100;
	else if (iTempLight>0x65)  
	   LightLevel=30;
	else
	   LightLevel=30;
	   
	if (LightLevel<=30) return; 
*/


	//运放 --3v (中-灵敏)
	if (iTempLight>0x62)
	   LightLevel=100;
	else if (iTempLight>0x60)  
	   LightLevel=30;
	else
	   LightLevel=30;
	   
	if (LightLevel<=30) return; 


    
    //iTempValue = AdTemp  & 0xff;
    iTempValue = (AdTemp >>2) & 0xff;

    iTempValue= iTempValue+  iTempValue+  iTempValue+  iTempValue+  iTempValue;
    //iTempValue= iTempValue+  iTempValue+  iTempValue;
   

    if((iTempValue<=52)&&(iTempValue>23))//0
    {
        Red=255;
        Green=(iTempValue+iTempValue+iTempValue);
        Blue=0;
    }
    if((iTempValue<=70)&&(iTempValue>52))//1
    {
        Red=255-((iTempValue-23)+(iTempValue-23)+(iTempValue-23));
        Green=255;
        Blue=0;
    }
    if((iTempValue<=127)&&(iTempValue>70))//2
    {
        Red=0;
        Green=255;
        Blue=(iTempValue-52+iTempValue-52+iTempValue-52);
    }
    if((iTempValue<=164)&&(iTempValue>127))//3
    {
        Red=0;
        Green=255-((iTempValue-70)+(iTempValue-70)+(iTempValue-70));
        Blue=255;
    }
    if((iTempValue<=211)&&(iTempValue>164))//4  //--4
    {
        Red=(iTempValue-127+iTempValue-127+iTempValue-127);
        Green=0;
        Blue=255;
    }
    if((iTempValue<255)&&(iTempValue>211))//5  //--5
    {
        Red=255;
        Green=0;
        Blue=255-((iTempValue-164)+(iTempValue-164)+(iTempValue-164));
    }
    
    if((iTempValue<=23)&&(iTempValue>0))//--6 --zz
    {
        Red=255;
        Green=0;
        Blue=255-((iTempValue-211)+(iTempValue-211)+(iTempValue-211));
    }       
	
	//AdMemory=CurrenTemp;  

} 

void ModeCtrl(void)
{
	word Buff;
	
	if(!bWorking) 
	{
		DataInit();
		RedValue=0;
		GreenValue=0;
		BlueValue=0;
		return;
	}
	if(!b2ms2) return;
	b2ms2=0;
	if(White)
	{
		RedValue=255;
		GreenValue=255;
		BlueValue=255;
		return;
	}
	switch(Mode)
	{
		case 0://声控
			//return;

			iAdcTimes++;
			if (iAdcTimes<10) return;
			iAdctimes=0;

			AdConvert();

			break;

		case 1://色环调色
			LightLevel=Value;
			break;

		case 2://色环颜色自动明暗
			Buff=101-Value;
			Buff>>=1;
			SpeedBuff++;
			if(SpeedBuff>=Buff)
			{
				SpeedBuff=0;
				if(!bTopLight)
				{
					LightLevel++;
					if(LightLevel>=100) 
					{
						LightLevel=100;
						bTopLight=1;
					}
				}
				else
				{
					if(LightLevel)  LightLevel--;
					if(!LightLevel) 
					{
						bTopLight=0; 
					}
				}
			}
			break;

		case 3://色环颜色快闪三次停1S
			LightLevel=100;
			mul_y1=101-Value;
			mul_x1=5;
			Byte_Mul_Byte();        //最快2*5ms, 最慢2*500ms
			Buff=mul_t2;
			SpeedBuff++;
			if(SpeedBuff>=Buff)
			{
				SpeedBuff=0;
				if(Count<3) 
				{
					if(bTopLight) 
					{
						bTopLight=0;
						Count++;
					}
					else 
					{
						bTopLight=1;
					}
				}
			}
			if(bTopLight) 
			{
				RedValue=0;
				GreenValue=0;
				BlueValue=0;
				return;
			}
			if(Count>=3) 
			{
				TimerWait++;
				if(TimerWait>=500)
				{
					TimerWait=1;
					Count=0;
					bTopLight=0;
				}
			}
			break;
		case 4://七色切换快闪三次停1S
			LightLevel=100;
			mul_y1=101-Value;
			mul_x1=5;
			Byte_Mul_Byte();        //最快2*5ms, 最慢2*500ms
			Buff=mul_t2;
			SpeedBuff++;
			if(SpeedBuff>=Buff)
			{
				SpeedBuff=0;
				if(Count<3) 
				{
					if(bTopLight) 
					{
						bTopLight=0;
						Count++;
					}
					else 
					{
						bTopLight=1;
					}
				}
			}
			if(bTopLight) 
			{
				RedValue=0;
				GreenValue=0;
				BlueValue=0;
				return;
			}
			if(Count>=3) 
			{
				TimerWait++;
				if(TimerWait>=500)
				{
					TimerWait=1;
					Count=0;
					bTopLight=0;
					ChangeNum++;
					if(ChangeNum>=7) ChangeNum=0;
				}
			}
ChangeColor:
			if(ChangeNum==0)
				RedColor();
			else if(ChangeNum==1)
				GreenColor();
			else if(ChangeNum==2)
				BlueColor();
			else if(ChangeNum==3)
				YellowColor();
			else if(ChangeNum==4)
				PurpleColor();
			else if(ChangeNum==5)
				CyanColor();
			else 
				WhiteColor();
			break;
		case 5://7色跳变
			LightLevel=100;
			mul_y1=101-Value;
			mul_x1=25;
			Byte_Mul_Byte();        //最快2*25ms, 最慢2*2500ms
			Buff=mul_t2;
			SpeedBuff++;
			if(SpeedBuff>=Buff)
			{
				SpeedBuff=0;
				ChangeNum++;
				if(ChangeNum>=7) ChangeNum=0;
			}
			goto ChangeColor;
			break;
		case 6://7色渐明渐暗
			Buff=101-Value;
			Buff>>=1;
			SpeedBuff++;
			if(SpeedBuff>=Buff)
			{
				SpeedBuff=0;
				if(!bTopLight)
				{
					if(LightLevel<100)  
					{
						LightLevel++;
						if(LightLevel>=100) 
						{
							LightLevel=100;
							bTopLight=1;
						}
					}
				}
				else
				{
					if(LightLevel)  LightLevel--;
					if(!LightLevel) 
					{
						bTopLight=0; 
						ChangeNum++;
						if(ChangeNum>=7) ChangeNum=0;
					}
				}
			}
			goto ChangeColor;
			break;
		case 7://7色平滑自动渐变
			Buff=101-Value;
			Buff>>=1;
			SpeedBuff++;
			if(SpeedBuff>=Buff)
			{
				SpeedBuff=0;
				if(LightFlow<255) LightFlow++;
				else
				{
					LightFlow=0;
					ChangeNum++;
					if(ChangeNum>=6) ChangeNum=0;
				}
			}
			if(ChangeNum==0)
			{
				RedValue=255;
				GreenValue=LightFlow;
				BlueValue=0;
			}
			else if(ChangeNum==1)
			{
				RedValue=255-LightFlow;
				GreenValue=255;
				BlueValue=0;
			}
			else if(ChangeNum==2)
			{
				RedValue=0;
				GreenValue=255;
				BlueValue=LightFlow;
			}	
			else if(ChangeNum==3)
			{
				RedValue=0;
				GreenValue=255-LightFlow;
				BlueValue=255;
			}
			else if(ChangeNum==4)
			{
				RedValue=LightFlow;
				GreenValue=0;
				BlueValue=255;
			}	
			else
			{
				RedValue=255;
				GreenValue=0;
				BlueValue=255-LightFlow;
			}
			return;

			break;
		default:
			mode=7;
			break;
	}

	//----------  以下调亮度  //--亮度等级 0-100  -----------
	//总的计算公式:  RedValue = Red * LightLevel/100

	//--调红色亮度: mul_t2 = Red *  LightLevel
	mul_y1=Red;
	mul_x1=LightLevel;
	Byte_Mul_Byte();

    //RedValue = mul_t2 / 100
	div_src2=mul_t2;
	div_val=100;
	Word_Div_Byte();
	RedValue=div_src2;

	//--调绿色亮度
	mul_y1=Green;
	mul_x1=LightLevel;
	Byte_Mul_Byte();
	div_src2=mul_t2;
	div_val=100;
	Word_Div_Byte();
	GreenValue=div_src2;
	
	//--调兰色亮度
	mul_y1=Blue;
	mul_x1=LightLevel;
	Byte_Mul_Byte();
	div_src2=mul_t2;
	div_val=100;
	Word_Div_Byte();
	BlueValue=div_src2;

}
void ModeCtrl_wc(void)
{
	word Buff;
	
	if(!bWorking) 
	{
		DataInit();
		WValue=0;
		CValue=0;
		return;
	}
	if(!b2ms2) return;
	b2ms2=0;
	if(White)
	{
		WValue=255;
		CValue=255;
		return;
	}
	switch(Mode)
	{
		WValue=255;
		CValue=255;
		Warm=200;
		Colour=200;
		case 0://声控
			return;
			break;
		case 1://色环调色
			//LightLevel=Value; //--zz
			//LightLevel=0x20;    //--zz
			LightLevel=0x1A;    //--zz
			break;
		case 2://色环颜色自动明暗
			Buff=101-Value;
			Buff>>=1;
			SpeedBuff++;
			if(SpeedBuff>=Buff)
			{
				SpeedBuff=0;
				if(!bTopLight)
				{
					LightLevel++;
					if(LightLevel>=100) 
					{
						LightLevel=100;
						bTopLight=1;
					}
				}
				else
				{
					if(LightLevel)  LightLevel--;
					if(!LightLevel) 
					{
						bTopLight=0; 
					}
				}
			}
			break;
		case 3://色环颜色快闪三次停1S
		
		case 4://七色切换快闪三次停1S
		

		case 5://7色跳变
		
		case 6://7色渐明渐暗
		
		case 7://7色平滑自动渐变
		default:
			mode=7;
			break;
	}


	//以下调亮度
	//Red=0;   //--zz
	//Green=0;  //--zz
	//Blue=0;  //--zz

    //-- WValue=Warm * LightLevel/100
	mul_y1=Warm;
	mul_x1=LightLevel;
	Byte_Mul_Byte();
	div_src2=mul_t2;
	div_val=100;
	Word_Div_Byte();
	WValue=div_src2;
	
    //-- CValue=Colour * LightLevel/100
	mul_y1=Colour;
	mul_x1=LightLevel;
	Byte_Mul_Byte();
	div_src2=mul_t2;
	div_val=100;
	Word_Div_Byte();
	CValue=div_src2;

}

void ReceiveProcessing_zyw(void)
{
	if(!b2ms1) return;
	b2ms1=0;
	if(ReceiveTimer)
	{                           //接收数据
		ReceiveTimer--;
		if(!ReceiveTimer)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
		{
			bQuick=1;
			bReceiveOk=1;
			DataCount=Number;
			Number=0;
			switch(SendFlag)
			{
				case 0:               //发送数据(接收数据正确则发送)
					if((DataCount==3) && (DataRecord[2]==0x55))
					{
						bCommOK=1;
						TimerComm=250;
						bNeedSend=1;
						bSend=1;
						
						if((DataRecord[2]==0x55) || (DataRecord[1]==0) )
						{   //主机地址
						    if(DataRecord[2]==0x55) bWorking=1;
						    //else if(DataRecord[0]==0) bWorking=0;

							/*if((DataRecord[0]==0x27) || (DataRecord[0]==28) )
							{//模式键+
								White=0;
								WCOK=0;
								RGBOK=1;
								if(bWorking)
								{
									Mode++;
									if(Mode>=8) Mode=0;
									DataInit();		
								}
							}
							*/
							if(DataRecord[0]==0x27) 
							{//模式键+
								White=0;
								WCOK=0;
								RGBOK=1;
								if(bWorking)
								{
									Mode++;
									if(Mode>=8) Mode=0;
									//if(Mode>=7) 
									//	Mode=0;
									//else
									//	Mode++;

									DataInit();		
								}
							}
							else if(DataRecord[0]==0x28) 
							{//模式键-
								White=0;
								WCOK=0;
								RGBOK=1;
								if(bWorking)
								{									
									if(Mode<=0) 
										Mode=7;
									else
										Mode--;

									DataInit();		
								}
							}
							
							else if(DataRecord[0]==0x20)
							{                 //色环调色
								White=0;
								WCOK=0;
								RGBOK=1;
								Mode=1;


							    //Value=20;//DataRecord[1];
								
								if((DataRecord[1]<=52)&&(DataRecord[1]>23))//0
								{						
										Red=255;
										Green=(DataRecord[1]+DataRecord[1]+DataRecord[1]);
										Blue=0;
								}
								if((DataRecord[1]<=70)&&(DataRecord[1]>52))//1
								{
										Red=255-((DataRecord[1]-23)+(DataRecord[1]-23)+(DataRecord[1]-23));
										Green=255;
										Blue=0;
								}
								if((DataRecord[1]<=127)&&(DataRecord[1]>70))//2
								{
										Red=0;
										Green=255;
										Blue=(DataRecord[1]-52+DataRecord[1]-52+DataRecord[1]-52);
								}
								if((DataRecord[1]<=164)&&(DataRecord[1]>127))//3
								{
										Red=0;
										Green=255-((DataRecord[1]-70)+(DataRecord[1]-70)+(DataRecord[1]-70));
										Blue=255;
								}
								if((DataRecord[1]<=211)&&(DataRecord[1]>164))//4
								{
										Red=(DataRecord[1]-127+DataRecord[1]-127+DataRecord[1]-127);
										Green=0;
										Blue=255;
								}
								if((DataRecord[1]<255)&&(DataRecord[1]>211))//5
								{
								    	Red=255;
								 		Green=0;
								 		Blue=255-((DataRecord[1]-164)+(DataRecord[1]-164)+(DataRecord[1]-164));
							    }

								if((DataRecord[1]<=23)&&(DataRecord[1]>0))//--6 --zz
								{
								    	Red=255;
								 		Green=0;
								 		Blue=255-((DataRecord[1]-211)+(DataRecord[1]-211)+(DataRecord[1]-211));
							    }								

							}
							else if(DataRecord[0]==0x21)
							{                 //关灯
								White=0;
								WCOK=0;
								RGBOK=1;
								Mode=1;
								Value=0X00;
							}
							else if(DataRecord[0]==0x22)
							{                 //开灯

								flg_LED_Open1=1;

								White=0;
								WCOK=0;
								RGBOK=1;
								Mode=1;
								//Value=0X20;
								Value=100;
							}
							else if(DataRecord[0]==0x23)
							{                 //亮度+
								White=0;
								WCOK=0;
								RGBOK=1;
								Mode=1;   //--色环调色

								//--Value=Value+30;
								//--if(Value>=100)Value=5;
								Value=Value  +LIGHTSTEP;               //--zz
								if(Value>=100) Value=100;

							}
							else if(DataRecord[0]==0x24)
							{                 //亮度-
								White=0;
								WCOK=0;
								RGBOK=1;
								Mode=1;    //--色环调色

								//--Value=Value-30;
								//--if(Value<=30)Value=100;
								if(Value<LIGHTSTEP) 
								   Value=0;
								else
								   Value=Value-LIGHTSTEP;
							}
							else if(DataRecord[0]==0x25)   //--待
							{                 //速度+
								White=0;
								WCOK=0;
								RGBOK=1;
								if(Mode!=1)
								{
								   Value=Value+30;        //--待
								   if(Value>=100) Value=20;
								}
								else{}
							}
							else if(DataRecord[0]==0x26)    //--待
							{                 //速度-
								White=0;
								WCOK=0;
								RGBOK=1;
								if(Mode!=1)
								{
								   Value=Value-20;      //--待
								   if(Value<=20)Value=20;
								}
								else{}
							}
							else if(DataRecord[0]==0xA5)
							{                 //音乐
								White=0;
								Mode=1;
								WCOK=0;
								RGBOK=1;

								/*Value=0X20;
								if(DataRecord[1]<0x55) 
									Red=DataRecord[1]+DataRecord[1];
								if((DataRecord[1]>=0x55)&&(DataRecord[1]<0XA5))
								{
									Green=DataRecord[1]+DataRecord[1];
									Red=0;
								}
								if((DataRecord[1]>=0xA5)&&(DataRecord[1]<=0XF0))
								{
									Blue=DataRecord[1]+DataRecord[1];
									Blue=0;
									Green=0;
								}
								--*/

								//--app播放音乐
								Value=100;
								//DataRecord[1]=DataRecord[1]+DataRecord[1];
								if (DataRecord[1]>85)
									DataRecord[1]=DataRecord[1]+DataRecord[1]+DataRecord[1]-255;
								else
									DataRecord[1]=DataRecord[1]+DataRecord[1]+DataRecord[1];


								if((DataRecord[1]<=52)&&(DataRecord[1]>23))//0
								{						
										Red=255;
										Green=(DataRecord[1]+DataRecord[1]+DataRecord[1]);
										Blue=0;
								}
								if((DataRecord[1]<=70)&&(DataRecord[1]>52))//1
								{
										Red=255-((DataRecord[1]-23)+(DataRecord[1]-23)+(DataRecord[1]-23));
										Green=255;
										Blue=0;
								}
								if((DataRecord[1]<=127)&&(DataRecord[1]>70))//2
								{
										Red=0;
										Green=255;
										Blue=(DataRecord[1]-52+DataRecord[1]-52+DataRecord[1]-52);
								}
								if((DataRecord[1]<=164)&&(DataRecord[1]>127))//3
								{
										Red=0;
										Green=255-((DataRecord[1]-70)+(DataRecord[1]-70)+(DataRecord[1]-70));
										Blue=255;
								}
								if((DataRecord[1]<=211)&&(DataRecord[1]>164))//4
								{
										Red=(DataRecord[1]-127+DataRecord[1]-127+DataRecord[1]-127);
										Green=0;
										Blue=255;
								}
								if((DataRecord[1]<255)&&(DataRecord[1]>211))//5
								{
								    	Red=255;
								 		Green=0;
								 		Blue=255-((DataRecord[1]-164)+(DataRecord[1]-164)+(DataRecord[1]-164));
							    }


								if((DataRecord[1]<=23)&&(DataRecord[1]>0))//--6 --zz
								{
								    	Red=255;
								 		Green=0;
								 		Blue=255-((DataRecord[1]-211)+(DataRecord[1]-211)+(DataRecord[1]-211));
							    }		

							}

							/*else if(DataRecord[0]==0x3e)
							{  					//暖光灯+   //--左
								W=1;
								C=0;
								Mode=1;
								WCOK=1;
								RGBOK=0;

								//Warm=Warm+20;
								//if(Warm>=255) Warm=0;  
								if( Warm>=235 )   //255-20
									Warm=255; 
								else
									Warm=Warm+20;

							}
							else if(DataRecord[0]==0x3f)
							{  					//暖光灯-    //--右
								W=1;
								C=0;
								Mode=1;
								WCOK=1;
								RGBOK=0;
								//Warm=Warm-20;
								//if(Warm<=20)Warm=255;
								if( Warm<=20 ) 
									Warm=0; 
								else
									Warm=Warm-20;

							}

							else if(DataRecord[0]==0x3C)
							{  					//冷光灯+   //--上
								W=0;
								C=1;
								Mode=1;
								WCOK=1;
								RGBOK=0;
								//Colour=Colour+20;
								//if(Colour>=255)Colour=0;
								if( Colour>=235 ) 
									Colour=255; 
								else
									Colour=Colour+20;
							}

							else if(DataRecord[0]==0x34)
							{  					//冷光灯-   //--下
								W=0;
								C=1;
								Mode=1;
								WCOK=1;
								RGBOK=0;
								//Colour=Colour-20;
								//if(Colour<=20)Colour=255;
								if( Colour<=20 ) 
									Colour=0; 
								else
									Colour=Colour-20;
							}
							*/

							else if(DataRecord[0]==0x3C)
							{  					//白光   //--上   
								W=0;
								C=1;
								Mode=1;
								WCOK=1;
								RGBOK=0;

								flg_WC_Open=C_OPEN ;
							}
							else if(DataRecord[0]==0x34)
							{  					//黄光   //--下  
								W=0;
								C=1;
								Mode=1;
								WCOK=1;
								RGBOK=0;

								flg_WC_Open=W_OPEN ;

							}
							else if(DataRecord[0]==0x3e)
							{  					//亮度-   //--左  	
								W=1;
								C=0;
								Mode=1;
								WCOK=1;
								RGBOK=0;

								if( Warm<=20 ) 
									Warm=0; 
								else
									Warm=Warm-20;

								if( Colour<=20 ) 
									Colour=0; 
								else
									Colour=Colour-20;

							}
							else if(DataRecord[0]==0x3f)
							{  					//亮度+    //--右    
								W=1;
								C=0;
								Mode=1;
								WCOK=1;
								RGBOK=0;

								if( Warm>=235 )   //255-20
									Warm=255; 
								else
									Warm=Warm+20;

								if( Colour>=235 ) 
									Colour=255; 
								else
									Colour=Colour+20;

							}


							else if(DataRecord[0]==0x32)
							{                 				//测试
								White=0;
								Mode=7;
							}
							else if(DataRecord[0]==0x39)
							{                 	//关WC灯
								White=0;
								WCOK=1;
								RGBOK=0;
								Mode=1;
								Value=0X00;

								//黄白单色
								flg_WC_Open_Old=flg_WC_Open;
								flg_WC_Open=0;

								Warm=0;
								Colour=0;
							}
							else if(DataRecord[0]==0x35)
							{                 	//开WC灯

								flg_LED_Open2=1;

								White=0;
								WCOK=1;
								RGBOK=0;
								Mode=1;
								Value=0X10;

								//黄白单色
								if (flg_WC_Open_Old==0)
								   flg_WC_Open=C_OPEN;
								else
									flg_WC_Open=flg_WC_Open_Old;

								Warm=130;
								Colour=130;

							}
							else if(DataRecord[2]==3)
							{                 	//白光
								if(bWorking)
								{
									White=1;
								}
							}
						//	Value=DataRecord[1];
						//	Value=DataRecord[0];
						//	Red=DataRecord[1];
						//	Green=DataRecord[1];
						//	Blue=DataRecord[1];
						}
					}
					break;
				default:
					SendFlag=1;
					break;
			}		

			bReceiveOk=0;
		}
	}
}


//===============================================================================
void	FPPA0 (void)
{ 
	.ADJUST_IC	SYSCLK=IHRC/4, IHRC=16MHz, VDD=2.5V, Bandgap=Off   
	disgint
	
	RamSize=_SYS(RAM_SIZE) - 1;
	A = 0;
	do
	{
		*RamSize = A;
	} while (RamSize--);
	
	PortInit();
	TimerInit();
	AdcInit();
//	SystemIni();
//	SetTxMode();
  
	fppen	=	0x03;   
	
	$ INTEN TM2,T16             // 允许定时器2中断
	clkmd.1	=	1;              //	WatchDog Enable
	engint
	
	AutoMode();
	GetRollData();
	
	/*
	bWorking=1;
	Mode=7;
	Value=90;
	Red=255;
	Green=0;
	Blue=0;
	*/


	flg_WC_Open=0;
	flg_WC_Open_Old=0;

	flg_LED_Open1=0;
	flg_LED_Open2=0;



    //---初始状态, 通电开灯---
	bWorking=1;

    flg_LED_Open2=1;
    //Value=0X10;

    W=0;
    C=1;
    Mode=1;
    WCOK=1;
    RGBOK=0;

	Colour=130;
	Warm=130;
    
    flg_WC_Open=C_OPEN ;    
     //---初始状态, 通电开灯 -- end---
  


	while(1)
	{
		wdreset;
		TimerCtrl();              //3s
		//ReceiveProcessing();
		ReceiveProcessing_zyw();
	//	KeyCtrl();
		if(WCOK==1) ModeCtrl_wc();
		if(RGBOK==1) ModeCtrl();

		//if(mode==0)	{ AdConvert(); }  //--zz
	
//		SendData();
	}

}

void	FPPA1 (void)
{
	while(1)
	{
		if(bSendEn)
		{
			TM2C=0x00;              //disable tm2
			Ptr=DataRecord;
			while( (Number<MAXLENGTH) && (Number<DataCount))
			{
				Number++;
				DataBuff=*Ptr;
				Ptr++;
				
				BitCount=0;
				p_TXD=0;
				#ifdef BUD4800
				.delay 410            //208us
				#else
				.delay 202            //104us
				#endif				
				while(BitCount<8)
				{
					if(DataBuff.0) 
						p_TXD=1;
					else
						p_TXD=0;
					#ifdef BUD4800
					.delay 407          //208us
					#else	
					.delay 199          //104us
					#endif
					DataBuff >>= 1;
					BitCount++;
				}
				p_TXD=1;
				#ifdef BUD4800
				.delay 415            //208us
				#else
				.delay 207            //104us
				#endif
			}
			Number=0;
			TM2C=0b00010000;        //system clock
			bSendEn=0;
		}
		else if(!bFactorySet)
		{
			if(!bWorking)
			{
				R=0;
				G=0;
				B=0;
				//====
				W=0;
				C=0;
				WCount=0;
				CCount=0;
				//=====
				RedCount=0;
				GreenCount=0;
				BlueCount=0;
			}
			else
			{


//RedValue=0;
//GreenValue=0;
//BlueValue=255;

				/*//通电初始开灯
                if(flg_LED_Open1==0 && flg_LED_Open2==0)
				{
                   RedValue=20;
				   GreenValue=20;
				   BlueValue=255;
				}
				*/

				RedCount++;
				if(RedCount<=RedValue) 
					R=1;
				else if(RedCount<=255)
					R=0;
				else 
					RedCount=0;
					
				GreenCount++;
				if(GreenCount<=GreenValue) 
					G=1;
				else if(GreenCount<=255)
					G=0;
				else 
					GreenCount=0;
					
				BlueCount++;
				if(BlueCount<=BlueValue) 
					B=1;
				else if(BlueCount<=255)
					B=0;
				else 
					BlueCount=0;



				//============
				//通电初始开灯
                /*if(flg_LED_Open1==0 && flg_LED_Open2==0)
				{
                   CValue=13;
				}
				*/

                if(flg_WC_Open==0) 
				{
					C=0;
					W=0;

				}
				else if(flg_WC_Open==C_OPEN ) 
				{
				   W=0;

				   CCount++;
				   if(CCount<=CValue) 
				      C=1;
				   else if(CCount<=255)
					  C=0;
				   else 
					  CCount=0;
				}
				else if(flg_WC_Open==W_OPEN ) 
				{
				   C=0;

				   WCount++;
				   if(WCount<=WValue) 
					  W=1;
				   else if(WCount<=255)
					  W=0;
				   else 
				  	  WCount=0;

				}

				//============

			}
		}
		else
		{
			R=0;
			G=0;
			B=0;
			//====
			W=0;
			C=0;
			//=====
		}
	}
}

void	Interrupt (void)
{
	pushaf		                  //	At PDK80CXX, not support the command
	
	if(Intrq.T16)
	{                           //2ms
		Intrq.T16=0;
		stt16 T16Data;
		
		b2ms0=1;
		b2ms1=1;
		b2ms2=1;
		if(SendDelay) SendDelay--;
		if(Timer400ms) Timer400ms--;
		if(TimerComm) 
		{
			TimerComm--;
			if(!TimerComm)
			{
				bCommOK=0;
				LINK=0;
			}
		}
		Timer250ms++;
		if(Timer250ms>=50)
		{
			Timer250ms=0;
			if(bCommOK) tog LINK;
		}
		Timer500ms++;
		if(Timer500ms>=250)
		{
			Timer500ms=0;
			HalfSecond++;
			if(HalfSecond.0)
			{
				if(Timer3s) Timer3s--;
				if(Timer20s) Timer20s--;
			}
		}
	}
	
	if(Intrq.TM2)  //--26us  9600对应于104us
	{                           //52us(4800) or 26us(9600)
		Intrq.TM2=0;
		VM_RS232_RX();
	}
  
	if (Intrq.PA0)
	{	
		Intrq.PA0	=	0;
	}
	if (Intrq.PB0)
	{	
		Intrq.PB0	=	0;
	}

	popaf		                    //	At PDK80CXX, not support the command
}
