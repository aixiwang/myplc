/********************************************************/
// CPU需要：STM32F103--RAM内存不小于64K	Flash内存不小于128K
// 本代码已在STM32F103RDT6测试通过
// 编辑日期：20150903
/********************************************************/



#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include <OS_Config.h>
#include "abs_addr.h"
#include <stdio.h>
#include <absacc.h> 

extern void Process_switch(void);
extern void TX_Process(void);
extern void Stm32_Clock_Init(u8 PLL);
extern u8 rx_end;
// extern u16 all_data[16600] __at (0x20005002);
extern void backup_data(void);
extern void recover_data(void);
extern void off_out(void);
u32  startup  __at (0x2000D5F0);
u8 power_down;
u8 Run_Flag=1;
#define BmpHeadSize (54)
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static void Delay(__IO uint32_t nCount)
  {  while(nCount--); }

int main(void)
{  
  u16 LED=0;
//  u16 Timer[2];
  Delay(10000);
  init_xy();		    //PLC 输入输出初始化
  Stm32_Clock_Init(9);
  startup=0X55AA55AA;
  init_xy();		     //PLC 输入输出初始化
  Delay(6);
  USART_DeInit(USART1);	 //串口初始化
  Delay(6);
  usart_init();	         //串口初始化
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
  TIM5_Init();	         //TIMER5 定时器初始化
  NVIC_Configuration();	 //中断程序全部等级划分
  power_down=5;
  while (1)
  {	
		y_refresh(); 
		x_refresh(); 
		all_prog_process();


		switch(rx_end)	  //  解析FX2N 三菱软件发送的命令
		{
			case 1  :
				  rx_end=0,
				  Process_switch(),
				  TX_Process(),
				  LED=800; 
				  break;	  //处理三菱软件的程序

			case 5  :
				  rx_end=0,
				  TX_Process(),
				  LED=800;  
				  break;	  //处理一次发送指令

			default :   
			     if(LED>=1) LED--;                                 
			      break;	   
		}

/********************************************************/
// CPU需要：STM32F103--RAM内存不小于48K	Flash内存不小于256K
// 本代码已在STM32F103RCT6 RDT6 VCT6 VET6测试通过
// 编辑日期：20151220
// editor by DaShi
/********************************************************/


		/*	低电压检测  断电保持数据
		if(!PVD)	   //MY PCB== !PVD
		{	
			if(Timer[0]==0)
			recover_data();
			if(Timer[0]<=60000)
			Timer[0]++;
		}
		else
		{
			all_data[0x180/2]=0;
			if(Timer[0]>=100)
			backup_data();
			Timer[0]=0;
		}
		
		//*/
	}
}


PUTCHAR_PROTOTYPE
{
  USART_SendData(USART2, (uint8_t) ch);
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
  return ch;
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{  while (1); }
#endif
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
