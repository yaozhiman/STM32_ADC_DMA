/**
  ******************************************************************************
  * @file DAC/DualModeDMA_SineWave/main.c
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main program body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  
  FF180200
  */

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "stm32f10x.h"
#include "stm32f10x_it.h"

#define ADC1_DR_Address ((uint32_t)&ADC1->DR)

static uint16_t ADCConvertedValue = 0;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void uart_write_raw(USART_TypeDef* UARTx,char *data,unsigned char len);
int fputc(int ch, FILE* f);
void USART1_Putc(unsigned char c);
void USART1_Puts(char* str);


//DACÕ®µ¿1 ‰≥ˆ≥ı ºªØ
void Dac1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );     // πƒ‹PORTAÕ®µ¿ ±÷”
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );     // πƒ‹DACÕ®µ¿ ±÷”

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                 // ∂Àø⁄≈‰÷√
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;          //ƒ£ƒ‚ ‰»Î
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4)    ;//PA.4  ‰≥ˆ∏ﬂ

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;    //≤ª π”√¥•∑¢π¶ƒ‹ TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//≤ª π”√≤®–Œ∑¢…˙
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//∆¡±Œ°¢∑˘÷µ…Ë÷√
	DAC_InitType.DAC_OutputBuffer= DAC_OutputBuffer_Enable;//DAC_OutputBuffer_Disable ;    //DAC1 ‰≥ˆª∫¥Êπÿ±’ BOFF1=1
	DAC_Init(DAC_Channel_1,&DAC_InitType);     //≥ı ºªØDACÕ®µ¿1

	DAC_Cmd(DAC_Channel_1, ENABLE); // πƒ‹DAC1
	DAC_SetChannel1Data(DAC_Align_12b_R, 0); //12Œª”“∂‘∆Î ˝æ›∏Ò Ω…Ë÷√DAC÷µ
}

void TIM2_Configuration(void)  
{   
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;   
 TIM_OCInitTypeDef TIM_OCInitStructure;   
 
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);   
 TIM_TimeBaseStructure.TIM_Period = 50;
 TIM_TimeBaseStructure.TIM_Prescaler = 71;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;   
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
 TIM_TimeBaseInit(TIM2, & TIM_TimeBaseStructure);  
      
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 TIM_OCInitStructure.TIM_Pulse = 25;   
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
 TIM_OC2Init(TIM2, & TIM_OCInitStructure);     
	
 TIM_InternalClockConfig(TIM2);  
 TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);   
 TIM_UpdateDisableConfig(TIM2, DISABLE);  
}  

void ADC_DMA_Config(void)  
{  
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(uint16_t*)&ADCConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//DMA_PeripheralDataSize_Word
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA_PeripheralDataSize_HalfWord
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular ;//DMA_Mode_Circular; ,DMA_Mode_Normal
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);           
  DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);
}

static char led_bit = 0;
void  DMA1_Channel1_IRQHandler(void)  
{  
	if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
	{  
		DMA_ClearITPendingBit(DMA1_IT_TC1);  
		
		if( led_bit == 0)
		{
			led_bit = 1;
			GPIO_SetBits(GPIOB,GPIO_Pin_5);
		}else{
			led_bit = 0;
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		}
		DAC_SetChannel1Data(DAC_Align_12b_R,ADCConvertedValue);
	}
}  

void PulseSenosrInit(void)  
{  
  ADC_InitTypeDef ADC_InitStructure;
  
  TIM2_Configuration();
  
  ADC_DMA_Config();
  
  ADC_DeInit(ADC1);
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode =DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode =DISABLE;
   
  ADC_InitStructure.ADC_ExternalTrigConv =ADC_ExternalTrigConv_T2_CC2;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;

  ADC_Init(ADC1, &ADC_InitStructure);  
	
	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);//????(12MHz),?RCC??????APB2=AHB??72MHz  
   
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1,ADC_SampleTime_1Cycles5);  
  //ADC_SMPR2 ADC_SMPR1 ???????????   
  //ADC_SQR1[19:0]?DC_SQR1[29:0]?DC_SQR3[29:0]  ???????????  ????????  
  //ADC???, ?3??? ????1,????  
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);//??????????(??ì??ì??????//??ADC?????,  
   
  ADC_DMACmd(ADC1, ENABLE);
   
  ADC_Cmd(ADC1, ENABLE);  //ADC??,??  ADC_ADON=1  
      
  ADC_ResetCalibration(ADC1);   //????  
   
  while(ADC_GetResetCalibrationStatus(ADC1));  //????????  
   
  ADC_StartCalibration(ADC1);  //????  ADC_RSTCAL=1; ????????  
   
  while(ADC_GetCalibrationStatus(ADC1));    //??????  ADC_CAL=0;    
   
   //ADC_SoftwareStartConvCmd(ADC1, ENABLE); //??????,ADC??DMA???????RAM??  
  //ADC_SWSTART=1 ?????? ?? ???????????  ???  ADC_EXTTRIG=1  
////  //??????STM32??)  
  TIM_Cmd(TIM2, ENABLE);//??????????  
  DMA_Cmd(DMA1_Channel1, ENABLE);//??DMA      
}



int main(void)
{
	SystemInit();
//	SysTick_Config(720000);
	RCC_Configuration();
	
	GPIO_Configuration();
	
//	USART1_Configuration();
//	USART2_Configuration();
	
	RCC_Configuration();
	PulseSenosrInit();
	
	Dac1_Init();
	NVIC_Configuration();

//	printf("Sending to hostname \n");

	while (1)
	{
	
	}
	return 0;
}


void uart_write_raw(USART_TypeDef* UARTx,char *data,unsigned char len)
{
	int i = 0;
	for(i = 0; i < len; i++)
	{
		USART_SendData(UARTx,data[i]);
		while( USART_GetFlagStatus(UARTx,USART_FLAG_TXE) == RESET);
	}
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO , ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//LED-RUN
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//adc
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA, ENABLE);   //??ADC?GPIOA??                        
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;        //??2  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;    //??????  
	GPIO_Init(GPIOA, &GPIO_InitStructure);     //GPIO?  

//	PWR_BackupAccessCmd(ENABLE);
//	RCC_LSEConfig(RCC_LSE_OFF);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  
	NVIC_InitStructure.NVIC_IRQChannel =DMA1_Channel1_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);   
	
}

void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;                   //¥Æø⁄≥ı ºªØΩ·ππÃÂ…˘√˜
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);

	//PA9,PA10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;             //π‹Ω≈9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       //∏¥”√Õ∆ÕÏ ‰≥ˆ
	GPIO_Init(GPIOA, &GPIO_InitStructure);                //TX≥ı ºªØ

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;            //π‹Ω≈10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //∏°ø’ ‰»Î
	GPIO_Init(GPIOA, &GPIO_InitStructure);                //RX≥ı ºªØ

	//≥ı ºªØ≤Œ ˝…Ë÷√
	USART_InitStructure.USART_BaudRate = 115200;                  //≤®Ãÿ¬ 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //◊÷≥§8Œª
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      //1ŒªÕ£÷π◊÷Ω⁄
	USART_InitStructure.USART_Parity = USART_Parity_No;         //Œﬁ∆Ê≈º–£—È
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Œﬁ¡˜øÿ÷∆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//¥Úø™RxΩ” ’∫ÕTx∑¢ÀÕπ¶ƒ‹
	USART_Init(USART1,&USART_InitStructure);                   //≥ı ºªØ

	//USART_ITConfig(USART1,USART_IT_TXE,ENABLE);                //‘ –Ì¥Æø⁄1∑¢ÀÕ÷–∂œ°£
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);               //‘ –Ì¥Æø⁄1Ω” ’÷–∂œ°£
	USART_Cmd(USART1,ENABLE);                                  //∆Ù∂Ø¥Æø⁄
	USART_ClearFlag(USART1,USART_FLAG_TC);                     //∑¢ÀÕÕÍ≥…±Í÷æŒª
}

void USART2_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;                   //¥Æø⁄≥ı ºªØΩ·ππÃÂ…˘√˜
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	//PA2-TXD,
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//PA3-RXD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//≥ı ºªØ≤Œ ˝…Ë÷√
	USART_InitStructure.USART_BaudRate = 9600;                  //≤®Ãÿ¬ 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //◊÷≥§8Œª
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      //1ŒªÕ£÷π◊÷Ω⁄
	USART_InitStructure.USART_Parity = USART_Parity_No;         //Œﬁ∆Ê≈º–£—È
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Œﬁ¡˜øÿ÷∆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//¥Úø™RxΩ” ’∫ÕTx∑¢ÀÕπ¶ƒ‹
	USART_Init(USART2,&USART_InitStructure);                   //≥ı ºªØ

	//USART_ITConfig(USART2,USART_IT_TXE,ENABLE);                //‘ –Ì¥Æø⁄1∑¢ÀÕ÷–∂œ°£
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);               //‘ –Ì¥Æø⁄1Ω” ’÷–∂œ°£
	USART_Cmd(USART2,ENABLE);                                  //∆Ù∂Ø¥Æø⁄
	USART_ClearFlag(USART2,USART_FLAG_TC);                     //∑¢ÀÕÕÍ≥…±Í÷æŒª
}

#define DEBUG_UART USART1

int fputc(int ch, FILE *f)
{
	USART_SendData(DEBUG_UART, (unsigned char) ch);
	while (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TXE) == RESET);
	return (ch);
}

void USART1_Putc(unsigned char c)
{
	USART_SendData(DEBUG_UART, c);
	while(USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TXE) == RESET );
}

void USART1_Puts(char * str)
{
	while(*str)
	{
		USART_SendData(DEBUG_UART, *str++);
		while(USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TXE) == RESET);
	}
}

/**
  * @brief  Inserts a delay time.
  * @param nCount: specifies the delay time length.
  * @retval : None
  */
void Delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
/**
  * @}
  */

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
