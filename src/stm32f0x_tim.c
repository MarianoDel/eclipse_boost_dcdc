/**
  ******************************************************************************
  * @file    Template_2/stm32f0x_tim.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   TIM functions.
  ******************************************************************************
  * @attention
  *
  * Use this functions to configure timers.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0x_tim.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_misc.h"

#include "hard.h"

//--- VARIABLES EXTERNAS ---//
extern volatile unsigned char timer_1seg;
extern volatile unsigned short timer_led_comm;
extern volatile unsigned short wait_ms_var;

//--- VARIABLES GLOBALES ---//

volatile unsigned short timer_1000 = 0;



//--- FUNCIONES DEL MODULO ---//
void Update_TIM3_CH1 (unsigned short a)
{
	TIM3->CCR1 = a;
}

void Update_TIM3_Freq (unsigned short a)
{
	TIM3->ARR = a;
}

void Update_TIM3_CH2 (unsigned short a)
{
	TIM3->CCR2 = a;
}

void Update_TIM3_CH3 (unsigned short a)
{
	TIM3->CCR3 = a;
}

void Update_TIM3_CH4 (unsigned short a)
{
	TIM3->CCR4 = a;
}

void Wait_ms (unsigned short wait)
{
	wait_ms_var = wait;

	while (wait_ms_var);
}

//-------------------------------------------//
// @brief  TIM configure.
// @param  None
// @retval None
//------------------------------------------//
void TIM3_IRQHandler (void)	//1 ms
{
	/*
	Usart_Time_1ms ();

	if (timer_1seg)
	{
		if (timer_1000)
			timer_1000--;
		else
		{
			timer_1seg--;
			timer_1000 = 1000;
		}
	}

	if (timer_led_comm)
		timer_led_comm--;

	if (timer_standby)
		timer_standby--;
	*/
	//bajar flag
	if (TIM3->SR & 0x01)	//bajo el flag
		TIM3->SR = 0x00;
}

void TIM_1_Init (void)
{
	if (!RCC_TIM1_CLK)
		RCC_TIM1_CLK_ON;

	//Configuracion del timer.
	TIM1->CR1 |= TIM_CR1_OPM;		//clk int / 1; upcounting; one pulse
	TIM1->CR2 |= TIM_CR2_MMS_1;		//UEV -> TRG0
	TIM1->SMCR |= TIM_SMCR_MSM | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_TS_1;
	//TIM1->SMCR = 0x0000;
	TIM1->CCMR1 = 0x0000;			//
	TIM1->CCMR2 = 0x0000;			//
	TIM1->CCER = 0x0000;
	//TIM1->ARR = 1024 - 172;	//cada tick 20.83ns; ok la int pero mide mal
	TIM1->ARR = 1024 - 80 + 26 + 10;	//cada tick 20.83ns; ok pero mide mal + periodo en cycles 7.5 / 14MHz = 535ns
	TIM1->CNT = 0;
	TIM1->PSC = 0;

	// Enable timer ver UDIS
	//TIM1->DIER |= TIM_DIER_UIE;
	TIM1->CR1 |= TIM_CR1_CEN;
}

void TIM_3_Init (void)
{

	//NVIC_InitTypeDef NVIC_InitStructure;

	if (!RCC_TIM3_CLK)
		RCC_TIM3_CLK_ON;

	//Configuracion del timer.
	TIM3->CR1 = 0x00;		//clk int / 1; upcounting
	TIM3->CR2 |= TIM_CR2_MMS_1;		//UEV -> TRG0
#ifdef BOOST_CONVENCIONAL
	TIM3->CCMR1 = 0x6060;			//CH1 CH2 output PWM mode 1
	TIM3->CCMR2 = 0x0000;			//CH4 y CH3 disable
	TIM3->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E | TIM_CCER_CC2E;	//CH1 enable on pin active low;CH2 enable on pin active high
#endif
#ifdef BOOST_WITH_CONTROL
	TIM3->CCMR1 = 0x0060;			//CH1 output PWM mode 1 CH2 disabled
	TIM3->CCMR2 = 0x0000;			//CH4 y CH3 disable
	TIM3->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;	//CH1 enable on pin active low;
#endif
#ifdef BUCK_BOOST_WITH_CONTROL
	TIM3->CCMR1 = 0x6060;			//CH1 CH2 output PWM mode 1
	TIM3->CCMR2 = 0x0000;			//CH4 y CH3 disable
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2P | TIM_CCER_CC2E;	//CH1 enable on pin active low;CH2 enable on pin active high
#endif

	TIM3->ARR = 1023;	//freq 46.8KHz
	TIM3->CNT = 0;
	TIM3->PSC = 0;
	//TIM3->EGR = TIM_EGR_UG;	//generate event

	// Enable timer ver UDIS
	//TIM3->DIER |= TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_CEN;

	//Timer sin Int
	//NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);

#ifdef BOOST_CONVENCIONAL
	//Configuracion Pines
	//Alternate Fuction
	GPIOA->AFR[0] = 0x11000000;	//PA7 -> AF1; PA6 -> AF1
	//GPIOB->AFR[0] = 0x00000011;	//PB1 -> AF1; PB0 -> AF1
	TIM3->CCR1 = 50;
#endif

#ifdef BOOST_WITH_CONTROL
	//Configuracion Pines
	//Alternate Fuction
	GPIOB->AFR[0] = 0x00010000;	//PB4 -> AF1;
	TIM3->CCR1 = 1;
#endif

#ifdef BUCK_BOOST_WITH_CONTROL
	//Configuracion Pines
	//Alternate Fuction
	GPIOB->AFR[0] = 0x00110000;	//PB5 -> AF1; PB4 -> AF1;
	TIM3->CCR1 = 1;
#endif

}

void TIM_6_Init (void)
{
	if (!RCC_TIM6_CLK)
		RCC_TIM6_CLK_ON;

	//Configuracion del timer.
	TIM6->CR1 = 0x00;		//clk int / 1; upcounting
	TIM6->PSC = 47;			//tick cada 1us
	TIM6->ARR = 0xFFFF;			//para que arranque
	//TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM14_IRQHandler (void)	//100uS
{

	if (TIM14->SR & 0x01)
		//bajar flag
		TIM14->SR = 0x00;
}


void TIM_14_Init (void)
{

	//NVIC_InitTypeDef NVIC_InitStructure;

	if (!RCC_TIM14_CLK)
		RCC_TIM14_CLK_ON;

	/*
	//Configuracion del timer.
	TIM14->ARR = 2000; //10m
	TIM14->CNT = 0;
	TIM14->PSC = 479;
	TIM14->EGR = TIM_EGR_UG;

	// Enable timer ver UDIS
	TIM14->DIER |= TIM_DIER_UIE;
	TIM14->CR1 |= TIM_CR1_CEN;

	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	//Configuracion del timer.
	TIM14->CR1 = 0x00;		//clk int / 1; upcounting; uev
	TIM14->PSC = 47;			//tick cada 1us
	TIM14->ARR = 0xFFFF;			//para que arranque
	TIM14->EGR |= 0x0001;
}

void TIM16_IRQHandler (void)	//100uS
{

	if (TIM16->SR & 0x01)
		//bajar flag
		TIM16->SR = 0x00;
}


void TIM_16_Init (void)
{

	NVIC_InitTypeDef NVIC_InitStructure;

	if (!RCC_TIM16_CLK)
		RCC_TIM16_CLK_ON;

	//Configuracion del timer.
	TIM16->ARR = 2000; //10m
	TIM16->CNT = 0;
	TIM16->PSC = 479;
	TIM16->EGR = TIM_EGR_UG;

	// Enable timer ver UDIS
	TIM16->DIER |= TIM_DIER_UIE;
	TIM16->CR1 |= TIM_CR1_CEN;

	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM17_IRQHandler (void)	//100uS
{

	//if (GPIOA_PIN0_OUT)
	//	GPIOA_PIN0_OFF;
	//else
	//	GPIOA_PIN0_ON;

	if (TIM17->SR & 0x01)
		//bajar flag
		TIM17->SR = 0x00;
}


void TIM_17_Init (void)
{

	NVIC_InitTypeDef NVIC_InitStructure;

	if (!RCC_TIM17_CLK)
		RCC_TIM17_CLK_ON;

	//Configuracion del timer.
	TIM17->ARR = 2000; //10m
	TIM17->CNT = 0;
	TIM17->PSC = 479;
	TIM17->EGR = TIM_EGR_UG;

	// Enable timer ver UDIS
	TIM17->DIER |= TIM_DIER_UIE;
	TIM17->CR1 |= TIM_CR1_CEN;

	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//--- end of file ---//


