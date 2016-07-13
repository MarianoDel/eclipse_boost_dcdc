/**
  ******************************************************************************
  * @file    Template_2/main.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Use this template for new projects with stm32f0xx family.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hard.h"
#include "core_cm0.h"
#include "stm32f0x_gpio.h"
#include "stm32f0x_tim.h"
#include "stm32f0xx_it.h"
#include "adc.h"

//#include <stdio.h>
//#include <string.h>




//--- VARIABLES EXTERNAS ---//
volatile unsigned char timer_1seg = 0;

volatile unsigned short timer_led_comm = 0;
volatile unsigned short wait_ms_var = 0;

volatile unsigned short adc_ch[4];

volatile unsigned char seq_ready = 0;

#define Iout_Sense	adc_ch[0]
#define Vin_Sense	adc_ch[1]
#define I_Sense		adc_ch[2]
#define Vout_Sense	adc_ch[3]

//--- VARIABLES GLOBALES ---//

// ------- de los timers -------
volatile unsigned short timer_standby;
volatile unsigned char filter_timer;
static __IO uint32_t TimingDelay;

volatile unsigned char door_filter;
volatile unsigned char take_sample;
volatile unsigned char move_relay;

volatile unsigned char secs = 0;
volatile unsigned short minutes = 0;

//------- de los PID ---------
#define KNUM	7			//ojo no pasar los 65536
#define KDEN	10
#define SP_VOUT		860			//Vout_Sense mide = Vout / 13
//#define SP		837			//Vout = 3.3 * 13 * SP / 1024

#define KPNUM	25			//ojo no pasar los 32768
#define KPDEN	100

#define KDNUM	2			//ojo no pasar los 32768
#define KDDEN	1

#define DMAX	717			//maximo D permitido	Dmax = 1 - Vinmin / Vout@1024adc
//#define DMAX	100			//maximo D permitido	Dmax = 1 - Vinmin / Vout@1024adc

#define MAX_I	74			//modificacion 13-07-16
							//Iout_Sense mide = Iout * 0.33
							//Iout = 3.3 * MAX_I / (0.33 * 1024)

#define MAX_I_MOSFET	193		//modificacion 13-07-16
								//I_Sense arriba de 620mV empieza a saturar la bobina

#define IDLE	0
#define LOOK_FOR_BREAK	1
#define LOOK_FOR_MARK	2
#define LOOK_FOR_START	3

//--- FUNCIONES DEL MODULO ---//
void TimingDelay_Decrement(void);
void Update_PWM (unsigned short);

// ------- del DMX -------
extern void EXTI4_15_IRQHandler(void);

//--- FILTROS DE SENSORES ---//
#define LARGO_FILTRO 16
#define DIVISOR      4   //2 elevado al divisor = largo filtro
//#define LARGO_FILTRO 32
//#define DIVISOR      5   //2 elevado al divisor = largo filtro
unsigned short vtemp [LARGO_FILTRO + 1];
unsigned short vpote [LARGO_FILTRO + 1];

//--- FIN DEFINICIONES DE FILTRO ---//


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
	unsigned char i;
	unsigned short medida = 0;
	int acc = 0;
	short error = 0;
	short val_p = 0;
	short val_d = 0;
	short val_dz = 0;
	short val_dz1 = 0;
	short d = 0;
//	unsigned char last_main_overload  = 0;
//	unsigned char last_function;
//	unsigned char last_program, last_program_deep;
//	unsigned short last_channel;
//	unsigned short current_temp = 0;

	//!< At this stage the microcontroller clock setting is already configured,
    //   this is done through SystemInit() function which is called from startup
    //   file (startup_stm32f0xx.s) before to branch to application main.
    //   To reconfigure the default setting of SystemInit() function, refer to
    //   system_stm32f0xx.c file

	//GPIO Configuration.
	GPIO_Config();

	//ACTIVAR SYSTICK TIMER
	if (SysTick_Config(48000))
	{
		while (1)	/* Capture error */
		{
			if (LED)
				LED_OFF;
			else
				LED_ON;

			for (i = 0; i < 255; i++)
			{
				asm (	"nop \n\t"
						"nop \n\t"
						"nop \n\t" );
			}
		}
	}


	//TIM Configuration.
	TIM_1_Init();
	TIM_3_Init();

	//--- COMIENZO PROGRAMA DE PRODUCCION

	//ADC configuration.
	AdcConfig();
	ADC1->CR |= ADC_CR_ADSTART;

	//--- Main loop ---//
	while(1)
	{
		//PROGRAMA DE PRODUCCION
		if (seq_ready)
		{
			//reviso el tope de corriente del mosfet
			if (I_Sense > MAX_I_MOSFET)
			{
				//corto el ciclo
				d = 0;
			}
			else
			{
				//VEO SI USO LAZO V O I
				if (Iout_Sense < MAX_I)
				{
					//LAZO V
					error = SP_VOUT - Vout_Sense;

					//proporcional
					//val_p = KPNUM * error;
					//val_p = val_p / KPDEN;
					acc = 8192 * error;
					val_p = acc >> 15;

					//derivativo
					//val_d = KDNUM * error;
					//val_d = val_d / KDDEN;
					//val_dz = val_d;
					acc = 65536 * error;
					val_dz = acc >> 15;
					val_d = val_dz - val_dz1;
					val_dz1 = val_dz;

					d = d + val_p + val_d;
					if (d < 0)
						d = 0;
					else if (d > DMAX)
						d = DMAX;
				}
				else
				{
					//LAZO I
					LED_ON;
					error = MAX_I - Iout_Sense;	//340 es 1V en adc

					//proporcional
					acc = 27852 * error;		//0.85
					val_p = acc >> 15;

					d = d + val_p;
					if (d < 0)
						d = 0;
					else if (d > DMAX)
						d = DMAX;
				}
			}

			Update_TIM3_CH1 (d);
			Update_TIM3_CH2 (Iout_Sense);	//muestro en pata PA7 el sensado de Iout

			seq_ready = 0;
			LED_OFF;
		}
	}	//termina while(1)

	return 0;
}


//--- End of Main ---//
void Update_PWM (unsigned short pwm)
{
	Update_TIM3_CH1 (pwm);
	Update_TIM3_CH2 (4095 - pwm);
}

void EXTI4_15_IRQHandler(void)		//nueva detecta el primer 0 en usart Consola PHILIPS
{


	if(EXTI->PR & 0x0100)	//Line8
	{
		EXTI->PR |= 0x0100;
	}
}


void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}

	if (wait_ms_var)
		wait_ms_var--;

	if (timer_standby)
		timer_standby--;

	if (take_sample)
		take_sample--;

	if (filter_timer)
		filter_timer--;

	/*
	//cuenta 1 segundo
	if (button_timer_internal)
		button_timer_internal--;
	else
	{
		if (button_timer)
		{
			button_timer--;
			button_timer_internal = 1000;
		}
	}
	*/
}




