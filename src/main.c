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
#include "dsp.h"

//#include <stdio.h>
//#include <string.h>


//--- VARIABLES EXTERNAS ---//
volatile unsigned char timer_1seg = 0;

volatile unsigned short timer_led_comm = 0;
volatile unsigned short wait_ms_var = 0;
volatile unsigned char seq_ready = 0;

#ifdef BOOST_CONVENCIONAL

volatile unsigned short adc_ch[4];

#define Iout_Sense	adc_ch[0]
#define Vin_Sense	adc_ch[1]
#define I_Sense		adc_ch[2]
#define Vout_Sense	adc_ch[3]
#endif

#ifdef BOOST_WITH_CONTROL

volatile unsigned short adc_ch[6];

#define Vin_Sense		adc_ch[0]
#define Iout_Sense		adc_ch[1]
#define I_Sense			adc_ch[2]
#define One_Ten_Sense	adc_ch[3]
#define One_Ten_Pote	adc_ch[4]
#define Vout_Sense		adc_ch[5]
#endif

#ifdef BUCK_BOOST_WITH_CONTROL

volatile unsigned short adc_ch[7];

#define Vin_Sense		adc_ch[0]
#define Iout_Sense		adc_ch[1]
#define Boost_Sense		adc_ch[2]
#define Buck_Sense		adc_ch[3]
#define One_Ten_Sense	adc_ch[4]
#define One_Ten_Pote	adc_ch[5]
#define Vout_Sense		adc_ch[6]
#endif

//----- para los filtros ------//
unsigned short v_pote_samples [32];
unsigned char v_pote_index;
unsigned int pote_sumation;



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
volatile int acc = 0;

						//todos se dividen por 32768
//#define KPV	49152		// 1.5
//#define KIV	3048		// I=0.093 y P=1.5 una placa ok y la otra no
//#define KIV	1024		// I=0.0625 y P=1.5 una placa ok y la otra no

//if ((defined BOOST_CONVENCIONAL) || (defined BOOST_WITH_CONTROL))

//todos se dividen por 128
#define KPV	128			// 1
#define KIV	64			// 0.5
#define KDV	0			// 0

//todos se dividen por 128
#define KPI	128			// 1
#define KII	16			// .125
#define KDI	0			// 0


#define K1V (KPV + KIV + KDV)
#define K2V (KPV + KDV + KDV)
#define K3V (KDV)

#define K1I (KPI + KII + KDI)
#define K2I (KPI + KDI + KDI)
#define K3I (KDI)



#ifdef BOOST_CONVENCIONAL
#define SP_VOUT		860			//Vout_Sense mide = Vout / 13
								//Vout = 3.3 * 13 * SP / 1024

#define DMAX	717				//maximo D permitido	Dmax = 1 - Vinmin / Vout@1024adc

#define MAX_I	72
//#define MAX_I	74				//modificacion 13-07-16
//								//Iout_Sense mide = Iout * 0.33
//								//Iout = 3.3 * MAX_I / (0.33 * 1024)


#define MAX_I_MOSFET	193		//modificacion 13-07-16
								//I_Sense arriba de 620mV empieza a saturar la bobina

#define MIN_VIN			300		//modificacion 13-07-16
								//Vin_Sense debajo de 2.39V corta @22V entrada 742
								//Vin_Sense debajo de 1.09V corta @10V entrada 337
#endif

#ifdef BOOST_WITH_CONTROL
#define SP_VOUT		955			//Vout_Sense mide = Vout / 13
								//Vout = 3.3 * 13 * SP / 1024

#define DMAX	800				//maximo D permitido	Dmax = 1 - Vinmin / Vout@1024adc

#define MAX_I	305
//#define MAX_I	153				//cuando uso Iot_Sense / 2
//								//Iout_Sense mide = Iout * 0.33
//								//Iout = 3.3 * MAX_I / (0.33 * 1024)


#define MAX_I_MOSFET	193		//modificacion 13-07-16
								//I_Sense arriba de 620mV empieza a saturar la bobina

#define MIN_VIN			300		//modificacion 13-07-16
								//Vin_Sense debajo de 2.39V corta @22V entrada 742
								//Vin_Sense debajo de 1.09V corta @10V entrada 337
#endif

#ifdef BUCK_BOOST_WITH_CONTROL
#define SP_VOUT		638			//566 23.35V
								//638 26V

#define DMAX	800				//maximo D permitido	Dmax = 1 - Vinmin / Vout@1024adc
#define DMAX_BUCK	950				//maximo D permitido	Dmax = 1 - Vinmin / Vout@1024adc

#define MAX_I	297
//#define MAX_I	153				//cuando uso Iot_Sense / 2
//								//Iout_Sense mide = Iout * 0.33
//								//Iout = 3.3 * MAX_I / (0.33 * 1024)


#define MAX_I_MOSFET	193		//modificacion 13-07-16
								//I_Sense arriba de 620mV empieza a saturar la bobina

#define MAX_VIN			953		//40V en entrada
#define MIN_VIN			233		//10V en entrada

#define CHANGE_MODE_THRESH	100

#define VBUCK_THRESH		(SP_VOUT - 70)		//10% abajo de la corriente de salida

#define IBUCK_THRESH		(MAX_I - 97)		//10% abajo de la corriente de salida
#define IBOOST_THRESH		(MAX_I + 30)		//10% arriba de la corrinete de salida
#define DMIN_THRESH		100

#endif

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
	unsigned int medida = 0;
	unsigned short pote_value;
	short error = 0;
//	short val_p = 0;
//	short val_d = 0;
//	short val_dz = 0;
//	short val_dz1 = 0;
//	short d_last = 0;

	short d = 0;
	short error_z1 = 0;
	short error_z2 = 0;

	short val_k1 = 0;
	short val_k2 = 0;
	short val_k3 = 0;

	unsigned char converter_mode = BOOST_MODE;
	unsigned char undersampling = 0;

	unsigned char change_mode_counter = 0;
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
#ifdef BOOST_WITH_CONTROL
			if (LED)
				LED_OFF;
			else
				LED_ON;
#endif

#ifdef BUCK_BOOST_WITH_CONTROL
			if (LEDR)
				LEDR_OFF;
			else
				LEDR_ON;
#endif

			for (i = 0; i < 255; i++)
			{
				asm (	"nop \n\t"
						"nop \n\t"
						"nop \n\t" );
			}
		}
	}


	//TIM Configuration.
#if ((defined BOOST_WITH_CONTROL) || (defined BOOST_CONVENCIONAL))
	TIM_3_Init();
#endif
#ifdef BUCK_BOOST_WITH_CONTROL
	TIM_1_Init();
	TIM_3_Init();
#endif

	//--- COMIENZO PROGRAMA DE PRODUCCION

	//ADC configuration.
	AdcConfig();
	ADC1->CR |= ADC_CR_ADSTART;

	//Inicializo el o los filtros
	//filtro pote
	v_pote_index = 0;
	pote_sumation = 0;
	pote_value = 0;


	//pruebo adc contra pwm
//	while (1)
//	{
//		//PROGRAMA DE PRODUCCION
//		if (seq_ready)
//		{
//			seq_ready = 0;
//			LED_ON;
//			//pote_value = MAFilter32Circular (One_Ten_Pote, v_pote_samples, p_pote, &pote_sumation);
//			pote_value = MAFilter32Pote (One_Ten_Pote);	//esto tarda 5.4us
//			//pote_value = MAFilter32 (One_Ten_Pote, v_pote_samples);	//esto tarda 32.4us
//			//pote_value = MAFilter8 (One_Ten_Pote, v_pote_samples);		//esto tarda 8.1us
//			LED_OFF;
//			Update_TIM3_CH1 (pote_value);
//		}
//	}

//	while(1)
//	{
//		if (OUTPUT_ENABLE)
//		{
//			Update_Buck(50);
//			Update_Boost(50);
//		}
//		else
//		{
//			Update_Buck(0);
//			Update_Boost(0);
//		}
//		if (LEDV)
//			LEDV_OFF;
//		else
//			LEDV_ON;
//
//		Wait_ms(300);
//	}

#ifdef BOOST_WITH_CONTROL
	MOSFET_ON;
#endif
#ifdef BUCK_BOOST_WITH_CONTROL
//	converter_mode = BOOST_MODE;
//	Update_Buck(1024);
//	Update_Boost(0);

	converter_mode = BUCK_MODE;
	//Update_Buck(100);
	Update_Boost(0);
	//while (1);
#endif

	//--- Main loop ---//
	while(1)
	{
		//PROGRAMA DE PRODUCCION
		if (seq_ready)
		{
#ifdef BUCK_BOOST_WITH_CONTROL
			switch (converter_mode)
			{
				case BUCK_MODE:
					//reviso el tope de corriente del mosfet
					if ((Buck_Sense > MAX_I_MOSFET) || (Vin_Sense > MAX_VIN))
					{
						//corto el ciclo
						d = 0;
					}
					else
					{
						//VEO SI USO LAZO V O I
						if (Vout_Sense > SP_VOUT)
						{
							//LAZO V
							error = SP_VOUT - Vout_Sense;

							//K1
							acc = K1V * error;		//5500 / 32768 = 0.167 errores de hasta 6 puntos
							val_k1 = acc >> 7;

							//K2
							acc = K2V * error_z1;		//K2 = no llega pruebo con 1
							val_k2 = acc >> 7;			//si es mas grande que K1 + K3 no lo deja arrancar

							//K3
							acc = K3V * error_z2;		//K3 = 0.4
							val_k3 = acc >> 7;

							d = d + val_k1 - val_k2 + val_k3;
							if (d < 0)
								d = 0;
							else if (d > DMAX_BUCK)		//no me preocupo si estoy con folding
								d = DMAX_BUCK;		//porque d deberia ser chico

							//Update variables PID
							error_z2 = error_z1;
							error_z1 = error;
						}
						else
						{
							//LAZO I
							LEDV_ON;

							undersampling--;
							if (!undersampling)
							{
								//undersampling = 10;		//funciona bien pero con saltos
								undersampling = 20;		//funciona bien pero con saltos

#ifdef WITH_POTE
								//con control por pote
								medida = MAX_I * pote_value;		//con filtro
								medida >>= 10;
								error = medida - Iout_Sense;	//340 es 1V en adc
#endif
#ifdef WITHOUT_POTE
								error = MAX_I - Iout_Sense;	//340 es 1V en adc
#endif


								acc = K1I * error;		//5500 / 32768 = 0.167 errores de hasta 6 puntos
								val_k1 = acc >> 7;

								//K2
								acc = K2I * error_z1;		//K2 = no llega pruebo con 1
								val_k2 = acc >> 7;			//si es mas grande que K1 + K3 no lo deja arrancar

								//K3
								acc = K3I * error_z2;		//K3 = 0.4
								val_k3 = acc >> 7;

								d = d + val_k1 - val_k2 + val_k3;
								if (d < 0)
									d = 0;
								else if (d > DMAX_BUCK)		//no me preocupo si estoy con folding
									d = DMAX_BUCK;		//porque d deberia ser chico

								//Update variables PID
								error_z2 = error_z1;
								error_z1 = error;
							}
						}
					}
#ifdef WITH_POTE
					pote_value = MAFilter8 (One_Ten_Pote, v_pote_samples);
#endif

					if (OUTPUT_ENABLE)
						Update_Buck (d);
					else
						Update_Buck (0);

					//Update_TIM3_CH2 (Iout_Sense);	//muestro en pata PA7 el sensado de Iout

					//pote_value = MAFilter32Circular (One_Ten_Pote, v_pote_samples, p_pote, &pote_sumation);
					//pote_value = MAFilter32 (One_Ten_Pote, v_pote_samples);
					//pote_value = MAFilter8 (One_Ten_Pote, v_pote_samples);
					//pote_value = MAFilter32Pote (One_Ten_Pote);
					seq_ready = 0;
					LEDV_OFF;

					//reviso si necesito un cambio de modo desde BUCK
					if ((Vin_Sense <= VBUCK_THRESH) &&
							(d == DMAX_BUCK)	&&
							(Iout_Sense < IBUCK_THRESH))
					{
						if (change_mode_counter < CHANGE_MODE_THRESH)
							change_mode_counter++;
						else
						{
							change_mode_counter = 0;
							converter_mode = BOOST_MODE;
							Update_Buck(1024);
							Update_Boost(0);
							d = 0;
							error_z1 = 0;
							error_z2 = 0;
							undersampling = 0;
						}
					}
					else if (change_mode_counter)
						change_mode_counter--;

					break;

				case BOOST_MODE:
					//reviso el tope de corriente del mosfet
					if ((Boost_Sense > MAX_I_MOSFET) || (Vin_Sense < MIN_VIN))
					{
						//corto el ciclo
						d = 0;
					}
					else
					{
						//VEO SI USO LAZO V O I
						if (Vout_Sense > SP_VOUT)
						{
							//LAZO V
							error = SP_VOUT - Vout_Sense;

							//K1
							acc = K1V * error;		//5500 / 32768 = 0.167 errores de hasta 6 puntos
							val_k1 = acc >> 7;

							//K2
							acc = K2V * error_z1;		//K2 = no llega pruebo con 1
							val_k2 = acc >> 7;			//si es mas grande que K1 + K3 no lo deja arrancar

							//K3
							acc = K3V * error_z2;		//K3 = 0.4
							val_k3 = acc >> 7;

							d = d + val_k1 - val_k2 + val_k3;
							if (d < 0)
								d = 0;
							else if (d > DMAX)		//no me preocupo si estoy con folding
								d = DMAX;		//porque d deberia ser chico

							//Update variables PID
							error_z2 = error_z1;
							error_z1 = error;
						}
						else
						{
							//LAZO I
							LEDV_ON;

							if (undersampling)
								undersampling--;
							else
							{
								//undersampling = 10;		//funciona bien pero con saltos
								undersampling = 20;		//funciona bien pero con saltos

#ifdef WITH_POTE
								//con control por pote
								medida = MAX_I * pote_value;		//con filtro
								medida >>= 10;
								error = medida - Iout_Sense;	//340 es 1V en adc
#endif
#ifdef WITHOUT_POTE
								error = MAX_I - Iout_Sense;	//340 es 1V en adc
#endif


								acc = K1I * error;		//5500 / 32768 = 0.167 errores de hasta 6 puntos
								val_k1 = acc >> 7;

								//K2
								acc = K2I * error_z1;		//K2 = no llega pruebo con 1
								val_k2 = acc >> 7;			//si es mas grande que K1 + K3 no lo deja arrancar

								//K3
								acc = K3I * error_z2;		//K3 = 0.4
								val_k3 = acc >> 7;

								d = d + val_k1 - val_k2 + val_k3;
								if (d < 0)
									d = 0;
								else if (d > DMAX)		//no me preocupo si estoy con folding
									d = DMAX;		//porque d deberia ser chico

								//Update variables PID
								error_z2 = error_z1;
								error_z1 = error;
							}
						}
					}
#ifdef WITH_POTE
					pote_value = MAFilter8 (One_Ten_Pote, v_pote_samples);
#endif

					if (OUTPUT_ENABLE)
						Update_Boost (d);
					else
						Update_Boost (0);

					//Update_TIM3_CH2 (Iout_Sense);	//muestro en pata PA7 el sensado de Iout

					//pote_value = MAFilter32Circular (One_Ten_Pote, v_pote_samples, p_pote, &pote_sumation);
					//pote_value = MAFilter32 (One_Ten_Pote, v_pote_samples);
					//pote_value = MAFilter8 (One_Ten_Pote, v_pote_samples);
					//pote_value = MAFilter32Pote (One_Ten_Pote);
					seq_ready = 0;
					LEDV_OFF;

					//reviso si necesito un cambio desde modo BOOST
					medida = Vout_Sense * 120;
					medida >>= 7;
//					if ((Vin_Sense >= Vout_Sense) &&
					if ((Vin_Sense >= medida) &&
							(d < DMIN_THRESH)	&&
							(Iout_Sense > IBOOST_THRESH))
					{
						if (change_mode_counter < CHANGE_MODE_THRESH)
							change_mode_counter++;
						else
						{
							change_mode_counter = 0;
							converter_mode = BUCK_MODE;
							Update_Buck(0);
							Update_Boost(0);
							d = 0;
							error_z1 = 0;
							error_z2 = 0;
							undersampling = 0;
						}
					}
					else if (change_mode_counter)
						change_mode_counter--;


					break;

				default:
					break;
			}
#endif
#ifdef BOOST_WITH_CONTROL
			if ((I_Sense > MAX_I_MOSFET) || (Vin_Sense < MIN_VIN))

			{
				//corto el ciclo
				d = 0;
			}
			else
			{
				//VEO SI USO LAZO V O I
				if (Vout_Sense > SP_VOUT)
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
					undersampling--;
					if (!undersampling)
					{
						//undersampling = 10;		//funciona bien pero con saltos
						undersampling = 20;		//funciona bien pero con saltos


						//con control por pote
//						medida = MAX_I * One_Ten_Pote;	//sin filtro
						medida = MAX_I * pote_value;		//con filtro
						medida >>= 10;
//						if (medida < 26)
//							medida = 26;
//						Iout_Sense >>= 1;
						error = medida - Iout_Sense;	//340 es 1V en adc
//						error = MAX_I - Iout_Sense;	//340 es 1V en adc
//						error = 24 - Iout_Sense;	//en 55mA esta inestable


//						if (Iout_Sense > 62)
//						{
							acc = K1V * error;		//5500 / 32768 = 0.167 errores de hasta 6 puntos
							val_k1 = acc >> 7;
							//val_k1 = acc >> 15;

							//K2
							acc = K2V * error_z1;		//K2 = no llega pruebo con 1
							val_k2 = acc >> 7;			//si es mas grande que K1 + K3 no lo deja arrancar
							//val_k2 = acc >> 15;

							//K3
							acc = K3V * error_z2;		//K3 = 0.4
							val_k3 = acc >> 7;
							//val_k3 = acc >> 15;
//						}
//						//else if (Iout_Sense < 52)
//						else
//						{
//							acc = 513 * error;		//5500 / 32768 = 0.167 errores de hasta 6 puntos
//							val_k1 = acc >> 7;
//							//val_k1 = acc >> 15;
//
//							//K2
//							acc = 512 * error_z1;		//K2 = no llega pruebo con 1
//							val_k2 = acc >> 7;			//si es mas grande que K1 + K3 no lo deja arrancar
//							//val_k2 = acc >> 15;
//
//							//K3
//							acc = K3V * error_z2;		//K3 = 0.4
//							val_k3 = acc >> 7;
//							//val_k3 = acc >> 15;
//						}

						d = d + val_k1 - val_k2 + val_k3;
						if (d < 0)
							d = 0;
						else if (d > DMAX)		//no me preocupo si estoy con folding
								d = DMAX;		//porque d deberia ser chico

						//Update variables PID
						error_z2 = error_z1;
						error_z1 = error;
					}
				}
			}

//			if (d != d_last)
//			{
//				Update_TIM3_CH1 (d);
//				d_last = d;
//			}
			pote_value = MAFilter8 (One_Ten_Pote, v_pote_samples);
			Update_TIM3_CH1 (d);

			//Update_TIM3_CH2 (Iout_Sense);	//muestro en pata PA7 el sensado de Iout

			//pote_value = MAFilter32Circular (One_Ten_Pote, v_pote_samples, p_pote, &pote_sumation);
			//pote_value = MAFilter32 (One_Ten_Pote, v_pote_samples);
			//pote_value = MAFilter8 (One_Ten_Pote, v_pote_samples);
			//pote_value = MAFilter32Pote (One_Ten_Pote);
			seq_ready = 0;
			LED_OFF;
#endif
		}	//fin seq_ready
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





