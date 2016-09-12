/*
 * hard.h
 *
 *  Created on: 28/11/2013
 *      Author: Mariano
 */

#ifndef HARD_H_
#define HARD_H_

//-------- Board Configuration -----------------------//
//#define VER_1_0
//#define VER_1_1
//#define VER_1_2
#define VER_2_0

//#define WITH_POTE
#define WITHOUT_POTE


#ifdef VER_1_0
#define BOOST_CONVENCIONAL	//placa anterior del tamaño de la F12V5A ultimo prog 13-07-16
#endif
#ifdef VER_1_1
#define BOOST_WITH_CONTROL		//placa nueva con control 1 a 10V o pote 20-07-16
#endif
#if (defined VER_2_0) || (defined VER_1_2)
#define BUCK_BOOST_WITH_CONTROL		//placa mas nueva con control 1 a 10V o pote + fuente buck 10-08-16
#endif

#ifdef VER_2_0
//GPIOA pin0
//GPIOA pin1
//GPIOA pin2
//GPIOA pin3
//GPIOA pin4
//GPIOA pin5
//GPIOA pin6		//Analog Inputs

//GPIOA pin7

//GPIOA pin8
#define LEDR ((GPIOA->ODR & 0x0100) != 0)
#define LEDR_ON	GPIOA->BSRR = 0x00000100
#define LEDR_OFF GPIOA->BSRR = 0x01000000

//GPIOA pin9
#define LEDV ((GPIOA->ODR & 0x0200) != 0)
#define LEDV_ON	GPIOA->BSRR = 0x00000200
#define LEDV_OFF GPIOA->BSRR = 0x02000000

//GPIOA pin10
//GPIOA pin11
//GPIOA pin12
//GPIOA pin13
//GPIOA pin14
//GPIOA pin15

//GPIOB pin0

//GPIOB pin1
#define JUMPER_CONF ((GPIOB->IDR & 0x0002) == 0)
#define OUTPUT_ENABLE	(!JUMPER_CONF)
//GPIOB pin3

//GPIOB pin4	//TIM3_CH1 para control PWM CTRL BUCK
//GPIOB pin5	//TIM3_CH2 para control PWM CTRL BOOST

//GPIOB pin6
//GPIOB pin7

#endif

#ifdef VER_1_2
//GPIOA pin0
//GPIOA pin1
//GPIOA pin2
//GPIOA pin3
//GPIOA pin4
//GPIOA pin5
//GPIOA pin6		//Analog Inputs

//GPIOA pin7
#define LEDR ((GPIOA->ODR & 0x0080) != 0)
#define LEDR_ON	GPIOA->BSRR = 0x00000080
#define LEDR_OFF GPIOA->BSRR = 0x00800000

//GPIOA pin8
//GPIOA pin9
//GPIOA pin10
//GPIOA pin11
//GPIOA pin12
//GPIOA pin13
//GPIOA pin14
//GPIOA pin15

//GPIOB pin0
#define LEDV ((GPIOB->ODR & 0x0001) != 0)
#define LEDV_ON	GPIOB->BSRR = 0x00000001
#define LEDV_OFF GPIOB->BSRR = 0x00010000

//GPIOB pin1
#define JUMPER_CONF ((GPIOB->IDR & 0x0002) == 0)
#define OUTPUT_ENABLE	(!JUMPER_CONF)
//GPIOB pin3

//GPIOB pin4	//TIM3_CH1 para control PWM
//GPIOB pin5	//TIM3_CH2 para control PWM

//GPIOB pin6
//GPIOB pin7

#endif

#ifdef VER_1_0
//GPIOA pin0
//GPIOA pin1
//GPIOA pin2		//Analog Inputs

//GPIOA pin3
//GPIOA pin4
//GPIOA pin5
//GPIOA pin6
//TIM3 alternate function

//GPIOA pin7
//GPIOA pin8
//GPIOA pin9
//GPIOA pin10
#define LED ((GPIOA->ODR & 0x0400) != 0)
#define LED_ON	GPIOA->BSRR = 0x00000400
#define LED_OFF GPIOA->BSRR = 0x04000000

//GPIOA pin11
//GPIOA pin12
//GPIOA pin13
//GPIOA pin14
//GPIOA pin15

//GPIOB pin0

//GPIOB pin1		//Analog Input

//GPIOB pin3
//GPIOB pin4
//GPIOB pin5
//GPIOB pin6
//GPIOB pin7
#endif

#ifdef VER_1_1
//GPIOA pin0
//GPIOA pin1
//GPIOA pin2		//Analog Inputs

//GPIOA pin3

//GPIOA pin4
//GPIOA pin5
//GPIOA pin6		//Analog Inputs

//GPIOA pin7
//GPIOA pin8
//GPIOA pin9
//GPIOA pin10
//GPIOA pin11
//GPIOA pin12
//GPIOA pin13
//GPIOA pin14
//GPIOA pin15

//GPIOB pin0
#define LED ((GPIOB->ODR & 0x0001) != 0)
#define LED_ON	GPIOB->BSRR = 0x00000001
#define LED_OFF GPIOB->BSRR = 0x00010000

//GPIOB pin1
#define JUMPER_CONF ((GPIOB->IDR & 0x0002) == 0)

//GPIOB pin3

//GPIOB pin4	//TIM3_CH1 para control PWM

//GPIOB pin5

//GPIOB pin6
#define MOSFET ((GPIOB->ODR & 0x0040) != 0)
#define MOSFET_ON	GPIOB->BSRR = 0x00000040
#define MOSFET_OFF GPIOB->BSRR = 0x00400000


//GPIOB pin7


#endif

//ESTADOS DEL PROGRAMA PRINCIPAL
#define BOOST_MODE	0
#define BUCK_MODE	1

//---- Temperaturas en el LM335
//37	2,572
//40	2,600
//45	2,650
//50	2,681
//55	2,725
//60	2,765
#define TEMP_IN_30		3226
#define TEMP_IN_35		3279
#define TEMP_IN_50		3434
#define TEMP_IN_65		3591


//ESTADOS DEL DISPLAY
#define DISPLAY_INIT		0
#define DISPLAY_SENDING		1
#define DISPLAY_SHOWING		2
#define DISPLAY_WAITING		3

#define DISPLAY_ZERO	10
#define DISPLAY_POINT	11
#define DISPLAY_LINE	12
#define DISPLAY_REMOTE	13
#define DISPLAY_PROG	14
#define DISPLAY_NONE	0xF0

#define DISPLAY_DS3		0x04
#define DISPLAY_DS2		0x02
#define DISPLAY_DS1		0x01

#define DISPLAY_TIMER_RELOAD	6		//166Hz / 3
#define SWITCHES_TIMER_RELOAD	10

#define SWITCHES_THRESHOLD_FULL	300		//3 segundos
#define SWITCHES_THRESHOLD_HALF	100		//1 segundo
#define SWITCHES_THRESHOLD_MIN	5		//50 ms

#define TIMER_STANDBY_TIMEOUT	6000	//6 segundos
#define DMX_DISPLAY_SHOW_TIMEOUT	30000	//30 segundos

#define S_FULL		10
#define S_HALF		3
#define S_MIN		1
#define S_NO		0

#define FUNCTION_DMX	1
#define FUNCTION_MAN	2

#endif /* HARD_H_ */
