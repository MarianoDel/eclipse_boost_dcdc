/*
 * adc.h
 *
 *  Created on: 10/09/2014
 *      Author: Mariano
 */

#ifndef ADC_H_
#define ADC_H_

#define RCC_ADC_CLK 		(RCC->APB2ENR & 0x00000200)
#define RCC_ADC_CLK_ON 		RCC->APB2ENR |= 0x00000200
#define RCC_ADC_CLK_OFF 	RCC->APB2ENR &= ~0x00000200

#define ADC_CH0		0x00000001
#define ADC_CH1		0x00000002
#define ADC_CH2		0x00000004

#define ADC_WITH_INT

unsigned short ADC_Conf (void);
unsigned short ReadADC1 (unsigned int);
unsigned short ReadADC1_SameSampleTime (unsigned int);
void SetADC1_SampleTime (void);
unsigned short ReadADC1Check (unsigned char);
void ADC1_COMP_IRQHandler (void);
void AdcConfig (void);

#endif /* ADC_H_ */
