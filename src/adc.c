/*
 * adc.c
 *
 *  Created on: 04/05/2015
 *      Author: Mariano
 */
#include "adc.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx.h"
#include "hard.h"


//--- VARIABLES EXTERNAS ---//
extern volatile unsigned short adc_ch [];

extern volatile unsigned char seq_ready;

//--- VARIABLES GLOBALES ---//
volatile unsigned short * p_channel;

//volatile unsigned char channel_conv = 0;


unsigned short ADC_Conf (void)
{
	unsigned short cal = 0;
	ADC_InitTypeDef ADC_InitStructure;

	if (!RCC_ADC_CLK)
		RCC_ADC_CLK_ON;

	ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv4);

	// preseteo de registros a default
	  /* ADCs DeInit */
	  ADC_DeInit(ADC1);

	  /* Initialize ADC structure */
	  ADC_StructInit(&ADC_InitStructure);

	  /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	  ADC_Init(ADC1, &ADC_InitStructure);

	//software by setting bit ADCAL=1.
	//Calibration can only be initiated when the ADC is disabled (when ADEN=0).
	//ADCAL bit stays at 1 during all the calibration sequence.
	//It is then cleared by hardware as soon the calibration completes
	cal = ADC_GetCalibrationFactor(ADC1);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	SetADC1_SampleTime ();

	return cal;
}

void AdcConfig (void)
{
#ifdef ADC_WITH_INT
	NVIC_InitTypeDef    NVIC_InitStructure;
#endif
	unsigned short cal = 0;

	if (!RCC_ADC_CLK)
		RCC_ADC_CLK_ON;

	// preseteo los registros a default, la mayoria necesita tener ADC apagado
	ADC1->CR = 0x00000000;
	ADC1->IER = 0x00000000;
	ADC1->CFGR1 = 0x00000000;
	ADC1->CFGR2 = 0x00000000;
	ADC1->SMPR = 0x00000000;
	ADC1->TR = 0x0FFF0000;
	ADC1->CHSELR = 0x00000000;

	//set clock
	ADC1->CFGR2 = ADC_ClockMode_SynClkDiv4;

	//set resolution & trigger
#ifdef BUCK_BOOST_WITH_CONTROL
	ADC1->CFGR1 |= ADC_Resolution_10b | ADC_ExternalTrigConvEdge_Rising | ADC_ExternalTrigConv_T1_TRGO;
#endif
#if ((defined BOOST_WITH_CONTROL) || (defined BOOST_CONVENCIONAL))
	ADC1->CFGR1 |= ADC_Resolution_10b | ADC_ExternalTrigConvEdge_Rising | ADC_ExternalTrigConv_T3_TRGO;
#endif

	//set sampling time
	ADC1->SMPR |= ADC_SampleTime_28_5Cycles;		//
	//ADC1->SMPR |= ADC_SampleTime_7_5Cycles;		//
	//ADC1->SMPR |= ADC_SampleTime_1_5Cycles;		//

	//set channel selection siempre convierte creciente, termina con el canal mas alto de la secuencia
	//ADC1->CHSELR |= ADC_Channel_0 | ADC_Channel_1 | ADC_Channel_2;
	//ADC1->CHSELR |= ADC_Channel_1 | ADC_Channel_2;
#ifdef BOOST_CONVENCIONAL
	ADC1->CHSELR |= ADC_Channel_0 | ADC_Channel_1 | ADC_Channel_2 | ADC_Channel_9;		//modificado 13-07-16
																					//Iout_Sense 	CH0
																					//Vin_Sense		CH1
																					//I_Sense		CH2
																					//Vout_Sense 	CH9
#endif

#ifdef BOOST_WITH_CONTROL
	ADC1->CHSELR |= ADC_Channel_0 | ADC_Channel_1 | ADC_Channel_2;		//modificado 20-07-16
	ADC1->CHSELR |= ADC_Channel_4 | ADC_Channel_5 | ADC_Channel_6;		//Vin_Sense		CH0
																		//Iout_Sense 	CH1
																		//I_Sense		CH2
																		//One_Ten_Sense	CH4
																		//One_Ten_Pote 	CH5
																		//Vout_Sense 	CH6
#endif

#ifdef BUCK_BOOST_WITH_CONTROL
	ADC1->CHSELR |= ADC_Channel_0 | ADC_Channel_1 | ADC_Channel_2;		//modificado 10-08-16
	ADC1->CHSELR |= ADC_Channel_3 | ADC_Channel_4 | ADC_Channel_5;		//Vin_Sense		CH0
	ADC1->CHSELR |= ADC_Channel_6;										//Iout_Sense 	CH1
																		//Boost_Sense	CH2
																		//Buck_Sense	CH3
																		//One_Ten_Sense	CH4
																		//One_Ten_Pote 	CH5
																		//Vout_Sense 	CH6
#endif

#ifdef ADC_WITH_INT
	//set interrupts
	ADC1->IER |= ADC_IT_EOC;

	/* Configure and enable ADC1 interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	//calibrar ADC
	cal = ADC_GetCalibrationFactor(ADC1);

	p_channel = &adc_ch[0];
	// Enable ADC1
	ADC1->CR |= ADC_CR_ADEN;
}

#ifdef ADC_WITH_INT
void ADC1_COMP_IRQHandler (void)
{
	if (ADC1->ISR & ADC_IT_EOC)
	{
#ifdef BUCK_BOOST_WITH_CONTROL
		if (ADC1->ISR & ADC_IT_EOSEQ)	//seguro que es channel6
		{
			p_channel = &adc_ch[6];
			*p_channel = ADC1->DR;
			p_channel = &adc_ch[0];
			seq_ready = 1;
		}
		else
		{
			*p_channel = ADC1->DR;		//
			if (p_channel < &adc_ch[6])
				p_channel++;
		}
#endif

#ifdef BOOST_CONVENCIONAL
		if (ADC1->ISR & ADC_IT_EOSEQ)	//seguro que es channel9
		{
			p_channel = &adc_ch[3];
			*p_channel = ADC1->DR;
			p_channel = &adc_ch[0];
			seq_ready = 1;
		}
		else
		{
			*p_channel = ADC1->DR;		//
			if (p_channel < &adc_ch[3])
				p_channel++;
		}
#endif

#ifdef BOOST_WITH_CONTROL
		if (ADC1->ISR & ADC_IT_EOSEQ)	//seguro que es channel6
		{
			p_channel = &adc_ch[5];
			*p_channel = ADC1->DR;
			p_channel = &adc_ch[0];
			seq_ready = 1;
		}
		else
		{
			*p_channel = ADC1->DR;		//
			if (p_channel < &adc_ch[5])
				p_channel++;
		}
#endif

		//clear pending
		ADC1->ISR |= ADC_IT_EOC | ADC_IT_EOSEQ;
	}
}
#endif

unsigned short ReadADC1 (unsigned int channel)
{
	uint32_t tmpreg = 0;
	//GPIOA_PIN4_ON;
	// Set channel and sample time
	//ADC_ChannelConfig(ADC1, channel, ADC_SampleTime_7_5Cycles);	//pifia la medicion 2800 o 3400 en ves de 4095
	//ADC_ChannelConfig(ADC1, channel, ADC_SampleTime_239_5Cycles);
	//ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);

	//ADC_ChannelConfig INTERNALS
	/* Configure the ADC Channel */
	ADC1->CHSELR = channel;

	/* Clear the Sampling time Selection bits */
	tmpreg &= ~ADC_SMPR1_SMPR;

	/* Set the ADC Sampling Time register */
	tmpreg |= (uint32_t)ADC_SampleTime_239_5Cycles;

	/* Configure the ADC Sample time register */
	ADC1->SMPR = tmpreg ;


	// Start the conversion
	ADC_StartOfConversion(ADC1);
	// Wait until conversion completion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	// Get the conversion value
	//GPIOA_PIN4_OFF;	//tarda 20us en convertir
	return ADC_GetConversionValue(ADC1);
}

//Setea el sample time en el ADC
void SetADC1_SampleTime (void)
{
	uint32_t tmpreg = 0;

	/* Clear the Sampling time Selection bits */
	tmpreg &= ~ADC_SMPR1_SMPR;

	/* Set the ADC Sampling Time register */
	tmpreg |= (uint32_t)ADC_SampleTime_239_5Cycles;

	/* Configure the ADC Sample time register */
	ADC1->SMPR = tmpreg ;
}


//lee el ADC sin cambiar el sample time anterior
unsigned short ReadADC1_SameSampleTime (unsigned int channel)
{
	// Configure the ADC Channel
	ADC1->CHSELR = channel;

	// Start the conversion
	ADC1->CR |= (uint32_t)ADC_CR_ADSTART;

	// Wait until conversion completion
	while((ADC1->ISR & ADC_ISR_EOC) == 0);

	// Get the conversion value
	return (uint16_t) ADC1->DR;
}

unsigned short ReadADC1Check (unsigned char channel)
{
	if (ADC1->CR & 0x01)			//reviso ADEN
		return 0xFFFF;

	//espero que este listo para convertir
	while ((ADC1->ISR & 0x01) == 0);	//espero ARDY = 1

	if ((ADC1->CFGR1 & 0x00010000) == 0)			//reviso DISCONTINUOS = 1
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00002000)					//reviso CONT = 0
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00000C00)					//reviso TRIGGER = 00
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00000020)					//reviso ALIGN = 0
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00000018)					//reviso RES = 00
		return 0xFFFF;

	//espero que no se este convirtiendo ADCSTART = 0
	while ((ADC1->CR & 0x02) != 0);	//espero ADCSTART = 0

	ADC1->CHSELR = 0x00000001;	//solo convierto CH0

	return 1;
}

