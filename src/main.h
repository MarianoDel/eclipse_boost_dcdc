/**
  ******************************************************************************
  * @file    Template_2/main.h
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx_adc.h"
//#include "stm32f0xx_can.h"
//#include "stm32f0xx_cec.h"
//#include "stm32f0xx_comp.h"
//#include "stm32f0xx_crc.h"
//#include "stm32f0xx_crs.h"
//#include "stm32f0xx_dac.h"
//#include "stm32f0xx_dbgmcu.h"
//#include "stm32f0xx_dma.h"
//#include "stm32f0xx_exti.h"
//#include "stm32f0xx_flash.h"
#include "stm32f0xx_gpio.h"
//#include "stm32f0xx_i2c.h"
//#include "stm32f0xx_iwdg.h"
#include "stm32f0xx_misc.h"
//#include "stm32f0xx_pwr.h"
#include "stm32f0xx_rcc.h"
//#include "stm32f0xx_rtc.h"
#include "stm32f0xx_spi.h"
//#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"
//#include "stm32f0xx_wwdg.h"
#include "system_stm32f0xx.h"
#include "stm32f0xx_it.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//--- My includes ---//
#include "stm32f0x_gpio.h"
#include "stm32f0x_tim.h"


//--- Exported types ---//
//--- Exported constants ---//
//--- Exported macro ---//
enum end_states
{
	WORKING = 1,
	END_OK,
	END_ERROR,
	END_TIMEOUT
};
//--- Exported functions ---//

//ESTADOS DEL PROGRAMA PRINCIPAL
#define STAND_BY	0
#define TX_LCD		1
#define TX_LCD2		2
#define TX_SERIE	3
#define TX_SERIE2	4
#define RX_SERIE	5
#define TEMP_SENSE	6
#define TX_SERIE_NC		7
#define TX_SERIE2_NC	8
#define CONNECT	9


//ESTADOS DEL LED DE COMUNICACION
//#define NO_COMM		0
//#define TX_NO_COMM	1
//#define TX_COMM			2
//#define COMM		3

//FUNCIONES DEL MAIN QUE USAN OTROS MODULOS???
void TimingDelay_Decrement(void);



/* Uncomment the line below if you will use the SPI peripheral as a Master */
#define SPI_MASTER
/* Uncomment the line below if you will use the SPI peripheral as a Slave */
//#define SPI_SLAVE

/* Uncomment the size of data to be transmetted (only one data size must be selected) */
#define SPI_DATASIZE_8
//#define SPI_DATASIZE_7
//#define SPI_DATASIZE_6
//#define SPI_DATASIZE_5


/* USE//R_TIMEOUT value for waiting loops. This timeout is just guarantee that the
   application will not remain stuck if the USART communication is corrupted.
   You may modify this timeout value depending on CPU frequency and application
   conditions (interrupts routines, number of data to transfer, baudrate, CPU
   frequency...). */
#define USER_TIMEOUT                    ((uint32_t)0x64) /* Waiting 1s */

/* Communication boards SPIx Interface */
#define SPIx                             SPI1
#define SPIx_CLK                         RCC_APB2Periph_SPI1
#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler

#ifdef USE_STM320518_EVAL
  #define SPIx_SCK_PIN                     GPIO_Pin_5
  #define SPIx_SCK_GPIO_PORT               GPIOA
  #define SPIx_SCK_GPIO_CLK                RCC_AHBPeriph_GPIOA
  #define SPIx_SCK_SOURCE                  GPIO_PinSource5
  #define SPIx_SCK_AF                      GPIO_AF_0

  #define SPIx_MISO_PIN                    GPIO_Pin_4
  #define SPIx_MISO_GPIO_PORT              GPIOB
  #define SPIx_MISO_GPIO_CLK               RCC_AHBPeriph_GPIOB
  #define SPIx_MISO_SOURCE                 GPIO_PinSource4
  #define SPIx_MISO_AF                     GPIO_AF_0

  #define SPIx_MOSI_PIN                    GPIO_Pin_7
  #define SPIx_MOSI_GPIO_PORT              GPIOA
  #define SPIx_MOSI_GPIO_CLK               RCC_AHBPeriph_GPIOA
  #define SPIx_MOSI_SOURCE                 GPIO_PinSource7
  #define SPIx_MOSI_AF                     GPIO_AF_0
#else
  #define SPIx_SCK_PIN                     GPIO_Pin_3
  #define SPIx_SCK_GPIO_PORT               GPIOB
  #define SPIx_SCK_GPIO_CLK                RCC_AHBPeriph_GPIOB
  #define SPIx_SCK_SOURCE                  GPIO_PinSource3
  #define SPIx_SCK_AF                      GPIO_AF_0

  #define SPIx_MISO_PIN                    GPIO_Pin_4
  #define SPIx_MISO_GPIO_PORT              GPIOB
  #define SPIx_MISO_GPIO_CLK               RCC_AHBPeriph_GPIOB
  #define SPIx_MISO_SOURCE                 GPIO_PinSource4
  #define SPIx_MISO_AF                     GPIO_AF_0

  #define SPIx_MOSI_PIN                    GPIO_Pin_5
  #define SPIx_MOSI_GPIO_PORT              GPIOB
  #define SPIx_MOSI_GPIO_CLK               RCC_AHBPeriph_GPIOB
  #define SPIx_MOSI_SOURCE                 GPIO_PinSource5
  #define SPIx_MOSI_AF                     GPIO_AF_0
#endif /* USE_STM320518_EVAL */

//#define TXBUFFERSIZE                     (countof(TxBuffer) - 1)
#define TXBUFFERSIZE                     16
#define RXBUFFERSIZE                     TXBUFFERSIZE

/* Define data size and data masks */
#ifdef SPI_DATASIZE_8
 #define SPI_DATASIZE                     SPI_DataSize_8b
 #define SPI_DATAMASK                     (uint8_t)0xFF
#elif defined (SPI_DATASIZE_7)
 #define SPI_DATASIZE                     SPI_DataSize_7b
 #define SPI_DATAMASK                     (uint8_t)0x7F
#elif defined (SPI_DATASIZE_6)
 #define SPI_DATASIZE                     SPI_DataSize_6b
 #define SPI_DATAMASK                     (uint8_t)0x3F
#elif defined (SPI_DATASIZE_5)
 #define SPI_DATASIZE                     SPI_DataSize_5b
 #define SPI_DATAMASK                     (uint8_t)0x1F
#else
 #error " Select at least one data size in main.h file "
#endif

#endif //--- End ---//


//--- END OF FILE ---//
