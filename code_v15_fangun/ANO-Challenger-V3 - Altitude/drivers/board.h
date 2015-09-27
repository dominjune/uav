
#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f10x.h"
#include "string.h"
#include "ANO_Math.h"

#define ARMAPI extern "C"

/***************LED GPIO����******************/
#define ANO_RCC_LED1			RCC_APB2Periph_GPIOC
#define ANO_GPIO_LED1		GPIOC
#define ANO_Pin_LED1			GPIO_Pin_14
#define ANO_RCC_LED2		RCC_APB2Periph_GPIOC
#define ANO_GPIO_LED2		GPIOC
#define ANO_Pin_LED2		GPIO_Pin_13
/*********************************************/
/***************I2C GPIO����******************/
#define ANO_GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_6
#define I2C_Pin_SDA		GPIO_Pin_7
#define ANO_RCC_I2C		RCC_APB2Periph_GPIOB
/*********************************************/
/***************SPI2 GPIO����******************/
#define ANO_GPIO_SPI2		GPIOB
#define ANO_GPIO_CE2		GPIOA
#define SPI2_Pin_SCK		GPIO_Pin_13
#define SPI2_Pin_MISO		GPIO_Pin_14
#define SPI2_Pin_MOSI		GPIO_Pin_15
#define SPI2_Pin_CE2		GPIO_Pin_8
#define SPI2_Pin_CSN		GPIO_Pin_12
#define RCC_GPIO_SPI2		RCC_APB2Periph_GPIOB
#define RCC_GPIO_CE2		RCC_APB2Periph_GPIOA
/*********************************************/

/***************Ӳ���ж����ȼ�******************/
#define NVIC_UART_P	5
#define NVIC_UART_S	1
#define NVIC_TIM4_P	2
#define NVIC_TIM4_S	1
/***********************************************/

#include "ANO_Config.h"
#include "ANO_Drv_LED.h"
#include "ANO_Drv_SPI2.h"
#include "ANO_Drv_I2C_soft.h"
#include "ANO_Drv_MPU6050.h"
#include "ANO_Drv_MS5611.h"
#include "ANO_Drv_Nrf24l01.h"
#include "ANO_Drv_Uart3.h"
#include "ANO_Drv_PWM.h"
#include "ANO_Drv_EEPROM.h"

void ANO_Hexacopter_board_Init(void);

void SysTick_IRQ(void);

uint32_t GetSysTime_us(void);
void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);

#endif /* __BOARD_H__ */

// 




