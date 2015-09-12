/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������
 * �ļ���  ��ANO_Drv_LED.cpp
 * ����    ��LED
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "ANO_Drv_LED.h"

ANO_LED led;

void ANO_LED::Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(ANO_RCC_LED1,ENABLE);
	RCC_APB2PeriphClockCmd(ANO_RCC_LED2,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1;
	GPIO_Init(ANO_GPIO_LED1, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED2;
	GPIO_Init(ANO_GPIO_LED2, &GPIO_InitStructure);
}

void ANO_LED::ON1(void)
{
	GPIO_SetBits(ANO_GPIO_LED1, ANO_Pin_LED1);			
}

void ANO_LED::ON2(void)
{		
	GPIO_SetBits(ANO_GPIO_LED2, ANO_Pin_LED2);	
}

void ANO_LED::OFF1(void)
{
	GPIO_ResetBits(ANO_GPIO_LED1, ANO_Pin_LED1);	
}

void ANO_LED::OFF2(void)
{
	GPIO_ResetBits(ANO_GPIO_LED2, ANO_Pin_LED2);		
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

