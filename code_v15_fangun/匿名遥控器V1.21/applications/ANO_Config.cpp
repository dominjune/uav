/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������ƴ�
 * �ļ���  ��ANO_Config.cpp
 * ����    �������ļ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/

#include "ANO_Config.h"

ANO_Config ano;


ANO_Config::ANO_Config(void)
{
	f.STICKMODE = 1;
}

//ָʾ��
void ANO_Config::Pilot_Light(void)
{
	static u8 cnt = 0;
	
	if(f.ACCELMODE)	// ���ģʽACCELMODE
	{
		cnt++;
		switch(cnt)
		{
			case 1:
				led.ON();
				break;
			case 2:
				led.OFF();
				break;	
			case 4:
				cnt = 0;
				break;
		}
	}
	else	//��ͨģʽSTICKMODE
	{
		led.ON(); 
	}
	
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
