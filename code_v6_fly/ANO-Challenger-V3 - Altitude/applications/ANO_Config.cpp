/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������
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
	f.ANGLE_MODE = 1;
}

//ָʾ��
void ANO_Config::Pilot_Light(void)
{
	static u8 cnt = 0,cnt2 = 0;
	
	
	if(f.ARMED && !f.ALTHOLD)
	{
		cnt++;
		switch(cnt)
		{
			case 1:
				led.ON1();
				break;
			case 20:
				led.OFF1();
				break;
			case 40:
				cnt = 0;
				break;			
		}
	}
	else if(f.ARMED && f.ALTHOLD)
	{
		cnt2++;		
		switch(cnt2)
		{
			case 1:
				led.ON1();
				led.OFF2();
				break;
			case 20:
				led.OFF1();
				led.ON2();
				break;
			case 40:
				cnt2 = 0;
				break;			
		}
	}
	else
	{
		led.ON1(); 
		led.ON2(); 
	}
	
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
