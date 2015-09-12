/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������,�費˼
 * �ļ���  ��AnoCopter.cpp
 * ����    ��������ս��V3΢�����������
 * ����汾��V1.0
 * ʱ��		 ��2015/9/6
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "ANO_Config.h"

int main(void)
{
	//��ʼ���ɿذ��Ӳ������
	ANO_Hexacopter_board_Init();
	
	//��ʼ������
	param.Init();
	
	//��ʼ��IMU�����Բ�����Ԫ��
	imu.Init();	
	
	nav.Init();
	
	while(1)
	{
		ANO_Loop();
	}
	
	return 0;
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
