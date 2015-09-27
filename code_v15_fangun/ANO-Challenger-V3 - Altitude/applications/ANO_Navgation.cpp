/******************** (C) COPYRIGHT 2014 ANO Tech *******************************
 * ����		 �������
 * �ļ���  ��ANO_Navgation.cpp
 * ����    ����������ϵ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "ANO_Navgation.h"
#include <math.h>

ANO_Navgation nav;

ANO_Navgation::ANO_Navgation()
{

}

void ANO_Navgation::Init(void)
{
	filter_Init();
	sensor_Init();
}

void ANO_Navgation::update(void)
{
	static int baroCnt = 0, ultraCnt = 0;
	static uint32_t lastTimeInertia, lastTimeBaro, lastTimeUltra, lastUltraAlt;
	static uint8_t ultraSameCnt = 0;
	float	deltaT;
	
	
	//����US100����������ź�
	if(ultraCnt == 10 || uart3.RxState3 == 2){	
			uart3.RxState3 = 0;
			uart3.Put_Char(0x55);		
			ultraCnt = 0;		

			deltaT = (GetSysTime_us() - lastTimeUltra) * 1e-6;
			lastTimeUltra = GetSysTime_us();
		
			//��ȡ�������߶Ȳ���ͨ�˲�
			ultraAlt = Ultra_LPF_2nd(&ultra_lpf_2nd, (float)ultraAltRaw);	
			//���㳬�����仯�ٶ�
			ultraVel = (ultraAlt - lastUltraAlt) / deltaT;
			lastUltraAlt = ultraAlt;
			//������ѹ�仯���ٶȣ�����800cm/s
			ultraVel = constrain_int32(ultraVel, -8000, 8000);    
			//����������С����
			ultraVel = applyDeadband(ultraVel, 100); 		
		
			if(ultraAltRaw == lastUltraAlt)
				ultraSameCnt++;
			else
				ultraSameCnt = 0;
			
			lastUltraAlt = ultraAltRaw;
			
			if(ultraAlt > 0 && ultraAlt < 1000 && ultraSameCnt < 30){
				ULTRA_IS_OK = 1;
			}
			else{
				ULTRA_IS_OK = 0;
			}
			
	}	
	ultraCnt++;
	
	if(baroCnt == 5){
		//��ѹ���������ݸ���
		ms5611.Update();
	}
	else if(baroCnt == 10){
		deltaT = (GetSysTime_us() - lastTimeBaro) * 1e-6;
		lastTimeBaro = GetSysTime_us();
		baroCnt = 0;
		
		//��ѹ���������ݸ���
		ms5611.Update();
		
		//��ȡ��ѹ�߶Ȳ���ͨ�˲�����У��ƫ��
		baroAlt = Baro_LPF_2nd(&baro_lpf_2nd, (float)ms5611.Get_BaroAlt());	
		
		//������ѹ�仯�ٶ�
		baroVel = (baroAlt -lastBaroAlt) / deltaT;
		lastBaroAlt = baroAlt;
		//������ѹ�仯���ٶȣ�����800cm/s
    baroVel = constrain_int32(baroVel, -800, 800);    
		//����������С����
    baroVel = applyDeadband(baroVel, 10);    

	}
	baroCnt++;
	
	
	deltaT = (GetSysTime_us() - lastTimeInertia) * 1e-6;
	lastTimeInertia = GetSysTime_us();
	
	accel.z= imu.Get_Accel_Ef().z;
	accel.z = (accel.z - ACC_1G) * 9.7925f * 100 / ACC_1G; 
	//acc.z *= accVelScale;
	velocity.z += accel.z * deltaT;
	position.z += velocity.z * deltaT;
	//float cf_factor = 0.4;
	velocity.z = baroVel * 0.005 + velocity.z * (1 - 0.005);
//	nav.position.z
	position.z = baroAlt * 0.005 + position.z * (1 - 0.005);
}

void ANO_Navgation::Reset(void)
{
	accel(0,0,0);
	velocity(0,0,0);
	position(0,0,0);
}


void ANO_Navgation::filter_Init()
{
	LPF_2nd_Factor_Cal(BARO_LOOP_TIME * 1e-6, BARO_LPF_CUT, &baro_lpf_2nd);
	
	LPF_2nd_Factor_Cal(ULTRA_LOOP_TIME * 1e-6, ULTRA_LPF_CUT, &ultra_lpf_2nd);
}

void ANO_Navgation::sensor_Init()
{
	//��ʼ����ѹ��
	ms5611.Init();
}




