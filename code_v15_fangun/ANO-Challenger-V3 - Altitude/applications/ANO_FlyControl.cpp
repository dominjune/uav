/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������
 * �ļ���  ��ANO_FlyControl.cpp
 * ����    �����п���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "ANO_FlyControl.h"

//int cnt, cnt2;
//int32_t PIDTerm[3];
const int first_angle = 1400;
const int second_angle = 4600;
const int tt = 50;
const int tt2 = 0;
ANO_FlyControl fc;

ANO_FlyControl::ANO_FlyControl()
{
	rollPitchRate = 150;
	yawRate = 50;
	
	//����PID����
	PID_Reset();
}

//����PID����
void ANO_FlyControl::PID_Reset(void)
{
	pid[PIDROLL].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDPITCH].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDYAW].set_pid(0.8, 0.45, 0, 200);
	pid[PIDANGLE].set_pid(5, 0, 0, 0);
	pid[PIDMAG].set_pid(2, 0, 0, 0);
 	pid[PIDVELZ].set_pid(1.5, 0.5, 0.002, 150);
 	pid[PIDALT].set_pid(1, 0, 0, 200);
}

//��������̬�⻷����
void ANO_FlyControl::Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_ADC;
	
	//����Ƕ����ֵ
	errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10; 
	errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10; 
	errorAngle[ROLL] = applyDeadband(errorAngle[ROLL], 2); 
	errorAngle[PITCH] = applyDeadband(errorAngle[PITCH], 2); 
	
	//��ȡ���ٶ�
	Gyro_ADC = imu.Gyro_lpf / 4.0f;
	
	//�õ��⻷PID���
	RateError[ROLL] = pid[PIDANGLE].get_p(errorAngle[ROLL]) - Gyro_ADC.x;
	RateError[PITCH] = pid[PIDANGLE].get_p(errorAngle[PITCH]) - Gyro_ADC.y;
	RateError[YAW] = ((int32_t)(yawRate) * rc.Command[YAW]) / 32 - Gyro_ADC.z;		
}

//��������̬�ڻ�����
void ANO_FlyControl::Attitude_Inner_Loop(void)
{
	static int cnt, cnt2;
	static uint32_t lastTime;
	float deltaT;
	//
	deltaT = (GetSysTime_us() - lastTime) * 1e-6;
	lastTime = GetSysTime_us();
	//
	int32_t PIDTerm[3];
	float tiltAngle = constrain_float( max(abs(imu.angle.x), abs(imu.angle.y)), 0 ,20);
	
	for(u8 i=0; i<3;i++)
	{
		//�����ŵ��ڼ��ֵʱ��������
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)	
			pid[i].reset_I();
		
		//�õ��ڻ�PID���
		PIDTerm[i] = pid[i].get_pid(RateError[i], PID_INNER_LOOP_TIME*1e-6);
	}
	
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300 - abs(rc.Command[YAW]), +300 + abs(rc.Command[YAW]));	
		
	//������б����
	if(!ano.f.FLAG)
	{
		rc.Command[THROTTLE] = (rc.Command[THROTTLE] - 1000) / cosf(radians(tiltAngle)) + 1000;
		motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
		cnt = 0;
		cnt2 = 0;
		imu.fanzhuan_angle.y = 0;
		//motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
	}
		
	
	//////////////��ת////////////////
	if(ano.f.FLAG)
	{
		//�ϳ�+��ת
		if(ano.f.flag == 0 && ano.f.flag2 == 0)	
		{
			if(cnt < tt)
			{
				cnt++;
				motor.up(1900,0,0,0);
			}
			else
			{
				if(abs(imu.fanzhuan_angle.y) <= first_angle)		// 90��
				{
					imu.fanzhuan_angle.y += imu.Gyro_lpf.y * deltaT;
				}
				else
				{
					ano.f.flag = 1;	
				}
				motor.fangun(1900,0,0,0);
			}
			
		}
		//��������
		else if(ano.f.flag == 1 && ano.f.flag2 == 0)
		{
			if(abs(imu.fanzhuan_angle.y) <= second_angle)		// 360��
			{
				imu.fanzhuan_angle.y += imu.Gyro_lpf.y * deltaT;
			}
			else
			{
				ano.f.flag2 = 1;
			}
			motor.balance(1900, 0,0,0);
		}
		//�ϳ�+�ֶ�
		else if(ano.f.flag == 1 && ano.f.flag2 == 1)
		{
			if(cnt2 < tt2)
			{			
				cnt2++;
				motor.up(1800,0,0,0);
			}
			else
			{
				ano.f.FLAG = 0;
			}		
		}
	}
	
}

/************************ (C) COPYRIGHT 2014 ANO TECH *****END OF FILE**********************/
