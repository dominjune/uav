/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：秋雨魂
 * 文件名  ：ANO_FlyControl.cpp
 * 描述    ：飞行控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
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
	
	//重置PID参数
	PID_Reset();
}

//重置PID参数
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

//飞行器姿态外环控制
void ANO_FlyControl::Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_ADC;
	
	//计算角度误差值
	errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10; 
	errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10; 
	errorAngle[ROLL] = applyDeadband(errorAngle[ROLL], 2); 
	errorAngle[PITCH] = applyDeadband(errorAngle[PITCH], 2); 
	
	//获取角速度
	Gyro_ADC = imu.Gyro_lpf / 4.0f;
	
	//得到外环PID输出
	RateError[ROLL] = pid[PIDANGLE].get_p(errorAngle[ROLL]) - Gyro_ADC.x;
	RateError[PITCH] = pid[PIDANGLE].get_p(errorAngle[PITCH]) - Gyro_ADC.y;
	RateError[YAW] = ((int32_t)(yawRate) * rc.Command[YAW]) / 32 - Gyro_ADC.z;		
}

//飞行器姿态内环控制
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
		//当油门低于检查值时积分清零
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)	
			pid[i].reset_I();
		
		//得到内环PID输出
		PIDTerm[i] = pid[i].get_pid(RateError[i], PID_INNER_LOOP_TIME*1e-6);
	}
	
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300 - abs(rc.Command[YAW]), +300 + abs(rc.Command[YAW]));	
		
	//油门倾斜补偿
	if(!ano.f.FLAG)
	{
		rc.Command[THROTTLE] = (rc.Command[THROTTLE] - 1000) / cosf(radians(tiltAngle)) + 1000;
		motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
		cnt = 0;
		cnt2 = 0;
		imu.fanzhuan_angle.y = 0;
		//motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
	}
		
	
	//////////////翻转////////////////
	if(ano.f.FLAG)
	{
		//上冲+翻转
		if(ano.f.flag == 0 && ano.f.flag2 == 0)	
		{
			if(cnt < tt)
			{
				cnt++;
				motor.up(1900,0,0,0);
			}
			else
			{
				if(abs(imu.fanzhuan_angle.y) <= first_angle)		// 90度
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
		//补偿惯性
		else if(ano.f.flag == 1 && ano.f.flag2 == 0)
		{
			if(abs(imu.fanzhuan_angle.y) <= second_angle)		// 360度
			{
				imu.fanzhuan_angle.y += imu.Gyro_lpf.y * deltaT;
			}
			else
			{
				ano.f.flag2 = 1;
			}
			motor.balance(1900, 0,0,0);
		}
		//上冲+手动
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
