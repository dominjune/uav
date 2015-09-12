/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：秋雨魂
 * 文件名  ：ANO_FlyControl.cpp
 * 描述    ：飞行控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "ANO_FlyControl.h"

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
	if(!ano.f.ALTHOLD)
		rc.Command[THROTTLE] = (rc.Command[THROTTLE] - 1000) / cosf(radians(tiltAngle)) + 1000;
	
	//PID输出转为电机控制量
	motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}

void ANO_FlyControl::Altitude_Outter_Loop(void)
{/*
	int32_t error;
	static uint8_t AltHoldChanged = 0;
	setVelocity.z = rc.rawData[THROTTLE] - RC_MID;
	if(AltHoldChanged)
	{
		AltHold = nav.position.z;
		AltHoldChanged = 0;
	}
	else if(setVelocity.z < 100)
	{
		error = constrain_int32(AltHold - nav.position.z, -500, 500);
		setVelocity.z = constrain_int32(pid[PIDALT].get_p(error), -200, +200);
	}
	else
	{
		AltHold = nav.position.z;
		AltHoldChanged = 1;
	}*/
	
	
	int32_t AltHoldDeadband = 100;
	static uint8_t isAltHoldChanged = 0;
	int32_t error;
	int16_t deltaThrottle = rc.rawData[THROTTLE] - RC_MID;
	
	if(abs(deltaThrottle) > AltHoldDeadband)
	{
		deltaThrottle = applyDeadband(deltaThrottle, AltHoldDeadband - 80);
		
		if(deltaThrottle > 0)
			setVelocity.z = deltaThrottle * deltaThrottle / 500;
		else
			setVelocity.z = -deltaThrottle * deltaThrottle / 700;
		
		AltHoldReset();
		isAltHoldChanged = 1;
	}
	else if(isAltHoldChanged)
	{
		AltHoldReset();
		isAltHoldChanged = 0;
	}
	else
	{
		error = constrain_int32(AltHold - nav.position.z, -500, 500);
		error = applyDeadband(error, 10);
		setVelocity.z = constrain_int32(pid[PIDALT].get_p(error), -200, +200);
	}
}

void ANO_FlyControl::Altitude_Inner_Loop(void)
{
	static uint32_t lastTime;
	float deltaT;
	int16_t baroPIDThrottle;
	int16_t throttleMid = 1550;
	float tiltAngle = constrain_float(max(abs(imu.angle.x),abs(imu.angle.y)), 0, 20);
	deltaT = (GetSysTime_us() - lastTime) * 1e-6;
	lastTime = GetSysTime_us();
	
	velError.z = setVelocity.z - nav.velocity.z;
	
	velPIDTerm.z = constrain_int32(pid[PIDVELZ].get_p(velError.z), -300, +300);
	velPIDTerm.z += pid[PIDVELZ].get_i(velError.z, deltaT);
	velPIDTerm.z += constrain_int32(pid[PIDVELZ].get_d(velError.z, deltaT), -200, 200);
	baroPIDThrottle = (throttleMid - 1000) / cosf(radians(tiltAngle)) + 1000 + velPIDTerm.z;
	
	if(ano.f.ALTHOLD)
		rc.Command[THROTTLE] = constrain_int32(baroPIDThrottle, RC_MINTHROTTLE, RC_MAXTHROTTLE);
	
}

void ANO_FlyControl::AltHoldReset(void)
{
	AltHold = nav.position.z;
}
/************************ (C) COPYRIGHT 2014 ANO TECH *****END OF FILE**********************/
