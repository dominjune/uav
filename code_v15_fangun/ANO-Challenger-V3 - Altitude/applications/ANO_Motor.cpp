/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������
 * �ļ���  ��ANO_Motor.cpp
 * ����    �����������غ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "ANO_Motor.h"

ANO_Motor motor;

const int faster = 1900;
const int slower = 1200;
const int u = 1700;

void ANO_Motor::stop(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	motorPWM[0] = throttle; //����
	motorPWM[1] = throttle; //ǰ��
	motorPWM[2] = throttle; //����
	motorPWM[3] = throttle; //ǰ��
	motorPWM[4] = throttle;	//��
	motorPWM[5] = throttle;	//��		
	
	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//���Ƶ��PWM����С�����ֵ
		//motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
			motorPWM[i] = constrain_uint16(motorPWM[i], 0, 0);
	}

	//���δ�������򽫵���������Ϊ���
	if(!ano.f.ARMED)	
		ResetPWM();

	if(!ano.f.ALTHOLD && rc.rawData[THROTTLE] < 1200)
		ResetPWM();

	//д����PWM
	pwm.SetPwm(motorPWM);
}

void ANO_Motor::up(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	motorPWM[0] = throttle; //����
	motorPWM[1] = throttle; //ǰ��
	motorPWM[2] = throttle; //����
	motorPWM[3] = throttle; //ǰ��
	motorPWM[4] = throttle;	//��
	motorPWM[5] = throttle;	//��		
	
	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//���Ƶ��PWM����С�����ֵ
			motorPWM[i] = constrain_uint16(motorPWM[i], throttle, throttle);
	}

	//���δ�������򽫵���������Ϊ���
	if(!ano.f.ARMED)	
		ResetPWM();

	if(!ano.f.ALTHOLD && rc.rawData[THROTTLE] < 1200)
		ResetPWM();

	//д����PWM
	pwm.SetPwm(motorPWM);
}

void ANO_Motor::fangun(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	motorPWM[0] = throttle; //����
	motorPWM[1] = 0; //ǰ��
	motorPWM[2] = throttle; //����
	motorPWM[3] = 0; //ǰ��
	motorPWM[4] = 0;	//��
	motorPWM[5] = 0;	//��		
	
	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//���Ƶ��PWM����С�����ֵ
		//motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
		if(i == 1 || i == 3)
			motorPWM[i] = constrain_uint16(motorPWM[i], slower, slower);	//ǰ�����
		else
			motorPWM[i] = constrain_uint16(motorPWM[i], faster, faster);	//����ԭ��
	}

	//���δ�������򽫵���������Ϊ���
	if(!ano.f.ARMED)	
		ResetPWM();

	if(!ano.f.ALTHOLD && rc.rawData[THROTTLE] < 1200)
		ResetPWM();

	//д����PWM
	pwm.SetPwm(motorPWM);
}

void ANO_Motor::balance(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	motorPWM[0] = 0; //����
	motorPWM[1] = throttle; //ǰ��
	motorPWM[2] = 0; //����
	motorPWM[3] = throttle; //ǰ��
	motorPWM[4] = 0;	//��
	motorPWM[5] = 0;	//��		
	
	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//���Ƶ��PWM����С�����ֵ 
		//motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
		if(i == 1 || i == 3)
			motorPWM[i] = constrain_uint16(motorPWM[i], faster, faster);	//ǰ�����
		else if(i == 0 || i == 2)
			motorPWM[i] = constrain_uint16(motorPWM[i], slower, slower);	//�������
		else
			motorPWM[i] = constrain_uint16(motorPWM[i], faster, faster);	//����ԭ��
	}

	//���δ�������򽫵���������Ϊ���
	if(!ano.f.ARMED)	
		ResetPWM();

	if(!ano.f.ALTHOLD && rc.rawData[THROTTLE] < 1200)
		ResetPWM();

	//д����PWM
	pwm.SetPwm(motorPWM);
}

void ANO_Motor::writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	//����X��
	motorPWM[0] = throttle - 0.5 * pidTermRoll + 0.866 *  pidTermPitch + pidTermYaw; //����
	motorPWM[1] = throttle - 0.5 * pidTermRoll - 0.866 *  pidTermPitch + pidTermYaw; //ǰ��
	motorPWM[2] = throttle + 0.5 * pidTermRoll + 0.866 *  pidTermPitch - pidTermYaw; //����
	motorPWM[3] = throttle + 0.5 * pidTermRoll - 0.866 *  pidTermPitch - pidTermYaw; //ǰ��
	motorPWM[4] = throttle - pidTermRoll - pidTermYaw;	//��
	motorPWM[5] = throttle + pidTermRoll + pidTermYaw;	//��	
	
	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//���Ƶ��PWM����С�����ֵ
		motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}

	//���δ�������򽫵���������Ϊ���
	if(!ano.f.ARMED)	
		ResetPWM();

	//if(!ano.f.ALTHOLD && rc.rawData[THROTTLE] < 1200)
	//S	ResetPWM();
	if(!ano.f.FLAG && rc.rawData[THROTTLE] < 1200)
		ResetPWM();
	//д����PWM
	pwm.SetPwm(motorPWM);
	
}

void ANO_Motor::getPWM(int16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
	*(pwm+4) = motorPWM[4];
	*(pwm+5) = motorPWM[5];	
}

void ANO_Motor::ResetPWM(void)
{
	for(u8 i=0; i< MAXMOTORS ; i++)
		motorPWM[i] = 1000;
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
