/******************** (C) COPYRIGHT 2014 ANO Tech *******************************
 * 作者		 ：秋雨魂
 * 文件名  ：ANO_Navgation.cpp
 * 描述    ：飞行器组合导航
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
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
	
	
	//发送US100超声波测距信号
	if(ultraCnt == 10 || uart3.RxState3 == 2){	
			uart3.RxState3 = 0;
			uart3.Put_Char(0x55);		
			ultraCnt = 0;		

			deltaT = (GetSysTime_us() - lastTimeUltra) * 1e-6;
			lastTimeUltra = GetSysTime_us();
		
			//获取超声波高度并低通滤波
			ultraAlt = Ultra_LPF_2nd(&ultra_lpf_2nd, (float)ultraAltRaw);	
			//计算超声波变化速度
			ultraVel = (ultraAlt - lastUltraAlt) / deltaT;
			lastUltraAlt = ultraAlt;
			//限制气压变化的速度，正负800cm/s
			ultraVel = constrain_int32(ultraVel, -8000, 8000);    
			//增加死区减小噪声
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
		//气压传感器数据更新
		ms5611.Update();
	}
	else if(baroCnt == 10){
		deltaT = (GetSysTime_us() - lastTimeBaro) * 1e-6;
		lastTimeBaro = GetSysTime_us();
		baroCnt = 0;
		
		//气压传感器数据更新
		ms5611.Update();
		
		//获取气压高度并低通滤波（已校零偏）
		baroAlt = Baro_LPF_2nd(&baro_lpf_2nd, (float)ms5611.Get_BaroAlt());	
		
		//计算气压变化速度
		baroVel = (baroAlt -lastBaroAlt) / deltaT;
		lastBaroAlt = baroAlt;
		//限制气压变化的速度，正负800cm/s
    baroVel = constrain_int32(baroVel, -800, 800);    
		//增加死区减小噪声
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
	//初始化气压计
	ms5611.Init();
}




