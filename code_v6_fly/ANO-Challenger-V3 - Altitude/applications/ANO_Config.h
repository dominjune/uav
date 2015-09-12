#ifndef __ANO_CONFIG_H
#define __ANO_CONFIG_H

#include "board.h"
#include "ANO_PID.h"
#include "ANO_Filter.h"
#include "ANO_IMU.h"
#include "ANO_Scheduler.h"
#include "ANO_DT.h"
#include "ANO_Motor.h"
#include "ANO_RC.h"
#include "ANO_FlyControl.h"
#include "ANO_Navgation.h"
#include "ANO_Param.h"

/*----------------------IMU--------------------*/
#define ANO_IMU_USE_DCM_CF
//#define ANO_IMU_USE_Quaternions_CF

//#define ANO_IMU_USE_LPF_1st
#define ANO_IMU_USE_LPF_2nd


#define IMU_LOOP_TIME					2000	//��λΪuS
#define PID_INNER_LOOP_TIME		2000	//��λΪus
#define PID_OUTER_LOOP_TIME		5000	//��λΪus

#define ACC_1G 			4096		//�ɼ��ٶȼƵ�����ȷ��
#define ACC_LPF_CUT 50.0f		//���ٶȵ�ͨ�˲�����ֹƵ��50Hz
#define GYRO_LPF_CUT 130.0f		//�����ǵ�ͨ�˲�����ֹƵ��130Hz

#define GYRO_CF_TAU 1.8f
/*---------------------------------------------*/


/*----------------------Navgation--------------------*/
#define BARO_LOOP_TIME					20000	//��λΪuS
#define BARO_LPF_CUT	5.0f 

#define ULTRA_LOOP_TIME					20000	//��λΪuS
#define ULTRA_LPF_CUT	50.0f 
/*--------------------------------------------------------*/


/*-------------------�������ݷ��ͷ�ʽѡ��-----------------*/
//#define ANO_DT_USE_Bluetooth
#define ANO_DT_USE_NRF24l01
/*--------------------------------------------------------*/


class ANO_Config
{
	
public:
	
	ANO_Config();

	class Factor{
		public:		
			float acc_lpf;		
			float gyro_cf;		
	}factor;

	class Flag{
		public:
			uint8_t CALIBRATED;
			uint8_t ARMED;
			uint8_t FAILSAFE;
			uint8_t ANGLE_MODE;
			uint8_t ALTHOLD;
	}f;
	
	//ָʾ��
	void Pilot_Light(void);
	
};

extern ANO_Config ano;

#endif

