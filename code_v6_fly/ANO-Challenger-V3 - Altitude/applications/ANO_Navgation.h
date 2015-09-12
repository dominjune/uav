#ifndef __ANO_NAVGATION_H
#define __ANO_NAVGATION_H

#include "ANO_Config.h"

class ANO_Navgation : public ANO_Filter
{
	
public:

	ANO_Navgation();

	int16_t baroVel, ultraVel;	

	int32_t baroAlt, ultraAlt, ultraAltRaw; 

	Vector3f accel, velocity, position;

	int32_t velRatio;

	LPF2ndData_t baro_lpf_2nd, ultra_lpf_2nd;


	void Init(void);
	void update(void);
	void Reset(void);
	
private:
	
	struct Factor{
			float velxy_cf;
			float velz_baro_cf;
			float velz_ultra_cf;
			float velz_cf_ki;
			float baro_cf;	
			float ultra_cf;		
	}factor;
	
	float accVelScale;
	//���ٶ�����
	Vector3f accDeadband;
	//��ʵ�������ٶ�
	int32_t GRAVITY_MSS;

 	int32_t lastBaroAlt, lastUltraAlt;

	int32_t baroOffset;
	
	uint8_t ULTRA_IS_OK;
	
	//�˲���������ʼ��
	void filter_Init(void);
	//��������ʼ��
	void sensor_Init(void);

};

extern ANO_Navgation nav;

#endif

