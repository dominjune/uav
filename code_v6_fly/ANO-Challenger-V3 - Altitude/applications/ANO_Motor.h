#ifndef __ANO_MOTOR_H
#define __ANO_MOTOR_H

#include "ANO_Config.h"

#define MINTHROTTLE 1100
#define MAXTHROTTLE 1900

class ANO_Motor
{

public:
	
	void writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw);
	
	void getPWM(int16_t* pwm);

private:
	
	int16_t motorPWM[6];	

	void ResetPWM(void);

};

extern ANO_Motor motor;

#endif





