#include "main.h"

int MotorDrive(TIM_HandleTypeDef* const pwmHandle, int duty)
{
	int pwm_val = 100+(duty*20);
	pwmHandle -> Instance -> CCR1 = pwm_val;
	return pwm_val;
}

