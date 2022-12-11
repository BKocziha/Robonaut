#include "main.h"

int MotorDrive(TIM_HandleTypeDef* const pwmHandle, int duty)
{
	int pwm_val = 100+(duty*20);
	pwmHandle -> Instance -> CCR1 = pwm_val;
	return pwm_val;
}

int MotorFollowControl(int* prev_error, int current_distance)
{
	int new_duty_motor;
	int reference_distance = 300;
	int error = reference_distance-current_distance;
	int d_error = error - *prev_error;
	new_duty_motor = -error/20 + d_error/100;
	*prev_error = error;
	if (new_duty_motor > 20)
		new_duty_motor = 20;
	if (new_duty_motor < 2)
		new_duty_motor = 2;
	return new_duty_motor;
}
