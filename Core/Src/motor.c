#include "main.h"
#include "motor.h"

int MotorDrive(TIM_HandleTypeDef* const pwmHandle, int duty)
{
	int pwm_val = 100+(duty*20);
	pwmHandle -> Instance -> CCR1 = pwm_val;
	return pwm_val;
}

int MotorFollowControl(int* prev_error, int current_distance_front, int current_distance_tilted, circuit_section circuit_section)
{
	int new_duty_motor;
	int current_distance;
	int reference_distance = 300;
	if (circuit_section == Slow_section || circuit_section == Slow_waiting){
		if (current_distance_tilted < current_distance_front){
			current_distance = current_distance_tilted;
		}
		else {
			current_distance = current_distance_front;
		}
	}
	else {
		current_distance = current_distance_front;
	}

	int error = reference_distance-current_distance;
	int d_error = error - *prev_error;
	new_duty_motor = -error/20 + d_error/50;
	*prev_error = error;
	switch(circuit_section) {
		case Slow_section:
			if (new_duty_motor > 18)
				new_duty_motor = 18;
			break;
		case Fast_section:
			if (new_duty_motor > 30)
				new_duty_motor = 30;
			break;
		case Braking:
			if (new_duty_motor > 10)
				new_duty_motor = 10;
			break;
		case Slow_waiting:
			if (new_duty_motor > 18)
				new_duty_motor = 18;
			break;
		case Acceleration:
			if (new_duty_motor > 30)
				new_duty_motor = 30;
			break;
	}
//	if (new_duty_motor > 25)
//2				new_duty_motor = 25;
		if (new_duty_motor < 2)
			new_duty_motor = 2;
		return new_duty_motor;
}
