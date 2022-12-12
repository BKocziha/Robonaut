/*
 * motor.h
 *
 *  Created on: Dec 9, 2022
 *      Author: kocziha
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

typedef enum circuit_section
	{Slow_section, Fast_section, Braking, Slow_waiting, Acceleration}
	circuit_section;

int MotorDrive(TIM_HandleTypeDef* const pwmHandle, int pwm_val);

int MotorFollowControl(int* prev_error, int current_distance_front, int current_distance_tilted, circuit_section circuit_section,
		int* integral, int slow_sec_nr);

#endif /* INC_MOTOR_H_ */
