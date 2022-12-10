/*
 * motor.h
 *
 *  Created on: Dec 9, 2022
 *      Author: kocziha
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

int MotorDrive(TIM_HandleTypeDef* const pwmHandle, int pwm_val);

#endif /* INC_MOTOR_H_ */
