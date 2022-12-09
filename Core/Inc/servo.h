#ifndef SERVO_H
#define SERVO_H

void ServoPosition(TIM_HandleTypeDef* const pwmHandle, double    angle);

float SteeringAngle(float p, float delta);

#endif
