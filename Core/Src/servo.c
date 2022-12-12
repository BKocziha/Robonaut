#include "main.h"
#include <math.h>

void ServoPosition(TIM_HandleTypeDef* const pwmHandle, double    angle){
    if(angle < 10){angle = 10;}
    if(angle>180){angle = 180;}
    //angle2CCR = ((angle/180+1)/20*60000);
    pwmHandle->Instance->CCR1 = (int)((angle/180+1)*3000);//angle2CCR;
}


float SteeringAngle(float p, float delta, float kp, float kd){
	float phi = atan(0.7826*tan(-kp*p-kd*delta));
	// Egyenesfutás miatt megváltoztatva
	float servoangle=90-phi*269.04;//88
	return servoangle;
}
