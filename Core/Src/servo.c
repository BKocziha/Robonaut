#include "main.h"


void ServoPosition(TIM_HandleTypeDef* const pwmHandle, double    angle){
    if(angle < 36){angle = 36;}
    if(angle>144){angle = 144;}
    //angle2CCR = ((angle/180+1)/20*60000);
    pwmHandle->Instance->CCR1 = (int)((angle/180+1)/20*60000);//angle2CCR;
}
