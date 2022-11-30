// Line sensor control

#include "main.h"
#include "linesensor.h"
#include "bluetooth.h"
#include <string.h>
#include <stdio.h>

// Az utolsó beadott érték mindig 33-nál nagyobb legyen!
void LS_LED_Light(SPI_HandleTypeDef *hspi, uint8_t *leds_to_light, uint8_t *fb_leds_on)
{
	uint8_t fb_leds_on_temp[4] = {0};
	for (int i=0; leds_to_light[i]<33; i++)
	{
		switch(leds_to_light[i]/8) {
		case 0:
			fb_leds_on_temp[3] = 1;
			for (int j=0; j<leds_to_light[i]%8; j++){
				fb_leds_on_temp[3] <<= 1;
			}
			fb_leds_on[3] |= fb_leds_on_temp[3];
			break;
		case 1:
			fb_leds_on_temp[2] = 1;
			for (int j=0; j<leds_to_light[i]%8; j++){
				fb_leds_on_temp[2] <<= 1;
			}
			fb_leds_on[2] |= fb_leds_on_temp[2];
			break;
		case 2:
			fb_leds_on_temp[1] = 1;
			for (int j=0; j<leds_to_light[i]%8; j++){
				fb_leds_on_temp[1] <<= 1;
			}
			fb_leds_on[1] |= fb_leds_on_temp[1];
			break;
		case 3:
			fb_leds_on_temp[0] = 1;
			for (int j=0; j<leds_to_light[i]%8; j++){
				fb_leds_on_temp[0] <<= 1;
			}
			fb_leds_on[0] |= fb_leds_on_temp[0];
			break;
		}
	}
	LS_LED_Send(hspi, fb_leds_on);
}

void LS_LED_Send(SPI_HandleTypeDef *hspi, uint8_t *leds_on)
{
	// Send bits with SPI
	HAL_SPI_Transmit(hspi, leds_on, 4, 100);

	// Latch enable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

	//Output enable
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LS_INF_Send(SPI_HandleTypeDef *hspi, uint8_t *infs_on)
{
	// Send bits with SPI
	HAL_SPI_Transmit(hspi, infs_on, 4, 100);

	// Inf Latch enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	// Inf Output enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

void LS_RUN_4_Leds(SPI_HandleTypeDef *hspi)
{
	// Starting state
	uint8_t leds_on[] = {1, 1, 1, 1};

	// Shift leds left
	for (int i = 0; i < 7; i++)
		  {
			  leds_on[0] <<= 1;
			  leds_on[1] <<= 1;
			  leds_on[2] <<= 1;
			  leds_on[3] <<= 1;
			  LS_LED_Send(hspi, leds_on);
			  HAL_Delay(100);
		  }

	// Shift leds right
	for (int i = 0; i < 7; i++)
		  {
			  leds_on[0] >>= 1;
			  leds_on[1] >>= 1;
			  leds_on[2] >>= 1;
			  leds_on[3] >>= 1;
			  LS_LED_Send(hspi, leds_on);
			  HAL_Delay(100);
		  }
}

void LS_ADC_ChipSelect(int CS)
{
	switch(CS) {
//	case 0:
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
}

void LS_BT_SendData(UART_HandleTypeDef *huart, unsigned char *BT_send_msg_buff, uint16_t *ADC_values, unsigned char *ADC_value_string)
{
	BT_send_msg_buff[0] = '\0';
	for (int k=0; k<16; k++){
	  sprintf((char*)ADC_value_string, "%d ", ADC_values[k]);
	  strcat((char*)BT_send_msg_buff, (char*)ADC_value_string);
	}
	BT_TransmitMsg(huart, BT_send_msg_buff);
	BT_send_msg_buff[0] = '\0';
	for (int k=16; k<32; k++){
	  sprintf((char*)ADC_value_string, "%d ", ADC_values[k]);
	  strcat((char*)BT_send_msg_buff, (char*)ADC_value_string);
	}
	strcat((char*)BT_send_msg_buff, "\n\r");
	BT_TransmitMsg(huart, BT_send_msg_buff);
}

double LS_Holavonal(uint16_t * ADC_values){
	double weighted_sum = 0;
	int sum = 0;
	double line;
	for(int i = 0;i<32;i++){
		weighted_sum += (ADC_values[i]-250)*i;
		sum += ADC_values[i];
	}
	sum -= 32*250;
	line = (double)weighted_sum/sum;
	return line;
}

double LS_Holavonal_favago(uint16_t *ADC_values){
    int m = 0;
    int sum = 0;
    for(int i=0; i<32; i++){
        if (ADC_values[i] > 3000){
            sum += i;
            m++;
        }
    }
    if(m == 0)
        return 0;
    return sum/m;
}

//double LS_Holavonal_favago(uint16_t *ADC_values){
//	int vonal[33] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
//								50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
//	int k = 0;
//	for(int i=0; i<32; i++){
//		if (ADC_values[i] > 3000){
//			vonal[k] = i;
//			k++;
//		}
//	}
//	int m;
//	int sum = 0;
//	for(m=0; vonal[m]!=50; m++){
//		sum += vonal[m];
//	}
//	if(m == 0)
//		return 0;
//	return sum/m;
//}
