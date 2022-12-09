// Line sensor control

#include "main.h"
#include "linesensor.h"
#include "bluetooth.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Az utolsó beadott érték mindig 33-nál nagyobb legyen!
void LS_LED_Light(SPI_HandleTypeDef *hspi, uint8_t *leds_to_light)
{
	uint8_t fb_leds_on[4] = {0};
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

void LS_BT_SendData(UART_HandleTypeDef *huart, unsigned char *BT_send_msg_buff, uint16_t *ADC_values)
{
	unsigned char ADC_value_string[20];
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

float LS_Holavonal(uint16_t * ADC_values){
	float weighted_sum = 0;
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

float LS_Holavonal_favago(uint16_t *ADC_values, int prev_value, int* summ, int* MA_sum){
    int m = 0;
    int sum = 0;
    float alpha = 0.1;
    *summ = 0;
    for(int i=0; i<32; i++){
        if (ADC_values[i] > 2500){
        	*summ += ADC_values[i];
            sum += i;
            m++;
        }
    }
    if(m == 0)
		return prev_value;
    *MA_sum = alpha * *summ + (1-alpha) * *MA_sum;
    return sum/m;
}

void LineSensor_FrontOnly(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi_led, SPI_HandleTypeDef *hspi_sense_front,
		uint16_t *ADC_values_front)
{
	uint8_t leds_on[4];
	uint8_t ADC_inputs[] = {0, 8, 16, 24, 32, 40, 48, 56};
	uint8_t ADC_received_msg[2];

	//LS_INF_Send(&hspi3, leds_off);

	// Turn on first set of LEDs
	leds_on[0] = 1;leds_on[1] = 1;leds_on[2] = 1;leds_on[3] = 1;
	LS_INF_Send(hspi_led, leds_on);
	HAL_Delay(1);
	// Retrieve data from first set of ADCs
	for (int i=1; i<5; i++)
	{
	  LS_ADC_ChipSelect(i);
	  HAL_SPI_TransmitReceive(hspi_sense_front, &ADC_inputs[0], ADC_received_msg, 2, 100);
	  ADC_values_front[(i-1)*8] = ADC_received_msg[1] | (ADC_received_msg[0] << 8);
	  LS_ADC_ChipSelect(0);
	}

	// Shift the LEDs by one
	for (int k=0; k<7; k++)
	{
	  leds_on[0] <<= 1;
	  leds_on[1] <<= 1;
	  leds_on[2] <<= 1;
	  leds_on[3] <<= 1;
	  LS_INF_Send(hspi_led, leds_on);
	  HAL_Delay(1);

	  //Retrieve data from the ADCs at the active LEDs
	  for (int i=1; i<5; i++)
	  {
		  LS_ADC_ChipSelect(i);
		  HAL_SPI_TransmitReceive(hspi_sense_front, &ADC_inputs[k+1], ADC_received_msg, 2, 100);
		  HAL_SPI_TransmitReceive(hspi_sense_front, &ADC_inputs[k+1], ADC_received_msg, 2, 100);
		  ADC_values_front[(i-1)*8+k+1] = ADC_received_msg[1] | (ADC_received_msg[0] << 8);
		  LS_ADC_ChipSelect(0);
	  }
	}
}

void LineSensor_FrontAndBack(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi_led, SPI_HandleTypeDef *hspi_sense_front,
		SPI_HandleTypeDef *hspi_sense_rear, uint16_t *ADC_values_front, uint16_t *ADC_values_rear)
{
	uint8_t leds_on[4];// = {1, 1, 1, 1};

	uint8_t ADC_inputs[] = {0, 8, 16, 24, 32, 40, 48, 56};
	uint8_t ADC_received_msg[2];

	//LS_INF_Send(&hspi3, leds_off);

	// Turn on first set of LEDs
	leds_on[0] = 1;leds_on[1] = 1;leds_on[2] = 1;leds_on[3] = 1;
	LS_INF_Send(hspi_led, leds_on);
	HAL_Delay(1);
	// Retrieve data from first set of ADCs - Front
	for (int i=1; i<5; i++)
	{
	  LS_ADC_ChipSelect(i);
	  HAL_SPI_TransmitReceive(hspi_sense_front, &ADC_inputs[0], ADC_received_msg, 2, 100);
	  HAL_SPI_TransmitReceive(hspi_sense_front, &ADC_inputs[0], ADC_received_msg, 2, 100);
	  ADC_values_front[(i-1)*8] = ADC_received_msg[1] | (ADC_received_msg[0] << 8);
	  LS_ADC_ChipSelect(0);
	}
	// Ez működjön, aztán lehet optimalizálni, hogy egy loopon belül kérjük be az adatokat mindettőtől
	// Retrieve data from first set of ADCs - Rear
	for (int i=1; i<5; i++)
	{
	  LS_ADC_ChipSelect(i);
	  HAL_SPI_TransmitReceive(hspi_sense_rear, &ADC_inputs[0], ADC_received_msg, 2, 100);
	  HAL_SPI_TransmitReceive(hspi_sense_rear, &ADC_inputs[0], ADC_received_msg, 2, 100);
	  ADC_values_rear[(i-1)*8] = ADC_received_msg[1] | (ADC_received_msg[0] << 8);
	  LS_ADC_ChipSelect(0);
	}

	// Shift the LEDs by one
	for (int k=0; k<7; k++)
	{
	  leds_on[0] <<= 1;
	  leds_on[1] <<= 1;
	  leds_on[2] <<= 1;
	  leds_on[3] <<= 1;
	  LS_INF_Send(hspi_led, leds_on);
	  HAL_Delay(1);

	  //Retrieve data from the ADCs at the active LEDs - Front
	  for (int i=1; i<5; i++)
	  {
		  LS_ADC_ChipSelect(i);
		  HAL_SPI_TransmitReceive(hspi_sense_front, &ADC_inputs[k+1], ADC_received_msg, 2, 100);
		  HAL_SPI_TransmitReceive(hspi_sense_front, &ADC_inputs[k+1], ADC_received_msg, 2, 100);
		  ADC_values_front[(i-1)*8+k+1] = ADC_received_msg[1] | (ADC_received_msg[0] << 8);
		  LS_ADC_ChipSelect(0);
	  }

	  // Szintén lehet optimalizálni
	  // Retrieve data from the ADCs at the active LEDs - Front
	  for (int i=1; i<5; i++)
	  {
		  LS_ADC_ChipSelect(i);
		  HAL_SPI_TransmitReceive(hspi_sense_rear, &ADC_inputs[k+1], ADC_received_msg, 2, 100);
		  HAL_SPI_TransmitReceive(hspi_sense_rear, &ADC_inputs[k+1], ADC_received_msg, 2, 100);
		  ADC_values_rear[(i-1)*8+k+1] = ADC_received_msg[1] | (ADC_received_msg[0] << 8);
		  LS_ADC_ChipSelect(0);
	  }
	}

//	line_pos[0] = 0;
//	for (int i=0; i<32; i++)
//	{
//		if (ADC_values_rear[i] > line_pos[0])
//			line_pos[0] = ADC_values_rear[i];
//	}

//	unsigned char BT_send_msg_buff[200];
//	LS_BT_SendData(huart, BT_send_msg_buff, ADC_values_rear);
}

void LS_feedback_all(SPI_HandleTypeDef *hspi_led, uint16_t *ADC_values)
{
	uint8_t fb_leds_to_light[5] = {50, 50, 50, 50, 50};
	uint8_t leds_off[4] = {0};
	int j = 0;
		for (int i=0; i<32; i++){
			if (ADC_values[i]>2500){
				fb_leds_to_light[j] = i;
				j++;
			}
		}

	LS_LED_Send(hspi_led, leds_off);
	LS_LED_Light(hspi_led, fb_leds_to_light);
}

void LS_feedback_led(SPI_HandleTypeDef *hspi_led, float *line_pos, bool feedback_rear)
{
	uint8_t fb_leds_to_light[5] = {50, 50, 50, 50, 50};
	uint8_t leds_off[4] = {0};

	if (feedback_rear){
		fb_leds_to_light[0] = (int)line_pos[1];
		fb_leds_to_light[1] = (int)line_pos[1]+1; // ?
	}
	else {
		fb_leds_to_light[0] = (int)line_pos[0];
		fb_leds_to_light[1] = (int)line_pos[0]+1; // ?
	}
	LS_LED_Send(hspi_led, leds_off);
	LS_LED_Light(hspi_led, fb_leds_to_light);
}

float LS_delta_angle(float p1, float p2){
    float delta = atan((p2-(31-p1))*6.5/460);
    return delta;
}



float LS_p(float f1){
    float p = (15.5-f1)*0.0065;//m-ben adja vissza a p-t
    return p;
}
