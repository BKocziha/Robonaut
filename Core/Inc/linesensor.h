#ifndef LINESENSOR_H
#define LINESENSOR_H

#include <stdbool.h>

void LS_LED_Light(SPI_HandleTypeDef *hspi, uint8_t *leds_to_light);

void LS_LED_Send(SPI_HandleTypeDef *hspi, uint8_t *leds_on);

void LS_INF_Send(SPI_HandleTypeDef *hspi, uint8_t *infs_on);

void LS_RUN_4_Leds(SPI_HandleTypeDef *hspi);

void LS_ADC_ChipSelect(int CS);

void LS_BT_SendData(UART_HandleTypeDef *huart, unsigned char *BT_send_msg_buff, uint16_t *ADC_values);

float LS_Holavonal(uint16_t * ADC_values);

float LS_Holavonal_favago(uint16_t *ADC_values);

void LineSensor_FrontOnly(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi_led, SPI_HandleTypeDef *hspi_sense_front,
		uint16_t *ADC_values_front);

void LineSensor_FrontAndBack(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi_led, SPI_HandleTypeDef *hspi_sense_front,
		SPI_HandleTypeDef *hspi_sense_rear, uint16_t *ADC_values_front, uint16_t *ADC_values_rear);

void LS_feedback_led(SPI_HandleTypeDef *hspi_led, float *line_pos, bool feedback_rear);

float LS_delta_angle(float p1, float p2);

float LS_p(float f1, float f2, float delta);

void LS_feedback_all(SPI_HandleTypeDef *hspi_led, uint16_t *ADC_values);

#endif
