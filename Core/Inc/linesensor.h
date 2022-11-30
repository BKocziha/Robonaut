#ifndef LINESENSOR_H
#define LINESENSOR_H

#include <stdbool.h>

void LS_LED_Light(SPI_HandleTypeDef *hspi, uint8_t *leds_to_light, uint8_t *fb_leds_on);

void LS_LED_Send(SPI_HandleTypeDef *hspi, uint8_t *leds_on);

void LS_INF_Send(SPI_HandleTypeDef *hspi, uint8_t *infs_on);

void LS_RUN_4_Leds(SPI_HandleTypeDef *hspi);

void LS_ADC_ChipSelect(int CS);

void LS_BT_SendData(UART_HandleTypeDef *huart, unsigned char *BT_send_msg_buff, uint16_t *ADC_values);

float LS_Holavonal(uint16_t * ADC_values);

float LS_Holavonal_favago(uint16_t *ADC_values);

float LineSensor_FrontOnly(SPI_HandleTypeDef *hspi_led, SPI_HandleTypeDef *hspi_sense);

float *LineSensor_FrontAndBack(UART_HandleTypeDef *huart,SPI_HandleTypeDef *hspi_led, SPI_HandleTypeDef *hspi_sense_front, SPI_HandleTypeDef *hspi_sense_rear, float *line_pos, bool feedback_rear);

#endif
