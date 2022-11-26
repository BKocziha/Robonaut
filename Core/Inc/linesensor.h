#ifndef LINESENSOR_H
#define LINESENSOR_H

void LS_LED_Send(SPI_HandleTypeDef *hspi, uint8_t *leds_on);

void LS_RUN_4_Leds(SPI_HandleTypeDef *hspi);

void LS_ADC_ChipSelect(int CS);

void LS_BT_SendData(UART_HandleTypeDef *huart, unsigned char *BT_send_msg_buff, uint16_t *ADC_values, unsigned char *ADC_value_string);

//typedef enum {
//	CS_0000 = 0,
//	CS_0001 = 1,
//	CS_0010 = 2,
//	CS_0011 = 3,
//	CS_0100 = 4,
//	CS_0101 = 5,
//	CS_0110 = 6,
//	CS_0111 = 7,
//	CS_1000 = 8,
//	CS_1001 = 9,
//	CS_1010 = 10,
//	CS_1011 = 11,
//	CS_1100 = 12,
//	CS_1101 = 13,
//	CS_1110 = 14,
//	CS_1111 = 15
//} ChipSelect;

#endif
