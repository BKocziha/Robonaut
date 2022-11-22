// Line sensor control

#include "main.h"

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

	// Send bits with SPI
	HAL_SPI_Transmit(hspi, leds_on, 4, 100);

	// Inf Latch enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	// Inf Output enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}
