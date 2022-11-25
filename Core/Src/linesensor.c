// Line sensor control

#include "main.h"
#include "linesensor.h"

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
	default:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	}
}

//uint16_t LS_Run_ADCs(SPI_HandleTypeDef *hspi, ChipSelect CS, uint8_t *ADC_number)
//{
//	// Pull down the appropriate Chip Select pins
//
//	return (uint16_t)CS;
//}
