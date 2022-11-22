// Bluetooth communication

#include <string.h>
#include "main.h"

void BT_TransmitMsg(UART_HandleTypeDef *huart, unsigned char *msg_to_send)
{
	uint32_t len = strlen((char*)msg_to_send);
	HAL_UART_Transmit(huart, (uint8_t*)msg_to_send, len, 100);
}

void BT_ReceiveMsg(UART_HandleTypeDef *huart, unsigned char *msg_to_receive) // Max üzenethossz: 10 karakter
{
	HAL_UART_Receive_IT(huart, msg_to_receive, 20);

}

void BT_ProcessMsg(unsigned char *msg)
{
	// Throw away the excess characters
	for (int i = 0; i < 20; i++)
	{
		if(msg[i] == '!')
		{
			msg[i] = 0;
			break;
		}
	}

	if(strcmp((char*)msg, "STOP") == 0)
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
}
