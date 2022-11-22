#ifndef BLUETOOTH_H
#define BLUETOOTH_H

void BT_TransmitMsg(UART_HandleTypeDef *huart, unsigned char *msg_to_send);

void BT_ReceiveMsg(UART_HandleTypeDef *huart, unsigned char *msg_to_receive);

void BT_ProcessMsg(unsigned char *msg);

#endif
