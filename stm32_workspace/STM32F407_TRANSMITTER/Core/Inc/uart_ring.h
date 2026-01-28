/*
 * usart_ring.h
 *
 *  Created on: 23.12.2025
 *      Author: mbeuler
 */

#ifndef INC_UART_RING_H_
#define INC_UART_RING_H_


#include "main.h"

#define UART_RX_BUF_SIZE 256

void UART_Ring_Init(UART_HandleTypeDef *huart);
void UART_Ring_IRQ_Handler(UART_HandleTypeDef *huart);
int  UART_Ring_GetByte(uint8_t *b);


#endif /* INC_UART_RING_H_ */
