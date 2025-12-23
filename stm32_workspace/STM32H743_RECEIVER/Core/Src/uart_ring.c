/*
 * uart_ring.c
 *
 *  Created on: 23.12.2025
 *      Author: Windows
 */

#include "uart_ring.h"

static uint8_t  rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static UART_HandleTypeDef *uart_handle = NULL;


void UART_Ring_Init(UART_HandleTypeDef *huart)
{
    uart_handle = huart;
    rx_head = rx_tail = 0;
}

static inline void ring_push(uint8_t b)
{
    uint16_t next = (rx_head + 1) % UART_RX_BUF_SIZE;
    if (next != rx_tail) {
        rx_buf[rx_head] = b;
        rx_head = next;
    }
}

int UART_Ring_GetByte(uint8_t *b)
{
    if (rx_head == rx_tail)
        return 0;

    *b = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

void UART_Ring_IRQ_Handler(UART_HandleTypeDef *huart)
{
    if (huart->Instance->ISR & USART_ISR_RXNE_RXFNE)
    {
        uint8_t b = (uint8_t)(huart->Instance->RDR & 0xFF);
        ring_push(b);
    }
}
