#ifndef __UARTBUF_H
#define __UARTBUF_H

#include "main.h"

#define buf_max 200

typedef struct
{
	uint16_t head;
	uint16_t tail;
	uint16_t lengh;
	uint8_t buff[buf_max];
}uart_buf;

extern uart_buf uartbuf1;

uint8_t write_buf(uart_buf *buf,uint8_t data);
uint8_t read_buf(uart_buf *buf,uint8_t *data);
void UART_IDLECallback(UART_HandleTypeDef *huart,DMA_HandleTypeDef * hdma_rx,uart_buf *buf,uint8_t *rdata,uint16_t size);

#endif
