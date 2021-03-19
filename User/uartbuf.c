#include "uartbuf.h"

uart_buf uartbuf1;

uint8_t write_buf(uart_buf *buf,uint8_t data)
{
	if(buf->lengh < buf_max)
	{
		buf->buff[buf->tail] = data;
		buf->tail = (buf->tail + 1)%buf_max;
		buf->lengh++;
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t read_buf(uart_buf *buf,uint8_t *data)
{
	if(buf->lengh != 0)
	{
		*data = buf->buff[buf->head];
		buf->head = (buf->head + 1)%buf_max;
		buf->lengh--;
		return 1;
	}
	else
	{
		return 0;
	}
}

void UART_IDLECallback(UART_HandleTypeDef *huart,DMA_HandleTypeDef * hdma_rx,uart_buf *buf,uint8_t *rdata,uint16_t size)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		//ø’œ–÷–∂œ
    HAL_UART_DMAStop(huart);  		
		uint16_t num = buf_max - __HAL_DMA_GET_COUNTER(hdma_rx);
		__HAL_DMA_DISABLE(hdma_rx);
		for(int i=0;i<num;i++)
		{
			if(write_buf(buf,*rdata) == 0)
			{
				printf("uart is full!\r\n");
				HAL_UART_Receive_DMA(huart,rdata, size); 
				return;
			}
			rdata++;
		}
		//ª÷∏¥DMAΩ” ‹
		HAL_UART_Receive_DMA(huart,rdata, size); 
		printf("uart is ok!\r\n");
	}
}

