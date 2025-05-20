/*
/ * @file       deca_uart.c
*
* @brief      HW specific definitions and functions for UART Interface
*
* @author     Decawave Software
*
* @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
*             All rights reserved.
*
*/

#include "port_platform.h"
#include "instance.h"
#include "stdbool.h"

// must be power of 2 to create fifo succesfully
#define RX_BUF_SIZE 256
/******************************************************************************
*
*                              APP global variables
*
******************************************************************************/
static uint8_t rx_buf[RX_BUF_SIZE];
static uint32_t rx_buf_index = 0;

static bool uart_rx_data_ready = false;

/******************************************************************************
 *
 *                              Uart Configuration
 *
 ******************************************************************************/

int __io_putchar (int ch)
{
	// TODO need to implement timeout
	// or may be not...
	while ( !LL_USART_IsActiveFlag_TXE( USART2 ) );
	LL_USART_TransmitData8(USART2, (uint8_t)ch);
	return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}

bool deca_uart_rx_data_ready(void)
{
    return uart_rx_data_ready;
}

extern bool SerialData;

void UART_RxCpltCallback( uint8_t data )
{

    /* Local echo*/
	__io_putchar((int)data);
	// wait for transmission complete
	while ( !LL_USART_IsActiveFlag_TC( USART2 ) );

	SerialData = true;

    if ( !uart_rx_data_ready  )
    {
    	/* to allow backspace in shell - thanks Yves, very useful feature */
    	if( data == '\b')
    	{
    		if(rx_buf_index !=0)
    		{
    			__io_putchar((int)' ');
    			__io_putchar((int)'\b');
    			while ( !LL_USART_IsActiveFlag_TC( USART2 ) );
    			rx_buf_index--;// = rx_buf_index - 1;
    			rx_buf[rx_buf_index] = 0;
    		}
    	}
    	else if ( data == '\r' )
        {
            uart_rx_data_ready = true;
            rx_buf[rx_buf_index] = 0;
        }
        else
        {
        	rx_buf[rx_buf_index++] = data;
        	//rx_buf_index = rx_buf_index + 1;

            if ( rx_buf_index >= RX_BUF_SIZE )
            {
                uart_rx_data_ready = true;	         // buffer is full, signal to proceed it
            }
        }
    }
    //printf("Callback: data=%c (0x%02X), index=%lu, ready=%d\n", data, data, rx_buf_index, uart_rx_data_ready);

}

void port_tx_msg(char *ptr, int len)
{
	_write(0, (char *)ptr,len);
}

/* @fn  deca_uart_receive
 *
 * @brief Function for receive data from UART buffer and store into rx_buf - either full or null-terminated if partial
 *
 * @param[in] address to buffer, max buffer size
 * @param[out] actual number of bytes in buffer
 * */
uint32_t deca_uart_receive( uint8_t * buffer, size_t size)
{
	//printf("deca_uart_receive: rx_buf_index=%lu\r\n", rx_buf_index);
	__disable_irq();

    uint32_t count = rx_buf_index;
    memcpy(buffer, rx_buf, MIN(rx_buf_index, size));
    rx_buf_index = 0;
    buffer[size-1] = 0;
    uint8_t data = '\n';

    __io_putchar(data);
    uart_rx_data_ready = false;

	__enable_irq();

    return count;
}

/****************************************************************************//**
 *
 *                          End of UART Configuration
 *
 *******************************************************************************/
