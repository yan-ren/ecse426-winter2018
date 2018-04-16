/**
  ******************************************************************************
  * File Name          : uart.h
  * Description        : Header File for uart.c
  ******************************************************************************
 **/ 
#ifndef _UART_H   /* Include guard */
#define _UART_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart6;

/* USART1 init function */

void MX_USART6_UART_Init(void);
#ifdef __cplusplus
}
#endif	 
#endif // UART_H_
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/





