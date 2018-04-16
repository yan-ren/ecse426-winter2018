/**
  ******************************************************************************
  * File Name          : tim.h
  * Description        : Header File for tim.c
  ******************************************************************************
 **/ 
#ifndef _TIM_H   /* Include guard */
#define _TIM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim3;

/* TIM3 init function */
void MX_TIM3_Init(void);

#ifdef __cplusplus
}
#endif	 
#endif // TIM_H_
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
