/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "lis3dsh.h"
#include "usart.h"
#include "math.h"
// grey PD2
// PURPLE PC12

/* Private variables ---------------------------------------------------------*/

LIS3DSH_InitTypeDef 		Acc_instance;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
/* Private variables ---------------------------------------------------------*/
//FIR_C filters
int x[] = {0, 0, 0, 0, 0};
float b[] = {0.2, 0.2, 0.2, 0.2, 0.2};
//
//ADC variables
uint32_t adcRawValue;
float adcFilteredValue = 0.0;
const int audioDataBufferSize = 10000;
uint8_t audioDataBuffer [audioDataBufferSize];
int audioBufferCounter = 0;
int recording = 0;
//Tap Detect variables
volatile int tapDetectTimer = 0;
volatile int tapCheckTimer = 0;
int taps = 0;
//



uint8_t status;
float Buffer[3];
float accX, accY, accZ;
uint8_t MyFlag = 0;
	
int systemState = ONE_TAP_STATE;	

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);

void TM_Delay_Init(void);
void TM_DelayMillis(uint32_t millis);

void initializeACC			(void);
void FIR_C(int input, float *output);

// LED operation functions
void ledON(void);
void Scenario_Signal(int ledtag);
//
// Accelerator detect functions
void tapFound(float* value, float* previous);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
	initializeACC	();	// Like any other peripheral, you need to initialize it. Refer to the its driver to learn more.
	MX_ADC1_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
//  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	// Step(1): Start the Timer as interrupt
	HAL_TIM_Base_Start(&htim2);
//	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_IT(&hadc1);
	
	TM_Delay_Init();
	// and example of sending a data through UART, but you need to configure the UART block:
	//	HAL_UART_Transmit(&huart5,"FinalProject\n",14,2000); 
	
	//
	float accelOutput[3];
	float previousAccelOutput[3];
	//

  while (1)
  {
 
//		if (MyFlag/200)
//		{

//			MyFlag = 0;
//			//Reading the accelerometer status register
//				LIS3DSH_Read (&status, LIS3DSH_STATUS, 1);
//				//The first four bits denote if we have new data on all XYZ axes, 
//		   	//Z axis only, Y axis only or Z axis only. If any or all changed, proceed
//				if ((status & 0x0F) != 0x00)
//				{
//					// read ACC
//					LIS3DSH_ReadACC(&Buffer[0]);
//					accX = (float)Buffer[0];
//					accY = (float)Buffer[1];
//					accZ = (float)Buffer[2];
//					printf("X: %4f     Y: %4f     Z: %4f \n", accX, accY, accZ);
//					//
//				}
//			}
		
		LIS3DSH_Read (&status, LIS3DSH_STATUS, 1);
		//The first four bits denote if we have new data on all XYZ axes, 
		//Z axis only, Y axis only or Z axis only. If any or all changed, proceed
		if ((status & 0x0F) != 0x00)
		{
			// read ACC
			LIS3DSH_ReadACC(previousAccelOutput);
//			accX = (float)accelOutput[0];
//			accY = (float)accelOutput[1];
//			accZ = (float)accelOutput[2];
			//printf("first: X: %4f     Y: %4f     Z: %4f \n", accX, accY, accZ);
			//
		}
		while(systemState == START_STATE){
			ledON();
			
			// tap detection
			if(tapDetectTimer >= TAP_DETECT_PERIOD){
				tapDetectTimer = 0;
				//
				LIS3DSH_Read (&status, LIS3DSH_STATUS, 1);
				//The first four bits denote if we have new data on all XYZ axes, 
				//Z axis only, Y axis only or Z axis only. If any or all changed, proceed
				if ((status & 0x0F) != 0x00)
				{
					// read ACC
					LIS3DSH_ReadACC(accelOutput);
//					accX = (float)accelOutput[0];
//					accY = (float)accelOutput[1];
//					accZ = (float)accelOutput[2];
					//printf("first: X: %4f     Y: %4f     Z: %4f \n", accX, accY, accZ);
					//
					
					// See if a tap has been found
					tapFound(accelOutput, previousAccelOutput);
			
					// the previous update has to be after the tap found	
					previousAccelOutput[0] = accelOutput[0];
					previousAccelOutput[1] = accelOutput[1];
					previousAccelOutput[2] = accelOutput[2];	
				}
			}
			
			if(tapCheckTimer >= TAP_CHECK_PERIOD){
				tapCheckTimer = 0;
				if(taps >= 2){
					systemState = TWO_TAP_STATE;
					taps = 0;
				}else if(taps > 0 && taps < 2){
					systemState = ONE_TAP_STATE;
					taps = 0;
				}
			}
		}
		
		while(systemState == ONE_TAP_STATE){
			//recording
			recording = 1;
			Scenario_Signal(0); // green led showing recording
			// testing
			
//			HAL_ADC_Start(&hadc1); 								/* start ADC conversion */
//			
//			/* wait for the conversion to be done and get data */
//			if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) { 
//				adcRawValue = HAL_ADC_GetValue(&hadc1); /* get the value */
//				printf("adcRawValue: %i \n", adcRawValue);

//			}
			//adcRawValue = HAL_ADC_GetValue(&hadc1);
			printf("adc raw value: %i \n", adcRawValue);
			//
			
			//check if recording finish
			if(audioBufferCounter == audioDataBufferSize){
				recording = 0;
				audioBufferCounter = 0;
				//transmit
				Scenario_Signal(1); // blue led showing transmitting
				
				TM_DelayMillis(3000);// delay 1s
				
			}
			
			// if push button detected, go back to start state
			
		}
		
		while(systemState == TWO_TAP_STATE){
			Scenario_Signal(2); // RED led showing TWO_TAP_STATE
			// read accelerameter and calculate pitch and roll
			
			
			// if push button detected, go back to start state
			
		}
		
  }


}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 1k, 1 milisecond
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function ADC timer*/
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
	if(systemState == ONE_TAP_STATE){
		while(audioBufferCounter < audioDataBufferSize && recording == 1){
			adcRawValue = HAL_ADC_GetValue(&hadc1);
			
			FIR_C(adcRawValue, &adcFilteredValue);
			float adcConvertedValue = adcFilteredValue/8;
			audioDataBuffer[audioBufferCounter] = (uint8_t) adcConvertedValue;
			printf("adcConvertedValue: %i\n", (uint8_t) adcConvertedValue);
			audioBufferCounter++;
		}
	}
}

void Scenario_Signal(int ledtag){
	if(ledtag == 0){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_RESET);
	}
	else if(ledtag == 1){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_SET);
	}
	else if(ledtag == 2){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_RESET);	
	}
	else if(ledtag == 3){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_RESET);
	}
}

void ledON()
{ 
	HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_SET);
}

/**
   * @brief A function used to calculate the pitch
	 * @param Takes in an XYZ acceleration vector
   * @retval Returns the pitch in degrees
   */
float calcPitch(float* xyz)
{
	float pitch;
	// x = 0 , y = 1, z = 2
	pitch = atan( xyz[0] / sqrt(pow(xyz[1], 2) + pow(xyz[2], 2)) ) * 180/PI;
	
	return pitch;
}
/**
   * @brief A function used to calculate the roll
	 * @param Takes in an XYZ acceleration vector
   * @retval Returns the roll in degrees
   */
float calcRoll(float* xyz)
{
	float roll;
	// x = 0 , y = 1, z = 2
	roll = atan( xyz[1] / sqrt(pow(xyz[0], 2) + pow(xyz[2], 2)) ) * 180/PI;
	
	return roll;
}

/**
   * @brief A function used to see a tap 
   * The difference is needed and a threshold of the difference of 2 values
   * Recall that it's at 25samples/sec so don't forget to X25 for threshold and difference
	 * @param Takes in values XYZ, sets a flag if "tap" is found
   * @retval None
   */
void tapFound(float* value, float* previous)
{
	float Xprev = previous[0];
	float Yprev = previous[1];
	float Zprev = previous[2];
	
	float Xthis = value[0];
	float Ythis = value[1];
	float Zthis = value[2];
	
	float Xdiff = (Xthis - Xprev);
	float Ydiff = (Ythis - Yprev);
	float Zdiff = (Zthis - Zprev);
	
	
	if (Xdiff >= (fabs)(40.0) || Ydiff >= (fabs)(40.0) || Zdiff >= (fabs)(30.0))
	{
		taps++;
	}
	// could have a signal here to notify double tap instead of a flag/integer value
//	if (TAP == 2 && msecond <= 1500000)
//	{
//		TAP = 0;
//		DOUBLETAP++ ;
//		msecond = 0;
//	}
	
}

/**
	* @brief FIR filter
	* @param value: sConfigOC.Pulse
	*/
void FIR_C(int input, float *output) {
	// shifting
	int j;
	for(j = 0; j < 4; ++j){
		x[j] = x[j+1];
	}
	// add new element in the end
	x[4] = input;

	float out = 0.0;
	int i;
	for(i = 0; i < 5; ++i) {
		out += x[i] * b[4 - i];
	}
	*output = out;
}

uint32_t multiplier;
/**
	* @brief Delay function Init
	*/
void TM_Delay_Init(void) {
    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    multiplier = HAL_RCC_GetHCLKFreq() / 4000000;
}

/**
	* @brief This function cause dealy in milisecond
	*	@param micros:	delay time in milisecond
	*/
void TM_DelayMillis(uint32_t millis) {
    /* Multiply millis with multipler */
    /* Substract 10 */
    millis = 1000 * millis * multiplier - 10;
    /* 4 cycles for one loop */
    while (millis--);
}

void initializeACC(void){
	
	Acc_instance.Axes_Enable				= LIS3DSH_XYZ_ENABLE;
	Acc_instance.AA_Filter_BW				= LIS3DSH_AA_BW_50;
	Acc_instance.Full_Scale					= LIS3DSH_FULLSCALE_2;
	Acc_instance.Power_Mode_Output_DataRate		= LIS3DSH_DATARATE_100;
	Acc_instance.Self_Test					= LIS3DSH_SELFTEST_NORMAL;
	Acc_instance.Continous_Update   = LIS3DSH_ContinousUpdate_Disabled;
	
	LIS3DSH_Init(&Acc_instance);	
	
	/* Enabling interrupt conflicts with push button
  ACC_Interrupt_Config.Dataready_Interrupt	= LIS3DSH_DATA_READY_INTERRUPT_ENABLED;
	ACC_Interrupt_Config.Interrupt_signal			= LIS3DSH_ACTIVE_HIGH_INTERRUPT_SIGNAL;
	ACC_Interrupt_Config.Interrupt_type				= LIS3DSH_INTERRUPT_REQUEST_PULSED;
	
	LIS3DSH_DataReadyInterruptConfig(&ACC_Interrupt_Config);
	*/
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
