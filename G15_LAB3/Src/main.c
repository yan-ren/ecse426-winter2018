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
#include "math.h"
#include <stdlib.h>
#include "keypad.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
volatile int adcTimer = 0;
volatile int pushButtonTimer = 0;
volatile int display7segTimer = 0;
volatile int digitToDisplay = 0;
volatile int timeDisplay1DigitTimer = 0;
volatile int pwmUpdateTimer = 0;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int x[] = {0, 0, 0, 0, 0};
float b[] = {0.2, 0.2, 0.2, 0.2, 0.2};
int count = 0;
float min_val = 10.0;
float max_val = 0.0;
float rms_counter = 0.0;
//
uint8_t adcRawValue;
float adcFilteredValue = 0.0;
float voltageReading;
//
char keypadInput;											// button pressed by the user
float inputValue = 0.0;
int debounce = 0;												// used to debounced the keypad
int keyLock = 1;												// mutex for keypressed
int systemState = START_STATE;					// State of the state machine
int pwmPhaseValue = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void TM_Delay_Init(void);
void TM_DelayMicros(uint32_t micros);
void TM_DelayMillis(uint32_t millis);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void plotPoint(float, float *);
void displayLEDValue(int number, int position);
void display(float value);
int getDigit(float value, int place);
void FIR_C(int input, float *output);
void userPWMSetValue(int value);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	TM_Delay_Init();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
	/* Initialize Keypad*/
	Keypad_Config();

  /* USER CODE BEGIN 2 */
	// start TIM3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	// Start TIM2 Timer
	HAL_TIM_Base_Start(&htim2);
	// Start ADC
	HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*************** START STATE **************************/
		/* start state, just display adc and wait for enter */
		while (systemState == START_STATE) {
			inputValue = 0.0;
			userPWMSetValue(99);
			printf("voltage reading: %f\n", voltageReading);
//			printf("MRS: %f, MIN: %f, MAX: %f\n", results[0], results[1], results[2]);

			/* update the value to be displayed */
			if(display7segTimer >= DISPLAY_7_SEGMENT_PERIOD) {
				display7segTimer = 0;
				display(voltageReading);
			}

			/* read which keypad button is pressed */
			keypadInput = readKeypad();
			if (keypadInput == '#') {
				systemState = INPUT_STATE_1;
			}
		}

		/************* INPUT STATE 1*****************/
		while (systemState == INPUT_STATE_1) {
			/* update the value to be displayed */
			if(display7segTimer >= DISPLAY_7_SEGMENT_PERIOD) {
				display7segTimer = 0;
				display(inputValue);
			}

			/* read input from keypad */
			keypadInput = readKeypad();
			if(keypadInput == '#'){
				systemState = INPUT_STATE_2;
			}
			else if (keypadInput != 'n'  && keypadInput != '*') {
				inputValue = (float)(keypadInput - '0');
//				printf("input value %f", inputValue);
			}
			else if(keypadInput == '*'){
				systemState = START_STATE;
			}
		}

		/************* INPUT STATE 2*****************/
		while (systemState == INPUT_STATE_2) {
			/* update the value to be displayed */
			if(display7segTimer >= DISPLAY_7_SEGMENT_PERIOD) {
				display7segTimer = 0;
				display(inputValue);
			}

			/* read input from keypad */
			keypadInput = readKeypad();
			if(keypadInput == '#'){
				systemState = MEASURE_STATE;
			}
			else if (keypadInput != 'n'  && keypadInput != '*') {
				inputValue = inputValue + (float)(keypadInput - '0') / 10.0;
//				printf("input value %f", inputValue);
			}
			else if(keypadInput == '*'){
				inputValue = roundf (inputValue - ((float)((int)(inputValue*10) % 10) / 10.0));
				systemState = INPUT_STATE_1;
			}
		}

		/************* MEASURE STATE *****************/
		while(systemState == MEASURE_STATE){

			/* update the value to be displayed */
			if(display7segTimer >= DISPLAY_7_SEGMENT_PERIOD) {
				display7segTimer = 0;
				display(voltageReading);
			}

			/* update PWN */
			if(pwmUpdateTimer >= PWM_UPDATE_PERIOD){
				pwmUpdateTimer = 0;
			if(voltageReading < inputValue){
				pwmPhaseValue = pwmPhaseValue + 1;
			}else if(voltageReading > inputValue){
				pwmPhaseValue = pwmPhaseValue - 1;
			}
			userPWMSetValue(pwmPhaseValue);
		}
			/* button holding*/
			int held = 0;
			while (	readKeypadNoDebounce() == '*' && held < 5)
			{
				TM_DelayMillis(1000);// delay 1s
				held++;
			}
			if (held > 2)
			{

				systemState = SLEEP_STATE;
			}
			else if(held > 0)
			{
				 // the button was released in less than 3 seconds
				  systemState = START_STATE;
			}

		}

		while(systemState == SLEEP_STATE)
		{
			// pwn output 0
			userPWMSetValue(0);
			// led off
			HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);

			int held = 0;
			while (	readKeypadNoDebounce() == '#' && held < 5)
			{
				TM_DelayMillis(1000);// delay 1s
				held++;
			}
			if (held > 2)
			{
				  // the button was released in longer than 3 seconds
				systemState = START_STATE;
			}
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

     /* Configure SysTick to generate an interrupt every microsecond */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
		/*1k*/
	 //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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

/* TIM2 init function */
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
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 336-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_PERIOD - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

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
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
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

	//7-segment display
	GPIO_InitStruct.Pin = SEG7_A | SEG7_B | SEG7_C | SEG7_D | SEG7_E | SEG7_F | SEG7_G | SEG7_DP | SEG7_OUT1 | SEG7_OUT2 | SEG7_OUT3 | SEG7_OUT4 ; 	// initialize the 7-segment display for the LEDs pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //set pins to output push pull mode
	GPIO_InitStruct.Pull = GPIO_NOPULL; // no pull for 7-segment pins
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; //Speed of pin

	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); // Initiate the pins with the setting from GPIO_INIT
	// when initialize, set all the pins to LOW
	HAL_GPIO_WritePin(GPIOE, SEG7_A | SEG7_B | SEG7_C | SEG7_D | SEG7_E | SEG7_F | SEG7_G | SEG7_DP | SEG7_OUT1 | SEG7_OUT2 | SEG7_OUT3 | SEG7_OUT4, GPIO_PIN_RESET);


}

/**
	* @brief Display float number in seven segment
	* @param float in x.xx format
	* @retval None
	*/
void display(float value) {
		/* change the digit to be viewed slower for the 7-segment slower for better display */
		if (timeDisplay1DigitTimer >= TIME_DISPLAY_1_DIGIT_PERIOD) {
			timeDisplay1DigitTimer = 0;
			digitToDisplay = (digitToDisplay + 1) % 3;
		}
		//display digits from left side, +1 for seven segments digit offset
	displayLEDValue(getDigit(value, digitToDisplay), digitToDisplay + 2);
}

/**
	* @brief Retrive the digits in float number
	* @param value:	x.xx
	* @param place: 2 10
	*/
int getDigit(float value, int place) {
	int tmp = (int) (value * 100);
	switch (place) {
		case 0:
			return (tmp - tmp % 100) / 100;
		case 1:
			return (tmp % 100 - tmp % 10) / 10;
		case 2:
			return tmp % 10;
		default:
			return 0;
	}
}

/**
	* @brief Update PWM pulse value
	* @param value: sConfigOC.Pulse
	*/
void userPWMSetValue(int value)
{
	if(value > PWM_PERIOD){
		value = PWM_PERIOD;
	}
	if(value < 0){
		value = 0;
	}
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
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

/**
	* @brief Allow to display the value on a segment display depending on the number and position
	* @param number: number to be displayed
  * @param position: digit of number to be displayed
	*/
void displayLEDValue(int number, int position){
	// set a specific value to set all the segment pins to low
	if (number == -1){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);
	}

	else if (number == 1){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);
	}

	else if (number == 2){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}

	else if (number == 3){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}

	else if (number == 4){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}

	else if (number == 5){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}

	else if (number == 6){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}

	else if (number == 7){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);
	}

	else if (number == 8){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}

	else if (number == 9){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}

	else if (number == 0){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);

	}

	// Set which display to display the number.
	if (position == -1){
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}

	else if (position == 1){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}

	else if (position == 2){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_SET);
	}

	else if (position == 3){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}

	else if (position == 4){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}
}

/**
	* @brief RMS MAX MIN calculation function
	* @param input
  * @param output
	*/
void plotPoint(float input, float* output) {

	if(count == 0) {
		min_val = input;
		max_val = input;
		rms_counter = input * input;
	} else {
		if(input < min_val) min_val = input;
		else if(input > max_val) max_val = input;
		rms_counter += input * input;
	}
	count = (count + 1) % 500;
	output[0] = sqrt(rms_counter / count);
	output[1] = min_val;
	output[2] = max_val;
}

/**
	* @brief ADC Conversion Call back Function
	* @param number: number to be displayed
  * @param position: digit of number to be displayed
	*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
	adcRawValue = HAL_ADC_GetValue(&hadc1);
	FIR_C(adcRawValue, &adcFilteredValue);
	voltageReading = (3.0 * adcRawValue / 255.0);
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
	* @brief This function cause dealy in microsecond
	*	@param micros:	delay time in microsecond
	*/
void TM_DelayMicros(uint32_t micros) {
    /* Multiply micros with multipler */
    /* Substract 10 */
    micros = micros * multiplier - 10;
    /* 4 cycles for one loop */
    while (micros--);
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
