/**
  ******************************************************************************
  * File Name          : segment_controller.c
  * Description        : Allow to display numbers on 4 7-segments displays
	* Author						 : Auguste Lalande, Felix Dube, Juan Morency Trudel
	* Version            : 1.0.0
	* Date							 : February, 2016
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "sevenSegmentController.h"
#include "cmsis_os.h"

volatile int digitToDisplay = 0;
volatile int timeDisplay1DigitTimer = 0;

/**
	* @brief GPIO for 7-segments display init
	* @param None
	* @retval None
	*/
void Display_GPIO_Config(void) {
	/* Initialize struct */
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable clock for GPOIE */
	__HAL_RCC_GPIOE_CLK_ENABLE();

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
		osDelay(2);
		digitToDisplay = (digitToDisplay + 1) % 3;
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
