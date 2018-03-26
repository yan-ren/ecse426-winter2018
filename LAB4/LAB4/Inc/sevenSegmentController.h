#ifndef SEGMENT_CONTROLLER_H
#define SEGMENT_CONTROLLER_H

#define SEG7_A GPIO_PIN_7			//Pin PE7
#define SEG7_B GPIO_PIN_8			//Pin PE8
#define SEG7_C GPIO_PIN_9			//Pin PE9
#define SEG7_D GPIO_PIN_10		//Pin PE10
#define SEG7_E GPIO_PIN_11		//Pin PE11
#define SEG7_F GPIO_PIN_12		//Pin PE12
#define SEG7_G GPIO_PIN_13		//Pin PE13
#define SEG7_DP GPIO_PIN_14		//Pin PE14

#define SEG7_OUT1 GPIO_PIN_2		//Pin PE2
#define SEG7_OUT2 GPIO_PIN_4		//Pin PE4
#define SEG7_OUT3 GPIO_PIN_5		//Pin PE5
#define SEG7_OUT4 GPIO_PIN_6		//Pin PE6

#define DISPLAY_7_SEGMENT_PERIOD 1 				/* if too high causes dimming */
#define TIME_DISPLAY_1_DIGIT_PERIOD 2

/* USER CODE END Private defines */
extern volatile int display7segTimer;
extern volatile int timeDisplay1DigitTimer;
extern volatile int digitToDisplay;

void Display_GPIO_Config(void);
void displayLEDValue(int number, int position);
void display(float value);
int getDigit(float value, int place);

#endif
