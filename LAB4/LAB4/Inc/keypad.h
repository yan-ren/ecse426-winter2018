#ifndef KEYPAD_H
#define KEYPAD_H

/* GROUP C
*/

#define col1 GPIO_PIN_1 //pin 1 on keypad
#define col2 GPIO_PIN_2 //pin 2 on keypad
#define col3 GPIO_PIN_4 //pin 3 on keypad
#define row1 GPIO_PIN_5 //pin 5 on keypad
#define row2 GPIO_PIN_10 //pin 6 on keypad
#define row3 GPIO_PIN_14 //pin 7 on keypad
#define row4 GPIO_PIN_15 //pin 8 on keypad

extern int debounce;
extern int keyLock;

char convertToChar(int col, int row);
void Keypad_Config(void);
int findRow(void);
char readKeypad(void);
char readKeypadNoDebounce(void);


#endif
