/**
  ******************************************************************************
  * @file    main.c 
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   This application contains an example which shows how implementing
  *          a proprietary Bluetooth Low Energy profile: the sensor profile.
  *          The communication is done using a Nucleo board and a Smartphone
  *          with BTLE.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "cube_hal.h"

#include "osal.h"
#include "sensor_service.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
#include "tim.h"
#include "gpio.h"
#include "uart.h"

#include <string.h>
#include <stdio.h>

uint8_t p_data [10000] = {0};
uint8_t p_data_p [2000] = {0};
//uint8_t p_data_r [1000];

uint8_t rece_buffer [1] = {100};

int count = 0;
int count_p = 0;
//int count_r = 0;

int rece_flag = 0;
int rece_flag_p = 0;
//int rece_flag_r = 0;

int send_flag = 0;
int send_flag_p = 0;
//int send_flag_r = 0;


/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @defgroup SensorDemo
 *  @{
 */

/** @defgroup MAIN 
 * @{
 */

/** @defgroup MAIN_Private_Defines 
 * @{
 */
/* Private defines -----------------------------------------------------------*/
#define BDADDR_SIZE 6

/**
 * @}
 */
 
/* Private macros ------------------------------------------------------------*/

/** @defgroup MAIN_Private_Variables
 * @{
 */
/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t set_connectable;
extern volatile int connected;
extern AxesRaw_t axes_data;
extern uint8_t SendData;
uint16_t sample[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */
/**
 * @}
 */

/** @defgroup MAIN_Private_Function_Prototypes
 * @{
 */
/* Private function prototypes -----------------------------------------------*/
void User_Process(AxesRaw_t* p_axes);
void User_Process_DATA(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/**
 * @}
 */

/**
 * @brief  Main function to show how to use the BlueNRG Bluetooth Low Energy
 *         expansion board to send data from a Nucleo board to a smartphone
 *         with the support BLE and the "BlueNRG" app freely available on both
 *         GooglePlay and iTunes.
 *         The URL to the iTunes for the "BlueNRG" app is
 *         http://itunes.apple.com/app/bluenrg/id705873549?uo=5
 *         The URL to the GooglePlay is
 *         https://play.google.com/store/apps/details?id=com.st.bluenrg
 *         The source code of the "BlueNRG" app, both for iOS and Android, is
 *         freely downloadable from the developer website at
 *         http://software.g-maps.it/
 *         The board will act as Server-Peripheral.
 *
 *         After connection has been established:
 *          - by pressing the USER button on the board, the cube showed by
 *            the app on the smartphone will rotate.
 *          
 *         The communication is done using a vendor specific profile.
 *
 * @param  None
 * @retval None
 */
int main(void){

  const char *name = "BlueG06";
  uint8_t SERVER_BDADDR[] = {0x11, 0x34, 0x00, 0xE1, 0x80, 0x03};
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  uint8_t  hwVersion;
  uint16_t fwVersion;
  
  int ret;
  
	HAL_Init();
  
  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /* Configure the system clock */
	/* SYSTEM CLOCK = 32 MHz */
  SystemClock_Config();
	
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART6_UART_Init();

	
		
  
  
  /* STM32Cube HAL library initialization:
   *  - Configure the Flash prefetch, Flash preread and Buffer caches
   *  - Systick timer is configured by default as source of time base, but user 
   *    can eventually implement his proper time base source (a general purpose 
   *    timer for example or other time source), keeping in mind that Time base 
   *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
   *    handled in milliseconds basis.
   *  - Low Level Initialization
   */
//  HAL_Init();
  
#if NEW_SERVICES
  /* Configure LED2 */
  BSP_LED_Init(LED2); 
#endif
  
  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /* Configure the system clock */
	/* SYSTEM CLOCK = 32 MHz */
  SystemClock_Config();

  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
  
  /* Initialize the BlueNRG HCI */
  HCI_Init();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();
    
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /* 
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  BlueNRG_RST();
  
  PRINTF("HWver %d, FWver %d", hwVersion, fwVersion);
	PRINTF("\n\n");
  
  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1; 
    /*
     * Change the MAC address to avoid issues with Android cache:
     * if different boards have the same MAC address, Android
     * applications unless you restart Bluetooth on tablet/phone
     */
    SERVER_BDADDR[5] = 0x02;
  }

  /* The Nucleo board must be configured as SERVER */
  Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret){
    PRINTF("Setting BD_ADDR failed.\n");
  }
  
  ret = aci_gatt_init();    
  if(ret){
    PRINTF("GATT_Init failed.\n");
  }

  if (bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x03, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret != BLE_STATUS_SUCCESS){
    PRINTF("GAP_Init failed.\n");
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(name), (uint8_t *)name);

  if(ret){
    PRINTF("aci_gatt_update_char_value failed.\n");            
    while(1);
  }
  
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
    PRINTF("BLE Stack Initialized.\n");
  }
  
  PRINTF("SERVER: BLE Stack Initialized\n");
  
  ret = Add_Acc_Service();
  
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Acc service added successfully.\n");
  else
    PRINTF("Error while adding Acc service.\n");
  
  ret = Add_Environmental_Sensor_Service();
  
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Environmental Sensor service added successfully.\n");
  else
    PRINTF("Error while adding Environmental Sensor service.\n");

#if NEW_SERVICES
  /* Instantiate Timer Service with two characteristics:
   * - seconds characteristic (Readable only)
   * - minutes characteristics (Readable and Notifiable )
   */
  ret = Add_Time_Service(); 
  
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Time service added successfully.\n");
  else
    PRINTF("Error while adding Time service.\n");  
  
  /* Instantiate LED Button Service with one characteristic:
   * - LED characteristic (Readable and Writable)
   */  
  ret = Add_LED_Service();

  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("LED service added successfully.\n");
  else
    PRINTF("Error while adding LED service.\n");  
#endif

		ret = Add_DATATRANSMIT_Service();
  
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Add_DATATRANSMIT_Service added successfully.\n");
  else
    PRINTF("Error while adding Add_DATATRANSMIT_Service.\n");
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);
	
	while(1){		
		
		while(1)
			{
				if(rece_flag == 1){
					HAL_UART_Receive(&huart6, p_data, 10000, 5000);
					rece_flag = 0;
					send_flag = 1;
					for(int i = 0 ; i < 10000; i++){
						printf("no %d: %d\n", i, p_data[i]);
					}
					break;
				}
				if(rece_flag_p == 1){
					HAL_UART_Receive(&huart6, p_data_p, 2000, 5000);
					rece_flag_p = 0;
					send_flag_p = 1;
					for(int i = 0 ; i < 2000; i++){
						printf("no %d: %d\n", i, p_data_p[i]);
					}
					break;
				}
				HAL_UART_Receive_IT(&huart6, rece_buffer, 1);
				while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY){};
				if(rece_buffer[0] == 0){
					rece_flag = 1;
					printf("receive flag changed. \n");
					
				}
				if(rece_buffer[0] == 1){
					rece_flag_p = 1;
					printf("receive pr flag changed. \n");
					
				}
			}
		
		printf("outta loop 1\n");
		while(1)
		{
			
			//printf("send data. \n");
			HCI_Process();
			//User_Process(&axes_data);
			//User_Process_DATA();
			printf("hci done. \n");
			
			if(set_connectable){
				setConnectable();
				set_connectable = FALSE;
			}  
			printf("conn done. \n");

  /* Check if the user has pushed the button */
			if(BSP_PB_GetState(BUTTON_KEY) == RESET)
			{
				while (BSP_PB_GetState(BUTTON_KEY) == RESET){
				};
				
				//BSP_LED_Toggle(LED2); //used for debugging (BSP_LED_Init() above must be also enabled)
				printf("in user process. \n");
				if(connected)
				{
					if (send_flag == 1){	
						//Data_Update(0);
						for(int i=0; i<10000; i++) {
							SendData = p_data[i];
							Data_Update(SendData);
							HAL_Delay(10);
							printf("SendData: %d\n", SendData );
						}	
		        printf("audio sent. \n");				
						send_flag = 0;
				#if NEW_SERVICES
						Update_Time_Characteristics();
				#endif
						break;
					}
					if (send_flag_p == 1){	
						//Data_Update(1);				
						for(int i=0; i<2000; i++) {
							SendData = p_data_p[i];
							Data_Update(SendData);
							HAL_Delay(15);
							printf("SendData: %d\n", SendData );
						}
						printf("pitch and roll sent. \n");	
						send_flag_p = 0;
				#if NEW_SERVICES
						Update_Time_Characteristics();
				#endif
						break;
					}
				}
			}
		}
	}
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  AxesRaw_t* p_axes
 * @retval None
 */
void User_Process(AxesRaw_t* p_axes)
{
  if(set_connectable){
    setConnectable();
    set_connectable = FALSE;
  }  

  /* Check if the user has pushed the button */
  if(BSP_PB_GetState(BUTTON_KEY) == RESET)
  {
    while (BSP_PB_GetState(BUTTON_KEY) == RESET);
    
    //BSP_LED_Toggle(LED2); //used for debugging (BSP_LED_Init() above must be also enabled)
    
    if(connected)
    {
      /* Update acceleration data */
      p_axes->AXIS_X += 10;
      p_axes->AXIS_Y -= 10;
      p_axes->AXIS_Z += 20;
      //PRINTF("ACC: X=%6d Y=%6d Z=%6d\r\n", p_axes->AXIS_X, p_axes->AXIS_Y, p_axes->AXIS_Z);
      Acc_Update(p_axes);
    }
  }
}

//void User_Process_DATA(void)
//{

//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	//HAL_GPIO_TogglePin( GPIOA, GPIO_PIN_5);
	//HAL_Delay(3);
	if(p_data[0] != 0)
	{
		//HAL_GPIO_TogglePin( GPIOA, GPIO_PIN_5);
	}
	else{
		//HAL_GPIO_TogglePin( GPIOA, GPIO_PIN_5);
	}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}

/**
 * @}
 */
 
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
