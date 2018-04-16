/**
  ******************************************************************************
  * @file    DataTransmit_EX.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    27-11-2014
  * @brief   Try to a set of data
  ******************************************************************************
  ******************************************************************************
  */
	/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup SensorDemo
 *  @{
 */
 
/** @defgroup SENSOR_SERVICE
 * @{
 */

/** @defgroup SENSOR_SERVICE_Private_Variables
 * @{
 */
 
 /* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _DATATRANSMIT_EX_H_
#define _DATATRANSMIT_EX_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"

#include <stdlib.h>

/**
 * @}
 */

/**
 * @}
 */
 
/**
 * @}
 */

/**
 * @}
 */
 
 /* Exported defines ----------------------------------------------------------*/   
#define IDB04A1 0
#define IDB05A1 1

/**
 * @brief Instantiate two new services:
 *        1. Timer Service with two characteristics
 *           - Seconds characteristic (Readable only)
 *           - Minutes characteristics (Readable and Notifiable)
 *        2. LED Button Service with one characteristic
 *           - LED characteristic (Readable and Writable)
 */
#define NEW_SERVICES 0
/**
 * @}
 */

/** @addtogroup SENSOR_SERVICE_Exported_Types
 *  @{
 */
typedef int i32_t;

/** 
 * @brief Structure containing acceleration value (in mg) of each axis.
 */
typedef struct {
/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup SensorDemo
 *  @{
 */
 
/** @defgroup SENSOR_SERVICE
 * @{
 */

/** @defgroup SENSOR_SERVICE_Private_Variables
 * @{
 */  
	int Data[10];
} DataRaw_t;
 
 
tBleStatus Add_DATATRANSMIT_Service(void);
tBleStatus Data_Update(DataRaw_t *data);
tBleStatus DATA_Notify(void);

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

