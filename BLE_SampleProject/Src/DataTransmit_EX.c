/**
  ******************************************************************************
  * @file    DataTransmit_EX.c
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
 
 #include "DataTransmit_EX.h"
 
/* Private variables ---------------------------------------------------------*/
//volatile uint8_t set_connectable = 1;
//volatile uint16_t connection_handle = 0;
//volatile uint8_t notification_enabled = FALSE;
//volatile AxesRaw_t axes_data = {0, 0, 0};
uint16_t DATAServHandle, DATACharHandle, DATA_UPCharHandle;
volatile DataRaw_t EXAMPLE = {{1, 1, 5, 6, 5, 8, 9, 5 , 4 , 7}};




 /* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
 
#if NEW_SERVICES
  #define COPY_DATATRANSMIT_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x99,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0x38, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_DATATRANSMIT_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x98,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9b,0x39, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_DATA_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x97,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9b,0x39, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#else
  #define COPY_DATATRANSMIT_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x99,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0x38, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_DATATRANSMIT_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x98,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9b,0x39, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_DATA_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x97,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9b,0x39, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/**
 * @brief  Add an accelerometer service using a vendor specific profile.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_DATATRANSMIT_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_DATATRANSMIT_UUID(uuid);
	
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &DATAServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
	
	/*COPY_DATA_UUID(uuid);
  ret =  aci_gatt_add_char(DATAServHandle, UUID_TYPE_128, uuid, 1,
                           CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 0, &DATA_UPCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;*/
  
  COPY_DATATRANSMIT_CHAR_UUID(uuid);  
	
  ret =  aci_gatt_add_char(DATAServHandle, UUID_TYPE_128, uuid, 6,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &DATACharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("Service DATATRANSMIT added. Handle 0x%04X DATACharHandle : 0x%04X\n",DATAServHandle,DATACharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding DATATRANSMIT service.\n");
  return BLE_STATUS_ERROR ;
    
}

/**
 * @brief  Update acceleration characteristic value.
 *
 * @param  Structure containing acceleration value in mg
 * @retval Status
 */
tBleStatus Data_Update(DataRaw_t *data)
{  
  tBleStatus ret;    
  uint8_t buff[20];
    
  STORE_LE_16(buff,data->Data[0]);
	STORE_LE_16(buff+2,data->Data[1]);
  STORE_LE_16(buff+4,data->Data[2]);
	STORE_LE_16(buff+6,data->Data[3]);
	STORE_LE_16(buff+8,data->Data[4]);
	STORE_LE_16(buff+10,data->Data[5]);
	STORE_LE_16(buff+12,data->Data[6]);
	STORE_LE_16(buff+14,data->Data[7]);
	STORE_LE_16(buff+16,data->Data[8]);
	STORE_LE_16(buff+18,data->Data[9]);
	
  ret = aci_gatt_update_char_value(DATAServHandle, DATACharHandle, 0, 20, buff);
	
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating ACC characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;	
}








 
 


/**
 * @brief  Send a notification detection.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus DATA_Notify(void)
{  
  uint8_t val;
  tBleStatus ret;
	
  val = 0x01;	
  ret = aci_gatt_update_char_value(DATAServHandle, DATA_UPCharHandle, 0, 1,
                                   &val);
	
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating DATA characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;	
}


