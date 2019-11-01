#include "ZComDef.h"
#include "OSAL.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_sleep.h"
#include "hal_mcu.h"
#include "SimpleApp.h"

#include "mt.h"
#include "IIC.h"

#define  MYDEVID                          0x11

// Application States
#define APP_INIT                           0    // Initial state
#define APP_START                          1    // Sensor has joined network
#define APP_BOUND                          2    // Sensor is bound to collector

// Application osal event identifiers
// Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                0x0001
#define MY_REPORT_EVT               0x0002

#define REPORT_DELAY                (1*1000)   //发送数据延时



#define MMA7660_XOUT  0x00   // 6-bit output value X 
#define MMA7660_YOUT  0x01   // 6-bit output value Y 
#define MMA7660_ZOUT  0x02   // 6-bit output value Z

#define IIC_Write  0x98
#define IIC_Read   0x98

uint8 MMA7660_XOut; 
uint8 MMA7660_YOut;
uint8 MMA7660_ZOut;



/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* LOCAL VARIABLES
*/

static uint8 myAppState = APP_INIT;

static uint16 myStartRetryDelay = 10000;      // milliseconds


/*********************************************************************
* GLOBAL VARIABLES
*/

// Inputs and Outputs for Switch device
#define NUM_IN_CMD_SENSOR                 2
#define NUM_OUT_CMD_SENSOR                3


const cId_t zb_InCmdList[NUM_IN_CMD_SENSOR] =
{
	ID_CMD_READ_REQ,
	ID_CMD_WRITE_REQ,
};
const cId_t zb_OutCmdList[NUM_OUT_CMD_SENSOR] =
{
	ID_CMD_READ_RES,
	ID_CMD_WRITE_RES,
	ID_CMD_REPORT,
};


#define TEMP_REPORT     0x01
#define BATTERY_REPORT 0x02


// Define SimpleDescriptor for Switch device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
	MY_ENDPOINT_ID,             //  Endpoint
	MY_PROFILE_ID,              //  Profile ID
	DEV_ID_SENSOR,              //  Device ID
	DEVICE_VERSION_SENSOR,      //  Device Version
	0,                          //  Reserved
	NUM_IN_CMD_SENSOR,          //  Number of Input Commands
	(cId_t *) zb_InCmdList,     //  Input Command List
	NUM_OUT_CMD_SENSOR,         //  Number of Output Commands
	(cId_t *) zb_OutCmdList     //  Output Command List
};

/********************************************************
*
*/
void zb_HanderMsg(osal_event_hdr_t *msg);

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void myReportData(void);


/*****************************************************************************
* @fn          zb_HandleOsalEvent
*
* @brief       The zb_HandleOsalEvent function is called by the operating
*              system when a task event is set
*
* @param       event - Bitmask containing the events that have been set
*
* @return      none
*/
void zb_HandleOsalEvent( uint16 event )
{

	if (event & ZB_ENTRY_EVENT) {
		uint8 startOptions;
		uint8 logicalType;

		zb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType );
		if ( logicalType != ZG_DEVICETYPE_ENDDEVICE )
		{
			logicalType = ZG_DEVICETYPE_ENDDEVICE;

			zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
		}

		// Do more configuration if necessary and then restart device with auto-start bit set
		// write endpoint to simple desc...dont pass it in start req..then reset
		zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
		if (startOptions != ZCD_STARTOPT_AUTO_START) {
			startOptions = ZCD_STARTOPT_AUTO_START;
			zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
		} 

		HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
		HalLedSet( HAL_LED_2, HAL_LED_MODE_FLASH );

                //iic初始化
                IICInit();
                halSleep(250); 
                RegWrite(IIC_Write,0x07,0x00);  //0x07 MODE Mode 默认模式Standby Mode is enabled
                RegWrite(IIC_Write,0x08,0x02);  //0x08 SR Auto-Wake/Sleep
                RegWrite(IIC_Write,0x07,0x11);  ///Auto-Sleep is enabled, Active mode 
                halSleep(250); 
	}

	if ( event & MY_START_EVT )
	{  
		zb_StartRequest();
	}

	if (event & MY_REPORT_EVT) {

		myReportData();
		osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, REPORT_DELAY );    
	}
}
/*********************************************************************
* @fn      zb_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 EVAL_SW4
*                 EVAL_SW3
*                 EVAL_SW2
*                 EVAL_SW1
*
* @return  none
*/
void zb_HandleKeys( uint8 shift, uint8 keys )
{

}
/******************************************************************************
* @fn          zb_StartConfirm
*
* @brief       The zb_StartConfirm callback is called by the ZigBee stack
*              after a start request operation completes
*
* @param       status - The status of the start operation.  Status of
*                       ZB_SUCCESS indicates the start operation completed
*                       successfully.  Else the status is an error code.
*
* @return      none
*/
void zb_StartConfirm( uint8 status )
{
	if ( status == ZB_SUCCESS )//成功加入zigbee网络
	{
		myAppState = APP_START;
		HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );//LED灯常亮
		// 设置定时器触发MY_REPORT_EVT事件
		osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, REPORT_DELAY );
	}
	else
	{
		// 入网失败，设置定时器触发MY_START_EVT事件来重启ZStack协议栈
		osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
	}
}
/******************************************************************************
* @fn          zb_SendDataConfirm
*
* @brief       The zb_SendDataConfirm callback function is called by the
*              ZigBee after a send data operation completes
*
* @param       handle - The handle identifying the data transmission.
*              status - The status of the operation.
*
* @return      none
*/
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
	if ( status != ZSuccess )
	{
		// Remove bindings to the existing collector
	}
	else
	{
		// send data ??
	}
}
/******************************************************************************
* @fn          zb_BindConfirm
*
* @brief       The zb_BindConfirm callback is called by the ZigBee stack
*              after a bind operation completes.
*
* @param       commandId - The command ID of the binding being confirmed.
*              status - The status of the bind operation.
*
* @return      none
*/
void zb_BindConfirm( uint16 commandId, uint8 status )
{


}
/******************************************************************************
* @fn          zb_AllowBindConfirm
*
* @brief       Indicates when another device attempted to bind to this device
*
* @param
*
* @return      none
*/
void zb_AllowBindConfirm( uint16 source )
{
}
/******************************************************************************
* @fn          zb_FindDeviceConfirm
*
* @brief       The zb_FindDeviceConfirm callback function is called by the
*              ZigBee stack when a find device operation completes.
*
* @param       searchType - The type of search that was performed.
*              searchKey - Value that the search was executed on.
*              result - The result of the search.
*
* @return      none
*/
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
}


void zb_HanderMsg(osal_event_hdr_t *msg)
{
}

/******************************************************************************
* @fn          zb_ReceiveDataIndication
*
* @brief       The zb_ReceiveDataIndication callback function is called
*              asynchronously by the ZigBee stack to notify the application
*              when data is received from a peer device.
*
* @param       source - The short address of the peer device that sent the data
*              command - The commandId associated with the data
*              len - The number of bytes in the pData parameter
*              pData - The data sent by the peer device
*
* @return      none
*/
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{

}


/*读取到MMA7660传感器的值
-------------------------------------------------------*/
static uint8 MMA7660_GetResult(uint8 Regs_Addr) 
{
  uint8 ret;
   
  if(Regs_Addr>MMA7660_ZOUT)
  return 0;
   
  ret=RegRead(IIC_Read,Regs_Addr);
  while(ret&0x40) 
  {
    RegRead(IIC_Read,Regs_Addr);
  } 
  return ret;
}


static void myReportData(void)
{
	int len = 5, i;
	uint8 dat[5];  

        
        MMA7660_XOut = MMA7660_GetResult(MMA7660_XOUT);
        halSleep(250); 
        MMA7660_YOut = MMA7660_GetResult(MMA7660_YOUT);
        halSleep(250); 
        MMA7660_ZOut = MMA7660_GetResult(MMA7660_ZOUT);
        halSleep(250); 
        

	//LED灯闪烁1次
	HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
	HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );

        if (MMA7660_XOut>99 || MMA7660_YOut>99 || MMA7660_ZOut>99)
          return ;
	//将节点信息进行封装
	dat[0] = 0x05;
	dat[1] = MMA7660_XOut;
        dat[2] = MMA7660_YOut;
        dat[3] = MMA7660_ZOut;
        
	//    for (i=0;i<len;i++)
	//        dat[2] += dat[i];
	//将数据包发送给协调器
	zb_SendDataRequest(0, ID_CMD_REPORT, len, dat, 0, AF_ACK_REQUEST, 0 );  
}