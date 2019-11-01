
/**************************************************************************************************
Filename:       SimpleApp.c
Revised:        $Date: 2007-12-04 11:46:18 -0800 (Tue, 04 Dec 2007) $
Revision:       $Revision: 16007 $

Description:    Sample application utilizing the Simple API.


Copyright 2007 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/******************************************************************************
* INCLUDES
*/

#include "ZComDef.h"
#include "OSAL.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "DebugTrace.h"
#include "SimpleApp.h"

#if defined( MT_TASK )
#include "osal_nv.h"
#endif
#include "mt_app.h"
#include "mt_uart.h"
#include "mt.h"


/*********************************************************************
* CONSTANTS
*/

// Application States
#define APP_INIT                           0
#define APP_START                          1

// Application osal event identifiers
#define MY_START_EVT                0x0001

// Same definitions as in SimpleSensor.c
#define TEMP_REPORT     0x01
#define BATTERY_REPORT 0x02
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
static int flag=0;

static uint8 myAppState = APP_INIT;
static uint8 myStartRetryDelay = 10;

#if defined( MT_TASK )
extern uint8 aExtendedAddress[8];
#endif

void zb_HanderMsg(osal_event_hdr_t *pMsg);
/*********************************************************************
* GLOBAL VARIABLES
*/
void NodeUartInit(void);
void NodeUartCallBack ( uint8 port, uint8 event );
// Inputs and Outputs for Switch device
#define NUM_OUT_CMD_COLLECTOR                2
#define NUM_IN_CMD_COLLECTOR                 3

// List of output and input commands for Switch device
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
	ID_CMD_READ_RES,
	ID_CMD_WRITE_RES,
	ID_CMD_REPORT,
};
const cId_t zb_OutCmdList[NUM_OUT_CMD_COLLECTOR] =
{
	ID_CMD_READ_REQ,
	ID_CMD_WRITE_REQ,
};

// Define SimpleDescriptor for Switch device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
	MY_ENDPOINT_ID,             //  Endpoint
	MY_PROFILE_ID,              //  Profile ID
	DEV_ID_COLLECTOR,          //  Device ID
	DEVICE_VERSION_COLLECTOR,  //  Device Version
	0,                          //  Reserved
	NUM_IN_CMD_COLLECTOR,      //  Number of Input Commands
	(cId_t *) zb_InCmdList,     //  Input Command List
	NUM_OUT_CMD_COLLECTOR,     //  Number of Output Commands
	(cId_t *) zb_OutCmdList              //  Output Command List
};

/******************************************************************************
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
	uint8 startOptions;
	uint8 logicalType;
	if (event & ZB_ENTRY_EVENT) {//zigbee协议栈入口事件处理
		zb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType );
		if ( logicalType != ZG_DEVICETYPE_COORDINATOR )//设置节点类型为协调器
		{
			logicalType = ZG_DEVICETYPE_COORDINATOR;
			//将节点类型信息写入NV存储区
			zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
		}

		// Do more configuration if necessary and then restart device with auto-start bit set
		// write endpoint to simple desc...dont pass it in start req..then reset


		zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
		if (startOptions != ZCD_STARTOPT_AUTO_START) {
			startOptions = ZCD_STARTOPT_AUTO_START;
			zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
		}
		//闪烁LED灯
		HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
		HalLedSet( HAL_LED_2, HAL_LED_MODE_FLASH );  
		NodeUartInit();
	}

}

void zb_HanderMsg(osal_event_hdr_t *msg)
{

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

	// If the device sucessfully started, change state to running
	if ( status == ZB_SUCCESS )
	{
		myAppState = APP_START;
		//zigbee网络成功建立，LED常亮
		HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
	}
	else
	{
		// Try again later with a delay
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
#include <stdio.h>


void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
	char buf[64],buf2[64];
	int ck,i;
	//接收到数据之后LED灯闪烁
	HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
	HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );


	switch(flag){
	case 1:
		//温湿度传感器
		if (len==4 && pData[0]==0x01 ) {  //&& pData[5]==ck
			sprintf(buf, "DEVID:%02X humidity:%02d temp:%02d\n", 
				pData[0], pData[1], pData[2]);
                        sprintf(buf2, "%02X%02d|%02d\n", pData[0], pData[1], pData[2]);
		} 
		break;
	case 2:
		//人体红外
		if (len==3 && pData[0]==0x02 ) {  
			sprintf(buf, "DEVID:%02X humenbody:%01d\n", 
				pData[0], pData[1]);
                        sprintf(buf2, "%02X%02d\n", pData[0], pData[1]);

		} 
		break;
	case 3:
		//超声波测距
		if (len==3 && pData[0]==0x03 ) {  
			sprintf(buf, "DEVID:%02X ultrasonic:%u\n", 
				pData[0], pData[1]);
                        sprintf(buf2, "%02X%02d\n", pData[0], pData[1]);

		} 
		break;
	case 4:
		//光敏
		if (len==3 && pData[0]==0x04 ) {  
			sprintf(buf, "DEVID:%02X photoresistance:%01d\n", 
				pData[0], pData[1]);  
                        sprintf(buf2, "%02X%02d\n", pData[0], pData[1]);

		}
		break;
	case 5:
		//三轴加速传感器
		if (len==5 && pData[0]==0x05 ) {  
			sprintf(buf, "DEVID:%02X x:%01d  y:%01d  z:%01d\n", 
				pData[0], pData[1], pData[2], pData[3]);  
                        sprintf(buf2, "%02X%02d|%02d|%02d\n", pData[0], pData[1], pData[2], pData[3]);

		}
		break;
                
	default:     
          
		//温湿度传感器
		if (len==4 && pData[0]==0x01 ) { 
			sprintf(buf, "DEVID:%02X humidity:%02d temp:%02d\n", 
				pData[0], pData[1], pData[2]);
                        sprintf(buf2, "%02X%02d|%02d\n", pData[0], pData[1], pData[2]);

		} 
		//人体红外
		if (len==3 && pData[0]==0x02 ) {  
			sprintf(buf, "DEVID:%02X humenbody:%01d\n", 
				pData[0], pData[1]);
                        sprintf(buf2, "%02X%02d\n", pData[0], pData[1]);

		} 
		//超声波测距
		if (len==3 && pData[0]==0x03 ) {  
			sprintf(buf, "DEVID:%02X ultrasonic:%u\n", 
				pData[0], pData[1]);
                        sprintf(buf2, "%02X%02d\n", pData[0], pData[1]);

		} 
		//光敏
		if (len==3 && pData[0]==0x04 ) {  
			sprintf(buf, "DEVID:%02X photoresistance:%01d\n", 
				pData[0], pData[1]);
                        sprintf(buf2, "%02X%02d\n", pData[0], pData[1]);

		} 
                //三轴加速传感器
		if (len==5 && pData[0]==0x05 ) {  
			sprintf(buf, "DEVID:%02X x:%01d  y:%01d  z:%01d\n", 
				pData[0], pData[1], pData[2], pData[3]);  
                        sprintf(buf2, "%02X%02d|%02d|%02d\n", pData[0], pData[1], pData[2], pData[3]);

		}
		break;
	}

//    sprintf(buf," flag is:%d \n", 
//				flag);
//    
	for(i=0;i<64;i++){
		if(buf[i]=='\0')break;
		HalUARTWrite (HAL_UART_PORT_0, &buf[i], 1);		//从串口写一个数据          
	}
	for(i=0;i<64;i++){
		if(buf2[i]=='\0')break;
		HalUARTWrite (HAL_UART_PORT_0, &buf2[i], 1);		//从串口写一个数据          
	}

}
///////////////////////////////////////////////////////////////////////////////
/* 节点串口初始化 */
void NodeUartInit(void)
{
	halUARTCfg_t 	uartConfig; 				//halUARTCfg_t类型的结构体变量
	/* 串口配置 */
	uartConfig.configured           = TRUE;
	uartConfig.baudRate             = HAL_UART_BR_38400;		//设置波特率为9600
	//禁止硬件流控，如果你的串口只有 RXD，TXD 和 GND 三条线，必须这么做；
	uartConfig.flowControl          = FALSE;
	uartConfig.rx.maxBufSize        = 128;		//最大接收缓冲区大小
	uartConfig.tx.maxBufSize        = 128; 		//最大发送缓冲区大小
	uartConfig.flowControlThreshold = (128 / 2);
	uartConfig.idleTimeout          = 6;			//空闲超时时间
	uartConfig.intEnable            = TRUE;		//允许中断
	uartConfig.callBackFunc         = NodeUartCallBack;	//设置串口接收回调函数

	/* 打开串口，完成初始化的工作*/
	HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
}


void NodeUartCallBack ( uint8 port, uint8 event )
{
#define RBUFSIZE 128
	(void)event;  // 故意不引用的参数，作保留用
	uint8  ch;
	static uint8 rbuf[RBUFSIZE];
	static uint8  rlen = 0;

	while (Hal_UART_RxBufLen(port))		//计算并返回接收缓冲区的长度
	{	
		HalUARTRead (port, &ch, 1);		        //从串口读一个数据
		HalUARTWrite (port, &ch, 1);		//从串口写一个数据
		flag =  ((int)ch)-48;
		if (rlen >= RBUFSIZE) rlen = 0;		//数据长度超过最大接收缓冲大小，则缓冲区清零
		if (ch == '\r') {				//如果读到回车字符
			HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );	//关闭LED灯
			HalLedSet( HAL_LED_2, HAL_LED_MODE_BLINK );	//使LED灯闪烁
			//      zb_SendDataRequest( 0, ID_CMD_REPORT, rlen, rbuf, 0, AF_ACK_REQUEST, 0 ); 	//发送数据

			rlen = 0;		//缓冲区清零
		}
		else
		{
			if (ch == '\n') {}
			else
				rbuf[rlen++] = ch;
		}  //将数据写到缓冲区
	}
}
