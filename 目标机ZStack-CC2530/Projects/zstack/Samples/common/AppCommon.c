
/**************************************************************************************************
Filename:       AppCommon.c  
Revised:        $Date: 2013-12-02 $
Revision:       $Revision: 1 $

Description:    The public part of user application.


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
#include "hal_adc.h"
#include "SimpleApp.h"
#include "mt.h"
#include "ZDApp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "UserApp.h"
#include "osal_nv.h"
#include "AddrMgr.h"
#include "rtg.h"
#include "nwk_util.h"
#include "AppCommon.h"
/*********************************************************************
* CONSTANTS
*/

// Application States
#define APP_INIT                           0    // Initial state
#define APP_START                          1    // Device has started/joined network

// Application osal event identifiers
#define __START_EVT                0x0010
#define __REPORT_EVT               0x0020

#define REPORT_DELAY                30
/*********************************************************************
* LOCAL VARIABLES
*/
static uint8 myAppState = APP_INIT;
static uint16 myStartRetryDelay = 10;      // seconds

/********************************************************
*
*/
#define K4  P0_1
#define K5  P0_4
void key_init(void);
void key_init(void)
{
  /* P0.4 按键检测*/
  P0SEL &= ~0x12;        //通用IO    
  P0DIR &= ~0x12;        //作输入  
}

/*********************************************************************
* GLOBAL VARIABLES
*/

// Inputs and Outputs for Switch device
#define NUM_IN_CMD_SWITCH                 2
#define NUM_OUT_CMD_SWITCH                2


const cId_t zb_InCmdList[NUM_IN_CMD_SWITCH] =
{
  ID_CMD_READ_REQ,
  ID_CMD_WRITE_REQ,
};
const cId_t zb_OutCmdList[NUM_OUT_CMD_SWITCH] =
{
  ID_CMD_READ_RES,
  ID_CMD_WRITE_RES,
};


// Define SimpleDescriptor for Switch device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_SENSOR,              //  Device ID
  DEVICE_VERSION_SWITCH,      //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_SWITCH,          //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_SWITCH,         //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};


/*********************************************************************/
static char wbuf[128];

static void process_package(char *pkg, int len);
static void my_report_proc(void);
static char* read_al(char *buf, int len);
static char* read_nb(char *buf, int len);
static uint16 _tm_delay, _tm_cnt;

static uint16 panid;
static uint8 logicalType;

static int _process_command_call(char *ptag, char *pval, char *pout);

void zb_HanderMsg(osal_event_hdr_t *msg);
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
    uint8 selType = ZG_DEVICETYPE_ENDDEVICE;
    
    key_init();  
    zb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType );
    if ( logicalType !=ZG_DEVICETYPE_ENDDEVICE && logicalType !=ZG_DEVICETYPE_ROUTER ) {
      selType = ZG_DEVICETYPE_ENDDEVICE;
      zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &selType);
      zb_SystemReset();
    }
    if ( K5 == 0 && logicalType !=ZG_DEVICETYPE_ENDDEVICE ) {
      selType = ZG_DEVICETYPE_ENDDEVICE;
      zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &selType);
      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON);
      while (K5 == 0);
      zb_SystemReset();
    }
    if ( K4 == 0 && logicalType != ZG_DEVICETYPE_ROUTER) { //按下
      selType = ZG_DEVICETYPE_ROUTER;
      zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &selType);
      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON);
      while (K4 == 0);
      zb_SystemReset();
    }
    
    // Do more configuration if necessary and then restart device with auto-start bit set
    // write endpoint to simple desc...dont pass it in start req..then reset
    zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
    if (startOptions != ZCD_STARTOPT_AUTO_START) {
      startOptions = ZCD_STARTOPT_AUTO_START;
      zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
      zb_SystemReset();
    } 
    osal_nv_read( ZCD_NV_PANID, 0, sizeof( panid ), &panid );    
    HalLedSet( HAL_LED_2, HAL_LED_MODE_FLASH );
    
    sensor_init(); 
  }
  
  if ( event & __START_EVT )
  {
    zb_StartRequest();
  }
  if (event & __REPORT_EVT) {
    if (_tm_cnt > 0) {
      my_report_proc();
      osal_start_timerEx( sapi_TaskID, __REPORT_EVT, _tm_delay * 1000);
      _tm_cnt--;
    }
  }
  if (event & 0x000F) {
    MyEventProcess( event );
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
  // If the device sucessfully started, change state to running
  if ( status == ZB_SUCCESS )
  {
    HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
    //osal_start_timerEx( sapi_TaskID, __REPORT_EVT, osal_rand() % REPORT_DELAY * 1000 );
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, __START_EVT, myStartRetryDelay * 1000 );
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
  if ( ( status == ZB_SUCCESS ) && ( myAppState == APP_START ) )
  {
  }
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
  uint16 pAddr = NLME_GetCoordShortAddr();
  
  HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
  HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );  
  if (command == 0) {
    process_package((char*)pData, len);
  }
}



////////////////////////////////////////////////////////////////////////////////
static int _process_command_call(char *ptag, char *pval, char *pout)
{
  int val;
  int ret = 0;
  
  val = atoi(pval);
  if (0 == strcmp("ECHO", ptag)) {
    ret = sprintf(pout, "ECHO=%s",pval);
  } else  
  if (0 == strcmp("PANID", ptag)) { 
    if (0 == strcmp("?", pval)) {
      uint16 tmp16;
      osal_nv_read( ZCD_NV_PANID, 0, sizeof( tmp16 ), &tmp16 );
      ret = sprintf(pout, "PANID=%u", tmp16);
    } else {
      uint8 startOptions = ZCD_STARTOPT_DEFAULT_NETWORK_STATE;
      uint16 tmp16;
      osal_nv_read( ZCD_NV_PANID, 0, sizeof( tmp16 ), &tmp16 );
      if (tmp16 != val) {
        osal_nv_write(ZCD_NV_PANID, 0, osal_nv_item_len( ZCD_NV_PANID ), &val);
        zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions ); //标记网络状态发生改变
      }
    } 
  } else
  if (0 == strcmp("CHANNEL", ptag)) { 
    static uint32 chs[] = {0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
        0x00010000, 0x00020000, 0x00040000,0x00080000,0x00100000,0x00200000,
        0x00400000,0x00800000,0x01000000,0x02000000,0x04000000}; 
    if (0 == strcmp("?", pval)) {
      uint32 tmp32;
      uint8 i;
      osal_nv_read( ZCD_NV_CHANLIST, 0, sizeof( tmp32 ), &tmp32 );
     
      for (i=0; i<16; i++) {
        if (tmp32 == chs[i]) break;
      }
      i += 11;
      ret = sprintf(pout, "CHANNEL=%u", i);
    } else {
      uint32 tmp32, t32;
      uint8 startOptions = ZCD_STARTOPT_DEFAULT_NETWORK_STATE;
      tmp32 = val - 11;
      osal_nv_read( ZCD_NV_CHANLIST, 0, sizeof( tmp32 ), &t32 );
      if (tmp32 < 16) {
        if (t32 != chs[tmp32]) {
          osal_nv_write(ZCD_NV_CHANLIST, 0, osal_nv_item_len( ZCD_NV_CHANLIST ), &chs[tmp32]);
          zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions ); //标记网络状态发生改变
        }
      }
    }
  } else
  /*if (0 == strcmp("RSSI", ptag)) {
    if (0 == strcmp("?", pval)) {
      ret = sprintf(pout, "RSSI=%d", rssi);
    }
  } else*/
  if (0 == strcmp("TYPE", ptag)) {
    if (0 == strcmp("?", pval)) {
      ret = sprintf(pout, "TYPE=%d%d%s", NODE_CATEGORY, logicalType, NODE_NAME);
    }
  } else
  if (0 == strcmp("NODE_CATEGORY", ptag)) {
    if (0 == strcmp("?", pval)) {
      ret = sprintf(pout, "NODE_CATEGORY=%d", NODE_CATEGORY);
    }
  } else
  if (0 == strcmp("NODE_TYPE", ptag)) {
    if (0 == strcmp("?", pval)) {
      ret = sprintf(pout, "NODE_TYPE=%d", logicalType);
    }else{
      logicalType = val;
      zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
    }
  } else
  if (0 == strcmp("NODE_NAME", ptag)) {
    if (0 == strcmp("?", pval)) {
      ret = sprintf(pout, "NODE_NAME=%s", NODE_NAME);
    }
  } else
  if (0 == strcmp("RTG", ptag)) {
    if (0 == strcmp("?", pval)) {
      int i;
      char *p;
      sprintf(pout, "RTG=");
      p = &pout[4];
      for (i=0; i<MAX_RTG_ENTRIES; i++) {
        rtgEntry_t *prtg;
        prtg = &rtgTable[i];
        if (prtg->status == RT_ACTIVE) {
          unsigned char mac[Z_EXTADDR_LEN];
          if (AddrMgrExtAddrLookup(prtg->nextHopAddress, mac) == TRUE) {
            MT_ReverseBytes( mac, Z_EXTADDR_LEN );
            sprintf(p, "%02X%02X", mac[6], mac[7]);
            p = p + strlen(p);              
          }
        }
      }
      if (strlen(pout) == 4) {
        sprintf(p, "NULL");
      }
      ret = strlen(pout);
    }
  } else//RTG
  if (0 == strcmp("AL", ptag)) { 
    if (0 == strcmp("?", pval)) {
      char *p;
      sprintf(pout, "AL=");
      p = read_al(pout+3, -1);
      if (strlen(p) == 0) {
        sprintf(pout+3, "NULL");
      }
      ret = strlen(pout);
    }
  } else//AL
  if (0 == strcmp("NB", ptag)) {
    if (0 == strcmp("?", pval)) {
      sprintf(pout, "NB=");
      read_nb(pout+strlen(pout), -1);
      if (strlen(pout) == 3) {
        sprintf(pout+3, "NULL");
      }
      ret = strlen(pout);
    }
  } else//NB
  if (0 == strcmp("AN", ptag)) { 
    if (0 == strcmp("?", pval)) {
      sprintf(pout, "AN=");
      read_al(pout+strlen(pout), -1);
      read_nb(pout+strlen(pout), -1);
      if (strlen(pout) == 3) {
        sprintf(pout+3, "NULL");
      }
      ret = strlen(pout);
    }
  } else //AN
  if (0 == strcmp("TPN", ptag)) { 
    /*  参数格式 x/y  表示在y分钟内上报x次数据 
     *  x = 0 停止上报,
     *  限制每分钟最大上报6次,最少上报1次
    */
    char *s = strchr(pval, '/');
    if (s != NULL) {
      int v1, v2;
      
      *s = 0;
      v1 = atoi(pval);
      v2 = atoi(s+1);
      
      if (v1 > 0 && v2 > 0) {
        _tm_delay = v2*60/v1;
        if (_tm_delay >= 10 && _tm_delay <= 65) {
          if (_tm_cnt == 0) {
            // start timer
            osal_start_timerEx( sapi_TaskID, __REPORT_EVT, (osal_rand()%_tm_delay) * (osal_rand()%1000));
          } 
          _tm_cnt = v1;
        }
      }
    }
  } //TPN  
  else {
    ret = usr_process_command_call(ptag, pval, pout);
  }
  return ret;
}

static void process_package(char *pkg, int len)
{  
  char *p;
  char *ptag = NULL;
  char *pval = NULL;

  char *pwbuf = wbuf+1;
  
  
  if (pkg[0] != '{' || pkg[len-1] != '}') return;
  pkg[len-1] = 0;
  p = pkg+1; 
  do {
    ptag = p;
    p = strchr(p, '=');
    if (p != NULL) {
      *p++ = 0;
      pval = p;
      p = strchr(p, ',');
      if (p != NULL) *p++ = 0;
      /*if (process_command_call != NULL) */{
        int ret;
        ret = _process_command_call(ptag, pval, pwbuf);
        if (ret > 0) {
          pwbuf += ret;
          *pwbuf++ = ',';
        }
      }
    }
  } while (p != NULL);
  if (pwbuf - wbuf > 1) {
    wbuf[0] = '{';
    pwbuf[0] = 0;
    pwbuf[-1] = '}';
    uint16 cmd = 0;
    
    HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );
    zb_SendDataRequest( 0, cmd, pwbuf-wbuf, (uint8*)wbuf, 0, AF_ACK_REQUEST, AF_DEFAULT_RADIUS );
  }
}

static void my_report_proc(void)
{
  unsigned char mac[Z_EXTADDR_LEN];
  sprintf(wbuf, "{PN=");
#if 0
  read_al(wbuf+strlen(wbuf), -1);
#else
  NLME_GetCoordExtAddr(mac);
  sprintf(wbuf+strlen(wbuf), "%02X%02X", mac[1],mac[0]);
#endif  
  read_nb(wbuf+strlen(wbuf), -1);
  if (strlen(wbuf) == 4) {
    sprintf(wbuf+4, "NULL");
  } 
  sprintf(wbuf+strlen(wbuf), ",TYPE=%d%d%s}", NODE_CATEGORY, logicalType, NODE_NAME);
  zb_SendDataRequest(0/*source*/, 0/*cmd*/, strlen(wbuf), (uint8*)wbuf, 0, AF_ACK_REQUEST, AF_DEFAULT_RADIUS );
}
static char* read_nb(char *buf, int len)
{
  int i;
  char *p;
  
  buf[0] = 0;
  p = buf;     
  for (i=0; i<MAX_NEIGHBOR_ENTRIES; i++) {
    neighborEntry_t *pnb = &neighborTable[i];
    if (pnb->panId == panid 
        && memcmp(pnb->neighborExtAddr,"\x00\x00\x00\x00\x00\x00\x00\x00", 8)!=0 
        && pnb->age <= NWK_ROUTE_AGE_LIMIT) {
          sprintf(p, "%02X%02X", pnb->neighborExtAddr[1], pnb->neighborExtAddr[0]);
          p = p + strlen(p);         
    }
  }
  return buf;
}
static char* read_al(char *buf, int len)
{
  uint8 assocCnt = 0, i;
  uint16 *assocList;
  unsigned char mac[Z_EXTADDR_LEN];
  char *p = buf;
  p[0] = 0;
 
  for (i=0; i<NWK_MAX_DEVICES; i++) {
    associated_devices_t *pa = &AssociatedDevList[i];
    if (pa->nodeRelation == CHILD_FFD_RX_IDLE || pa->nodeRelation == CHILD_FFD) {
      if (pa->age > NWK_ROUTE_AGE_LIMIT) {
        if (TRUE == AddrMgrExtAddrLookup(pa->shortAddr,  mac)) {
          AssocRemove(mac);
        }
      }
    }
  }
  
  assocList = AssocMakeList( &assocCnt );
  for (i=0; i<assocCnt; i++) { 
    if (TRUE == AddrMgrExtAddrLookup(assocList[i],  mac)) {
      MT_ReverseBytes( mac, Z_EXTADDR_LEN );
      sprintf(p, "%02X%02X", mac[6], mac[7]);
      p = p + strlen(p);    
    }
  }
  return buf;
}

int8 ZXBeeBegin(void)
{
  sprintf(wbuf, "{}");
  return strlen(wbuf);
}
int8 ZXBeeAdd(char* tag, char* val)
{
  sprintf(&wbuf[strlen(wbuf)-1], "%s=%s,", tag, val);
  return strlen(wbuf);
}
char* ZXBeeEnd(void)
{
  sprintf(&wbuf[strlen(wbuf)-1], "}");
  return (char*)wbuf; 
}
////////////////////////////////////////////////////////////////////////////////

