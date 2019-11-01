/**************************************************************************************************
  Filename:       zcl_se.c
  Revised:        $Date: 2010-10-28 15:07:54 -0700 (Thu, 28 Oct 2010) $
  Revision:       $Revision: 24272 $

  Description:    Zigbee Cluster Library - SE (Smart Energy) Profile.


  Copyright 2007-2010 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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


/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_se.h"
#include "DebugTrace.h"

#if defined ( INTER_PAN )
  #include "stub_aps.h"
#endif


#include "zcl_key_establish.h"


/*********************************************************************
 * MACROS
 */
// Clusters that are supported thru Inter-PAN communication
#define INTER_PAN_CLUSTER( id )  ( (id) == ZCL_CLUSTER_ID_SE_PRICING || \
                                   (id) == ZCL_CLUSTER_ID_SE_MESSAGE )

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef struct zclSECBRec
{
  struct zclSECBRec          *next;
  uint8                       endpoint; // Used to link it into the endpoint descriptor
  zclSE_AppCallbacks_t       *CBs;     // Pointer to Callback function
} zclSECBRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static zclSECBRec_t *zclSECBs = (zclSECBRec_t *)NULL;
static uint8 zclSEPluginRegisted = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static ZStatus_t zclSE_HdlIncoming(  zclIncoming_t *pInMsg );
static ZStatus_t zclSE_HdlInSpecificCommands( zclIncoming_t *pInMsg );

#ifdef ZCL_SIMPLE_METERING
static ZStatus_t zclSE_ProcessInSimpleMeteringCmds( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_GetProfileCmd( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_GetProfileRsp( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_ReqMirrorCmd( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_ReqMirrorRsp( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_MirrorRemCmd( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_MirrorRemRsp( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
#endif

#ifdef ZCL_PRICING
static ZStatus_t zclSE_ProcessInPricingCmds( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_Pricing_GetCurrentPrice( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_Pricing_GetScheduledPrice( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_Pricing_PublishPrice( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
#endif

#ifdef ZCL_MESSAGE
static ZStatus_t zclSE_ProcessInMessageCmds( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_Message_DisplayMessage( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_Message_CancelMessage( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_Message_GetLastMessage( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_Message_MessageConfirmation( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
#endif

#ifdef ZCL_LOAD_CONTROL
static ZStatus_t zclSE_ProcessInLoadControlCmds( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_LoadControl_LoadControlEvent( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_LoadControl_CancelLoadControlEvent( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_LoadControl_CancelAllLoadControlEvents( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_LoadControl_ReportEventStatus( zclIncoming_t *pInMsg, zclSE_AppCallbacks_t *pCBs );
static ZStatus_t zclSE_ProcessInCmd_LoadControl_GetScheduledEvents( zclIncoming_t *pInMsg,
                                                                    zclSE_AppCallbacks_t *pCBs );
#endif

#ifdef ZCL_SIMPLE_METERING
/*********************************************************************
 * @fn      zclSE_SimpleMetering_Send_GetProfileCmd
 *
 * @brief   Call to send out a Get Profile Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   channel - returned inteval (delivered @ 0; received @ 1)
 * @param   endTime - UTC time for the starting time of requested interval
 * @param   numOfPeriods - number of periods requested
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_SimpleMetering_Send_GetProfileCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                      uint8 channel, uint32 endTime, uint8 numOfPeriods,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[6];

  buf[0] = channel;
  osal_buffer_uint32( &buf[1], endTime );
  buf[5] = numOfPeriods;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_SIMPLE_METERING,
                          COMMAND_SE_GET_PROFILE_CMD, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, 6, buf );
}

/*********************************************************************
 * @fn      zclSE_SimpleMetering_Send_GetProfileRsp
 *
 * @brief   Call to send out a Get Profile Response
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   endTime - UTC time for the starting time of requested interval
 * @param   intervals - data buffer holding an array of interval data captured
 *          using the period
 *          specified by the ProfileIntervalPeriod attribute. Data is organized
 *          in a reverse chronological order, the most recent interval is
 *          transmitted first and the oldest interval is transmitted last.
 *          Invalid intervals intervals should be marked as 0xFFFFFF
 * @param   len - length of the intervals buffer
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_SimpleMetering_Send_GetProfileRsp( uint8 srcEP, afAddrType_t *dstAddr,
                                                   uint32 endTime, uint8 rspStatus, uint8 profileIntervalPeriod,
                                                   uint8 numOfPeriodDelivered, uint24 *intervals,
                                                   uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 *pBuf;
  uint8 i;
  uint8 len;
  ZStatus_t status;

  // endTime + status + profileIntervalPeriod + numOfEntry + array
  len = 4 + 1 + 1 + 1 + (3 * numOfPeriodDelivered);
  buf = osal_mem_alloc( len );
  if ( buf == NULL )
    return (ZMemError);

  osal_buffer_uint32( buf, endTime );
  buf[4] = rspStatus;
  buf[5] = profileIntervalPeriod;

  // Starting of the array of uint24
  buf[6] = numOfPeriodDelivered;   // Number of entries in the array
  pBuf = &buf[7];
  for( i = 0; i< numOfPeriodDelivered; i++ )
  {
    pBuf = osal_buffer_uint24( pBuf, *intervals++ );
  }

  status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_SIMPLE_METERING,
                          COMMAND_SE_GET_PROFILE_RSP, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, len, buf );
  osal_mem_free( buf );
  return status;
}

/*********************************************************************
 * @fn      zclSE_SimpleMetering_Send_ReqMirrorRsp
 *
 * @brief   Call to send out a Request Mirror Response
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_SimpleMetering_Send_ReqMirrorRsp( uint8 srcEP, afAddrType_t *dstAddr,
                                                      uint16 endpointId,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  ZStatus_t status;
  uint8 buf[2];

  buf[0] = (uint8) endpointId ;
  buf[1] = (uint8)( endpointId >> 8 );

  status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_SIMPLE_METERING,
                          COMMAND_SE_REQ_MIRROR_RSP, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, 2, buf );

  return status ;
}

/*********************************************************************
 * @fn      zclSE_SimpleMetering_Send_RemMirrorRsp
 *
 * @brief   Call to send out a Remove Mirror Response
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_SimpleMetering_Send_RemMirrorRsp( uint8 srcEP, afAddrType_t *dstAddr,
                                                      uint16 endpointId,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  ZStatus_t status;
  uint8 buf[2];

  buf[0] = (uint8) endpointId ;
  buf[1] = (uint8)( endpointId >> 8 );

  status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_SIMPLE_METERING,
                          COMMAND_SE_MIRROR_REM_RSP, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, 2, buf );

  return status ;
}
#endif


#ifdef ZCL_PRICING

/*********************************************************************
 * Call to send out a Get Scheduled Price Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   cmd - command payload
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_Pricing_Send_GetScheduledPrice( uint8 srcEP, afAddrType_t *dstAddr,
                                            zclCCGetScheduledPrice_t *cmd,
                                            uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[5];
  ZStatus_t status;

  osal_buffer_uint32( buf, cmd->startTime );
  buf[4] = cmd->numEvents;

  status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_PRICING,
                          COMMAND_SE_GET_SCHEDULED_PRICE, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0,
                          seqNum, 5, buf );

  return status;
}

/*********************************************************************
 * @fn      zclSE_Pricing_Send_PublishPrice
 *
 * @brief   Call to send out a Publish Price Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_Pricing_Send_PublishPrice( uint8 srcEP, afAddrType_t *dstAddr,
                                            zclCCPublishPrice_t *cmd,
                                            uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 *pBuf;
  uint16 bufLen;
  ZStatus_t status;

  bufLen = PACKET_LEN_SE_PUBLISH_PRICE + cmd->rateLabel.strLen;
  buf = osal_mem_alloc( bufLen );
  if ( buf == NULL )
    return (ZMemError);

  pBuf = buf;

  pBuf = osal_buffer_uint32( pBuf, cmd->providerId );
  *pBuf++ = cmd->rateLabel.strLen;
  pBuf = osal_memcpy( pBuf, cmd->rateLabel.pStr, cmd->rateLabel.strLen );
  pBuf = osal_buffer_uint32( pBuf, cmd->issuerEventId );
  pBuf = osal_buffer_uint32( pBuf, cmd->currentTime );
  *pBuf++ = cmd->unitOfMeasure;
  *pBuf++ = LO_UINT16( cmd->currency );
  *pBuf++ = HI_UINT16( cmd->currency );
  *pBuf++ = cmd->priceTrailingDigit;
  *pBuf++ = cmd->priceTier;
  pBuf = osal_buffer_uint32( pBuf, cmd->startTime );
  *pBuf++ = LO_UINT16( cmd->durationInMinutes );
  *pBuf++ = HI_UINT16( cmd->durationInMinutes );
  pBuf = osal_buffer_uint32( pBuf, cmd->price );
  *pBuf++ = cmd->priceRatio;
  pBuf = osal_buffer_uint32( pBuf, cmd->generationPrice );
  *pBuf = cmd->generationPriceRatio;

  status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_PRICING,
                          COMMAND_SE_PUBLISH_PRICE, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0,
                          seqNum, bufLen, buf );
  osal_mem_free(buf);

  return status;
}
#endif


#ifdef ZCL_MESSAGE
/*********************************************************************
 * @fn      zclSE_Message_Send_DisplayMessage
 *
 * @brief   Call to send out a Display Message Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   cmd - command
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_Message_Send_DisplayMessage( uint8 srcEP, afAddrType_t *dstAddr,
                                             zclCCDisplayMessage_t *cmd,
                                             uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 *pBuf;
  uint16 bufLen;
  ZStatus_t status;

  // msgId + msgCtrl + start time + duration + msgLen + msg
  bufLen = 4 + 1 + 4 + 2 + 1 + cmd->msgString.strLen;

  buf = osal_mem_alloc( bufLen );

  if ( buf == NULL )
    return (ZMemError);

  pBuf = buf;
  pBuf = osal_buffer_uint32( buf, cmd->messageId );  // Streamline the uint32 data
  *pBuf++ = cmd->messageCtrl.transmissionMode |
           (cmd->messageCtrl.importance << SE_PROFILE_MSGCTRL_IMPORTANCE)|
           (cmd->messageCtrl.confirmationRequired << SE_PROFILE_MSGCTRL_CONFREQUIRED);
  pBuf = osal_buffer_uint32( pBuf, cmd->startTime );
  *pBuf++ = LO_UINT16( cmd->durationInMinutes );
  *pBuf++ = HI_UINT16( cmd->durationInMinutes );
  *pBuf++ = cmd->msgString.strLen;

  osal_memcpy( pBuf, cmd->msgString.pStr, cmd->msgString.strLen );

  status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_MESSAGE,
                          COMMAND_SE_DISPLAY_MESSAGE, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, bufLen, buf );
  osal_mem_free(buf);
  return status;
}

/*********************************************************************
 * @fn      zclSE_Message_Send_CancelMessage
 *
 * @brief   Call to send out a Cancel Message Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   msgId - Message ID
 * @param   msgCtrl - Message Control
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_Message_Send_CancelMessage( uint8 srcEP, afAddrType_t *dstAddr,
                                                      uint32 msgId, uint8 msgCtrl,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[5];

  osal_buffer_uint32( buf, msgId );  // Streamline the uint32 data
  buf[4] = msgCtrl;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_MESSAGE,
                          COMMAND_SE_CANCEL_MESSAGE, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, 5, buf );
}

/*********************************************************************
 * @fn      zclSE_Message_Send_MessageConfirmation
 *
 * @brief   Call to send out a Message Confirmation
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   msgId - Message ID
 * @param   confirmTime - Confirmation Time
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_Message_Send_MessageConfirmation( uint8 srcEP, afAddrType_t *dstAddr,
                                                      uint32 msgId, uint32 confirmTime,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[8];

  osal_buffer_uint32( buf, msgId );  // Streamline the uint32 data
  osal_buffer_uint32( &buf[4], confirmTime );  // Streamline the uint32 data

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_MESSAGE,
                          COMMAND_SE_MESSAGE_CONFIRMATION, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, 8, buf );
}
#endif

#ifdef ZCL_LOAD_CONTROL
/*********************************************************************
 * @fn      zclSE_LoadControl_Send_LoadControlEvent
 *
 * @brief   Call to send out a Load Control Event
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   cmd - zclCCLoadControlEvent_t
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_LoadControl_Send_LoadControlEvent( uint8 srcEP, afAddrType_t *dstAddr,
                                                      zclCCLoadControlEvent_t* cmd,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 *pBuf;
  ZStatus_t status;

  buf = osal_mem_alloc( PACKET_LEN_SE_LOAD_CONTROL_EVENT );

  if ( buf == NULL )
    return (ZMemError);

  pBuf = buf;
  pBuf = osal_buffer_uint32( pBuf, cmd->issuerEvent );
  pBuf = osal_buffer_uint24( pBuf, cmd->deviceGroupClass );
  pBuf = osal_buffer_uint32( pBuf, cmd->startTime );
  *pBuf++ = LO_UINT16( cmd->durationInMinutes );
  *pBuf++ = HI_UINT16( cmd->durationInMinutes );
  *pBuf++ = cmd->criticalityLevel;
  *pBuf++ = cmd->coolingTemperatureOffset;
  *pBuf++ = cmd->heatingTemperatureOffset;
  *pBuf++ = LO_UINT16( cmd->coolingTemperatureSetPoint );
  *pBuf++ = HI_UINT16( cmd->coolingTemperatureSetPoint );
  *pBuf++ = LO_UINT16( cmd->heatingTemperatureSetPoint );
  *pBuf++ = HI_UINT16( cmd->heatingTemperatureSetPoint );
  *pBuf++ = cmd->averageLoadAdjustmentPercentage;
  *pBuf++ = cmd->dutyCycle;
  *pBuf = cmd->eventControl;

  status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_LOAD_CONTROL,
                          COMMAND_SE_LOAD_CONTROL_EVENT, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum,
                          PACKET_LEN_SE_LOAD_CONTROL_EVENT, buf );
  osal_mem_free(buf);

  return status;
}

/*********************************************************************
 * @fn      zclSE_LoadControl_Send_CancelLoadControlEvent
 *
 * @brief   Call to send out a Cancel Load Control Event
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_LoadControl_Send_CancelLoadControlEvent( uint8 srcEP, afAddrType_t *dstAddr,
                                                      zclCCCancelLoadControlEvent_t *cmd,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[PACKET_LEN_SE_CANCEL_LOAD_CONTROL_EVENT];
  uint8 *pBuf;

  pBuf = buf;

  pBuf = osal_buffer_uint32( pBuf, cmd->issuerEventID );
  pBuf = osal_buffer_uint24( pBuf, cmd->deviceGroupClass );
  *pBuf++ = cmd->cancelControl;
  pBuf = osal_buffer_uint32( pBuf, cmd->effectiveTime );

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_LOAD_CONTROL,
                          COMMAND_SE_CANCEL_LOAD_CONTROL_EVENT, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum,
                          PACKET_LEN_SE_CANCEL_LOAD_CONTROL_EVENT, buf );
}

/*********************************************************************
 * @fn      zclSE_LoadControl_Send_ReportEventStatus
 *
 * @brief   Call to send out a Report Event Status
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   cmd - zclCCReportEventStatus_t
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_LoadControl_Send_ReportEventStatus( uint8 srcEP, afAddrType_t *dstAddr,
                                                      zclCCReportEventStatus_t* cmd,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 *pBuf;
  ZStatus_t status;

  buf = osal_mem_alloc( PACKET_LEN_SE_REPORT_EVENT_STATUS );

  if ( buf == NULL )
    return (ZMemError);

  pBuf = buf;
  pBuf = osal_buffer_uint32( pBuf, cmd->issuerEventID );
  *pBuf++ = cmd->eventStatus;
  pBuf = osal_buffer_uint32( pBuf, cmd->eventStartTime );
  *pBuf++ = cmd->criticalityLevelApplied;
  *pBuf++ = LO_UINT16( cmd->coolingTemperatureSetPointApplied );
  *pBuf++ = HI_UINT16( cmd->coolingTemperatureSetPointApplied );
  *pBuf++ = LO_UINT16( cmd->heatingTemperatureSetPointApplied );
  *pBuf++ = HI_UINT16( cmd->heatingTemperatureSetPointApplied );
  *pBuf++ = cmd->averageLoadAdjustment;
  *pBuf++ = cmd->dutyCycleApplied;
  *pBuf++ = cmd->eventControl;
  *pBuf++ = cmd->signatureType;

  zclGeneral_KeyEstablishment_ECDSASign( buf, PACKET_LEN_SE_REPORT_EVENT_STATUS_ONLY, pBuf);

  status = zcl_SendCommand(srcEP, dstAddr, ZCL_CLUSTER_ID_SE_LOAD_CONTROL,
                           COMMAND_SE_REPORT_EVENT_STATUS, TRUE,
                           ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum,
                           PACKET_LEN_SE_REPORT_EVENT_STATUS, buf );
  osal_mem_free(buf);

  return status;
}

/*********************************************************************
 * @fn      zclSE_LoadControl_Send_GetScheduledEvent
 *
 * @brief   Call to send out a Get Scheduled Event
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   cmd - command format
 * @param   disableDefaultRsp - disable default response
 * @param   seqNum - ZCL sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSE_LoadControl_Send_GetScheduledEvent( uint8 srcEP, afAddrType_t *dstAddr,
                                                      zclCCGetScheduledEvent_t *cmd,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[PACKET_LEN_SE_GET_SCHEDULED_EVENT];

  osal_buffer_uint32( buf, cmd->startTime );
  buf[4] = cmd->numEvents;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SE_LOAD_CONTROL,
                          COMMAND_SE_GET_SCHEDULED_EVENT, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum,
                          PACKET_LEN_SE_GET_SCHEDULED_EVENT, buf );
}

#endif

/*********************************************************************
 * @fn      zclSE_RegisterCmdCallbacks
 *
 * @brief   Register an applications command callbacks
 *
 * @param   endpoint - application's endpoint
 * @param   callbacks - pointer to the callback record.
 *
 * @return  ZMemError if not able to allocate
 */
ZStatus_t zclSE_RegisterCmdCallbacks( uint8 endpoint, zclSE_AppCallbacks_t *callbacks )
{
  zclSECBRec_t *pNewItem;
  zclSECBRec_t *pLoop;

  // Register as a ZCL Plugin
  if ( !zclSEPluginRegisted )
  {
    zcl_registerPlugin( ZCL_CLUSTER_ID_SE_PRICING,
                        ZCL_CLUSTER_ID_SE_PRE_PAYMENT,
                        zclSE_HdlIncoming );
    zclSEPluginRegisted = TRUE;
  }

  // Fill in the new profile list
  pNewItem = osal_mem_alloc( sizeof( zclSECBRec_t ) );
  if ( pNewItem == NULL )
    return (ZMemError);

  pNewItem->next = (zclSECBRec_t *)NULL;
  pNewItem->endpoint = endpoint;
  pNewItem->CBs = callbacks;

  // Find spot in list
  if ( zclSECBs == NULL )
  {
    zclSECBs = pNewItem;
  }
  else
  {
    // Look for end of list
    pLoop = zclSECBs;
    while ( pLoop->next != NULL )
      pLoop = pLoop->next;

    // Put new item at end of list
    pLoop->next = pNewItem;
  }
  return ( ZSuccess );
}

#if defined( ZCL_LOAD_CONTROL ) || defined( ZCL_SIMPLE_METERING ) || \
    defined( ZCL_PRICING ) || defined( ZCL_MESSAGE )
/*********************************************************************
 * @fn      zclSE_FindCallbacks
 *
 * @brief   Find the callbacks for an endpoint
 *
 * @param   endpoint
 *
 * @return  pointer to the callbacks
 */
static zclSE_AppCallbacks_t *zclSE_FindCallbacks( uint8 endpoint )
{
  zclSECBRec_t *pCBs;

  pCBs = zclSECBs;
  while ( pCBs )
  {
    if ( pCBs->endpoint == endpoint )
      return ( pCBs->CBs );
    pCBs = pCBs->next;
  }
  return ( (zclSE_AppCallbacks_t *)NULL );
}

#endif
/*********************************************************************
 * @fn      zclSE_HdlIncoming
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library or Profile commands for attributes
 *          that aren't in the attribute list
 *
 * @param   pInHdlrMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSE_HdlIncoming(  zclIncoming_t *pInMsg )
{
  ZStatus_t stat = ZSuccess;

#if defined ( INTER_PAN )
  if ( StubAPS_InterPan( pInMsg->msg->srcAddr.panId, pInMsg->msg->srcAddr.endPoint ) &&
       !INTER_PAN_CLUSTER(pInMsg->msg->clusterId) )
  {
    return ( stat ); // Cluster not supported thru Inter-PAN
  }
#endif
  if ( zcl_ClusterCmd( pInMsg->hdr.fc.type ) )
  {
    // Is this a manufacturer specific command?
    if ( pInMsg->hdr.fc.manuSpecific == 0 )
    {
      stat = zclSE_HdlInSpecificCommands( pInMsg );
    }
    else
    {
      // We don't support any manufacturer specific command.
      stat = ZFailure;
    }
  }
  else
  {
    // Handle all the normal (Read, Write...) commands -- should never get here
    stat = ZFailure;
  }
  return ( stat );
}

/*********************************************************************
 * @fn      zclSE_HdlInSpecificCommands
 *
 * @brief   Function to process incoming Commands specific
 *          to this cluster library

 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSE_HdlInSpecificCommands( zclIncoming_t *pInMsg )
{
  ZStatus_t stat;
  zclSE_AppCallbacks_t *pCBs;

  // make sure endpoint exists

#if defined( ZCL_LOAD_CONTROL ) || defined( ZCL_SIMPLE_METERING ) || \
    defined( ZCL_PRICING ) || defined( ZCL_MESSAGE )

  pCBs = zclSE_FindCallbacks( pInMsg->msg->endPoint );
  if ( pCBs == NULL )
    return ( ZFailure );

#endif
  switch ( pInMsg->msg->clusterId )			
  {
#ifdef ZCL_SIMPLE_METERING
    case ZCL_CLUSTER_ID_SE_SIMPLE_METERING:
      stat = zclSE_ProcessInSimpleMeteringCmds( pInMsg, pCBs );
      break;
#endif

#ifdef ZCL_PRICING
    case ZCL_CLUSTER_ID_SE_PRICING:
      stat = zclSE_ProcessInPricingCmds( pInMsg, pCBs );
      break;
#endif

#ifdef ZCL_MESSAGE
    case ZCL_CLUSTER_ID_SE_MESSAGE:
      stat = zclSE_ProcessInMessageCmds( pInMsg, pCBs );
      break;
#endif

#ifdef ZCL_LOAD_CONTROL
    case ZCL_CLUSTER_ID_SE_LOAD_CONTROL:
      stat = zclSE_ProcessInLoadControlCmds( pInMsg, pCBs );
      break;
#endif
    default:
      stat = ZFailure;
      break;
  }

  return ( stat );
}

#ifdef ZCL_SIMPLE_METERING
/*********************************************************************
 * @fn      zclSE_ProcessInSimpleMeteringCmds
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis

 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSE_ProcessInSimpleMeteringCmds( zclIncoming_t *pInMsg,
                                                     zclSE_AppCallbacks_t *pCBs )
{
  ZStatus_t stat = ZSuccess;

  if ( zcl_ServerCmd( pInMsg->hdr.fc.direction ) )
  {
    // Process Client commands, received by server
    if( pInMsg->hdr.commandID == COMMAND_SE_GET_PROFILE_CMD )
    {
      stat = zclSE_ProcessInCmd_SimpleMeter_GetProfileCmd( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_REQ_MIRROR_RSP )
    {
      stat = zclSE_ProcessInCmd_SimpleMeter_ReqMirrorRsp( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_MIRROR_REM_RSP )
    {
      stat = zclSE_ProcessInCmd_SimpleMeter_MirrorRemRsp( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }
  else
  {
    // Process Server commands, received by client
    if ( pInMsg->hdr.commandID == COMMAND_SE_GET_PROFILE_RSP )
    {
      stat = zclSE_ProcessInCmd_SimpleMeter_GetProfileRsp( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_REQ_MIRROR_CMD )
    {
      stat = zclSE_ProcessInCmd_SimpleMeter_ReqMirrorCmd( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_MIRROR_REM_CMD )
    {
      stat = zclSE_ProcessInCmd_SimpleMeter_MirrorRemCmd( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_SimpleMeter_GetProfileCmd
 *
 * @brief   Process in the received Get Profile Command.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZCL_STATUS_CMD_HAS_RSP @ Supported and do
 *                                           not need default rsp
 *                      ZCL_STATUS_INVALID_FIELD @ Range checking
 *                                           failure
 *
 */
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_GetProfileCmd( zclIncoming_t *pInMsg,
                                                                zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnSimpleMeter_GetProfileCmd )
  {
    zclCCGetProfileCmd_t cmd;

    cmd.channel = pInMsg->pData[0];
    cmd.endTime = osal_build_uint32( &pInMsg->pData[1], 4 );
    cmd.numOfPeriods = pInMsg->pData[5];

    // Range checking
    if ( cmd.channel > MAX_INTERVAL_CHANNEL_SE_SIMPLE_METERING )
    {
      return ZCL_STATUS_INVALID_FIELD;
    }
    pCBs->pfnSimpleMeter_GetProfileCmd( &cmd, &(pInMsg->msg->srcAddr),
                                       pInMsg->hdr.transSeqNum  );

    return ZCL_STATUS_CMD_HAS_RSP;
  }
  else
  {
    return ZFailure;
  }
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_SimpleMeter_GetProfileRsp
 *
 * @brief   Process in the received Get Profile Response.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 *                      ZCL_STATUS_INVALID_FIELD @ Range checking
 *                                           failure
 *                      ZCL_STATUS_SOFTWARE_FAILURE @ ZStack memory allocation failure
 */
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_GetProfileRsp( zclIncoming_t *pInMsg,
                                                                zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnSimpleMeter_GetProfileRsp )
  {
    uint24 *pBuf24;
    uint8  *pBuf8;
    uint8  i;
    zclCCGetProfileRsp_t cmd;

    cmd.endTime = osal_build_uint32( &pInMsg->pData[0], 4 );
    cmd.status = pInMsg->pData[4];
    cmd.profileIntervalPeriod = pInMsg->pData[5];
    cmd.numOfPeriodDelivered = pInMsg->pData[6];

    // Range Checking
    if ( cmd.profileIntervalPeriod > MAX_PROFILE_INTERVAL_PERIOD_SE_SIMPLE_METERING )
    {
       return ZCL_STATUS_INVALID_FIELD;
    }

    // Convert the byte stream to array of uint24
    pBuf8 = &pInMsg->pData[7];  // Pointer to the start of the array of bytes

    // Pointer to the start of the array of uint24
    pBuf24 = (uint24*)osal_mem_alloc( cmd.numOfPeriodDelivered *
                                     sizeof(uint24) );
    if ( pBuf24 == NULL )
      return ZCL_STATUS_SOFTWARE_FAILURE;  // Memory allocation error

    cmd.intervals = pBuf24;
    for( i = 0; i < cmd.numOfPeriodDelivered; i++ )
    {
      *(pBuf24++) = osal_build_uint32( pBuf8, 3 );
      pBuf8 += 3;
    }

    pCBs->pfnSimpleMeter_GetProfileRsp( &cmd, &(pInMsg->msg->srcAddr),
                                       pInMsg->hdr.transSeqNum );

    osal_mem_free( cmd.intervals );

    return ZSuccess;
  }
  else
  {
    return ZFailure;
  }
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_SimpleMeter_ReqMirrorCmd
 *
 * @brief   Process in the received Request Mirror Command.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_ReqMirrorCmd( zclIncoming_t *pInMsg,
                                                                  zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnSimpleMeter_ReqMirrorCmd )
  {
    pCBs->pfnSimpleMeter_ReqMirrorCmd( &(pInMsg->msg->srcAddr),
                                       pInMsg->hdr.transSeqNum );
    return ZSuccess;
  }
  else
    return ZFailure;
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_SimpleMeter_ReqMirrorRsp
 *
 * @brief   Process in the received Request Mirror Response.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_ReqMirrorRsp( zclIncoming_t *pInMsg,
                                                                  zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnSimpleMeter_ReqMirrorRsp )
  {
    zclCCReqMirrorRsp_t cmd;

    cmd.endpointId = BUILD_UINT16( pInMsg->pData[0], pInMsg->pData[1] );

    pCBs->pfnSimpleMeter_ReqMirrorRsp( &cmd, &(pInMsg->msg->srcAddr),
                                       pInMsg->hdr.transSeqNum );
    return ZSuccess ;
  }
  else
    return ZFailure;
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_SimpleMeter_MirrorRemCmd
 *
 * @brief   Process in the received Mirror Removed Command.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_MirrorRemCmd( zclIncoming_t *pInMsg,
                                                                  zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnSimpleMeter_MirrorRemCmd )
  {
    pCBs->pfnSimpleMeter_MirrorRemCmd( &(pInMsg->msg->srcAddr),
                                       pInMsg->hdr.transSeqNum );
    return ZSuccess;
  }
  else
    return ZFailure;
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_SimpleMeter_MirrorRemRsp
 *
 * @brief   Process in the received Mirror Removed Response.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_SimpleMeter_MirrorRemRsp( zclIncoming_t *pInMsg,
                                                                  zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnSimpleMeter_MirrorRemRsp )
  {
    zclCCMirrorRemRsp_t cmd;

    cmd.endpointId = pInMsg->pData[0] | ( (uint16)pInMsg->pData[1] << 8 );

    pCBs->pfnSimpleMeter_MirrorRemRsp( &cmd, &(pInMsg->msg->srcAddr),
                                       pInMsg->hdr.transSeqNum );
    return ZSuccess;
  }
  else
    return ZFailure;
}

#endif


#ifdef ZCL_PRICING
/*********************************************************************
 * @fn      zclSE_ProcessInPricingCmds
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis

 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSE_ProcessInPricingCmds( zclIncoming_t *pInMsg,
                                              zclSE_AppCallbacks_t *pCBs )
{
  ZStatus_t stat = ZSuccess;

  if ( zcl_ServerCmd( pInMsg->hdr.fc.direction ) )
  {
    // Process Client commands, received by server
    if( pInMsg->hdr.commandID == COMMAND_SE_GET_CURRENT_PRICE )
    {
      stat = zclSE_ProcessInCmd_Pricing_GetCurrentPrice( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_GET_SCHEDULED_PRICE )
    {
      stat = zclSE_ProcessInCmd_Pricing_GetScheduledPrice( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }
  else
  {
    // Process Server commands, received by client
    if ( pInMsg->hdr.commandID == COMMAND_SE_PUBLISH_PRICE )
    {
      stat = zclSE_ProcessInCmd_Pricing_PublishPrice( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_Pricing_GetCurrentPrice
 *
 * @brief   Process in the received Get Current Price.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZCL_STATUS_CMD_HAS_RSP @ Supported and do
 *                                           not need default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_Pricing_GetCurrentPrice( zclIncoming_t *pInMsg,
                                                              zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnPricing_GetCurrentPrice )
  {
    zclCCGetCurrentPrice_t cmd;

    cmd.option = pInMsg->pData[0];

    pCBs->pfnPricing_GetCurrentPrice( &cmd,  &(pInMsg->msg->srcAddr),
                                     pInMsg->hdr.transSeqNum );

    return ZCL_STATUS_CMD_HAS_RSP;
  }
  else
  {
    return ZFailure;
  }
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_Pricing_GetScheduledPrice
 *
 * @brief   Process in the received Get Scheduled Price.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZCL_STATUS_CMD_HAS_RSP @ Supported and do
 *                                           not need default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_Pricing_GetScheduledPrice( zclIncoming_t *pInMsg,
                                                                zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnPricing_GetScheduledPrice )
  {
    zclCCGetScheduledPrice_t cmd;

    cmd.startTime = osal_build_uint32( pInMsg->pData, 4 );
    cmd.numEvents = pInMsg->pData[4];

    pCBs->pfnPricing_GetScheduledPrice( &cmd, &(pInMsg->msg->srcAddr),
                                       pInMsg->hdr.transSeqNum );
    return ZCL_STATUS_CMD_HAS_RSP;
  }
  else
  {
    return ZFailure;
  }
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_Pricing_PublishPrice
 *
 * @brief   Process in the received Publish Price.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and need default rsp
 *                      ZCL_STATUS_INVALID_FIELD @ Range checking
 *                                           failure
 *                      ZCL_STATUS_SOFTWARE_FAILURE @ ZStack memory allocation failure
 */
static ZStatus_t zclSE_ProcessInCmd_Pricing_PublishPrice( zclIncoming_t *pInMsg,
                                                           zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnPricing_PublishPrice )
  {
    zclCCPublishPrice_t cmd;

    // Parse the command and do range check
    if ( zclSE_ParseInCmd_PublishPrice( &cmd, &(pInMsg->pData[0]),
                                        pInMsg->pDataLen ) == ZSuccess )
    {
      if ( cmd.unitOfMeasure > MAX_UNIT_OF_MEASURE_SE )
      {
        // Free the memory allocated in zclSE_ParseInCmd_PublishPrice()
        if ( cmd.rateLabel.pStr != NULL )
        {
          osal_mem_free( cmd.rateLabel.pStr );
        }
        return ZCL_STATUS_INVALID_FIELD;
      }

      pCBs->pfnPricing_PublishPrice( &cmd, &(pInMsg->msg->srcAddr),
                                    pInMsg->hdr.transSeqNum );

      // Free the memory allocated in zclSE_ParseInCmd_PublishPrice()
      if ( cmd.rateLabel.pStr != NULL )
      {
        osal_mem_free( cmd.rateLabel.pStr );
      }

      return ZSuccess;
    }
    else
    {
      return ZCL_STATUS_SOFTWARE_FAILURE;
    }
  }
  else
  {
    return ZFailure;
  }
}
#endif


#ifdef ZCL_MESSAGE
/*********************************************************************
 * @fn      zclSE_ProcessInMessageCmds
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis

 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSE_ProcessInMessageCmds( zclIncoming_t *pInMsg,
                                              zclSE_AppCallbacks_t *pCBs )
{
  ZStatus_t stat = ZSuccess;

  if ( zcl_ServerCmd( pInMsg->hdr.fc.direction ) )
  {
    // Process Client commands, received by server
    if ( pInMsg->hdr.commandID == COMMAND_SE_GET_LAST_MESSAGE )
    {
      stat = zclSE_ProcessInCmd_Message_GetLastMessage( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_MESSAGE_CONFIRMATION )
    {
      stat = zclSE_ProcessInCmd_Message_MessageConfirmation( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }
  else
  {
    // Process Server commands, received by client
    if( pInMsg->hdr.commandID == COMMAND_SE_DISPLAY_MESSAGE )
    {
      stat = zclSE_ProcessInCmd_Message_DisplayMessage( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_CANCEL_MESSAGE )
    {
      stat = zclSE_ProcessInCmd_Message_CancelMessage( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_Message_DisplayMessage
 *
 * @brief   Process in the received Display Message Command.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 *                      ZCL_STATUS_SOFTWARE_FAILURE @ ZStack memory allocation failure
 */
static ZStatus_t zclSE_ProcessInCmd_Message_DisplayMessage( zclIncoming_t *pInMsg,
                                                             zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnMessage_DisplayMessage )
  {
    zclCCDisplayMessage_t cmd;

    if ( zclSE_ParseInCmd_DisplayMessage( &cmd,  &(pInMsg->pData[0]), pInMsg->pDataLen ) == ZSuccess )
    {
      pCBs->pfnMessage_DisplayMessage( &cmd, &(pInMsg->msg->srcAddr),
                                    pInMsg->hdr.transSeqNum );

      // Free memory allocated in zclSE_ParseInCmd_DiplayMessage()
      if ( cmd.msgString.pStr != NULL )
        osal_mem_free( cmd.msgString.pStr );

      return ZSuccess;
    }
  }

  return ZFailure;

}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_Message_CancelMessage
 *
 * @brief   Process in the received Cancel Message Command.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_Message_CancelMessage( zclIncoming_t *pInMsg,
                                                            zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnMessage_CancelMessage )
  {
    zclCCCancelMessage_t cmd;

    cmd.messageId = osal_build_uint32( &(pInMsg->pData[0]), 4 );

    // Message Control
    cmd.messageCtrl.transmissionMode = pInMsg->pData[4] & 0x03;    // bit 0&1
    cmd.messageCtrl.importance = ( pInMsg->pData[4] >> SE_PROFILE_MSGCTRL_IMPORTANCE ) & 0x03; // bit 2&3
    cmd.messageCtrl.confirmationRequired = ( pInMsg->pData[4] >> SE_PROFILE_MSGCTRL_CONFREQUIRED ) & 0x01;  // bit 7

    pCBs->pfnMessage_CancelMessage( &cmd, &(pInMsg->msg->srcAddr),
                                   pInMsg->hdr.transSeqNum );

    return ZSuccess;
  }
  else
  {
    return ZFailure;
  }
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_Message_GetLastMessage
 *
 * @brief   Process in the received Get Last Message Command.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZCL_STATUS_CMD_HAS_RSP @ Supported and do
 *                                           not need default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_Message_GetLastMessage( zclIncoming_t *pInMsg,
                                                             zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnMessage_GetLastMessage )
  {
    pCBs->pfnMessage_GetLastMessage( &(pInMsg->msg->srcAddr), pInMsg->hdr.transSeqNum );
    return ZCL_STATUS_CMD_HAS_RSP;
  }
  else
  {
    return ZFailure;
  }
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_Message_MessageConfirmation
 *
 * @brief   Process in the received Message Confirmation.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZSuccess @ Supported and send default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_Message_MessageConfirmation( zclIncoming_t *pInMsg,
                                                                  zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnMessage_MessageConfirmation )
  {
    zclCCMessageConfirmation_t cmd;

    cmd.messageId = osal_build_uint32( &(pInMsg->pData[0]), 4 );
    cmd.confirmTime = osal_build_uint32( &(pInMsg->pData[4]), 4 );

    pCBs->pfnMessage_MessageConfirmation( &cmd, &(pInMsg->msg->srcAddr),
                                         pInMsg->hdr.transSeqNum );
    return ZSuccess;
  }
  else
  {
    return ZFailure;
  }
}
#endif


#ifdef ZCL_LOAD_CONTROL
/*********************************************************************
 * @fn      zclSE_ProcessInLoadControlCmds
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis

 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSE_ProcessInLoadControlCmds( zclIncoming_t *pInMsg,
                                                  zclSE_AppCallbacks_t *pCBs )
{
  ZStatus_t stat = ZSuccess;

  if ( zcl_ServerCmd( pInMsg->hdr.fc.direction ) )
  {
    // Process Client commands, received by server
    if ( pInMsg->hdr.commandID == COMMAND_SE_REPORT_EVENT_STATUS )
    {
      stat = zclSE_ProcessInCmd_LoadControl_ReportEventStatus( pInMsg, pCBs );
    }
    else if( pInMsg->hdr.commandID == COMMAND_SE_GET_SCHEDULED_EVENT )
    {
      stat = zclSE_ProcessInCmd_LoadControl_GetScheduledEvents( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }
  else
  {
    // Process Server commands, received by client
    if( pInMsg->hdr.commandID == COMMAND_SE_LOAD_CONTROL_EVENT )
    {
      stat = zclSE_ProcessInCmd_LoadControl_LoadControlEvent( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_CANCEL_LOAD_CONTROL_EVENT )
    {
      stat = zclSE_ProcessInCmd_LoadControl_CancelLoadControlEvent( pInMsg, pCBs );
    }
    else if ( pInMsg->hdr.commandID == COMMAND_SE_CANCEL_ALL_LOAD_CONTROL_EVENT )
    {
      stat = zclSE_ProcessInCmd_LoadControl_CancelAllLoadControlEvents( pInMsg, pCBs );
    }
    else
      stat = ZFailure;
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_LoadControl_LoadControlEvent
 *
 * @brief   Process in the received Load Control Event.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t - ZFailure @ Unsupported
 *                      ZCL_STATUS_CMD_HAS_RSP @ Supported and do
 *                                           not need default rsp
 */
static ZStatus_t zclSE_ProcessInCmd_LoadControl_LoadControlEvent( zclIncoming_t *pInMsg,
                                                                  zclSE_AppCallbacks_t *pCBs )
{
  uint8 status = ZSuccess;

  if ( pCBs->pfnLoadControl_LoadControlEvent )
  {
    zclCCLoadControlEvent_t cmd;

    zclSE_ParseInCmd_LoadControlEvent( &cmd, &(pInMsg->pData[0]), pInMsg->pDataLen );

    // Range checking
    if ( cmd.durationInMinutes > MAX_DURATION_IN_MINUTES_SE_LOAD_CONTROL )
    {
      status = ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd.criticalityLevel > MAX_CRITICAL_LEVEL_SE_LOAD_CONTROL )
    {
      status = ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd. coolingTemperatureSetPoint != SE_OPTIONAL_FIELD_TEMPERATURE_SET_POINT &&
         cmd. coolingTemperatureSetPoint > MAX_TEMPERATURE_SETPOINT_SE_LOAD_CONTROL )
    {
      status = ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd. heatingTemperatureSetPoint != SE_OPTIONAL_FIELD_TEMPERATURE_SET_POINT &&
         cmd. heatingTemperatureSetPoint > MAX_TEMPERATURE_SETPOINT_SE_LOAD_CONTROL )
    {
      status = ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd.averageLoadAdjustmentPercentage != SE_OPTIONAL_FIELD_INT8 &&
        (cmd.averageLoadAdjustmentPercentage < MIN_AVERAGE_LOAD_ADJUSTMENT_PERCENTAGE_SE ||
         cmd.averageLoadAdjustmentPercentage > MAX_AVERAGE_LOAD_ADJUSTMENT_PERCENTAGE_SE ) )
    {
      status = ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd. dutyCycle != SE_OPTIONAL_FIELD_UINT8 &&
         cmd. dutyCycle > MAX_DUTY_CYCLE_SE_LOAD_CONTROL )
    {
      status = ZCL_STATUS_INVALID_FIELD;
    }

    // If any of the four fields is optional, set them all to optional
    if ( cmd.coolingTemperatureOffset == SE_OPTIONAL_FIELD_UINT8 ||
         cmd.heatingTemperatureOffset == SE_OPTIONAL_FIELD_UINT8 ||
         cmd.coolingTemperatureSetPoint == SE_OPTIONAL_FIELD_UINT16 ||
         cmd.heatingTemperatureSetPoint == SE_OPTIONAL_FIELD_UINT16 )
    {
      cmd.coolingTemperatureOffset = SE_OPTIONAL_FIELD_UINT8;
      cmd.heatingTemperatureOffset = SE_OPTIONAL_FIELD_UINT8;
      cmd.coolingTemperatureSetPoint = SE_OPTIONAL_FIELD_UINT16;
      cmd.heatingTemperatureSetPoint = SE_OPTIONAL_FIELD_UINT16;
    }

    pCBs->pfnLoadControl_LoadControlEvent( &cmd, &(pInMsg->msg->srcAddr), status, pInMsg->hdr.transSeqNum );

    // The Load Control Event command has response, therefore,
    // inform zclto not to send default response.
    return ZCL_STATUS_CMD_HAS_RSP;
  }
  else
    return ZFailure;  // Not supported
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_LoadControl_CancelLoadControlEvent
 *
 * @brief   Process in the received Cancel Load Control Event.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 */
static ZStatus_t zclSE_ProcessInCmd_LoadControl_CancelLoadControlEvent( zclIncoming_t *pInMsg,
                                                                         zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnLoadControl_CancelLoadControlEvent )
  {
    zclCCCancelLoadControlEvent_t cmd;

    zclSE_ParseInCmd_CancelLoadControlEvent( &cmd, &(pInMsg->pData[0]), pInMsg->pDataLen );

    pCBs->pfnLoadControl_CancelLoadControlEvent( &cmd, &(pInMsg->msg->srcAddr), pInMsg->hdr.transSeqNum );
    return ZSuccess;
  }
  else
    return ZFailure;
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_LoadControl_CancelAllLoadControlEvent
 *
 * @brief   Process in the received Cancel All Load Control Event.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 */
static ZStatus_t zclSE_ProcessInCmd_LoadControl_CancelAllLoadControlEvents( zclIncoming_t *pInMsg,
                                                                             zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnLoadControl_CancelAllLoadControlEvents )
  {
    zclCCCancelAllLoadControlEvents_t cmd;

    cmd.cancelControl = pInMsg->pData[0];

    pCBs->pfnLoadControl_CancelAllLoadControlEvents( &cmd, &(pInMsg->msg->srcAddr), pInMsg->hdr.transSeqNum );
    return ZSuccess;
  }
  else
    return ZFailure;
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_LoadControl_ReportEventStatus
 *
 * @brief   Process in the received Load Control Event.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 */
static ZStatus_t zclSE_ProcessInCmd_LoadControl_ReportEventStatus( zclIncoming_t *pInMsg,
                                                                    zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnLoadControl_ReportEventStatus )
  {

    zclCCReportEventStatus_t cmd;

    zclSE_ParseInCmd_ReportEventStatus( &cmd, &(pInMsg->pData[0]), pInMsg->pDataLen );

    // Range Checking
    if ( cmd.eventStatus != EVENT_STATUS_LOAD_CONTROL_EVENT_REJECTED &&
        (cmd.eventStatus == 0 ||
         cmd.eventStatus > EVENT_STATUS_LOAD_CONTROL_EVENT_SUPERSEDED ) )
    {
      return ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd.criticalityLevelApplied > MAX_CRITICAL_LEVEL_SE_LOAD_CONTROL )
    {
      return ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd.coolingTemperatureSetPointApplied != SE_OPTIONAL_FIELD_TEMPERATURE_SET_POINT &&
         cmd.coolingTemperatureSetPointApplied > MAX_TEMPERATURE_SETPOINT_SE_LOAD_CONTROL )
    {
      return ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd.heatingTemperatureSetPointApplied != SE_OPTIONAL_FIELD_TEMPERATURE_SET_POINT &&
         cmd.heatingTemperatureSetPointApplied > MAX_TEMPERATURE_SETPOINT_SE_LOAD_CONTROL )
    {
      return ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd.averageLoadAdjustment != SE_OPTIONAL_FIELD_INT8 &&
        (cmd.averageLoadAdjustment < MIN_AVERAGE_LOAD_ADJUSTMENT_PERCENTAGE_SE ||
         cmd.averageLoadAdjustment > MAX_AVERAGE_LOAD_ADJUSTMENT_PERCENTAGE_SE ) )
    {
      return ZCL_STATUS_INVALID_FIELD;
    }

    if ( cmd.dutyCycleApplied != SE_OPTIONAL_FIELD_UINT8 &&
         cmd.dutyCycleApplied > MAX_DUTY_CYCLE_SE_LOAD_CONTROL )
    {
      return ZCL_STATUS_INVALID_FIELD;
    }

    pCBs->pfnLoadControl_ReportEventStatus( &cmd, &(pInMsg->msg->srcAddr), pInMsg->hdr.transSeqNum );

    return ZSuccess;
  }
  else
    return ZFailure;
}

/*********************************************************************
 * @fn      zclSE_ProcessInCmd_LoadControl_GetScheduledEvents
 *
 * @brief   Process in the received Get Scheduled Event.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 */
static ZStatus_t zclSE_ProcessInCmd_LoadControl_GetScheduledEvents( zclIncoming_t *pInMsg,
                                                                    zclSE_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnLoadControl_GetScheduledEvents )
  {
    zclCCGetScheduledEvent_t cmd;

    cmd.startTime = osal_build_uint32( pInMsg->pData, 4);
    cmd.numEvents = pInMsg->pData[4];

    pCBs->pfnLoadControl_GetScheduledEvents( &cmd, &(pInMsg->msg->srcAddr), pInMsg->hdr.transSeqNum );
    return ZSuccess;
  }
  else
    return ZFailure;
}

#endif

#ifdef ZCL_PRICING
/*********************************************************************
 * @fn      zclSE_ParseInCmd_PublishPrice
 *
 * @brief   Parse received Publish Price Command.
 *
 * @param   cmd - pointer to the output data struct
 * @param   buf - pointer to the input data buffer
 * @param   len - length of the input buffer
 *
 */
ZStatus_t zclSE_ParseInCmd_PublishPrice( zclCCPublishPrice_t *cmd, uint8 *buf, uint8 len )
{
  (void)len;  // Intentionally unreferenced parameter

  // Parse the command buffer
  cmd->providerId = osal_build_uint32( buf, 4 );
  buf += 4;

  // Notice that rate label is a variable length UTF-8 string
  cmd->rateLabel.strLen = *buf++;
  if ( cmd->rateLabel.strLen == SE_OPTIONAL_FIELD_UINT8 )
  {
    // If character count is 0xFF, set string length to 0
    cmd->rateLabel.strLen = 0;
  }

  if ( cmd->rateLabel.strLen != 0 )
  {
    cmd->rateLabel.pStr = osal_mem_alloc( cmd->rateLabel.strLen );
    if ( cmd->rateLabel.pStr == NULL )
    {
      return ZMemError;
    }
    osal_memcpy( cmd->rateLabel.pStr, buf, cmd->rateLabel.strLen );
    buf += cmd->rateLabel.strLen;
  }
  else
  {
    cmd->rateLabel.pStr = NULL;
  }

  cmd->issuerEventId = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->currentTime = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->unitOfMeasure = *buf++;
  cmd->currency = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;

  cmd->priceTrailingDigit = *buf++;
  cmd->priceTier = *buf++;
  cmd->startTime = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->durationInMinutes = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;

  cmd->price = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->priceRatio = *buf++;
  cmd->generationPrice = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->generationPriceRatio = *buf;

  return ZSuccess;
}
#endif

#ifdef ZCL_MESSAGE
/*********************************************************************
 * @fn      zclSE_ParseInCmd_DisplayMessage
 *
 * @brief   Parse received Display Message Command.
 *
 * @param   cmd - pointer to the output data struct
 * @param   buf - pointer to the input data buffer
 * @param   len - length of the input buffer
 *
 * @return  ZStatus_t - ZSuccess or ZFailure due to memory allocation failure
 */
ZStatus_t zclSE_ParseInCmd_DisplayMessage( zclCCDisplayMessage_t *cmd, uint8 *buf, uint8 len )
{
  (void)len;  // Intentionally unreferenced parameter

  cmd->messageId = osal_build_uint32( buf, 4 );

  // Message control bitmap
  cmd->messageCtrl.transmissionMode = buf[4] & 0x03;    // bit 0&1
  cmd->messageCtrl.importance = ( buf[4] >> SE_PROFILE_MSGCTRL_IMPORTANCE ) & 0x03; // bit 2&3
  cmd->messageCtrl.confirmationRequired = (buf[4] >> SE_PROFILE_MSGCTRL_CONFREQUIRED ) & 0x01;  // bit 7

  cmd->startTime = osal_build_uint32( &(buf[5]), 4 );
  cmd->durationInMinutes = BUILD_UINT16( buf[9], buf[10] );
  cmd->msgString.strLen = buf[11];

  // Copy the message string
  if( cmd->msgString.strLen != 0 )
  {
    cmd->msgString.pStr = osal_mem_alloc( cmd->msgString.strLen );
    if ( cmd->msgString.pStr == NULL )
      return ZMemError;
    osal_memcpy( cmd->msgString.pStr, &(buf[12]), cmd->msgString.strLen );
  }
  else
  {
    cmd->msgString.pStr = NULL;
  }
  return ZSuccess;
}
#endif

#ifdef ZCL_LOAD_CONTROL
/*********************************************************************
 * @fn      zclSE_ParseInCmd_LoadControlEvent
 *
 * @brief   Parse received Load Control Event.
 *
 * @param   cmd - pointer to the output data struct
 * @param   buf - pointer to the input data buffer
 * @param   len - length of the input buffer
 *
 */
void zclSE_ParseInCmd_LoadControlEvent( zclCCLoadControlEvent_t *cmd, uint8 *buf, uint8 len )
{
  (void)len;  // Intentionally unreferenced parameter

  // Maybe add checking for buffer length later
  // Skipped right now to leave MT input to guarantee
  // proper buffer length
  cmd->issuerEvent = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->deviceGroupClass = osal_build_uint32( buf, 3 );
  buf += 3;

  cmd->startTime = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->durationInMinutes = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;

  cmd->criticalityLevel = *buf++;
  cmd->coolingTemperatureOffset = *buf++;
  cmd->heatingTemperatureOffset = *buf++;

  cmd->coolingTemperatureSetPoint = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;

  cmd->heatingTemperatureSetPoint = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;

  cmd->averageLoadAdjustmentPercentage = *buf++;
  cmd->dutyCycle = *buf++;
  cmd->eventControl = *buf++;

  return;

}

/*********************************************************************
 * @fn      zclSE_ParseInCmd_CancelLoadControlEvent
 *
 * @brief   Parse received Cancel Load Control Event Command.
 *
 * @param   cmd - pointer to the output data struct
 * @param   buf - pointer to the input data buffer
 * @param   len - length of the input buffer
 *
 */
void zclSE_ParseInCmd_CancelLoadControlEvent( zclCCCancelLoadControlEvent_t *cmd, uint8 *buf, uint8 len )
{
  (void)len;  // Intentionally unreferenced parameter

  // Maybe add checking for buffer length later
  // Skipped right now to leave MT input to guarantee
  // proper buffer length
  cmd->issuerEventID = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->deviceGroupClass = osal_build_uint32( buf, 3 );
  buf += 3;

  cmd->cancelControl = *buf++;
  cmd->effectiveTime = osal_build_uint32( buf, 4 );

  return;

}

/*********************************************************************
 * @fn      zclSE_ParseInCmd_ReportEventStatus
 *
 * @brief   Parse received Report Event Status.
 *
 * @param   cmd - pointer to the output data struct
 * @param   buf - pointer to the input data buffer
 * @param   len - length of the input buffer
 *
 */
void zclSE_ParseInCmd_ReportEventStatus( zclCCReportEventStatus_t *cmd, uint8 *buf, uint8 len )
{
  (void)len;  // Intentionally unreferenced parameter

  // Maybe add checking for buffer length later
  // Skipped right now to leave MT input to guarantee
  // proper buffer length
  cmd->issuerEventID = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->eventStatus = *buf++;

  cmd->eventStartTime = osal_build_uint32( buf, 4 );
  buf += 4;

  cmd->criticalityLevelApplied = *buf++;
  cmd->coolingTemperatureSetPointApplied = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;

  cmd->heatingTemperatureSetPointApplied = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;

  cmd->averageLoadAdjustment = *buf++;
  cmd->dutyCycleApplied = *buf++;
  cmd->eventControl = *buf++;
  cmd->signatureType = *buf++;

  osal_memcpy( cmd->signature, buf, SE_PROFILE_SIGNATURE_LENGTH );

  return;

}
#endif
/********************************************************************************************
*********************************************************************************************/
