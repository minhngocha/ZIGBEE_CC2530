/**************************************************************************************************
  Filename:       SerialApp.c
  Revised:        $Date: 2009-03-29 10:51:47 -0700 (Sun, 29 Mar 2009) $
  Revision:       $Revision: 19585 $

  Description -   Serial Transfer Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
  This sample application is basically a cable replacement
  and it should be customized for your application. A PC
  (or other device) sends data via the serial port to this
  application's device.  This device transmits the message
  to another device with the same application running. The
  other device receives the over-the-air message and sends
  it to a PC (or other device) connected to its serial port.

  This application doesn't have a profile, so it handles everything directly.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
   INCLUDES
*/

#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "SerialApp.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "hal_drivers.h"
#include "hal_key.h"
#if defined ( LCD_SUPPORTED )
#include "hal_lcd.h"
#endif
#include "hal_led.h"
#include "hal_uart.h"

/*********************************************************************
   MACROS
*/
#include "stdio.h"
#include "mac_api.h"
#define ROUTER_1     '1'
#define ROUTER_2     '2'
#define ROUTER_3     '3'

#define CHECK_ROUTER  ROUTER_1


#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE || ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ROUTER
#define PERIODIC_TIME_EVT       0X1000
#endif //#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE

#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE || ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ROUTER
#define START_END_DEVICE        0X2000
#endif //#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE

#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR
#define START_COORDINATOR       0X4000
#endif //#if ZSTACK_DEVICE_BUILD == ZG_DEVICETYPE_COORDINATOR

#define USER_DEBUG       FALSE
uint8   count_delay   =    0;
uint16  count_transmit =    0;
/*********************************************************************
   CONSTANTS
*/

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_115200
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  64
#endif

#if !defined( SERIAL_APP_RX_SZ )
#define SERIAL_APP_RX_SZ  128
#endif

#if !defined( SERIAL_APP_TX_SZ )
#define SERIAL_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SERIAL_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SERIAL_APP_LOOPBACK )
#define SERIAL_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  80
#endif

#define SERIAL_APP_RSP_CNT  4

// This list should be filled with Application specific Cluster IDs.
const cId_t SerialApp_ClusterList[SERIALAPP_MAX_CLUSTERS] =
{
  SERIALAPP_CLUSTERID1,
  SERIALAPP_CLUSTERID2
};

const SimpleDescriptionFormat_t SerialApp_SimpleDesc =
{
  SERIALAPP_ENDPOINT,              //  int   Endpoint;
  SERIALAPP_PROFID,                //  uint16 AppProfId[2];
  SERIALAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SERIALAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SERIALAPP_FLAGS,                 //  int   AppFlags:4;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SerialApp_ClusterList,  //  byte *pAppInClusterList;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)SerialApp_ClusterList   //  byte *pAppOutClusterList;
};

const endPointDesc_t SerialApp_epDesc =
{
  SERIALAPP_ENDPOINT,
  &SerialApp_TaskID,
  (SimpleDescriptionFormat_t *)&SerialApp_SimpleDesc,
  noLatencyReqs
};

/*********************************************************************
   TYPEDEFS
*/

/*********************************************************************
   GLOBAL VARIABLES
*/

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.

/*********************************************************************
   EXTERNAL VARIABLES
*/

/*********************************************************************
   EXTERNAL FUNCTIONS
*/

/*********************************************************************
   LOCAL VARIABLES
*/

static uint8 SerialApp_MsgID;

static afAddrType_t SerialApp_TxAddr;
static uint8 SerialApp_TxSeq;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX + 1];
static uint8 SerialApp_TxLen;

static afAddrType_t SerialApp_RxAddr;
static uint8 SerialApp_RxSeq;
static uint8 SerialApp_RspBuf[SERIAL_APP_RSP_CNT];

/*********************************************************************
   LOCAL FUNCTIONS
*/

static void SerialApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void SerialApp_HandleKeys( uint8 shift, uint8 keys );
static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
static void SerialApp_Send(void);
static void SerialApp_Resp(void);
static void SerialApp_CallBack(uint8 port, uint8 event);



/******************************************************************************************************
   FUNCTIONs USER &&    VARIABLE USER

*/
static afAddrType_t SerialApp_user;

void user_debug(char *tx_buf);
static void GenericApp_SendTheMessage(void);
static void Serial_user_callback(uint8 port, uint8 event);
/*****************************************************************************************************/



void user_debug(char *tx_buf)
{
  while (*tx_buf != 0x00)
  {
    HalUARTWrite( SERIAL_APP_PORT, (uint8 *) tx_buf, 1 ) ;
    tx_buf++;
  }
}


/*********************************************************************
   @fn      SerialApp_Init

   @brief   This is called during OSAL tasks' initialization.

   @param   task_id - the Task ID assigned by OSAL.

   @return  none
*/
void SerialApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;

  SerialApp_TaskID = task_id;
  SerialApp_RxSeq = 0xC3;

  afRegister( (endPointDesc_t *)&SerialApp_epDesc );

  RegisterForKeys( task_id );

  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = HAL_UART_FLOW_OFF;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH;
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = Serial_user_callback;

  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);


#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SerialApp", HAL_LCD_LINE_2 );
#endif

  ZDO_RegisterForZDOMsg( SerialApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( SerialApp_TaskID, Match_Desc_rsp );

  // set tx power maximum//
  uint8 value_txpower = 0xF5;
  MAC_MlmeSetReq(MAC_PHY_TRANSMIT_POWER, &value_txpower);

#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE  || ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ROUTER
  osal_start_timerEx(SerialApp_TaskID, START_END_DEVICE, 2500);
#endif

#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR
  osal_start_timerEx(SerialApp_TaskID, START_COORDINATOR , 2000);
#endif

}

/*********************************************************************
   @fn      SerialApp_ProcessEvent

   @brief   Generic Application Task event processor.

   @param   task_id  - The OSAL assigned task ID.
   @param   events   - Bit map of events to process.

   @return  Event flags of all unprocessed events.
*/
UINT16 SerialApp_ProcessEvent( uint8 task_id, UINT16 events )
{
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    afIncomingMSGPacket_t *MSGpkt;

    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SerialApp_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          SerialApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          SerialApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_INCOMING_MSG_CMD:
          SerialApp_ProcessMSGCmd( MSGpkt );
          break;

        default:
          break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    return ( events ^ SYS_EVENT_MSG );
  }

  if ( events & SERIALAPP_SEND_EVT )
  {
    SerialApp_Send();
    return ( events ^ SERIALAPP_SEND_EVT );
  }

  if ( events & SERIALAPP_RESP_EVT )
  {
    SerialApp_Resp();
    return ( events ^ SERIALAPP_RESP_EVT );
  }


#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE || ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ROUTER
  if ( events & PERIODIC_TIME_EVT )
  {
    osal_start_timerEx(SerialApp_TaskID, PERIODIC_TIME_EVT , 5000);
    GenericApp_SendTheMessage();
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_FLASH );
    return ( events ^ PERIODIC_TIME_EVT );
  }
#endif //#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE


#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE || ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ROUTER
  if ( events & START_END_DEVICE )
  {
    if (count_delay == 1)
    {
      SerialApp_HandleKeys( 0, HAL_KEY_SW_2 );
  //    GenericApp_SendTheMessage();
      if ( osal_start_timerEx(SerialApp_TaskID, PERIODIC_TIME_EVT , 1000) == SUCCESS)
      {
#if USER_DEBUG
        user_debug(" periodic is start \r\n");
        user_debug(" send HELLO WORLD in 3s\r\n");
#endif
      }
      else
      {
        user_debug(" periodis start false \r\n");
      }
      osal_stop_timerEx(SerialApp_TaskID, START_END_DEVICE);
    }
    if (count_delay == 0)
    {
      osal_start_timerEx(SerialApp_TaskID, START_END_DEVICE, 2500);
      count_delay++;
    }
    return ( events ^ START_END_DEVICE );
  }
#endif //#if ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE

  return ( 0 );  // Discard unknown events.
}

/*********************************************************************
   @fn      SerialApp_ProcessZDOMsgs()

   @brief   Process response messages

   @param   none

   @return  none
*/
static void SerialApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
#if USER_DEBUG
  user_debug("SerialApp_ProcessZDOMsgs \r\n");
#endif
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            SerialApp_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
            SerialApp_TxAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            SerialApp_TxAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
   @fn      SerialApp_HandleKeys

   @brief   Handles all key events for this device.

   @param   shift - true if in shift/alt.
   @param   keys  - bit field for key events.

   @return  none
*/
void SerialApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t txAddr;

  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      txAddr.addrMode = Addr16Bit;
      txAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &txAddr, NLME_GetShortAddr(),
                            SerialApp_epDesc.endPoint,
                            SERIALAPP_PROFID,
                            SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                            SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                            FALSE );
#if USER_DEBUG
      user_debug("Initiate an End Device Bind Request for the mandatory endpoint \r\n");
#endif
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate a Match Description Request (Service Discovery)
      txAddr.addrMode = AddrBroadcast;
      txAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &txAddr, NWK_BROADCAST_SHORTADDR,
                        SERIALAPP_PROFID,
                        SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                        SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                        FALSE );
#if USER_DEBUG
      user_debug("Initiate a Match Description Request (Service Discovery)\r\n");
#endif
    }
  }
}

/*********************************************************************
   @fn      SerialApp_ProcessMSGCmd

   @brief   Data message processor callback. This function processes
            any incoming data - probably from other devices. Based
            on the cluster ID, perform the intended action.

   @param   pkt - pointer to the incoming message packet

   @return  TRUE if the 'pkt' parameter is being used and will be freed later,
            FALSE otherwise.
*/
void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )
{
  uint8 stat;
  uint8 seqnb;
  uint8 delay;
#if USER_DEBUG
  user_debug("SerialApp_ProcessMSGCmd \r\n");
#endif
  switch ( pkt->clusterId )
  {
    // A message with a serial data block to be transmitted on the serial port.
    case SERIALAPP_CLUSTERID1:
      // Store the address for sending and retrying.
      osal_memcpy(&SerialApp_RxAddr, &(pkt->srcAddr), sizeof( afAddrType_t ));

      seqnb = pkt->cmd.Data[0];

      
#if ZSTACK_DEVICE_BUILD  ==   DEVICE_BUILD_ROUTER
      if (pkt->cmd.Data[0] == 'R' && pkt->cmd.Data[1] == 'T' && pkt->cmd.Data[2] == CHECK_ROUTER)
      {
        if ( HalUARTWrite( SERIAL_APP_PORT, pkt->cmd.Data + 3 , pkt->cmd.DataLength - 3) )  //+1
        {
          SerialApp_RxSeq = seqnb;
          stat = OTA_SUCCESS;
        }
        else
        {
          stat = OTA_SER_BUSY;
        }
      }
#else
      if ( HalUARTWrite( SERIAL_APP_PORT, pkt->cmd.Data, (pkt->cmd.DataLength) ) )  //+1
      {
        // Save for next incoming message
#if USER_DEBUG
        user_debug("serial port transmit data success \r\n");
#endif
        SerialApp_RxSeq = seqnb;
        stat = OTA_SUCCESS;
      }
      else
      {
        stat = OTA_SER_BUSY;
#if USER_DEBUG
        user_debug("serial port transmit data not success \r\n");
#endif
      }
#endif 
      delay = (stat == OTA_SER_BUSY) ? SERIALAPP_NAK_DELAY : SERIALAPP_ACK_DELAY;

      // Build & send OTA response message.
      SerialApp_RspBuf[0] = stat;
      SerialApp_RspBuf[1] = seqnb;
      SerialApp_RspBuf[2] = LO_UINT16( delay );
      SerialApp_RspBuf[3] = HI_UINT16( delay );
      osal_set_event( SerialApp_TaskID, SERIALAPP_RESP_EVT );
      osal_stop_timerEx(SerialApp_TaskID, SERIALAPP_RESP_EVT);
      break;

    // A response to a received serial data block.
    case SERIALAPP_CLUSTERID2:

      if ((pkt->cmd.Data[1] == SerialApp_TxSeq) &&
          ((pkt->cmd.Data[0] == OTA_SUCCESS) || (pkt->cmd.Data[0] == OTA_DUP_MSG)))
      {
        SerialApp_TxLen = 0;
        osal_stop_timerEx(SerialApp_TaskID, SERIALAPP_SEND_EVT);
      }
      else
      {
        // Re-start timeout according to delay sent from other device.
        delay = BUILD_UINT16( pkt->cmd.Data[2], pkt->cmd.Data[3] );
        osal_start_timerEx( SerialApp_TaskID, SERIALAPP_SEND_EVT, delay );
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
   @fn      SerialApp_Send

   @brief   Send data OTA.

   @param   none

   @return  none
*/
static void SerialApp_Send(void)
{
#if USER_DEBUG
  user_debug("SerialApp_Send \r\n");
#endif
#if SERIAL_APP_LOOPBACK
#if USER_DEBUG
  user_debug("SerialApp_Send with SERIAL_APP_LOOPBACK is true \r\n");
#endif
  if (SerialApp_TxLen < SERIAL_APP_TX_MAX)
  {
    SerialApp_TxLen += HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf + SerialApp_TxLen + 1,
                                   SERIAL_APP_TX_MAX - SerialApp_TxLen);
  }

  if (SerialApp_TxLen)
  {
    (void)SerialApp_TxAddr;
    if (HalUARTWrite(SERIAL_APP_PORT, SerialApp_TxBuf + 1, SerialApp_TxLen))
    {
      SerialApp_TxLen = 0;
    }
    else
    {
      osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
    }
  }
#else

  if (!SerialApp_TxLen &&
      (SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf + 1, SERIAL_APP_TX_MAX)))
  {
    // Pre-pend sequence number to the Tx message.
    SerialApp_TxBuf[0] = ++SerialApp_TxSeq;
#if USER_DEBUG
    user_debug("SerialApp_Send with SERIAL_APP_LOOPBACK is false \r\n");
#endif
  }

  if (SerialApp_TxLen)
  {
#if USER_DEBUG
    user_debug("before AF_DataRequest \r\n");
#endif
    if (afStatus_SUCCESS != AF_DataRequest(&SerialApp_TxAddr,
                                           (endPointDesc_t *)&SerialApp_epDesc,
                                           SERIALAPP_CLUSTERID1,
                                           SerialApp_TxLen + 1, SerialApp_TxBuf,
                                           &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
    {
      osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
    }
#if USER_DEBUG
    user_debug("after AF_DataRequest \r\n");
#endif
  }
#endif
}

/*********************************************************************
   @fn      SerialApp_Resp

   @brief   Send data OTA.

   @param   none

   @return  none
*/
static void SerialApp_Resp(void)
{
  if (afStatus_SUCCESS != AF_DataRequest(&SerialApp_RxAddr,
                                         (endPointDesc_t *)&SerialApp_epDesc,
                                         SERIALAPP_CLUSTERID2,
                                         SERIAL_APP_RSP_CNT, SerialApp_RspBuf,
                                         &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
  {
    osal_set_event(SerialApp_TaskID, SERIALAPP_RESP_EVT);
  }
}

/*********************************************************************
   @fn      SerialApp_CallBack

   @brief   Send data OTA.

   @param   port - UART port.
   @param   event - the UART port event flag.

   @return  none
*/
static void SerialApp_CallBack(uint8 port, uint8 event)
{
  (void)port;

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
      (SerialApp_TxLen < SERIAL_APP_TX_MAX))
#else
      !SerialApp_TxLen)
#endif
  {
    SerialApp_Send();
  }
}

static void Serial_user_callback(uint8 port, uint8 event)
{
 char TX_transmit[25];
#if ZSTACK_DEVICE_BUILD  ==   DEVICE_BUILD_ROUTER
  byte count_RX = 3;  // ly do khi bao bien count_RX o day den tranh truong hop cong dong bien khi truyen sai hoac truyen nham ban tin
  TX_transmit[0] = 'R';
  TX_transmit[1] = 'T';
  TX_transmit[2] = CHECK_ROUTER;
#endif

#if ZSTACK_DEVICE_BUILD  ==   DEVICE_BUILD_COORDINATOR
byte count_RX = 0;
#endif
  uint8 icho;
  while (Hal_UART_RxBufLen(port))
  {
    HalUARTRead(port, &icho, 1);
    if (icho != 'x')
    {
      TX_transmit[count_RX] = icho;
      count_RX += 1;
    }
    if (icho == 'x')
    {
      SerialApp_user.addrMode = (afAddrMode_t) Addr16Bit; ///AddrBroadcast;
      SerialApp_user.endPoint = SERIALAPP_ENDPOINT;
      SerialApp_user.addr.shortAddr = 0xFFFC;    //  send to all router
       
      if ( AF_DataRequest( &SerialApp_user, (endPointDesc_t *)&SerialApp_epDesc,
                           SERIALAPP_CLUSTERID1,
                           (byte) count_RX,
                           (byte *)&TX_transmit,
                           &SerialApp_MsgID,
                           AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
        // nothing
      }

    }
  }
  // debug for switch 2
}

static void GenericApp_SendTheMessage(void)
{
//  char TX_transmit[2];
//  TX_transmit[0] = 'R';
//  TX_transmit[1] = '1';
//  SerialApp_user.addrMode = (afAddrMode_t) Addr16Bit; ///AddrBroadcast;
//      SerialApp_user.endPoint = SERIALAPP_ENDPOINT;
//      SerialApp_user.addr.shortAddr = 0xFFFC;    //  send to all router
//       
//      if ( AF_DataRequest( &SerialApp_user, (endPointDesc_t *)&SerialApp_epDesc,
//                           SERIALAPP_CLUSTERID1,
//                           (byte) 2,
//                           (byte *)&TX_transmit,
//                           &SerialApp_MsgID,
//                           AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
//      {
//        // nothing
//      }
}
/*********************************************************************
*********************************************************************/
