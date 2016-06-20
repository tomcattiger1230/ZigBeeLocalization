/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


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

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  CALBRATION_N_CLUSTERID,
  LZQR_GROUP_CLUSTERID   //id for localization of quadrotor group 
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Router_Periodic_DstAddr;       // address for router
afAddrType_t SampleApp_Tester_DstAddr;                // address for tester
afAddrType_t SampleApp_Quadrotor_Location_DstAddr;    // address for quadrotor/end device


aps_Group_t SampleApp_Group_1;                        // group 1 for calibration of parameter n (quadrotor, router, coord)
aps_Group_t SampleApp_Group_2;                        // group 2 for communication between quadrotors (quadrotor, coord)

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

//user setting for quadrotor

//buf for storing rssi values from 4 anchors
int8 perRssiBuf1[RSSI_AVG_WINDOW_SIZE]={0};     // array for storing the rssi reading from router 1
int8 perRssiBuf2[RSSI_AVG_WINDOW_SIZE]={0};     // array for storing the rssi reading from router 2
int8 perRssiBuf3[RSSI_AVG_WINDOW_SIZE]={0};     // array for storing the rssi reading from router 3
int8 perRssiBuf4[RSSI_AVG_WINDOW_SIZE]={0};     // array for storing the rssi reading from router 4
//buf for nvalue
float pernBuf1[8]={0};  // array for storing the n-value of router 1
float pernBuf2[8]={0};  // array for storing the n-value of router 2
float pernBuf3[8]={0};  // array for storing the n-value of router 3
float pernBuf4[8]={0};  // array for storing the n-value of router 4
//temp rssivalue
int8 rssivalue;         // temporary rssi value
// index for loop innenhalb the window
int8 i1=0;             
int8 i2=0;
int8 i3=0;
int8 i4=0;
// index for moving filter
int imof1[2]={0,1};
int imof2[2]={0,1};
int imof3[2]={0,1};
int imof4[2]={0,1};
// index for moving filter of n value
int nindex1[2]={0,1};
int nindex2[2]={0,1};
int nindex3[2]={0,1};
int nindex4[2]={0,1};
// sum/average Rssi value
float sumRssi1=0;
float sumRssi2=0;
float sumRssi3=0;
float sumRssi4=0;

//localization parameter
//float distance[4]={0,0,0,0};
float RssiValue[4]={0,0,0,0};                   // array for storing rssi values
float nValue[4]={3.59f,3.94f,2.63f,5.6f};       // initialized n parameter
float AValue[4]={47.52f,46.53,43.56,43.56};     // initialized A parameter
float AnchorX[4]={2.4f, 0.0f, 0.0f, 2.4f};      // router position in x direction
float AnchorY[4]={4.2f, 4.2f, 0.0f, 0.0f};      // router position in y direction
float AnchorZ[4]={0.0f, 0.0f, 0.0f, 0.0f};      // router position in z direction

//altitude information
float locationX[4]={0.0f, 0.0f, 0.0f, 0.0f};    // quadrotor position in x direction
float locationY[4]={0.0f, 0.0f, 0.0f, 0.0f};    // quadrotor position in y direction
float locationZ[4]={0.0f, 0.0f, 0.0f, 0.0f};    // quadrotor position in z direction

//LParameter tempParameter;                       // structure parameter, which is not in used

// device id
int DeviceID=3;                                 // device id to identify the device
                                                // for coordinator meanless; for router is the anchor's id, except id 6,
                                                // which is preparing for tester; for quadrotor, it is the id.
/*********************************************************************
 * LOCAL FUNCTIONS
 */
// button setting for tester
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
//deal with message(Co)
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
//deal with message(Ed)
void SampleApp_QuadrotorMSGCB( afIncomingMSGPacket_t *pckt );
//deal with message(Ro)
void SampleApp_AnchorMSGCB( afIncomingMSGPacket_t *pckt );
//tester calibration a(Ed)
void SampleApp_Tester_calibrationMSGCB(afIncomingMSGPacket_t *pckt );
//router send period message
void SampleApp_Router_SendPeriodicMessage( int index );
//quadrotor send their location in the group
void SampleApp_Quadrotor_SendLocation( int index );
//router calibration n
void SampleApp_Router_calibrationNMessage(int index, float nvalue);
//tester calibration A
void SampleApp_Tester_calibrationAMessage( int index , float A );

//void SampleApp_SendFlashMessage( uint16 flashTime );
//void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);

//float gaussianfilter(int8* rssi, int length);
//float averagefilter(int8* rssi, int length);
//float calibrationN(int localID, int index, float rssivalue_f, float AnchorX[4], float AnchorY[4], float AnchorZ[4], float A[4]);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
  HalUARTWrite(0,"UartInit OK\n", sizeof("UartInit OK\n"));//提示信息
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Router broadcast to everyone


  // Setup for the flash command's destination address - Group 1
//  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
//  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
//  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, only end devices and coordinator start out in Group 1
  SampleApp_Group_1.ID = SAMPLEAPP_CalN_GROUP; //0x0001
  //SampleApp_Group.name[0]=0; or just as bellow
  osal_memcpy( SampleApp_Group_1.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group_1 );
  
   // Only end devices and coordinator can joint this Group 2
  SampleApp_Group_2.ID = SAMPLEAPP_QUADROTOR_GROUP; //0x0002
  //SampleApp_Group.name[0]=0; or just as bellow
  osal_memcpy( SampleApp_Group_2.name, "Group 2", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group_2 );


 

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
  
  
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
  //HalUARTWrite(0, MSGpkt->cmd.Data, MSGpkt->cmd.DataLength); //输出接收到的数据
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {        
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
                  if ( (SampleApp_NwkState == DEV_ZB_COORD) 
                        //|| (SampleApp_NwkState == DEV_ROUTER)
                     //|| (SampleApp_NwkState == DEV_END_DEVICE)
                          
                      )
                  {
                   SampleApp_MessageMSGCB( MSGpkt );
                  }
                  else if((SampleApp_NwkState == DEV_END_DEVICE))
                  {
                   
                    SampleApp_QuadrotorMSGCB(MSGpkt);
                   
                    
                  }
                   else if(SampleApp_NwkState == DEV_ROUTER)
                  {
                     if(DeviceID!=6)
                    SampleApp_AnchorMSGCB(MSGpkt);
                     else
                     SampleApp_Tester_calibrationMSGCB(MSGpkt);
                  }
//                  else if((SampleApp_NwkState == DEV_END_DEVICE)&&(DeviceID==6))
//                  {
//                    SampleApp_Tester_calibrationMSGCB(MSGpkt);
//                  }
                    
          break;

        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                 (SampleApp_NwkState == DEV_ROUTER)&&(DeviceID!=6)
             //|| (SampleApp_NwkState == DEV_END_DEVICE)
                  
              )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
               
          }
          else if((SampleApp_NwkState == DEV_END_DEVICE)&&(DeviceID!=6))
          {
              //Start sending quadrotor location after calculation
               osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_LOCATION_EVT,
                              SAMPLEAPP_SEND_PERIODIC_LOCATION_TIMEOUT );
                  
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  // anchor sends period message to help ed to localise itself 
  if ( (events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT)&&(DeviceID!=6) )
  {
    // Send the periodic message
    SampleApp_Router_SendPeriodicMessage(DeviceID);

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x0003)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  
   // Send a location message out (ED)
    if ( (events & SAMPLEAPP_SEND_PERIODIC_LOCATION_EVT)&&(DeviceID!=6) )
  {
    // Send the periodic message
    SampleApp_Quadrotor_SendLocation(DeviceID);

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_LOCATION_EVT,
        (SAMPLEAPP_SEND_PERIODIC_LOCATION_TIMEOUT + (osal_rand() & 0x000E)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_LOCATION_EVT);
  }
  
  
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 && (DeviceID==6))
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    //SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
     
     static int aindex;
     if(aindex!=4)
     {
        SampleApp_Tester_calibrationAMessage( aindex+1 , RssiValue[aindex] );
        aindex++;
     }
     else
     {
       aindex=0;
     }
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    
//    aps_Group_t *grp;
//    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
//    if ( grp )
//    {
//      // Remove from the group
//      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
//    }
//    else
//    {
//      // Add to the flash group
//      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
//    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt ) //接收数据
{
  //uint16 flashTime;
    float avalue=0.0f;
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
   //   printRSSIandLQI(pkt);
    // HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
      break;
//    case SAMPLEAPP_PERIODIC_CLUSTERID:
//      { // unsigned char* rssivalue;
//        //  uint8 rssiE=25;
//         // int abc=3;
//         // rssivalue=&rssiE;
//       // int8 rssivalue;
//        rssivalue=0xff-pkt->rssi;
//        unsigned char rssi_buf[3];
//        rssi_buf[0]='-';
//        rssi_buf[1]=rssivalue/10+0x30;
//        rssi_buf[2]=rssivalue%10+0x30;
//     // HalUARTWrite(0, "Rx:", 3);        //提示信息
//      HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
//  
//                
//    //  HalUARTWrite(0, "\n", 1); 
//      HalUARTWrite(0, "RSSI:", 5);  
//      HalUARTWrite(0,rssi_buf,3);
////     HalUARTWrite(0, "\r\n", 2); 
//////     
////     uint8 lqivalue;
////     lqivalue=pkt->LinkQuality;
////     unsigned char lqi_buf[3];
////      lqi_buf[0]=lqivalue/100+0x30;
////        lqi_buf[1]=(lqivalue%100)/10+0x30;
////        lqi_buf[2]=lqivalue%10+0x30;
////      HalUARTWrite(0, "LQI:", 4);  
////      HalUARTWrite(0,lqi_buf,3);
////     HalUARTWrite(0, "\n", 1);         //回车换行
//      break;
//      }
    
    

    case LZQR_GROUP_CLUSTERID: 
      {
       // HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
        
        
   
        
        
        break;
      }
    case CALBRATION_N_CLUSTERID:
      {
      // HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
        byte alphaindex;
        alphaindex=*(pkt->cmd.Data);
        if(alphaindex=='A')
         {  
              byte index;
              index=*(pkt->cmd.Data+1);
              avalue=10.0f*(float)((int)(*(pkt->cmd.Data+3))-48)+(float)((int)(*(pkt->cmd.Data+4))-48)+0.1f*(float)((int)(*(pkt->cmd.Data+6))-48);
              if(avalue>0)   
              {
                  switch (index)
                       case '1':
                          {
                                  AValue[0]=avalue;
                                  break;
                           }
                       case '2':
                          {
                                  AValue[1]=avalue;
                                  break;
                           }
                       case '3':
                          {
                                  AValue[2]=avalue;
                                  break;
                           }
                       case '4':
                          {
                                  AValue[3]=avalue;
                                  break;
                           }
            
                  break;
              }
           }
      }

  }
}

/*********************************************************************
 * @fn      SampleApp_QuadrotorMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_QuadrotorMSGCB( afIncomingMSGPacket_t *pkt ) //receive data for ed
{
  float avalue=0.0f;
  switch ( pkt->clusterId )
  {
     
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      { 
        
        byte index;
        index=*(pkt->cmd.Data+1);
        rssivalue=(0xff-pkt->rssi);
        
        switch (index)
        {
            case '1':
              {
//                //average filter
//                perRssiBuf1[i1]=rssivalue;
//                
//                if(i1==31)
//                {
//                  i1=0;
//                  /*average filter*/
////                  RssiValue[0]=averagefilter(perRssiBuf1,RSSI_AVG_WINDOW_SIZE);
//                  /*gausian filter*/
//                  RssiValue[0]=gaussianfilter(perRssiBuf1,RSSI_AVG_WINDOW_SIZE);
//
//                }
//                i1++;
//                
                //moving average filter
                moving_average(rssivalue, perRssiBuf1, RSSI_AVG_WINDOW_SIZE-1, imof1, &RssiValue[0]);
              }
            
                  break;
            case '2':
              {
//                perRssiBuf2[i2]=rssivalue;
//                
//                if(i2==31)
//                {
//                  i2=0;
//                  /*average filter*/
////                  RssiValue[1]=averagefilter(perRssiBuf2,RSSI_AVG_WINDOW_SIZE);
//                  /*gausian filter*/
//                  RssiValue[1]=gaussianfilter(perRssiBuf2,RSSI_AVG_WINDOW_SIZE);
//                }
//                i2++;
                //moving average filter
                moving_average(rssivalue, perRssiBuf2, RSSI_AVG_WINDOW_SIZE-1, imof2, &RssiValue[1]);
              }
                  break;
            case '3':
              {
//                perRssiBuf3[i3]=rssivalue;
//                
//                if(i3==31)
//                {
//                  i3=0;
//                   /*average filter*/
////                  RssiValue[2]=averagefilter(perRssiBuf3,RSSI_AVG_WINDOW_SIZE);
//                  /*gausian filter*/
//                  RssiValue[2]=gaussianfilter(perRssiBuf3,RSSI_AVG_WINDOW_SIZE);
//                }
//                i3++;
                //moving average filter
                moving_average(rssivalue, perRssiBuf3, RSSI_AVG_WINDOW_SIZE-1, imof3, &RssiValue[2]);
              }
                  break;
            case '4':
              {
//                perRssiBuf4[i4]=rssivalue;
//                
//                if(i4==31)
//                {
//                  i4=0;
//                   /*average filter*/
////                  RssiValue[3]=averagefilter(perRssiBuf4,RSSI_AVG_WINDOW_SIZE);
//                  /*gausian filter*/
//                  RssiValue[3]=gaussianfilter(perRssiBuf4,RSSI_AVG_WINDOW_SIZE);
//                }
//                i4++;
                //moving average filter
                 moving_average(rssivalue, perRssiBuf4, RSSI_AVG_WINDOW_SIZE-1, imof4, &RssiValue[3]);
              }
                  break;    
            default:
            
                   break;
         }
        break;
      }

    case LZQR_GROUP_CLUSTERID: // position of other quadrotors
          {
            //HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
            
       
            
            
           // break;
          }
    

     case CALBRATION_N_CLUSTERID:
    // {
        HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
        byte alphaindex;
        alphaindex=*(pkt->cmd.Data);
        if(alphaindex=='A')
         {  
              byte index;
              index=*(pkt->cmd.Data+1);
              avalue=10.0f*(float)((int)(*(pkt->cmd.Data+3))-48)+(float)((int)(*(pkt->cmd.Data+4))-48)+0.1f*(float)((int)(*(pkt->cmd.Data+6))-48);
              if(avalue>0)   
              {
                  switch (index)
                       case '1':
                          {
                                  AValue[0]=avalue;
                                  break;
                           }
                       case '2':
                          {
                                  AValue[1]=avalue;
                                  break;
                           }
                       case '3':
                          {
                                  AValue[2]=avalue;
                                  break;
                           }
                       case '4':
                          {
                                  AValue[3]=avalue;
                                  break;
                           }
            
                  break;
              }
         } 
               else if(alphaindex=='N')
             {
                      byte index;
                      index=*(pkt->cmd.Data+1);

                      float nvalue;
              //        float tempn=0.0f;
                      
                      if(((int)(*(pkt->cmd.Data+3))-48)>=0&&((int)(*(pkt->cmd.Data+3))-48)<=9)
                      {  nvalue=(float)((int)(*(pkt->cmd.Data+3))-48)+0.1f*(float)((int)(*(pkt->cmd.Data+5))-48)+0.01f*(float)((int)(*(pkt->cmd.Data+6))-48);
                          
                           switch (index)
                           {  
                                case '1':
                                 {
//                                   pernBuf1[nindex1]=nvalue;
//                                   nindex1++;
//                                   if(nindex1==7)
//                                   {
//                                     nindex1=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf1[i];
//                                     }
//                                      nValue[0]=tempn;
//                                   }
                                   
                                     moving_average_f(nvalue, pernBuf1, 7, nindex1, &nValue[0]);
                                   
                                  
                                   break;
                                 }
                                case '2':
                                 {
//                                   pernBuf2[nindex2]=nvalue;
//                                   nindex2++;
//                                   if(nindex2==7)
//                                   {
//                                     nindex2=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf2[i];
//                                     }
//                                      nValue[1]=tempn;
//                                   }
                                      moving_average_f(nvalue, pernBuf2, 7, nindex2, &nValue[1]);
                                   break;
                                 }
                                 case '3':
                                 {
//                                   pernBuf3[nindex3]=nvalue;
//                                   nindex3++;
//                                   if(nindex3==7)
//                                   {
//                                     nindex3=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf3[i];
//                                     }
//                                      nValue[2]=tempn;
//                                   }
                                   moving_average_f(nvalue, pernBuf3, 7, nindex3, &nValue[2]);
                                   break;
                                 }
                                 case '4':
                                 {
//                                   pernBuf4[nindex4]=nvalue;
//                                   nindex4++;
//                                   if(nindex4==7)
//                                   {
//                                     nindex4=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf4[i];
//                                     }
//                                      nValue[3]=tempn;
//                                   }
                                    moving_average_f(nvalue, pernBuf4, 7, nindex4, &nValue[3]);
                                   break;
                                         }
                                break;
                            }
                          }
                     }
         }
      
 
 
 // }

}

/*********************************************************************
 * @fn      SampleApp_Tester_calibrationMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_Tester_calibrationMSGCB( afIncomingMSGPacket_t *pkt ) //receive data for ed
{

  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      { 
        
       //  int8 rssivalue;
        HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
        byte index;
        index=*(pkt->cmd.Data+1);
        rssivalue=(0xff-pkt->rssi);
        
        switch (index)
        {
            case '1':
              {
//                perRssiBuf1[i1]=rssivalue;
//                
//                if(i1==9)
//                {
//                  i1=0;
//                  /*average filter*/
////                  RssiValue[0]=averagefilter(perRssiBuf1,10);
//                  /*gausian filter*/
//                    RssiValue[0]=gaussianfilter(perRssiBuf1,10);
//                }
//                i1++;
                moving_average(rssivalue, perRssiBuf1, 11, imof1, &RssiValue[0]);
              }
            
                  break;
            case '2':
              {
//                perRssiBuf2[i2]=rssivalue;
//                
//                if(i2==9)
//                {
//                  i2=0;
//                  /*average filter*/
////                  RssiValue[1]=averagefilter(perRssiBuf2,10);
//                  /*gausian filter*/
//                  RssiValue[1]=gaussianfilter(perRssiBuf2,10);
//                }
//                i2++;
                 moving_average(rssivalue, perRssiBuf2, 11, imof2, &RssiValue[1]);
              }
                  break;
            case '3':
              {
//                perRssiBuf3[i3]=rssivalue;
//                
//                if(i3==9)
//                {
//                  i3=0;
//                   /*average filter*/
////                  RssiValue[2]=averagefilter(perRssiBuf3,10);
//                  /*gausian filter*/
//                  RssiValue[2]=gaussianfilter(perRssiBuf3,10);
//                }
//                i3++;
                 moving_average(rssivalue, perRssiBuf3, 11, imof3, &RssiValue[2]);
              }
                  break;
            case '4':
              {
//                perRssiBuf4[i4]=rssivalue;
//                
//                if(i4==9)
//                {
//                  i4=0;
//                   /*average filter*/
////                  RssiValue[3]=averagefilter(perRssiBuf4,10);
//                  /*gausian filter*/
//                    RssiValue[3]=gaussianfilter(perRssiBuf4,10);
//                }
//                i4++;
                 moving_average(rssivalue, perRssiBuf4, 11, imof4, &RssiValue[3]);
              }
                  break;    
            default:
            
                   break;
         }
        break;
      }

     
   
  }

}


/*********************************************************************
 * @fn      SampleApp_AnchorMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_AnchorMSGCB( afIncomingMSGPacket_t *pkt ) //receive data for ed
{
  float avalue=0.0f;
  int switchindex;
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      { 
        
        
       
        
        byte index;
        index=*(pkt->cmd.Data+1);
        rssivalue=(0xff-pkt->rssi);
        
        switch (index)
        {
            case '1':
              {
//                switchindex=1;
//                perRssiBuf1[i1]=rssivalue;
//                
//                if(i1==(RSSI_AVG_WINDOW_SIZE-1))
//                {
//                  i1=0;
//   
//                  /*average filter*/
////                  RssiValue[1]=averagefilter(perRssiBuf2,RSSI_AVG_WINDOW_SIZE);
//                  /*gausian filter*/
//                  RssiValue[switchindex-1]=gaussianfilter(perRssiBuf1,RSSI_AVG_WINDOW_SIZE);
//                 if( (nValue[switchindex-1]=calibrationN(DeviceID,switchindex,RssiValue,nValue,
//                                                         AnchorX,AnchorY,AnchorZ,AValue))<10)
//                 {SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
//                  
//                }
//                i1++;
                
                //moving average filter
                switchindex=1;
                moving_average(rssivalue, perRssiBuf1, RSSI_AVG_WINDOW_SIZE-1, imof1, &RssiValue[switchindex-1]);
                if( (nValue[switchindex-1]=calibrationN(DeviceID,switchindex,RssiValue,nValue,
                                                    AnchorX,AnchorY,AnchorZ,AValue))<10)
                   {SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
              }
            
                  break;
            case '2':
              {
//                switchindex=2;
//                perRssiBuf2[i2]=rssivalue;
//                
//                if(i2==31)
//                {
//                  i2=0;
//
//                  /*average filter*/
//  //                RssiValue[1]=averagefilter(perRssiBuf2,RSSI_AVG_WINDOW_SIZE);
//                  /*gaussian filter*/
//                   RssiValue[switchindex-1]=gaussianfilter(perRssiBuf2,RSSI_AVG_WINDOW_SIZE);
//                   
//                  if(  (nValue[switchindex-1]=calibrationN(DeviceID,switchindex,RssiValue,nValue,AnchorX,AnchorY,AnchorZ,AValue))<10)
//                  { SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
//                }
//                i2++;
                //moving average filter
                switchindex=2;
                moving_average(rssivalue, perRssiBuf2, RSSI_AVG_WINDOW_SIZE-1, imof2, &RssiValue[switchindex-1]);
                if(  (nValue[switchindex-1]=calibrationN(DeviceID,switchindex,RssiValue,
                                                         nValue,AnchorX,AnchorY,AnchorZ,AValue))<10)
               { SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
              }
                  break;
            case '3':
              {
//                switchindex=3;
//                perRssiBuf3[i3]=rssivalue;
//                
//                if(i3==31)
//                {
//                  i3=0;              
//                  /*average filter*/
////                  RssiValue[2]=averagefilter(perRssiBuf3,RSSI_AVG_WINDOW_SIZE);
//                  /*gaussian filter*/
//                   RssiValue[switchindex-1]=gaussianfilter(perRssiBuf3,RSSI_AVG_WINDOW_SIZE);
//                   if((nValue[switchindex-1]=calibrationN(DeviceID,switchindex,RssiValue,nValue,AnchorX,AnchorY,AnchorZ,AValue))<10)
//                   {SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
//                }
//                i3++;
                 //moving average filter
                switchindex=3;
                moving_average(rssivalue, perRssiBuf3, RSSI_AVG_WINDOW_SIZE-1, imof3, &RssiValue[switchindex-1]);
                if(  (nValue[switchindex-1]=calibrationN(DeviceID,switchindex,RssiValue,
                                                         nValue,AnchorX,AnchorY,AnchorZ,AValue))<10)
               { SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
              }
                  break;
            case '4':
              {
//                switchindex=4;
//                perRssiBuf4[i4]=rssivalue;
//                
//                if(i4==31)
//                {
//                  i4=0;
//
//                     /*average filter*/
////                  RssiValue[3]=averagefilter(perRssiBuf4,RSSI_AVG_WINDOW_SIZE);
//                  /*gaussian filter*/
//                   RssiValue[switchindex-1]=gaussianfilter(perRssiBuf4,RSSI_AVG_WINDOW_SIZE);
//                   
//                    if((nValue[switchindex-1]=calibrationN(DeviceID,4,RssiValue,nValue,AnchorX,AnchorY,AnchorZ,AValue))<10)
//                    {SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
//                }
//                i4++;
                 //moving average filter
                switchindex=4;
                moving_average(rssivalue, perRssiBuf4, RSSI_AVG_WINDOW_SIZE-1, imof4, &RssiValue[switchindex-1]);
                if(  (nValue[switchindex-1]=calibrationN(DeviceID,switchindex,RssiValue,
                                                         nValue,AnchorX,AnchorY,AnchorZ,AValue))<10)
               { SampleApp_Router_calibrationNMessage(switchindex,nValue[switchindex-1]);}
              }
                  break;    
            default:
            
                   break;
         }
        break;
      }

           case CALBRATION_N_CLUSTERID:
      {
        HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
        byte alphaindex;
        alphaindex=*(pkt->cmd.Data);
        if(alphaindex=='A')
         {  
              byte index;
              index=*(pkt->cmd.Data+1);
              
              avalue=10.0f*(float)((int)(*(pkt->cmd.Data+3))-48)+(float)((int)(*(pkt->cmd.Data+4))-48)+0.1f*(float)((int)(*(pkt->cmd.Data+6))-48);
              if(avalue>0)   
              {
                  switch (index)
                       case '1':
                          {
                                  AValue[0]=avalue;
                                  break;
                           }
                       case '2':
                          {
                                  AValue[1]=avalue;
                                  break;
                           }
                       case '3':
                          {
                                  AValue[2]=avalue;
                                  break;
                           }
                       case '4':
                          {
                                  AValue[3]=avalue;
                                  break;
                           }
            
                  break;
              }
         } 
               else if(alphaindex=='N')
             {
                      byte index;
                      index=*(pkt->cmd.Data+1);
                      float nvalue;
//                      float tempn=0.0f;
                      
                      if(((int)(*(pkt->cmd.Data+3))-48)>=0&&((int)(*(pkt->cmd.Data+3))-48)<=9)
                      {  
                        
                        nvalue=(float)((int)(*(pkt->cmd.Data+3))-48)+0.1f*(float)((int)(*(pkt->cmd.Data+5))-48)+0.01f*(float)((int)(*(pkt->cmd.Data+6))-48);
                          
                           switch (index)
                           {  
                                case '1':
                                 {
//                                   pernBuf1[nindex1]=nvalue;
//                                   nindex1++;
//                                   if(nindex1==7)
//                                   {
//                                     nindex1=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf1[i];
//                                     }
//                                      nValue[0]=tempn;
//                                   }
                                     moving_average_f(nvalue, pernBuf1, 7, nindex1, &nValue[0]);
                                   
                                  
                                   break;
                                 }
                                case '2':
                                 {
//                                   pernBuf2[nindex2]=nvalue;
//                                   nindex2++;
//                                   if(nindex2==7)
//                                   {
//                                     nindex1=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf2[i];
//                                     }
//                                      nValue[1]=tempn;
//                                   }
                                      moving_average_f(nvalue, pernBuf2, 7, nindex2, &nValue[1]);
                                   break;
                                 }
                                 case '3':
                                 {
//                                   pernBuf3[nindex3]=nvalue;
//                                   nindex3++;
//                                   if(nindex3==7)
//                                   {
//                                     nindex3=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf3[i];
//                                     }
//                                      nValue[2]=tempn;
//                                   }
                                      moving_average_f(nvalue, pernBuf3, 7, nindex3, &nValue[2]);
                                   break;
                                 }
                                 case '4':
                                 {
//                                   pernBuf4[nindex4]=nvalue;
//                                   nindex4++;
//                                   if(nindex4==7)
//                                   {
//                                     nindex4=0;
//                                     for (int i=0; i<8; i++)
//                                     {
//                                        tempn+=pernBuf4[i];
//                                     }
//                                      nValue[3]=tempn;
//                                   }
                                      moving_average_f(nvalue, pernBuf4, 7, nindex4, &nValue[3]);
                                   break;
                                         }
                                break;
                            }
                          }
               }
        break;
         }
  }

}
/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_Router_SendPeriodicMessage( int index )
{
    
  // Router broadcast to everyone
  SampleApp_Router_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Router_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Router_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
  
  uint8 data[2]="A";
  data[1]=0x30+index;
  if ( AF_DataRequest( &SampleApp_Router_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       osal_strlen("A1"),
                       data,
                       &SampleApp_TransID,
                       AF_SKIP_ROUTING,//AF_DISCV_ROUTE,  
                       1//AF_DEFAULT_RADIUS 
                        ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
 * @fn      SampleApp_Router_calibrationNMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_Router_calibrationNMessage( int index , float nvalue )
{
    
  // Router broadcast to everyone
//  SampleApp_Router_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
//  SampleApp_Router_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
//  SampleApp_Router_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
  SampleApp_Router_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrGroup;  //AddrGroup
  SampleApp_Router_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Router_Periodic_DstAddr.addr.shortAddr =  SAMPLEAPP_CalN_GROUP;    //SAMPLEAPP_QUADROTOR_GROUP
 

 
  float tempn=nvalue;  
  if(nvalue>0)
  {
          uint8 data[7]="N";
          data[1]=0x30+index;

          data[2]=':';
          
          uint8 number8;
          number8=(uint8)tempn;
          data[3]=number8+'0';
          data[4]='.';
          tempn=10*(tempn-(float)number8);
          number8=(uint8)tempn;
          data[5]=number8+'0';
          tempn=10*(tempn-(float)number8);
          number8=(uint8)tempn;
          data[6]=number8+'0';
                      
          
          if ( AF_DataRequest( &SampleApp_Router_Periodic_DstAddr, &SampleApp_epDesc,
                               CALBRATION_N_CLUSTERID,
                               7,//osal_strlen("A1")+1,
                               data,
                               &SampleApp_TransID,
                               AF_DISCV_ROUTE,//AF_DISCV_ROUTE,  
                               1//AF_DEFAULT_RADIUS 
                                ) == afStatus_SUCCESS )
          {
          }
          else
          {
            // Error occurred in request to send.
          }
  }
}

/*********************************************************************
 * @fn      SampleApp_Tester_calibrationAMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_Tester_calibrationAMessage( int index , float A )
{
//    SampleApp_Tester_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
//  SampleApp_Tester_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
//  SampleApp_Tester_DstAddr.addr.shortAddr = 0xFFFF;
    
  SampleApp_Tester_DstAddr.addrMode = (afAddrMode_t)AddrGroup;  //AddrGroup
  SampleApp_Tester_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Tester_DstAddr.addr.shortAddr =  SAMPLEAPP_CalN_GROUP;    //SAMPLEAPP_QUADROTOR_GROUP
 


  float tempn=A;  
  if(A>=0)
  {
          uint8 data[7]="A";
          data[1]=0x30+index;

          data[2]=':';
          
          uint8 number8;
          number8=(uint8)(tempn/10);
          data[3]=number8+'0';
          number8=(uint8)(((int)tempn)%10);
          data[4]=number8+'0';
          data[5]='.';
          tempn=10*(tempn-(int)tempn);
          number8=(uint8)tempn;
          data[6]=number8+'0';
               
          
          if ( AF_DataRequest( &SampleApp_Tester_DstAddr, &SampleApp_epDesc,
                               CALBRATION_N_CLUSTERID,
                               7,//osal_strlen("A1")+1,
                               data,
                               &SampleApp_TransID,
                               AF_DISCV_ROUTE,//AF_DISCV_ROUTE,  
                               1//AF_DEFAULT_RADIUS 
                                ) == afStatus_SUCCESS )
          {
          }
          else
          {
            // Error occurred in request to send.
          }
  }
}


/*********************************************************************
 * @fn      SampleApp_Quadrotor_SendLocation
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_Quadrotor_SendLocation( int index )
{
  
  SampleApp_Quadrotor_Location_DstAddr.addrMode = (afAddrMode_t)AddrGroup;  //AddrGroup
  SampleApp_Quadrotor_Location_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Quadrotor_Location_DstAddr.addr.shortAddr = SAMPLEAPP_QUADROTOR_GROUP;    //SAMPLEAPP_QUADROTOR_GROUP
  
  float xposition;
  float yposition;
  float zposition;
  
  zposition=locationZ[index-1];
  // using min-max algorithm
 // SampleApp_MinMax_localization(&xposition, &yposition, zposition, nValue,
   //                           RssiValue, AValue, AnchorX, AnchorY);
  
  // using weighted centroid localization algorithm
  SampleApp_WCL_localization(&xposition, &yposition, nValue,RssiValue,AValue,AnchorX,AnchorY);
  locationX[index-1]=xposition;
  locationY[index-1]=yposition;
  
  

  uint8 data[18]="Q";
  data[1]=0x30+index;
  data[2]='[';
  data[17]=']';
  uint8 number8;
  number8=(uint8)xposition;
  data[3]=number8+'0';
  data[4]='.';
  xposition=10*(xposition-(float)number8);
  number8=(uint8)xposition;
  data[5]=number8+'0';
  xposition=10*(xposition-(float)number8);
  number8=(uint8)xposition+'0';
  data[6]=number8;
  data[7]=',';
  number8=(uint8)yposition;
  data[8]=number8+'0';
  data[9]='.';
  yposition=10*(yposition-(float)number8);
  number8=(uint8)yposition;
  data[10]=number8+'0';
  yposition=10*(yposition-(float)number8);
  number8=(uint8)yposition+'0';
  data[11]=number8;
  data[12]=',';
  number8=(uint8)zposition;
  data[13]=number8+'0';
  data[14]='.';
  zposition=10*(zposition-(float)number8);
  number8=(uint8)zposition;
  data[15]=number8+'0';
  zposition=10*(zposition-(float)number8);
  number8=(uint8)zposition+'0';
  data[16]=number8;
 // data[18]='\0';
 
  if ( AF_DataRequest( &SampleApp_Quadrotor_Location_DstAddr, &SampleApp_epDesc,
                         LZQR_GROUP_CLUSTERID,
                         18,//osal_strlen("Q1")+1,
                         data,
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,//AF_SKIP_ROUTING,  
                         AF_DEFAULT_RADIUS// 1
                           ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }


}


///*********************************************************************
// * @fn      SampleApp_MinMax_localization
// *
// * @brief   calculate the position of a quadrotor
// *
// * @param   rssi_i readings from different anchors, the result will be stored in xposition and yposition
// *
// * @return  none
// */
//void SampleApp_MinMax_localization(float* xposition, float* yposition, float zposition)
//{
//    //initial the parameter 
//    float x_left=-10;
//    float x_right=10;
//    float y_bottom=-10;
//    float y_top=10;
//    float gainconst=1.16f;
//    float naverage=0.0f;
//    for(int i=0;i<4;i++)
//    {
//      naverage+=nValue[i];
//    }
//    //calculate the distances based on the calibrated parameter n and A
//    for (int i=0; i<4; i++)
//    {
//      distance[i]=pow(10,((RssiValue[i]/gainconst-AValue[i])/(10*naverage)));
//      distance[i]=pow((distance[i]*distance[i]-zposition*zposition),0.5);
//    }
//    
//    
//    for(int i=0; i<4; i++)
//    {
//        //max of  
//       if((AnchorX[i]-distance[i])>x_left)
//       {x_left=(AnchorX[i]+distance[i]);}
//       
//       if((AnchorX[i]+distance[i])<x_right)
//       {x_right=(AnchorX[i]+distance[i]);}
//       
//       if((AnchorY[i]-distance[i])>y_bottom)
//       {y_bottom=(AnchorY[i]+distance[i]);}
//       
//       if((AnchorY[i]+distance[i])<y_top)
//       {y_top=(AnchorY[i]+distance[i]);}
//    }
//    *xposition=(x_left+x_right)/2;
//    *yposition=(y_bottom+y_top)/2;
//    
//}
//
//
///*********************************************************************
// * @fn      SampleApp_WCL_localization
// *
// * @brief   calculate the position of a quadrotor
// *
// * @param   rssi_i readings from different anchors, the result will be stored in xposition and yposition
// *
// * @return  none
// */
//void SampleApp_WCL_localization(float* xposition, float* yposition)
//{
//    //initial the parameter 
//    int g=4;   // const. parameter for WCL
//    float w[4]; //weight
//    double sumw=0.0; // sum of weights
//    float naverage=0.0f;
//    for(int i=0;i<4;i++)
//    {
//      naverage+=nValue[i];
//    }
//    
//    //calculate the distances based on the calibrated parameter n and A
//    for (int i=0; i<4; i++)
//    {
//      distance[i]=pow(10,((RssiValue[i]/1.16f-AValue[i])/(10*naverage)));
//      w[i]=pow(distance[i],g);
//      sumw+=w[i];
//    }
//
//    
//     
//    *xposition=AnchorX[0]*w[0]/sumw+AnchorX[1]*w[1]/sumw+AnchorX[2]*w[2]/sumw+AnchorX[3]*w[3]/sumw;
//    *yposition=AnchorY[0]*w[0]/sumw+AnchorY[1]*w[1]/sumw+AnchorY[2]*w[2]/sumw+AnchorY[3]*w[3]/sumw;
//    
//}
//
///*********************************************************************
// * @fn      gaussianfilter
// *
// * @brief   filter for received rssi data
// *
// * @param   rssi rssi array, the result will be stored in xposition and yposition
// *
// * @return  none
// */
//float gaussianfilter(int8* rssi, int length)
//{
//  float mu=0.0;
//  float variance=0.0;
//  float newmean=0.0;
//  
//  for (int i=0; i<length; i++)
//  {
//    mu+=*(rssi+i);
//  }
//  mu=mu/length;
//  
//  for (int i=0; i<length; i++)
//  {
//    float temp=*(rssi+i);
//    variance+=(float)pow((mu-temp),2);
//  }
//  variance=(float)(variance/length);
//  
//  float intervalmin=mu-variance;
//  float intervalmax=mu+variance;
//  
//  int index=0;
//  for (int i=0; i<length; i++)
//  {
//    float temp=*(rssi+i);
//    if(temp<=intervalmax&&temp>=intervalmin)
//       {
//         index++;
//         newmean+=temp;
//       }
//  }
//  
//       if(index!=0)
//       {
//         newmean=newmean/index;
//       }
//       
//  return newmean;
//}
//
///*********************************************************************
// * @fn      averagefilter
// *
// * @brief   filter for received rssi data
// *
// * @param   rssi rssi array
// *
// * @return  none
// */
//float averagefilter(int8* rssi, int length)
//{
//  int32 mu=0;
// 
//  float newmean=0.0;
//  
//  for(int i=0; i<length; i++)
//   {
//                  mu+=(int32)*(rssi+i);
//   }
//                 newmean=((float)mu)/((float)length);
//             
//       
//  return newmean;
//}
//
///*********************************************************************
// * @fn      calibrationN
// *
// * @brief   calibration of n
// *
// * @param   rssi rssi array
// *
// * @return  none
// */
//float calibrationN(int localID, int index, float rssivalue_f, float AnchorX[4], float AnchorY[4], float AnchorZ[4], float A[4])
//{
//  float n;
//  if(localID!=index)
//  {
//      
//      float d=0.0;
//      d=(AnchorX[localID-1]-AnchorX[index-1])*(AnchorX[localID-1]-AnchorX[index-1])+
//        (AnchorY[localID-1]-AnchorY[index-1])*(AnchorY[localID-1]-AnchorY[index-1])+
//          (AnchorZ[localID-1]-AnchorZ[index-1])*(AnchorZ[localID-1]-AnchorZ[index-1]);
//     // d+=pow((AnchorY[localID-1]-AnchorY[index-1]),2);
//     // d+=pow((AnchorZ[localID-1]-AnchorZ[index-1]),2);
//      d=pow(d,0.5);
//      if(d!=1)
//        n=(rssivalue_f-A[index-1])/10/(log10(d));
//      else
//        n=nValue[localID-1];
//  }
//  else
//  {
//    n=nValue[localID-1];
//  }
//  return n;
//}
/*********************************************************************
*********************************************************************/
