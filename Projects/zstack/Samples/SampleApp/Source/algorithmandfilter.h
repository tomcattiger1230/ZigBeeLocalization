/**************************************************************************************************
  Filename:       algorithmandfilter.h
  Revised:        $Date: 2016-01-09 $
  Revision:       $Revision: 1 $

  Description:   algorithm and filter function for localization 
**************************************************************************************************/

#ifndef ALGORITHMANDFILTER_H
#define ALGORITHMANDFILTER_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "math.h"
#include "stdlib.h"
#include "AF.h"
#include "hal_uart.h"
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define SAMPLEAPP_ENDPOINT           20

#define SAMPLEAPP_PROFID             0x0F08
#define SAMPLEAPP_DEVICEID           0x0001
#define SAMPLEAPP_DEVICE_VERSION     0
#define SAMPLEAPP_FLAGS              0

#define SAMPLEAPP_MAX_CLUSTERS       3
#define SAMPLEAPP_PERIODIC_CLUSTERID 1
#define CALBRATION_N_CLUSTERID       2
#define LZQR_GROUP_CLUSTERID         3  //#define SAMPLEAPP_COM_CLUSTERID      3
   

//#define RSSI_AVG_WINDOW_SIZE     11  //size of RSSI moving average


// Send Message Timeout
//#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT   30     // Every 3 seconds ==3000 now is 50ms
//#define SAMPLEAPP_SEND_PERIODIC_LOCATION_TIMEOUT 300

// Application Events (OSAL) - These are bit weighted definitions.
//#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT       0x0001
//#define SAMPLEAPP_SEND_PERIODIC_LOCATION_EVT  0x0002
  
// Group ID for Flash Command
//#define SAMPLEAPP_CalN_GROUP                 0x0001
//#define SAMPLEAPP_QUADROTOR_GROUP             0x0002
  
// Flash Command Duration - in milliseconds
//#define SAMPLEAPP_FLASH_DURATION              1000

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void SampleApp_Init( uint8 task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 SampleApp_ProcessEvent( uint8 task_id, uint16 events );

//print out rssi and lqi
void printRSSIandLQI(afIncomingMSGPacket_t *pkt, int specindex);
//localization algorithms
void SampleApp_MinMax_localization(float* xposition, float* yposition, float zposition, float nValue[4], float RssiValue[4],
                                   float AValue[4], float AnchorX[4], float AnchorY[4],float OffsetA[4]);
void SampleApp_WCL_localization(float* xposition, float* yposition, float nValue[4], float RssiValue[4],
                                   float AValue[4], float AnchorX[4], float AnchorY[4],float OffsetA[4]);

//filter
//moving average filter for rssi
void moving_average(int8 in, int8 moavarray[], int length, int fil_cnt[2], float *out);
void moving_average_f(float in, float moavarray[], int length, int fil_cnt[2], float *out);
//gaussian filter
float gaussianfilter(int8* rssi, int length);
//average filter
float averagefilter(int8* rssi, int length);
//calibration n-value from router
float calibrationN(int localID, int index, float rssivalue_f[4], float nValue[4],
                   float AnchorX[4], float AnchorY[4], float AnchorZ[4], float A[4]);
float calibrationA(int localID, int index, float rssivalue_f[4], float AValue[4], 
                   float AnchorX[4], float AnchorY[4], float AnchorZ[4], float A[4]);


/*********************************************************************
*********************************************************************/



#ifdef __cplusplus
}
#endif

#endif /* ALGORITHMANDFILTER_H */
