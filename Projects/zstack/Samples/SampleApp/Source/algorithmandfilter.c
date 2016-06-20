/**************************************************************************************************
  Filename:       algorithmandfilter.c
  Revised:        $Date: 2016-01-09  $
  Revision:       $Revision: 1 $

  Description:    algorithmandfilter.


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "algorithmandfilter.h"

void printRSSIandLQI(afIncomingMSGPacket_t *pkt, int specindex)
{
  
         if(!specindex)
        { 
          int8 rssivalue;
        rssivalue=0xff-pkt->rssi;
        unsigned char rssi_buf[3];
        rssi_buf[0]='-';
        rssi_buf[1]=rssivalue/10+0x30;
        rssi_buf[2]=rssivalue%10+0x30;
         HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //print out the data through uart
                
    //  HalUARTWrite(0, "\n", 1); 
      HalUARTWrite(0, "RSSI:", 5);  
      HalUARTWrite(0,rssi_buf,3);
//     HalUARTWrite(0, "\r\n", 2); 
//     
     uint8 lqivalue;
     lqivalue=pkt->LinkQuality;
     unsigned char lqi_buf[3];
      lqi_buf[0]=lqivalue/100+0x30;
        lqi_buf[1]=(lqivalue%100)/10+0x30;
        lqi_buf[2]=lqivalue%10+0x30;
      HalUARTWrite(0, "LQI:", 4);  
      HalUARTWrite(0,lqi_buf,3);
     HalUARTWrite(0, "\n", 1);         //create a new line
        }
        else if (specindex==(int)*(pkt->cmd.Data+1)-48)
        {
                    int8 rssivalue;
        rssivalue=0xff-pkt->rssi;
        unsigned char rssi_buf[3];
        rssi_buf[0]='-';
        rssi_buf[1]=rssivalue/10+0x30;
        rssi_buf[2]=rssivalue%10+0x30;
         HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //print out the data through uart
                
    //  HalUARTWrite(0, "\n", 1); 
      HalUARTWrite(0, "RSSI:", 5);  
      HalUARTWrite(0,rssi_buf,3);
//     HalUARTWrite(0, "\r\n", 2); 
//     
     uint8 lqivalue;
     lqivalue=pkt->LinkQuality;
     unsigned char lqi_buf[3];
      lqi_buf[0]=lqivalue/100+0x30;
        lqi_buf[1]=(lqivalue%100)/10+0x30;
        lqi_buf[2]=lqivalue%10+0x30;
      HalUARTWrite(0, "LQI:", 4);  
      HalUARTWrite(0,lqi_buf,3);
     HalUARTWrite(0, "\n", 1);         //create a new line
        }
      
                        
                  
//                   unsigned char rssi_buf[3];
//                     rssi_buf[0]='-';
//                    rssi_buf[1]=(int32)sumRssi1/10+0x30;
//                    rssi_buf[2]=(int32)sumRssi1%10+0x30;
//                   HalUARTWrite(0,rssi_buf,3);
}



/*********************************************************************
 * @fn      SampleApp_MinMax_localization
 *
 * @brief   calculate the position of a quadrotor
 *
 * @param   rssi_i readings from different anchors, the result will be stored in xposition and yposition
 *
 * @return  none
 */
void SampleApp_MinMax_localization(float* xposition, float* yposition, float zposition, float nValue[4], float RssiValue[4],
                                   float AValue[4], float AnchorX[4], float AnchorY[4], float OffsetA[4])
{
    //initial the parameter 
    float x_left=-10;
    float x_right=10;
    float y_bottom=-10;
    float y_top=10;
    float gainconst=1.0f;
    //float naverage=0.0f;
    float distance[4]={0.0f};
//    for(int i=0;i<4;i++)
//    {
//      naverage+=nValue[i];
//    }
//    naverage=naverage/4.0f;
    //calculate the distances based on the calibrated parameter n and A
    for (int i=0; i<4; i++)
    {
      distance[i]=pow(10,((RssiValue[i]/gainconst-AValue[i]+OffsetA[i])/(10*nValue[i])));
      distance[i]=pow((distance[i]*distance[i]-zposition*zposition),0.5);
    }
    
    
    for(int i=0; i<4; i++)
    {
      
        //max of  
       if((AnchorX[i]-distance[i])>x_left)
       {x_left=(AnchorX[i]-distance[i]);}
       
       if((AnchorX[i]+distance[i])<x_right)
       {x_right=(AnchorX[i]+distance[i]);}
       
       if((AnchorY[i]-distance[i])>y_bottom)
       {y_bottom=(AnchorY[i]-distance[i]);}
       
       if((AnchorY[i]+distance[i])<y_top)
       {y_top=(AnchorY[i]+distance[i]);}
    }
    *xposition=(x_left+x_right)/2;
    *yposition=(y_bottom+y_top)/2;
    
}


/*********************************************************************
 * @fn      SampleApp_WCL_localization
 *
 * @brief   calculate the position of a quadrotor
 *
 * @param   rssi_i readings from different anchors, the result will be stored in xposition and yposition
 *
 * @return  none
 */
void SampleApp_WCL_localization(float* xposition, float* yposition, float nValue[4], float RssiValue[4],
                                   float AValue[4], float AnchorX[4], float AnchorY[4], float OffsetA[4])
{
    //initial the parameter 
    float g=2.78;   // const. parameter for WCL  2.78
    float w[4]; //weight
    float sumw=0.0f; // sum of weights
    float naverage=0.0f;
    float distance[4]={0.0f, 0.0f, 0.0f, 0.0f};
    float rssigain=1.0f;
    
      //average n from 4 nodes
//    for(int i=0;i<4;i++)
//    {
//      naverage+=nValue[i];
//    }
//    
//  
//    naverage=naverage/4.0f;
    
    
    //calculate the distances based on the calibrated parameter n and A
    for (int i=0; i<4; i++)
    {
      distance[i]=pow(10,((RssiValue[i]/rssigain-AValue[i]+OffsetA[i])/(10*nValue[i])));
      w[i]=pow(distance[i],-g);
      sumw+=w[i];
    }

    
     
    *xposition=AnchorX[0]*w[0]/sumw+AnchorX[1]*w[1]/sumw+AnchorX[2]*w[2]/sumw+AnchorX[3]*w[3]/sumw;
    *yposition=AnchorY[0]*w[0]/sumw+AnchorY[1]*w[1]/sumw+AnchorY[2]*w[2]/sumw+AnchorY[3]*w[3]/sumw;
    
}

/*********************************************************************
 * @fn      moving_average
 *
 * @brief   filter for moving average
 *
 * @param   
 *
 * @return  none
 */
void moving_average(int8 in, int8 moavarray[], int length, int fil_cnt[2], float *out)
{
  if(++fil_cnt[0]>length)
  {
    fil_cnt[0]=0;
    fil_cnt[1]=1;
  }
  else  
  {
    fil_cnt[1]=(fil_cnt[0]==length)?0:(fil_cnt[0]+1);
  }
  moavarray[fil_cnt[0]]=in;
  *out+=(in-(moavarray[fil_cnt[1]]))/(float)(length);
  
  
}


/*********************************************************************
 * @fn      moving_average
 *
 * @brief   filter for moving average
 *
 * @param   
 *
 * @return  none
 */
void moving_average_f(float in, float moavarray[], int length, int fil_cnt[2], float *out)
{
  if(++fil_cnt[0]>length)
  {
    fil_cnt[0]=0;
    fil_cnt[1]=1;
  }
  else  
  {
    fil_cnt[1]=(fil_cnt[0]==length)?0:(fil_cnt[0]+1);
  }
  moavarray[fil_cnt[0]]=in;
  *out+=(in-(moavarray[fil_cnt[1]]))/(float)(length);
  
  
}



/*********************************************************************
 * @fn      gaussianfilter
 *
 * @brief   filter for received rssi data
 *
 * @param   rssi rssi array, the result will be stored in xposition and yposition
 *
 * @return  none
 */
float gaussianfilter(int8* rssi, int length)
{
  float mu=0.0;
  float variance=0.0;
  float newmean=0.0;
  
  for (int i=0; i<length; i++)
  {
    mu+=*(rssi+i);
  }
  mu=mu/length;
  
  for (int i=0; i<length; i++)
  {
    float temp=*(rssi+i);
    variance+=(float)pow((mu-temp),2);
  }
  variance=(float)(variance/length);
  
  float intervalmin=mu-variance;
  float intervalmax=mu+variance;
  
  int index=0;
  for (int i=0; i<length; i++)
  {
    float temp=*(rssi+i);
    if(temp<=intervalmax&&temp>=intervalmin)
       {
         index++;
         newmean+=temp;
       }
  }
  
       if(index!=0)
       {
         newmean=newmean/index;
       }
       
  return newmean;
}

/*********************************************************************
 * @fn      averagefilter
 *
 * @brief   filter for received rssi data
 *
 * @param   rssi rssi array
 *
 * @return  none
 */
float averagefilter(int8* rssi, int length)
{
  int32 mu=0;
 
  float newmean=0.0;
  
  for(int i=0; i<length; i++)
   {
                  mu+=(int32)*(rssi+i);
   }
                 newmean=((float)mu)/((float)length);
             
       
  return newmean;
}

/*********************************************************************
 * @fn      calibrationN
 *
 * @brief   calibration of n
 *
 * @param   rssi rssi array
 *
 * @return  none
 */
float calibrationN(int localID, int index, float rssivalue_f[4], float nValue[4], 
                   float AnchorX[4], float AnchorY[4], float AnchorZ[4], float A[4])
{
  float n;
  float offset=4.0f;
  float tempA[4]={48.0f, 48.0f, 55.0f, 56.0f} ;
  if(localID!=index)
  {
      
      float d=0.0f;
      d=(AnchorX[localID-1]-AnchorX[index-1])*(AnchorX[localID-1]-AnchorX[index-1])+
        (AnchorY[localID-1]-AnchorY[index-1])*(AnchorY[localID-1]-AnchorY[index-1])+
          (AnchorZ[localID-1]-AnchorZ[index-1])*(AnchorZ[localID-1]-AnchorZ[index-1]);
     // d+=pow((AnchorY[localID-1]-AnchorY[index-1]),2);
     // d+=pow((AnchorZ[localID-1]-AnchorZ[index-1]),2);
      d=pow(d,0.5);   // distance between two anchors 
     
      if(d!=1)
      //  n=(rssivalue_f[index-1]-A[index-1]+offset)/10.0f/(log10(d));
         n=(rssivalue_f[index-1]-tempA[index-1]+offset)/10.0f/(log10(d));
      else
        n=nValue[localID-1];
  }
  else
  {
    n=nValue[localID-1];
  }
  return n;
}

/*********************************************************************
 * @fn      calibrationA
 *
 * @brief   calibration of A from anchors
 *
 * @param   rssi rssi array
 *
 * @return  none
 */
float calibrationA(int localID, int index, float rssivalue_f[4], float nValue[4], 
                   float AnchorX[4], float AnchorY[4], float AnchorZ[4], float AValue[4])
{
  float Aresult;
  float offset=2.0f;
  if(localID!=index)
  {
      
      float d=0.0f;
      d=(AnchorX[localID-1]-AnchorX[index-1])*(AnchorX[localID-1]-AnchorX[index-1])+
        (AnchorY[localID-1]-AnchorY[index-1])*(AnchorY[localID-1]-AnchorY[index-1])+
          (AnchorZ[localID-1]-AnchorZ[index-1])*(AnchorZ[localID-1]-AnchorZ[index-1]);
     
     
      d=pow(d,0.5);   // distance between two anchors 
     
      if(d!=1)
        Aresult=rssivalue_f[index-1]-10*nValue[index-1]*log10(d)+offset;
      else
        Aresult=AValue[localID-1];
  }
  else
  {
    Aresult=AValue[localID-1];
  }
  return Aresult;
}


/*********************************************************************
*********************************************************************/