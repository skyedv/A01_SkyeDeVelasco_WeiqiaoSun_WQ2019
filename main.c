//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution. 
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
//
//*****************************************************************************


//*****************************************************************************
//
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_uart.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "timer.h"
#include "hw_apps_rcm.h"
#include "gpio.h"

//Common interface includes
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "timer_if.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"

//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"
#define TIMER_FREQ      80000000



#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "a26ypaoxj1nj7v-ats.iot.us-west-2.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                1    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2019  /* Current year */
#define HOUR                19    /* Time - hours */
#define MINUTE              05    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/CC3200_Thing/shadow HTTP/1.1\n\r"
#define GETHEADER "GET /things/CC3200_Thing/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: a3ix757rgh8hoa-ats.iot.us-west-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \""
#define DATA2 "\"\r\n}}}\r\n\r\n"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
//
//IR input variables
volatile unsigned long Input_intcount;
volatile unsigned char Input_intflag;
static unsigned long time1;
static unsigned long time2;
static unsigned long time3;
static unsigned long time4;
static unsigned long tdiff;
static unsigned long tdiff2;
int value=0;
volatile unsigned long count=0;
int i=0;
int but=12;
int x=0;
int y=0;
int letter=0;
static unsigned long key[];
char msgtx[8];
char msgrx[8];
int index=0;
long RetVal;
int flag;
char msg[200];
static unsigned long val;
static unsigned long ZERO[]={0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1};
static unsigned long ONE[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1};
static unsigned long TWO[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,0,1,0,0,0,0,0,0,1,0,1,1,1,1,1,1};
static unsigned long THREE[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1};
static unsigned long FOUR[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,0,0,1,0,0,0,0,0,1,1,0,1,1,1,1,1};
static unsigned long FIVE[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,1,0,1,0,0,0,0,0,0,1,0,1,1,1,1,1};
static unsigned long SIX[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,1,1};
static unsigned long SEVEN[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1};
static unsigned long EIGHT[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,0,0,0,1,0,0,0,0,1,1,1,0,1,1,1,1};
static unsigned long NINE[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,0,0,0,1,1,0,1,1,1,1};
static unsigned long MUTE[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,0,0,0,0,1,0,0,0,1,1,1,1,0,1,1,1};
static unsigned long LAST[]= {0,0,0,0,0,0,1,0,1,1,1,1,1,1,0,1,1,1,1,0,1,0,0,0,0,0,0,1,0,1,1,1};
//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);
static int http_get(int);
//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************


static void GPIOA0IntHandler(void) { // Input handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (GPIOA0_BASE, true);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);        // clear interrupts on GPIOA0
    value = MAP_GPIOPinRead(GPIOA0_BASE,0x10)   ;
    //Report("value =  %d\r\n",value);
    if (value > 0){
        time1 = count;
        //Report("time1 =  %d\r\n",time1);
    }
    else if (value ==0){
        time2 = count;
        tdiff = (time2 - time1);
        //Report("tdiff =  %d\r\n",tdiff);
        Input_intflag=1;

    }
    if (Input_intflag ==1){
                       Input_intflag = 0;


                       if (tdiff <=110){
                           val =0;
                           key[i] = val;
                            i++;
                       }
                       else if(tdiff>110){
                            val=1;
                            key[i] = val;
                            i++;
                       }
                       if (tdiff >200){

                            i = 0;
                       }

                       if (i>=32){

                            //Report("key= %c",key);
                            i = 0;
                            //for(j=0;j<32;j++){
                              //     Report("%d",key[j]);


                            //}
                           // Report("\n%d",ONE);

                            time4 =count;
                            tdiff2 = time4-time3;
                            //Report("%d\n\r",tdiff2);
                            if(strcomp(key,ZERO)==0){
                                //Report("ZERO\n\r");
                                if (x>=128){
                                    y= y + 8;
                                    x= 0;
                                }
                                if (y>=64){
                                    y = 0;
                                }
                                drawChar(x, y, 32, BLACK, WHITE, 1);    //space
                                msgtx[index]= ' ';
                                index++;
                                x = x+8;
                                but = 0;
                            }
                            else if(strcomp(key,ONE)==0){
                                //Report("One\n\r");

                                but = 1;
                            }
                            else if(strcomp(key,TWO)==0){
                                //Report("TWO\n\r");
                                if((but==2)&&((tdiff2)<= 50000)){

                                    x= x-8;
                                    index--;
                                   if(letter==65){
                                             drawChar(x, y, 66, BLACK, WHITE, 1); //B
                                             msgtx[index]='b';
                                             letter = 66;
                                   }
                                   else if(letter ==66)    {
                                             drawChar(x, y, 67, BLACK, WHITE, 1); //C
                                             msgtx[index]='c';
                                             letter = 67;
                                   }
                                   else if(letter== 67){
                                             drawChar(x, y, 65, BLACK, WHITE, 1);   //A
                                             msgtx[index]='a';
                                             letter = 65;
                                   }
                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 65, BLACK, WHITE, 1);
                                    msgtx[index]='a';
                                    letter = 65;
                                }

                                x = x+8;
                                index++;
                                but = 2;
                            }
                            else if(strcomp(key,THREE)==0){
                                //Report("THREE\n\r");
                                if((but==3)&&((tdiff2)<= 50000)){

                                    x= x-8;
                                    index--;
                                   if(letter==68){
                                             drawChar(x, y, 69, BLACK, WHITE, 1); //E
                                             msgtx[index]='e';
                                             letter = 69;
                                   }
                                   else if(letter ==69)    {
                                             drawChar(x, y, 70, BLACK, WHITE, 1); //F
                                             msgtx[index]='f';
                                             letter = 70;
                                   }
                                   else if(letter== 70){
                                             drawChar(x, y, 68, BLACK, WHITE, 1);   //D
                                             msgtx[index]='d';
                                             letter = 68;
                                   }
                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 68, BLACK, WHITE, 1);    //D
                                    msgtx[index]='d';
                                    letter = 68;
                                }
                                index++;
                                x = x+8;
                                but = 3;
                            }
                            else if(strcomp(key,FOUR)==0){
                                //Report("FOUR\n\r");
                                if((but==4)&&((tdiff2)<= 50000)){

                                    x= x-8;
                                    index--;
                                   if(letter==71){
                                             drawChar(x, y, 72, BLACK, WHITE, 1); //H
                                             msgtx[index]='h';
                                             letter = 72;
                                   }
                                   else if(letter ==72)    {
                                             drawChar(x, y, 73, BLACK, WHITE, 1); //I
                                             msgtx[index]='i';
                                             letter = 73;
                                   }
                                   else if(letter== 73){
                                             drawChar(x, y, 71, BLACK, WHITE, 1);   //G
                                             msgtx[index]='g';
                                             letter = 71;
                                   }
                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 71, BLACK, WHITE, 1);        //G
                                    msgtx[index]='g';
                                    letter = 71;
                                }

                                x = x+8;
                                index++;
                                but = 4;
                            }
                            else if(strcomp(key,FIVE)==0){
                                //Report("FIVE\n\r");
                                if((but==5)&&((tdiff2)<= 50000)){
                                    index--;
                                    x= x-8;
                                   if(letter==74){
                                             drawChar(x, y, 75, BLACK, WHITE, 1); //K
                                             msgtx[index]='k';
                                             letter = 75;
                                   }
                                   else if(letter ==75)    {
                                             drawChar(x, y, 76, BLACK, WHITE, 1); //L
                                             msgtx[index]='l';
                                             letter = 76;
                                   }
                                   else if(letter== 76){
                                             drawChar(x, y, 74, BLACK, WHITE, 1);   //J
                                             msgtx[index]='j';
                                             letter =74;
                                   }
                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 74, BLACK, WHITE, 1);
                                    msgtx[index]='j';
                                    letter = 74;
                                }
                                index++;
                                x = x+8;
                                but = 5;
                            }
                            else if(strcomp(key,SIX)==0){
                                //Report("SIX\n\r");
                                if((but==6)&&((tdiff2)<= 50000)){
                                    index--;
                                    x= x-8;
                                   if(letter==77){
                                             drawChar(x, y, 78, BLACK, WHITE, 1); //N
                                             msgtx[index]='n';
                                             letter = 78;
                                   }
                                   else if(letter ==78)    {
                                             drawChar(x, y, 79, BLACK, WHITE, 1); //O
                                             msgtx[index]='o';
                                             letter = 79;
                                   }
                                   else if(letter== 79){
                                             drawChar(x, y, 77, BLACK, WHITE, 1);   //M
                                             msgtx[index]='m';
                                             letter = 77;
                                   }
                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 77, BLACK, WHITE, 1);//M
                                    msgtx[index]='m';
                                    letter = 77;
                                }
                                index++;
                                x = x+8;
                                but = 6;
                            }
                            else if(strcomp(key,SEVEN)==0){
                                //Report("SEVEN\n\r");
                                if((but==7)&&((tdiff2)<= 50000)){
                                    index--;
                                    x= x-8;
                                   if(letter==80){
                                             drawChar(x, y, 81, BLACK, WHITE, 1); //Q
                                             msgtx[index]='q';
                                             letter = 81;
                                   }
                                   else if(letter ==81)    {
                                             drawChar(x, y, 82, BLACK, WHITE, 1); //R
                                             msgtx[index]='r';
                                             letter = 82;
                                   }
                                   else if(letter== 82){
                                             drawChar(x, y, 83, BLACK, WHITE, 1);   //S
                                             msgtx[index]='s';
                                             letter = 83;
                                   }
                                   else if(letter== 83){
                                             drawChar(x, y, 80, BLACK, WHITE, 1);   //P
                                             msgtx[index]='p';
                                             letter = 80;
                                   }
                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 80, BLACK, WHITE, 1);//P
                                    msgtx[index]='p';
                                    letter = 80;
                                }
                                index++;
                                x = x+8;
                                but = 7;
                            }
                            else if(strcomp(key,EIGHT)==0){
                                //Report("EIGHT\n\r");
                                if((but==8)&&((tdiff2)<= 50000)){
                                    index--;
                                    x= x-8;
                                   if(letter==84){
                                             drawChar(x, y, 85, BLACK, WHITE, 1); //U
                                             msgtx[index]='u';
                                             letter = 85;
                                   }
                                   else if(letter ==85)    {
                                             drawChar(x, y, 86, BLACK, WHITE, 1); //V
                                             msgtx[index]='v';
                                             letter = 86;
                                   }
                                   else if(letter== 86){
                                             drawChar(x, y, 84, BLACK, WHITE, 1);   //T
                                             msgtx[index]='t';
                                             letter = 84;
                                   }

                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 84, BLACK, WHITE, 1);//T
                                    msgtx[index]='t';
                                    letter = 84;
                                }
                                index++;
                                x = x+8;
                                but = 8;
                            }
                            else if(strcomp(key,NINE)==0){
                                //Report("NINE\n\r");
                                if((but==9)&&((tdiff2)<= 50000)){
                                    index--;
                                    x= x-8;
                                   if(letter==87){
                                             drawChar(x, y, 88, BLACK, WHITE, 1); //X
                                             msgtx[index]='x';
                                             letter = 88;
                                   }
                                   else if(letter ==88)    {
                                             drawChar(x, y, 89, BLACK, WHITE, 1); //Y
                                             msgtx[index]='y';
                                             letter = 89;
                                   }
                                   else if(letter== 89){
                                             drawChar(x, y,90, BLACK, WHITE, 1);   //Z
                                             msgtx[index]='z';
                                             letter = 90;
                                   }
                                   else if(letter== 90){
                                             drawChar(x, y, 87, BLACK, WHITE, 1);   //W
                                             msgtx[index]='w';
                                             letter = 87;
                                   }
                                }
                                else    {
                                    if (x>=128){
                                        y= y + 8;
                                        x= 0;
                                    }
                                    if (y>=64){
                                        y = 0;
                                    }
                                    drawChar(x, y, 87, BLACK, WHITE, 1);    //W
                                    msgtx[index]='w';
                                    letter = 87;
                                }
                                x = x+8;
                                index++;
                                but = 9;
                            }
                            else if(strcomp(key,MUTE)==0){
                                //Report("MUTE\n\r");
                                x = x-8;
                                index--;
                                if (index<0){
                                    index=0;
                                }
                                drawChar(x, y, 32, BLACK, WHITE, 1);    //space
                                msgtx[index]=' ';
                                but = 10;
                            }
                            else if(strcomp(key,LAST)==0){  // send
                                //Report("LAST\n\r");


                                msg[0] = msgtx[0];

                                msg[8] = '\0';

                                flag = 1;
                                //Enables UART interrupts
                                index=0;

                                memset(msgtx, ' ', 8);
                                but = 11;
                            }
                            //Report("msg=%c",msgtx[index-1]);
                            //Report("index=%d",index);
                            time3 = time4;
                       }


    }
}

void
TimerBaseIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);

    count ++;

}


int
strcomp( unsigned long t1[],  unsigned long t2[])
{
    int d;
    int j;
    d =0;
    for(j=0;j<32;j++){
        if (t1[j]==t2[j]){
             d = d+0;
        }
        else if (t1[j]!=t2[j]){
        d ++;
        }
    }
    return d;
}


void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End breadcrumb: s18_df
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}




long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP;
//    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 // does not work (-340, handshake fails)
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket 
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }



/////////////////////////////////
// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
/////////////////////////////////


    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
        UART_PRINT("Device has connected to the website (UNVERIFIED):");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    GPIO_IF_LedConfigure(LED1|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    unsigned long ulStatus;
    long lRetVal = -1;
    long pin = 0;
    long pin0 = 0;
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("My terminal works!\n\r");

    //Connect the CC3200 to the local access point




    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

       //
       // Initialising the Terminal.
       //
       InitTerm();
       // Clearing the Terminal.
       //
       ClearTerm();
       memset(msgrx, ' ', 8);
       memset(msgtx, ' ', 8);

       // Display the Banner
       //
       // Reset the peripheral
       //
       MAP_PRCMPeripheralReset(PRCM_GSPI);
       MAP_SPIReset(GSPI_BASE);
           //
           // Configure SPI interface
           //
       MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                            SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                            (SPI_HW_CTRL_CS |
                            SPI_4PIN_MODE |
                            SPI_TURBO_OFF |
                            SPI_CS_ACTIVELOW |
                            SPI_WL_8));

       MAP_SPIEnable(GSPI_BASE);
       Adafruit_Init();
       fillScreen(WHITE);

          // Register the interrupt handlers
          //
          MAP_GPIOIntRegister(GPIOA0_BASE, GPIOA0IntHandler);
          //
          // Configure rising edge interrupts on SW2 and SW3
          //
          MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x10, GPIO_BOTH_EDGES);

          ulStatus = MAP_GPIOIntStatus (GPIOA0_BASE, false);
          MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);
          // clear global variables
          Input_intcount=0;
          Input_intflag=0;
          i =0;
          // Enable SW2 and SW3 interrupts
          MAP_GPIOIntEnable(GPIOA0_BASE, 0X10);
          //
             // Enable Timer
             //
             g_ulBase = TIMERA0_BASE;
             //
             // Base address for second timer

             //
             // Configuring the timers
             //
             Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

             //
             // Setup the interrupts for the timer timeouts.
             //
             Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
             //
             // Turn on the timers feeding values in mSec
             //
             MAP_TimerLoadSet(g_ulBase,TIMER_A,800);
                 //
                 // Enable the GPT
                 //
                 MAP_TimerEnable(g_ulBase,TIMER_A);

                    //http_get(lRetVal);

          while (1) {
              pin =  GPIOPinRead(GPIOA1_BASE, 0x1);
              Report("pin=%d \r\n",pin);
              if (pin){
                  msg[1] = '0';
              }
              else {
                  msg[1] = '1';
              }

              if (pin!=pin0){
                  flag = 1;
                  pin0 = pin;
              }

              if(flag ==1){
                  lRetVal = connectToAccessPoint();
                  lRetVal = set_time();
                  if(lRetVal < 0) {
                     UART_PRINT("Unable to set time in the device");
                     LOOP_FOREVER();
                     }
                  lRetVal = tls_connect();
                  if(lRetVal < 0) {
                      ERR_PRINT(lRetVal);
                  }
                      //Connect to the website with TLS encryption
                  http_post(lRetVal);
                  sl_Stop(SL_STOP_TIMEOUT);


                      flag=0;
              }
          }
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_post(int iTLSSockID){


    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;
    Report("%d",iTLSSockID);
    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1);
    dataLength += strlen(msg);
    dataLength += strlen(DATA2);
    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

    strcpy(pcBufHeaders, msg);
    pcBufHeaders += strlen(msg);

    strcpy(pcBufHeaders, DATA2);
    pcBufHeaders += strlen(DATA2);


    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }
    drawChar(0, 4, 's', BLACK, WHITE, 1);
    drawChar(4, 4, 'e', BLACK, WHITE, 1);
    drawChar(8, 4, 'n', BLACK, WHITE, 1);
    drawChar(12, 4, 't', BLACK, WHITE, 1);
    return 0;
}

static int http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];

    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");


    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Get the packet from the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
        if(lRetVal < 0) {
            UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
            sl_Close(iTLSSockID);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return lRetVal;
        }
        lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
        if(lRetVal < 0) {
            UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
            //sl_Close(iSSLSockID);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
               return lRetVal;
        }
        else {
            acRecvbuff[lRetVal+1] = '\0';
            UART_PRINT(acRecvbuff);
            UART_PRINT("\n\r\n\r");
        }



    return 0;
}
