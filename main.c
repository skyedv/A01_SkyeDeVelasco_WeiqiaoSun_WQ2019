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
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_SPI_Demo
// or
// docs\examples\CC32xx_SPI_Demo.pdf
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_uart.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_apps_rcm.h"
#include "gpio.h"

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"
#include "i2c_if.h"
#include "timer_if.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"

#define APPLICATION_VERSION     "1.1.1"
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
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;



#if defined(ccs)
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
static unsigned long msgtx[8];
static unsigned long msgrx[8];
int index=0;
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
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus;

    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        //Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{

    unsigned long ulUserData;
    unsigned long ulDummy;

    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    //Message("Enabled SPI Interface in Master Mode\n\r");

    //
    // User input
    //
    //Report("Press any key to transmit data....");

    //
    // Read a character from UART terminal
    //
    ulUserData = MAP_UARTCharGet(UARTA0_BASE);


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);

    //
    // Report to the user
    //
    //Report("\n\rSend      %s",g_ucTxBuff);
    //Report("Received  %s",g_ucRxBuff);

    //
    // Print a message
    //
    //Report("\n\rType here (Press enter to exit) :");

    //
    // Initialize variable
    //
    ulUserData = 0;

    //
    // Enable Chip select
    //
    MAP_SPICSEnable(GSPI_BASE);

    //
    // Loop until user "Enter Key" is
    // pressed
    //
    while(ulUserData != '\r')
    {
        //
        // Read a character from UART terminal
        //
        ulUserData = MAP_UARTCharGet(UARTA0_BASE);

        //
        // Echo it back
        //
        MAP_UARTCharPut(UARTA0_BASE,ulUserData);

        //
        // Push the character over SPI
        //
        MAP_SPIDataPut(GSPI_BASE,ulUserData);

        //
        // Clean up the receive register into a dummy
        // variable
        //
        MAP_SPIDataGet(GSPI_BASE,&ulDummy);
    }

    //
    // Disable chip select
    //
    MAP_SPICSDisable(GSPI_BASE);
}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{
    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,SLAVE_MSG,sizeof(SLAVE_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

    //
    // Enable Interrupts
    //
    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    //Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
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
static void
BoardInit(void)
{
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



int gradient(signed char Buff){
    float divider = 2;              //speed control
    float gradient = Buff /divider;
    return (int) gradient;
}


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
                                send(msgtx);
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

void send(unsigned long line[8]){
    //Disables UART interrupt while sending characters
    MAP_UARTIntDisable(UARTA0_BASE, UART_INT_RX | UART_INT_RT);
    int i;
    for(i = 0; i < 8; i++)
    {
        MAP_UARTCharPut(UARTA0_BASE, line[i]);
        MAP_UtilsDelay(80000);
    }
    MAP_UARTIntEnable(UARTA0_BASE, UART_INT_RX | UART_INT_RT);
}

void receive(){
    int i;
    unsigned long ulStatus;
    //int c;

    ulStatus = MAP_UARTIntStatus(UARTA0_BASE, true);
    UARTIntClear(UARTA0_BASE, ulStatus );
    MAP_UtilsDelay(80000);
    for(i = 0; i<8; i++)
    {
        msgrx[i] = MAP_UARTCharGet(UARTA0_BASE);
        //Report("receive:%c",msgrx[i]);

        MAP_UtilsDelay(80000);


        drawChar(8*i, 64,msgrx[i] , BLACK, WHITE, 1);
        MAP_UtilsDelay(80000);
    }
    memset(msgrx, ' ', 8);
    UARTIntEnable(UARTA0_BASE, UART_INT_RX|UART_INT_RT);
}



//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    unsigned long ulStatus;
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();
    memset(msgrx, ' ', 8);
    memset(msgtx, ' ', 8);
    //
    // Display the Banner
    //


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
    //UART0 IntI


        MAP_UARTConfigSetExpClk(UARTA0_BASE, MAP_PRCMPeripheralClockGet(PRCM_UARTA0),
                    UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                    UART_CONFIG_PAR_NONE));
        UARTEnable(UARTA0_BASE);

        MAP_UARTFIFODisable(UARTA0_BASE);
        // Set interrupt handlers
        MAP_UARTIntRegister(UARTA0_BASE,receive);
        MAP_UARTIntClear(UARTA0_BASE, UART_INT_RX);
        MAP_UARTIntEnable(UARTA0_BASE, UART_INT_RX|UART_INT_RT);
        UARTFIFOEnable(UARTA0_BASE);
    //test();

    // I2C Init
        //
    //
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





       while (1) {
       }
}
