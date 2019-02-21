//This code realizes DTMF detection using Goertzel algorithm on AVR Atmega128 board
//Version 1.0 - 04 July 2014 by Omayma Said, FH-SWF

#include <math.h>
#include <stdbool.h>
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

long int goertzel(int sample[], long int coeff, int N)  ;
void post_test(void);

//-------Global variables--------//

int N=410;                       // block size
volatile int samples[410];       // buffer to store N samples
volatile int count;             // samples count
volatile bool flag;             // flag set when the samples buffer is full with N samples
volatile bool new_dig;          // flag set when inter-digit interval (pause) is detected

int power_all[8];               // array to store calculated power of 8 frequencies

int coeff[8];                   // array to store the calculated coefficients
int f_tone[8]={697, 770, 852, 941, 1209, 1336, 1477, 1633}; // frequencies of rows & columns

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

#define SPI_IF_BIT_RATE  400000
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

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

static unsigned long msgtx[8];
static unsigned long msgrx[8];
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

void
TimerBaseIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);

    //MAP_GPIOPinWrite(Base, Pin, Val);

    count ++;

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




//-------Start Main--------//

void main(void)
    {
int i;

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
            //enable timer
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
          MAP_TimerLoadSet(g_ulBase,TIMER_A,500);
              //
              // Enable the GPT
              //
              MAP_TimerEnable(g_ulBase,TIMER_A);





       while (1) {
       }

ADCSRA = 0x8F;              // 10001111   - ADEN-ADSC-ADFR-ADIF-ADIE-ADPS2-ADPS1-ADPS0
                            // ADEN Enable
                            // ADIE Interrupt Enable
                            // Division factor 128 - ADC clock 125 KHz

ADCSRA |= 1<<ADFR;          // Free Running Mode
ADCSRA |= 1<<ADSC;          // ADSC Start Conversion
ADMUX=0;                    // Select ADMUX channel

sei();                      // Enable Global Interrupts

    coeff[0] = 31548;
    coeff[1] = 31281;
    coeff[2] = 30950;
    coeff[3] = 30556;
    coeff[4] = 29143;
    coeff[5] = 28360;
    coeff[6] = 27408;
    coeff[7] = 26258;

while (1)
    {
      count=0;  //rest count
      flag=0;   //reset flag

    while(flag==0); // wait till N samples are read in the buffer and the flag set by the ADC ISR

        {
            for (i=0;i<8;i++)
                power_all[i]=goertzel(samples, coeff[i], N); // call goertzel to calculate the power at each frequency and store it in the power_all array

            post_test(); // call post test function to validate the data and display the pressed digit if applicable
        }

    }

}
//-------End of Main--------//




//---------------------------------------------------------------//

//-------ADC ISR--------//
ISR(ADC_vect)
{
    if (count<N)
        samples[count++]=ADC>>2;    // scale down ADC reading, store the value in the samples buffer & increment the count
    else if (count==N)              // if the buffer is full with N samples
        flag=1;                     // set flag to 1 to start decoding
}


//---------------------------------------------------------------//

//-------Goertzel function---------------------------------------//
long int goertzel(int sample[], long int coeff, int N)
//---------------------------------------------------------------//
{
//initialize variables to be used in the function
int Q, Q_prev, Q_prev2,i;
long prod1,prod2,prod3,power;

    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero

    for (i=0; i<N; i++) // loop N times and calculate Q, Q_prev, Q_prev2 at each iteration
        {
            Q = (sample[i]) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
            Q_prev2 = Q_prev;                                    // shuffle delay elements
            Q_prev = Q;
        }

        //calculate the three products used to calculate power
        prod1=( (long) Q_prev*Q_prev);
        prod2=( (long) Q_prev2*Q_prev2);
        prod3=( (long) Q_prev *coeff)>>14;
        prod3=( prod3 * Q_prev2);

        power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down

        return power;
}


//---------------------------------------------------------------//

//-------Post-test function---------------------------------------//
void post_test(void)
//---------------------------------------------------------------//
{
//initialize variables to be used in the function
int i,row,col,max_power;

 char row_col[4][4] =       // array with the order of the digits in the DTMF system
    {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
    };

// find the maximum power in the row frequencies and the row number

    max_power=0;            //initialize max_power=0

    for(i=0;i<4;i++)        //loop 4 times from 0>3 (the indecies of the rows)
        {
        if (power_all[i] > max_power)   //if power of the current row frequency > max_power
            {
            max_power=power_all[i];     //set max_power as the current row frequency
            row=i;                      //update row number
            }
        }


// find the maximum power in the column frequencies and the column number

    max_power=0;            //initialize max_power=0

    for(i=4;i<8;i++)        //loop 4 times from 4>7 (the indecies of the columns)
        {
        if (power_all[i] > max_power)   //if power of the current column frequency > max_power
            {
            max_power=power_all[i];     //set max_power as the current column frequency
            col=i;                      //update column number
            }
        }


if(power_all[col]==0 && power_all[row]==0) //if the maximum powers equal zero > this means no signal or inter-digit pause
    new_dig=1;                             //set new_dig to 1 to display the next decoded digit


    if((power_all[col]>1000 && power_all[row]>1000) && (new_dig==1)) // check if maximum powers of row & column exceed certain threshold AND new_dig flag is set to 1
        {
            write_lcd(1,row_col[row][col-4]);                       // display the digit on the LCD
            dis_7seg(8,row_col[row][col-4]);                        // display the digit on 7-seg
            new_dig=0;                                              // set new_dig to 0 to avoid displaying the same digit again.
        }
}
