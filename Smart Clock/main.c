/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM� FS USB Demo Board,
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/


//  ========================    INCLUDES    ========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"

#include "OledGraphics.h"


//  ========================    CONFIGURATION   ========================

#if defined(PIC18F46J50_PIM)
//Watchdog Timer Enable bit:
#pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
//PLL Prescaler Selection bits:
#pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
//Stack Overflow/Underflow Reset Enable bit:
#pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
//Extended Instruction Set Enable bit:
#pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
//CPU System Clock Postscaler:
#pragma config CPUDIV = OSC1        //No CPU system clock divide
//Code Protection bit:
#pragma config CP0 = OFF            //Program memory is not code-protected
//Oscillator Selection bits:
#pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
//Secondary Clock Source T1OSCEN Enforcement:
#pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
//Low-Power Timer1 Oscillator Enable bit:
#pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
//Fail-Safe Clock Monitor Enable bit:
#pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
//Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
#pragma config IESO = OFF           //Two-Speed Start-up disabled
//Watchdog Timer Postscaler Select bits:
#pragma config WDTPS = 32768        //1:32768
//DSWDT Reference Clock Select bit:
#pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
//RTCC Reference Clock Select bit:
#pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
//Deep Sleep BOR Enable bit:
#pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
//Deep Sleep Watchdog Timer Enable bit:
#pragma config DSWDTEN = OFF        //Disabled
//Deep Sleep Watchdog Timer Postscale Select bits:
#pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
//IOLOCK One-Way Set Enable bit:
#pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
//MSSP address mask:
#pragma config MSSP7B_EN = MSK7     //7 Bit address masking
//Write Protect Program Flash Pages:
#pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
//Write Protection End Page (valid when WPDIS = 0):
#pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
//Write/Erase Protect Last Page In User Flash bit:
#pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
//Write Protect Disable bit:
#pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored

#else
#error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//  ========================    Global VARIABLES    ========================
#pragma udata
//You can define Global Data Elements here
unsigned char RA0='0', RA1='1', RA2='2', RA3='3';
char cord[60][2] = {
{63,1},{66,1},{69,2},{72,3},{75,4},//0 ++ (63+30sin(x)/31-30cos(x)) -+
{78,5},{80,7},{83,9},{85,11},{87,13},
{89,16},{90,19},{91,22},{92,25},{93,28},
{93,31},{93,34},{92,37},{91,40},{90,43},//3 -+ (63+30cos(x)/31+30sin(x)) --
{89,46},{87,48},{85,51},{83,53},{81,55},
{78,57},{75,58},{72,59},{69,60},{66,61},
{63,61},{60,61},{57,60},{54,59},{51,58},//6 -- (63-30sin(x)/31+30cos(x)) +-
{48,57},{45,55},{43,53},{41,51},{39,49},
{37,46},{36,43},{35,40},{34,37},{33,34},
{33,31},{33,28},{34,25},{35,22},{36,19},//9 +- (63-30cos(x)/31-30sin(x)) ++
{37,16},{39,13},{41,11},{43,9},{45,7},
{48,5},{51,4},{54,3},{57,2},{60,1} };

/*
char cord[60][2] = {
{63,1},{65,3},{67,5},{69,7},{71,9},//0++
{73,11},{75,13},{77,15},{79,17},{81,19},
{83,21},{85,23},{87,25},{89,27},{91,29},
{93,31},{91,33},{89,35},{87,37},{85,39},//3 -+
{83,41},{81,43},{79,45},{77,47},{75,49},
{73,51},{71,53},{69,55},{67,57},{65,59},
{63,61},{61,59},{59,57},{57,55},{55,53},//6--
{53,51},{51,49},{49,47},{47,45},{45,43},
{43,41},{41,39},{39,37},{37,35},{35,33},
{33,31},{35,29},{37,27},{39,25},{41,23},//9+-
{43,21},{45,19},{47,17},{49,15},{51,13},
{53,11},{55,9},{57,7},{59,5},{61,3} };
*/

typedef struct {
    int day;
    int month;
} date;
typedef struct {
    int second;
    int minute;
    int hour;
} time;
time Time;
date Date;
char xhtemp,yhtemp,xmtemp=63,ymtemp=1;////////////////////////////////////////////////////////////////////////////////////////////////////
BOOL tickSec,tickMin,tickHour, coverFlag = 0,timeflag = 0,interval_24 = 1,AM = 0,analogOnflag = 0,Pedometerflag=0,pedometerOnflag=0;
int mode = 1,step2seconds[1],step24hours[1],cmStep=2;
//  ========================    PRIVATE PROTOTYPES  ========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();
BOOL CheckButtonPressed(void);
//  ========================    VECTOR REMAPPING    ========================
#if defined(__18CXX)
//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.  However, the current Microchip USB bootloader
//examples are intended to occupy addresses 0x00-0x7FF or
//0x00-0xFFF depending on which bootloader is used.  Therefore,
//the bootloader code remaps these vectors to new locations
//as indicated below.  This remapping is only necessary if you
//wish to program the hex file generated from this project with
//the USB bootloader.  If no bootloader is used, edit the
//usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_SD_BOOTLOADER
#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
#else
#define REMAPPED_RESET_VECTOR_ADDRESS     0x00
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
#endif
#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
extern void _startup (void);        // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset (void)
{
    _asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR (void)
{
    _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR (void)
{
    _asm goto YourLowPriorityISRCode _endasm
}
#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR (void)
{
    _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR (void)
{
    _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"
#pragma code
//  ========================    Application Interrupt Service Routines  ========================
//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode
void YourHighPriorityISRCode()
{
    static char disp=0 ;
    if(INTCONbits.T0IF)
    {
        Time.second++;
		tickSec = 1;
        if(Time.second == 60)
        {
            Time.second = 0;
            Time.minute++;
			tickMin = 1;
        }

        if(Time.minute == 60)
        {
            Time.minute = 0;
            Time.hour++;
			tickHour = 1;
        }

        if(Time.hour == 24)
        {
            Time.hour = 0;
            timeflag = 1;
        }

        if (timeflag)
        {
            if((Date.month == 1 || Date.month == 3 || Date.month == 5 || Date.month == 7 || Date.month == 8 || Date.month == 10 || Date.month == 12) && Date.day == 31)
            {
                Date.month++;
                Date.day = 1;
            }
            else if ((Date.month == 4 || Date.month == 6 || Date.month == 9 || Date.month == 11) && Date.day == 30)
            {
                Date.month++;
                Date.day = 1;
            }
            else if(Date.month == 2 && Date.day == 28)
            {
                Date.month++;
                Date.day = 1;
            }
            else
                Date.day++;

            timeflag = 0;
        }
        TMR0H = 0x48;
        TMR0L = 0xe5;
        INTCONbits.TMR0IF = 0b0 ;
    }
} //This return will be a "retfie fast", since this is in a #pragma interrupt section
#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode()
{
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.

} //This return will be a "retfie", since this is in a #pragma interruptlow section
#endif




//  ========================    Board Initialization Code   ========================
#pragma code
#define ROM_STRING rom unsigned char*

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void)
{
    /* Initialize the mTouch library */
    mTouchInit();

    /* Call the mTouch callibration function */
    mTouchCalibrate();

    /* Initialize the accelerometer */
    InitBma150();

    /* Initialize the oLED Display */
    ResetDevice();
    FillDisplay(0x00);
	Date.day=1;
    Date.month=1;
    Time.second=0;
    Time.minute=0;
    Time.hour=0;
	T0CON = 0x07 ;// Initialize Timer Interrupt
    RCONbits.IPEN = 1 ;			//Prio Enable
    INTCON2bits.TMR0IP = 1 ;	//Use Hi-Prio
    INTCON = 0xE0 ;				//Enable Timer Interrupt
    T0CON |= 0x80;				//Start the Timer
}//end UserInit


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    // Soft Start the APP_VDD
    while(!AppPowerReady())
        ;

#if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
    //On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
    //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
    //This allows the device to power up at a lower initial operating frequency, which can be
    //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
    //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
    //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while(pll_startup_counter--);
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.
#endif

#if defined(PIC18F46J50_PIM)
    //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
#endif

    UserInit();

}//end InitializeSystem



//  ========================    Application Code    ========================


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
static char line[24];


void ProtectoledPutString(unsigned char *ptr,unsigned char page, unsigned char col,BOOL flag)
{
    INTCONbits.GIE = 0;
    oledPutString( ptr, page, col,flag) ;
    INTCONbits.GIE = 1 ;
}

void BoldProtectoledPutString(unsigned char *ptr,unsigned char page, unsigned char col,BOOL flag)
{
    INTCONbits.GIE = 0 ;
    BoldoledPutString( ptr, page, col,flag);
    INTCONbits.GIE = 1 ;
}

BOOL CheckButtonPressed(void)
{
    if(PORTBbits.RB0 == 0)
        return TRUE;
    else
        return FALSE;
}


int checkUDButtons(unsigned int i){
  if(i>700)
        return 0;
    else return 1;
}

int cheackNavigationButtons(unsigned int i){
    if(i>550)
        return 0;
    else return 1;
}

void longLRcheck()
{
	 while( cheackNavigationButtons(mTouchReadButton(RA3)))
        {}

        while( cheackNavigationButtons(mTouchReadButton(RA0)))
		{}
}	

void clearScreen() {
    int i;
    for(i=0; i<8; i++) {
        oledPutROMString("                                 ", i, 0);
    }
}

void clearScreenRow(int i) {
    oledPutROMString("                                 ", i, 0);
}

	
void menuClock(time t)
{
    int timeprint;
    sprintf(timeprint, "%02d", t.second);
    ProtectoledPutString(timeprint, 0 ,2*58,1);
    sprintf(timeprint, "%02d", t.minute);
    ProtectoledPutString(timeprint, 0 ,2*49,1);

    if(interval_24)
    {
        sprintf(timeprint, "%02d", t.hour);
        ProtectoledPutString(timeprint, 0 ,2*40,1);
    }
    else
    {
		if(t.hour == 0)
				sprintf(timeprint, "%02d:", 12);
        else if(t.hour >12)
            sprintf(timeprint, "%02d", ((t.hour + 1) % 13));
        else
            sprintf(timeprint, "%02d", (t.hour % 13));
        ProtectoledPutString(timeprint, 0 ,2*40,1);

        if(t.hour>=0 && t.hour <=11)
        {
            sprintf(line,"AM");
            ProtectoledPutString(line, 0 ,2*32,1);
        }
        else
        {
            sprintf(line,"PM");
            ProtectoledPutString(line, 0 ,2*32,1);
        }

    }
    sprintf(line,":");
    ProtectoledPutString(line, 0 ,2*55,1);
    ProtectoledPutString(line, 0 ,2*46,1);

}

void setDisplay()
{
    char currChoice=1;
    clearScreen();
	longLRcheck();
    while(1)
    {
        sprintf(line,"Display");
        ProtectoledPutString(line, 0, 0,1);
        menuClock(Time);

		if(checkUDButtons(mTouchReadButton(RA1)))    //Pressed up           
			if(currChoice > 1) 
			currChoice--;

    	if( checkUDButtons(mTouchReadButton(RA2))) //Pressed down
 			if(currChoice < 2) 
			currChoice++;

            sprintf(line, "Digital mode");
            if(currChoice == 1)ProtectoledPutString(line, 1 ,2*6,0);
            else ProtectoledPutString(line, 1 ,2*6,1);
            sprintf(line, "Analog mode");
            if(currChoice == 2)ProtectoledPutString(line, 2 ,2*6,0);
            else ProtectoledPutString(line, 2 ,2*6,1);

        if( cheackNavigationButtons(mTouchReadButton(RA3)) ) // L to go back
        {
            clearScreen();
			longLRcheck();
            return 0;
        }
        if( cheackNavigationButtons(mTouchReadButton(RA0)) ) // R to go right
        {
			longLRcheck();
            if(currChoice == 1)
            {
                    mode = 1;
					analogOnflag=0;
                    clearScreen();
                    return 0;
            }
            else if( currChoice == 2)
            {
                    mode = 2;
					analogOnflag = 1;
                    clearScreen();
                    return 0;
            }
        }
        DelayMs(60);
    }
}
 
void setInterval()
{
    char currChoice=1;
    clearScreen();
	longLRcheck();
    while(1)
    {
    
        sprintf(line,"Interval");
        ProtectoledPutString(line, 0, 0,1);
        menuClock(Time);

		if(checkUDButtons(mTouchReadButton(RA1)))    //Pressed up           
			if(currChoice > 1) 
			currChoice--;

    	if( checkUDButtons(mTouchReadButton(RA2))) //Pressed down
 			if(currChoice < 2) 
			currChoice++;
       
            sprintf(line, "24h mode");
            if(currChoice == 1)ProtectoledPutString(line, 1 ,2*6,0);
            else ProtectoledPutString(line, 1 ,2*6,1);
            sprintf(line, "12h mode");
            if(currChoice == 2)ProtectoledPutString(line, 2 ,2*6,0);
            else ProtectoledPutString(line, 2 ,2*6,1);

        if( cheackNavigationButtons(mTouchReadButton(RA3)) ) // L to go back
        {
            clearScreen();
			longLRcheck();
            return 0;
        }
        if( cheackNavigationButtons(mTouchReadButton(RA0)) ) // R to go right
        {
			longLRcheck();
            if(currChoice == 1)
            {
                    interval_24 = 1;
                    clearScreen();
                    return 0;
            }
            else if( currChoice == 2)
            {
                    interval_24 = 0;
                    clearScreen();
                    return 0;
            }
        }
        DelayMs(60);
    }
}

void displayDate()
{
    sprintf(line, "%02d", Date.day);
    ProtectoledPutString(line, 7 ,98,1);
    sprintf(line,"/");
    ProtectoledPutString(line, 7 ,110,1);
    sprintf(line, "%02d", Date.month);
    ProtectoledPutString(line, 7 ,116,1);
}

void AMPM(time t)
{
	if(t.hour>=0 && t.hour <=11)
    {
    	sprintf(line,"AM");
    	ProtectoledPutString(line, 7 ,2*0,1);
    }
    else
    {
    	sprintf(line,"PM");
    	ProtectoledPutString(line, 7 ,2*0,1);
    }
}

void analogClockDisplay()
{
		char xhand,yhand, i;
		int t;
		for(i=0;i<12;i++)
		{
			xhand = cord[i*5][0];
			yhand = cord[i*5][1];
			if(i==0)
			yhand+=4;
			else if(0<i&&i<3)
			{
			xhand-=2;
			yhand+=2;
			}	
			else if(i==3)
			xhand-=4;
			else if(3<i&&i<6)
			{
			xhand-=2;
			yhand-=2;
			}
			else if(i==6)
			{
			yhand-=4;
			}	
			else if(6<i&&i<9)
			{
			xhand+=2;
			yhand-=2;
			}
			else if(i==9)
			xhand+=4;
			else if(9<i&&i<12)
			{
			xhand+=2;
			yhand+=2;
			}
			if(i%3==0)
			drawLine( xhand, yhand, cord[i*5][0], cord[i*5][1], fat );
			else
			drawLine( xhand, yhand, cord[i*5][0], cord[i*5][1], thin );
		}	
			if(Time.second == 0)
				drawLine( 63, 31, cord[0][0], cord[0][1], thin );	
			else
				drawLine( 63, 31, cord[Time.second-1][0], cord[Time.second-1][1], thin );
			xmtemp = cord[Time.minute][0]-(cord[Time.minute][0]-63)/5;
			ymtemp = cord[Time.minute][1]-(cord[Time.minute][1]-31)/5;
			drawLine( 63, 31, xmtemp, ymtemp, thick );
			xhtemp = 63�+(cord[((Time.hour%12)*5)][0]-63)/2;
			yhtemp = 31�+(cord[((Time.hour%12)*5)][1]-31)/2;
			drawLine( 63, 31, xhtemp, yhtemp, fat );	
}

void analogClock()
{
	if(tickSec)
	{	
			if(Time.second == 0)
				drawLine( 63, 31, cord[59][0], cord[59][1], thin );	
			else
				drawLine( 63, 31, cord[Time.second-1][0], cord[Time.second-1][1], thin );
		drawLine( 63, 31, cord[Time.second][0], cord[Time.second][1], thin );
		tickSec = 0;
	}
	if(tickMin)
	{	
		drawLine( 63, 31, xmtemp, ymtemp, thick );
	
		xmtemp = cord[Time.minute][0]-(cord[Time.minute][0]-63)/5;
		ymtemp = cord[Time.minute][1]-(cord[Time.minute][1]-31)/5;

		drawLine( 63, 31, xmtemp, ymtemp, thick );
		tickMin = 0;

		if(coverFlag==1 && Time.hour!=Time.minute)
		{
		drawLine( 63, 31, xhtemp, yhtemp, fat );
		coverFlag=0;
		}
	}

	if(tickHour)
	{	
		drawLine( 63, 31, xhtemp, yhtemp, fat );
		xhtemp = 63�+(cord[((Time.hour%12)*5)][0]-63)/2;
		yhtemp = 31�+(cord[((Time.hour%12)*5)][1]-31)/2;
		drawLine( 63, 31, xhtemp, yhtemp, fat );
		tickHour = 0;
	}
	AMPM(Time);
}

void digitalClock(time t)
{

    int timeprint;
/*
		char D[2] = {0X7E, 0};
		//oledPutString( D, 4, 10*6,1);
       	oledPutImage(RR, 5, 5,1,1);
*/
        sprintf(timeprint, "%02d", t.second);
        BoldProtectoledPutString(timeprint, 2 ,2*51,1);
        
        sprintf(timeprint, "%02d:", t.minute);
        BoldProtectoledPutString(timeprint, 2 ,2*27,1);

        if(interval_24)
        {
            sprintf(timeprint, "%02d:", t.hour);
            BoldProtectoledPutString(timeprint, 2 ,2*3,1);
        }
        else
        {
			if(t.hour == 0)
				sprintf(timeprint, "%02d:", 12);
            else if(t.hour >12)
                sprintf(timeprint, "%02d:", ((t.hour + 1) % 13));
            else
                sprintf(timeprint, "%02d:", (t.hour % 13));
            BoldProtectoledPutString(timeprint, 2 ,2*3,1);
            AMPM(t);
        }
}

void setClock()
{
    time timetemp;
    char c =1;
    timetemp.hour = Time.hour;
    timetemp.minute = Time.minute;
    timetemp.second = Time.second;
    clearScreen();
    longLRcheck();
    while(1)
    {
        sprintf(line,"Set Time");
        ProtectoledPutString(line, 0, 0,1);
        menuClock(Time);
        digitalClock(timetemp);
        sprintf(line,"     ");
        switch(c)
        {
        case 0:
        {
            clearScreen();
            return 0;
        }
        case 1:
        {
            ProtectoledPutString(line, 6 ,1*2,0);
            if( checkUDButtons(mTouchReadButton(RA1)))    //Pressed up
                timetemp.hour= (timetemp.hour +1)%24 ;

            if( checkUDButtons(mTouchReadButton(RA2))) //Pressed down
                if(timetemp.hour == 0)
					timetemp.hour = 23;
				else
					timetemp.hour--;
            break;
        }
        case 2:
        {
            ProtectoledPutString(line, 6 ,25*2,0);
            if( checkUDButtons(mTouchReadButton(RA1)))    //Pressed up
                timetemp.minute = (timetemp.minute +1)%60;

            if( checkUDButtons(mTouchReadButton(RA2))) //Pressed down
                if(timetemp.minute == 0)
					timetemp.minute = 59;
				else
					timetemp.minute--;
            break;
        }
        case 3:
        {
            ProtectoledPutString(line, 6 ,49*2,0);
            if( checkUDButtons(mTouchReadButton(RA1)))    //Pressed up
                timetemp.second = (timetemp.second +1)%60;

            if( checkUDButtons(mTouchReadButton(RA2))) //Pressed down
                if(timetemp.second == 0)
					timetemp.second = 59;
				else
					timetemp.second--;
            break;
        }
        case 4:
        {
            Time.hour = timetemp.hour;
            Time.minute = timetemp.minute;
            Time.second = timetemp.second;
            clearScreen();
            return 0;

        }
        }

        if( cheackNavigationButtons(mTouchReadButton(RA3)) && c>=0 ) // L to go back
        {
            c--;
            clearScreenRow(6);
			longLRcheck();
        }

        if( cheackNavigationButtons(mTouchReadButton(RA0)) ) // R to go right
        {
            c++;
            clearScreenRow(6);
			longLRcheck();
        }
        DelayMs(60);
    }
}

int datecheck(date* t, int mode)
{
    if(mode)
    {
        if(t->month == 1 || t->month == 3 || t->month == 5 || t->month == 7 || t->month == 8 || t->month == 10 || t->month == 12)
        {
            if(t->day < 31)
                return 1;
            else
                return 0;
        }
        else if (t->month == 4 || t->month == 6 || t->month == 9 || t->month == 11)
        {
            if(t->day < 30)
                return 1;
            else
                return 0;
        }
        else if(t->month == 2)
        {
            if(t->day < 28)
                return 1;
            else
                return 0;
        }
        else
            return 0;
    }
    else
    {
        if ((t->month == 4 || t->month == 6 || t->month == 9 || t->month == 11) && t->day > 30)
        {
            t->day = 30;
            return 0;
        }
        else if(t->month == 2 && t->day >28)
        {
            t->day = 28;
            return 0;
        }
        else
            return 1;
    }
}

void setDate()
{
    char step = 1;
    time timetemp;
    date datemp;
    datemp.month = Date.month;
    datemp.day = Date.day;
    timetemp.hour = Time.hour;
    timetemp.minute = Time.minute;
    clearScreen();
    longLRcheck();
    while(1)
    {
        sprintf(line,"Set Date");
        ProtectoledPutString(line, 0, 0,1);
        sprintf(line, "%02d/", datemp.day);
        BoldProtectoledPutString(line, 2 ,2*15,1);
        sprintf(line, "%02d", datemp.month);
        BoldProtectoledPutString(line, 2 ,2*39,1);
        menuClock(Time);
        sprintf(line,"     ");
        switch(step)
        {
        case 0:
        {
            clearScreen();
            return 0;
        }
        case 1:
        {
            ProtectoledPutString(line, 6 ,13*2,0);

                if( checkUDButtons(mTouchReadButton(RA1)) &&  datecheck(&datemp,1))    //Pressed up
                    datemp.day++;

                if( checkUDButtons(mTouchReadButton(RA2)) && datemp.day > 1) //Pressed down
                    datemp.day--; 
            break;
        }
        case 2:
        {
            ProtectoledPutString(line, 6 ,37*2,0);
           
                if( checkUDButtons(mTouchReadButton(RA1)) &&  datemp.month < 12)    //Pressed up
                    datemp.month++;
                datecheck(&datemp,0);

                if( checkUDButtons(mTouchReadButton(RA2)) && datemp.month > 1) //Pressed down
                    datemp.month--;
                datecheck(&datemp,0);
            break;
        }
        case 3:
        {
            Date.month = datemp.month;
            Date.day = datemp.day;
            clearScreen();
            return 0;
        }
        }
        if( cheackNavigationButtons(mTouchReadButton(RA3)) && step>=0 ) // L to go back
        {
            step--;
            clearScreenRow(6);
			longLRcheck();
        }
        if( cheackNavigationButtons(mTouchReadButton(RA0)) ) // R to go right
        {
            step++;
            clearScreenRow(6);
			longLRcheck();
        }
        DelayMs(60);
    }
}

void pedometerDisplay()
{
	int i;
	char lineG[2] = {0X82, 0};
	char point[2] = {0X86, 0};
	
 	sprintf(line,"Pedometer");
    ProtectoledPutString(line, 0, 0,1);
    sprintf(line,"%d",100);
    ProtectoledPutString(line, 1, 0,1); 
	for(i=24;i<128;i+=8)
	oledPutString(lineG, 1, i,1);
    sprintf(line,"%d",66);
    ProtectoledPutString(line, 3, 0,1); 
	for(i=24;i<128;i+=8)
	oledPutString(lineG, 3, i,1);
    sprintf(line,"%d",33);
    ProtectoledPutString(line, 5, 0,1);
	for(i=24;i<128;i+=8)
	oledPutString(lineG, 5, i,1);
	for(i=24;i<120;i+=5)
	oledPutString(point, 7, i,1);
}
void pedometerGraph()//*************************************************************************************************************************************************************************************
{
	menuClock(Time);
}
void pedometerInfo()
{
	int cmStepTemp=cmStep;
	sprintf(line,"Pedometer");
    ProtectoledPutString(line, 0, 0,1);
	
	longLRcheck();
	//menuClock(Time);

    while(1)
    {

        sprintf(line, "%d", cmStepTemp);
       	BoldProtectoledPutString(line, 2 ,2*15,1);
        sprintf(line, "cm");     
 		BoldProtectoledPutString(line, 2 ,2*39,1);
        menuClock(Time);
        sprintf(line,"     ");

  /*          if(checkUDButtons(mTouchReadButton(RA1)))    //Pressed up
			{
            	cmStepTemp++;
				sprintf(line, "%02d", cmStepTemp);
         	    BoldProtectoledPutString(line, 2 ,2*15,1);
			}

            if(checkUDButtons(mTouchReadButton(RA2))) //Pressed down
			{
                cmStepTemp--;
				sprintf(line, "%02d", cmStepTemp);
       			BoldProtectoledPutString(line, 2 ,2*15,1);
			}

        if( cheackNavigationButtons(mTouchReadButton(RA3))) // L to go back
        {
		clearScreen();
		return 0;
        }
        if( cheackNavigationButtons(mTouchReadButton(RA0))) // R to go right
        {
        cmStep=cmStepTemp;
		clearScreen();
		return 0;
        }
        DelayMs(60);
*/
    }

}
void setTraverse(char c){
	switch(c){
		case 1: setDisplay();break;
		case 2: setInterval();break;
		case 3: setClock();break;
		case 4: setDate();break;
		case 5: clearScreen(); mode = 3; break;
		case 6: 
			mode = 4;
			clearScreen();
			pedometerOnflag=1;
			break;
		default: break;
	}
}
void setMenu() 
{
    char currChoice=3;
    clearScreen();
    while(1)
    {
        sprintf(line,"Settings");
        ProtectoledPutString(line, 0, 0,1);
        menuClock(Time);

		if(checkUDButtons(mTouchReadButton(RA1)))    //up           
			if(currChoice > 1) 
			currChoice--;

    	if( checkUDButtons(mTouchReadButton(RA2))) //down
 			if(currChoice < 6) 
			currChoice++;

        sprintf(line, "Display Mode");//menu
        if(currChoice == 1)ProtectoledPutString(line, 1 ,2*6,0);
        else ProtectoledPutString(line, 1 ,2*6,1);
        sprintf(line, "12/24H Interval");
        if(currChoice == 2)ProtectoledPutString(line, 2 ,2*6,0);
        else ProtectoledPutString(line, 2 ,2*6,1);
        sprintf(line, "Set Time");
        if(currChoice == 3)ProtectoledPutString(line, 3 ,2*6,0);
        else ProtectoledPutString(line, 3 ,2*6,1);
        sprintf(line, "Set Date");
        if(currChoice == 4)ProtectoledPutString(line, 4 ,2*6,0);
        else ProtectoledPutString(line, 4 ,2*6,1);
		sprintf(line, "Pedometer Info");
        if(currChoice == 5)ProtectoledPutString(line, 5 ,2*6,0);
        else ProtectoledPutString(line, 5 ,2*6,1);
		sprintf(line, "Pedometer Graph");
        if(currChoice == 6)ProtectoledPutString(line, 6 ,2*6,0);
        else ProtectoledPutString(line, 6 ,2*6,1);
		

        if( cheackNavigationButtons(mTouchReadButton(RA3)) ) // L back
        {
            clearScreen();
			tickSec = 1;
			tickMin = 1;
			tickHour = 1;
            return 0;
        }
        if( cheackNavigationButtons(mTouchReadButton(RA0)) ) // R accept
            setTraverse(currChoice);
		  if(currChoice >= 5 && cheackNavigationButtons(mTouchReadButton(RA0))) break;
        DelayMs(60);
    }
}

int buttonLongPress(void)
{
    long checkCount;
    for(checkCount=0; checkCount<15000; checkCount++)
    {
        if(!(CheckButtonPressed()))
            return 0;
        if(checkCount%4000 == 0)
        {
            if(mode==1)
			digitalClock(Time);
			else if (mode==2)
				analogClock();
			else if (mode==3)
				pedometerInfo();
			else if (mode==4)
				pedometerGraph();
        }
    }
    clearScreen();
    return 1;
}

void Clock()
{
    clearScreen();
	mode = 4;
	pedometerOnflag=1;
    while(1)
    {
	switch(mode){
		case 1:
       		displayDate();
			digitalClock(Time);
			goto defaultt;
		case 2:
			if(analogOnflag)
			{
				analogClockDisplay();
				analogOnflag--;
			}
			displayDate();
			analogClock();
			goto defaultt;
		case 3:
			pedometerInfo();
			goto defaultt;
		case 4:
		if(pedometerOnflag)
			{
				pedometerDisplay();
				pedometerOnflag--;
			}
			pedometerGraph();
			goto defaultt;
		default:
        	defaultt:if(CheckButtonPressed())
        	{
           		if(buttonLongPress())
				{
               		analogOnflag++;
					pedometerOnflag++;
               		setMenu();
				}
       		}	
		}
    }
}

void main(void)
{
    InitializeSystem();
    Clock();
}//end main


/** EOF main.c *************************************************/