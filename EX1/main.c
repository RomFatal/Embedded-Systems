//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL
//rom&&tal
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"

#include "delays.h"

//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
     #pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
   //PLL Prescaler Selection bits:
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
   //Stack Overflsb/Underflsb Reset Enable bit:
     #pragma config STVREN = ON            //Reset on stack overflsb/underflsb enabled
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
     #pragma config LPT1OSC = OFF        //Timer1 oscillator configured for msber power operation
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



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here
char clear[] = " ";

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();


//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
  
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
  #endif  

  #pragma code
  
//	========================	Application Interrupt Service Routines	========================
  
  #pragma interrupt YourHighPriorityISRCode
  void YourHighPriorityISRCode()
  {
  }
  #pragma interruptlsb YourLowPriorityISRCode
  void YourLowPriorityISRCode()
  {
  
  }  
#endif




//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

void UserInit(void)
{
 
  mTouchInit();


  mTouchCalibrate();


  InitBma150(); 


   ResetDevice();  
   FillDisplay(0x00);
   oledPutROMString((ROM_STRING)" PIC18F Starter Kit  ",0,0);
}


static void InitializeSystem(void)
{
	
   while(!AppPowerReady())
		;

    #if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1;  
        while(pll_startup_counter--);
    }
    #endif

    #if defined(PIC18F46J50_PIM)
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
    
    UserInit();

}//end InitializeSystem



//	========================	Application Code	========================

void postive_num(int *num)
{
	if (*num < 0)
		*num *= -1;
}

void printScreen(void) 
{

	char x[] = "X:";//
	char y[] = "Y:";
	char btn[] = "BTN:";
	char temp[] = "TMP:";
	char AD[] = "A/D:";
	char line[2] = {0X82, 0};
	char degree[] = {0X7F, 0};
	char divider[] = {0X80,0};
	char accel[] = "accelerat";
	char BracketL[] = {0X5B,0};
	char BracketR[] = {0X5D,0};
	int count;
	for(count = 0; count< 127; count++)
		oledPutSingleColumn(0x00, 0, 0+count);//clean LCD

	oledPutString(temp, 0, 0*6);
	oledPutString(degree, 0, 6*6);
	oledPutString(btn, 1, 0*6);
	oledPutString(AD, 2, 10*6);
	

	oledRepeatString(line,76,3,40);
	oledPutString(divider,3,18*2);
	oledPutString(divider,3,60*2);
	oledPutString(x, 3, 1*2);
	oledPutString(y, 4, 1*2);

	oledPutString(accel, 5, 6*6);

	oledPutString(BracketL,6,0*2);
	oledPutString(BracketL,7,0*2);
	oledPutString(BracketR,6,62*2);
	oledPutString(BracketR,7,62*2);

	oledPutString(divider,6,31*2);
	oledPutString(divider,7,31*2);

}

void temperature(void) 
{
	BYTE BMA_temp;
	int curr_temp;
	char cprint[3];
	BMA_temp = BMA150_ReadByte(8);
	curr_temp = ((BMA_temp * 127.5) / 0xff) -30;
	sprintf(cprint, "%2d", curr_temp);
	oledPutString(cprint, 0, 4*6);
}


void pressButton (void) 
{
	char nopress[2] = {0X58,0};
	char press[2] = {0X84,0};

	if(PORTBbits.RB0 == 0) 
		oledPutString(press, 1, 4*6);
	else 
		oledPutString(nopress, 1, 4*6);
}

int oldSlider;

void potentiometer(void) 
{

	int count;
	int newSlider;
	unsigned int result;
	char line[2] = {0X82, 0};
	char point[2] = {0X81, 0};
	char cprint[5];

	ADCON0 = 0x13;
	while(ADCON0 & 0x02);
	result = ADRESH;
	result = result << 8;
	result = result | ADRESL;
	sprintf(cprint, "%4d", result);
	oledPutString(cprint, 2, 14*6);
	newSlider = result / 64;

	if(newSlider!=oldSlider)
{
	oledPutString(line, 3, (8+oldSlider)*5);
	oledPutString(point, 3, (8+newSlider)*5);
	oldSlider=newSlider;
}
}

void sideButtons(void) 
{
	int left = mTouchReadButton(3);
	int right =  mTouchReadButton(0);
	char cRight[2] = {0X85, 0};
	char cLeft[2] = {0X86, 0};
	if(left < 700) 
		oledPutString(cLeft, 4, 10*6);
	else 
		oledPutString(clear, 4, 10*6);
	if(right < 700) 
		oledPutString(cRight, 4, 11*6);	
	else 
		oledPutString(clear, 4, 11*6);
}

int xyzcal(BYTE lsb, BYTE msb, int accel)
{
		accel = (((int)msb) << 8) + lsb;
		accel >>= 6;
		if(accel & 0x0200) 
			accel |= 0xfc00;	

	return accel;

}

int getParameter(int num) 
{
	BYTE lsb, msb;
	int printXY;
	char count;
	static int x = 0;
	static int y = 0;
	static int z = 0;
	static int minX = 0;
	static int minY = 0;
	static int maxX = 0;
	static int maxY = 0;

	lsb = BMA150_ReadByte(num);
	if(lsb & 0x01 == 1) 
	{
		msb = BMA150_ReadByte(num+1);
		if(num == 2)
			x = xyzcal(lsb,msb,x);
		else if(num == 4)
			y = xyzcal(lsb,msb,y);
		else if(num == 6)
			z= xyzcal(lsb,msb,z);

		
		if(num == 6) 
		{
			if(z < -60) 
			{
				for(count = 0; count < 40; count++) 
					Delay1KTCYx(0);

				lsb = BMA150_ReadByte(num);
				msb = BMA150_ReadByte(num+1);
				z = xyzcal(lsb,msb,z);
				if(z >= -60) 
					return 0;

				minY = 0;
				minX = 0;
				maxX = 0;
				maxY = 0;
				for(count = 0; count < 60; count++)  
				{
					oledPutSingleColumn(0x00, 6, 4+count);
					oledPutSingleColumn(0x00, 6, 66+count);
					oledPutSingleColumn(0x00, 7, 4+count);
					oledPutSingleColumn(0x00, 7, 66+count);
				}
			}
		}
		else 
		{
			if(num == 2) 
			{
				sprintf(printXY, "%4d", x);
				oledPutString(printXY, 3, 2*6); 
				if(x > maxX) 
				{
					maxX = x;
					return maxX;
				}
				else if(x < minX) 
				{
					minX = x;
					return minX;
				}
			}
			else if(num == 4) 
			{
				sprintf(printXY, "%4d", y);
				oledPutString(printXY, 4, 2*6); 
				if(y > maxY) 
				{
					maxY = y;
					return maxY;
				}
				else if(y < minY) 
				{
					minY = y;
					return minY;
				}
			}
			
		}
	}
	return 0;
}


void printAcc (int axle, int num)
{
int b_length;
	unsigned char count;
	b_length = axle / 9;
	postive_num(&b_length);
	if(axle > 0) 
		for(count = 0; count < b_length; count++) 
		{
			if (num == 2)
				oledPutSingleColumn(0x18, 6, 67+count);
			else if (num == 4)
				oledPutSingleColumn(0x18, 7, 67+count);
		}
	else if(axle < 0) 
		for(count = 0; count < b_length; count++) 
			{
			if	(num == 2)
				oledPutSingleColumn(0x18, 6, 61-count);
			else if	(num == 4)
				oledPutSingleColumn(0x18, 7, 61-count);
		}
}


void accelerated(void) 
{
	int accel;
	accel = getParameter(2);
	printAcc(accel, 2);
	accel = getParameter(4);
	printAcc(accel, 4);
	getParameter(6);
}

//	========================	Main	========================
void main(void) {
    InitializeSystem();
	printScreen();

    while(1)						
    {
		potentiometer();
		temperature();
		pressButton();
		sideButtons();
		accelerated();
    }
}
//end main


/** EOF main.c *************************************************/
//#endif
