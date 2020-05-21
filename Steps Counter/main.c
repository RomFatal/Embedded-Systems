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

   
    
    UserInit();

}//end InitializeSystem



//	========================	Application Code	========================

int ThresHold=13;
int ThresMode=10;
int ThresMin=13;
int ThresMax=30;

int xyzcal(BYTE lsb, BYTE msb, int accel)
{
		accel = (((int)msb) << 8) + lsb;
		accel >>= 6;
		if(accel & 0x0200) 
			accel |= 0xfc00;	

	return accel;
}

int stepsValidation() 
{

	BYTE lsbX, msbX,lsbY, msbY,lsbZ, msbZ;
	int x,y,z,xMax,xMin,yMax,yMin,zMax,zMin,i,j,index,tmp,changesFlag=0,x1,y1,z1;
	BOOL step=0;
	static char X[25];
	static char Y[25];
	static char Z[25];
	static int Ms = 0;
	char tmpp[5]="";//tmp

	lsbX = BMA150_ReadByte(2);
	msbX = BMA150_ReadByte(3);
	lsbY = BMA150_ReadByte(4);
	msbY = BMA150_ReadByte(5);
	lsbZ = BMA150_ReadByte(6);
	msbZ = BMA150_ReadByte(7);

	X[Ms] = x = xyzcal(lsbX,msbX,x);
	Y[Ms] = y = xyzcal(lsbY,msbY,y);
	Z[Ms] = z = xyzcal(lsbZ,msbZ,z);

//print
sprintf(tmpp,"X:");
oledPutString(tmpp, 3, 0);
sprintf(tmpp, "%4d", x);
oledPutString(tmpp, 3, 10);

sprintf(tmpp, "Y:");
oledPutString(tmpp, 3, 40);
sprintf(tmpp, "%4d", y);
oledPutString(tmpp, 3, 50);

sprintf(tmpp, "Z:");
oledPutString(tmpp, 3, 90);
sprintf(tmpp, "%4d", z);
oledPutString(tmpp, 3, 100);

//end

	for(i=0;i<20;i++)////////////////////////////////////////////////////////////////
		for(j=0;j<20;j++)
		{
		changesFlag=0;

		if(X[i]>X[j])
		{
		xMax=X[i]; xMin=X[j];
		}
		else
		{
		xMax=X[j]; xMin=X[i];
		}
		if(Y[i]>Y[j])
		{
		yMax=Y[i]; yMin=Y[j];
		}
		else
		{
		yMax=Y[j]; yMin=Y[i];
		}
		if(Z[i]>Z[j])
		{
		zMax=Z[i]; zMin=Z[j];
		}
		else
		{
		zMax=X[j]; zMin=Z[i];
		}
		if(xMax-xMin>ThresMin&&xMax-xMin<ThresMax)//if was a resunable change in X axis
		if(((xMax+xMin)/2)>=x-ThresHold&&((xMax+xMin)/2)<=x+ThresHold)//check if could be a step
		if(!(y<=12+ThresMode&&y>=12-ThresMode&&(z<=34+ThresMode&&z>=34-ThresMode||z<=42+ThresMode&&z>=42-ThresMode)))//Thres mode
		if(!((y<=20+ThresMode&&y>=20-ThresMode||y<=30+ThresMode&&y>=30-ThresMode)&&z<=46+ThresMode&&z>=46-ThresMode))//Thres mode
		if(!((x<=30+ThresMode&&x>=30-ThresMode||x<=20+ThresMode&&x>=20-ThresMode)&&z<=55+ThresMode&&z>=55-ThresMode))//Thres mode
		if(!(x<=-15+ThresMode&&x>=-15-ThresMode&&z<=52+ThresMode&&z>=52-ThresMode))//Thres mode
		changesFlag++;
		if(yMax-yMin>ThresMin&&yMax-yMin<ThresMax)//if was a resunable change in X axis
		if(((yMax+yMin)/2)>=y-ThresHold&&((yMax+yMin)/2)<=y+ThresHold)//check if could be a step
		if(!(x<=62+ThresMode&&x>=62-ThresMode&&(z<=8+ThresMode&&z>=8-ThresMode||z<=22+ThresMode&&z>=22-ThresMode)))//Thres mode
		if(!(x<=45+ThresMode&&x>=45-ThresMode&&z<=35+ThresMode&&z>=35-ThresMode))//Thres mode
		if(!(x<=27+ThresMode&&x>=27-ThresMode&&z<=55+ThresMode&&z>=55-ThresMode))//Thres mode
		if(!(x<=-47+ThresMode&&x>=-47-ThresMode&&z<=37+ThresMode&&z>=37-ThresMode))//Thres mode
		changesFlag++;
		if(zMax-xMin>ThresMin&&zMax-zMin<ThresMax)//if was a resunable change in Z axis
		if(((zMax+xMin)/2)>=z-ThresHold&&((zMax+xMin)/2)<=z+ThresHold)//check if could be a step
		if(!(x<=40+ThresMode&&x>=40-ThresMode&&(y<=13+ThresMode&&y>=13-ThresMode||y<=-2+ThresMode&&y>=-2-ThresMode)))//Thres mode
		if(!(x<=52+ThresMode&&x>=50-ThresMode&&(y<=-15+ThresMode&&y>=-15-ThresMode||y<=5+ThresMode&&y>=5-ThresMode)))//Thres mode
		if(!(x<=10+ThresMode&&x>=10-ThresMode&&y<=28+ThresMode&&y>=28-ThresMode))//Thres mode
		if(!(x<=10+ThresMode&&x>=10-ThresMode&&y<=28+ThresMode&&y>=28-ThresMode))//Thres mode
		if(!((x<=5+ThresMode&&x>=5-ThresMode||x<=20+ThresMode&&x>=20-ThresMode)&&y<=40+ThresMode&&y>=40-ThresMode))//Thres mode
		changesFlag++;

		if(changesFlag>=2) //if could be a step in at lest 2 axis we assuem step haapend
		{

		if (i>j)//delete step value from array
		for(index=j;index>i;index++)
		{
			X[index]=1; Y[index]=1; Z[index]=1; 
		}
		else
		for(index=i;index>j;index++)
		{
			X[index]=1; Y[index]=1; Z[index]=1; 
		}
		step=1;//step successfully
		}
		sprintf(tmpp, "%04d", i);
		oledPutString(tmpp, 2, 20); 
		sprintf(tmpp, "%04d", j);
		oledPutString(tmpp, 2, 60); 
		}

	if (Ms>20)
	Ms+=1;
	else
	Ms=0;
	DelayMs(6);
	return step;
}

void accelerated(void) 
{
	int accel,in=1,out=0;
	static int t=0;
	char line[25]="hallelujah";
	char x[] = "Func:";//
	BOOL o;

oledPutString(x, 0, 0);
o=stepsValidation();

if(o)
{
sprintf(x, "%4d", in);
oledPutString(x, 0, 20);
t++;
}

else
sprintf(x, "%4d", out);
oledPutString(x, 0, 20);

sprintf(x, "Steps:");
oledPutString(x, 5, 0);
sprintf(x, "%4d", t);
oledPutString(x, 5, 40); 
}

//	========================	Main	========================
void main(void) {
    InitializeSystem();
    while(1)						
    {
		accelerated();
    }
}

