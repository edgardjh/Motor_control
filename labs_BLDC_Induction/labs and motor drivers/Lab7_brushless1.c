//
//      Lab5_1: TMS320F28335
//      (c) Frank Bormann
//
//###########################################################################
//
// FILE:	Lab5_1.c
//
// TITLE:	DSP28335ControlCARD; Digital Output
//			4 - bit - counter at 4 LEDs LD1(GPIO9), LD2(GPIO11), LD3(GPIO34)
//			and LD4 (GPIO49)
//			software delay loop; watchdog disabled
//			template file for Lab5_1
//###########################################################################
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  3.0 | 02 May 2008 | F.B. | Lab5_1 for F28335;
//  3.1 | 06 Nov 2009 | F.B  | Lab5_1 for F28335 and PE revision5
//###########################################################################
#include "DSP2833x_Device.h"

// Prototype statements for functions found within this file.


interrupt void cpu_timer0_isr(void);  //added function  part2
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitCpuTimers(void);
extern void InitPieVectTable(void);  //added number 19
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);  //added function part1

void Setup_ePWM5B(void);
void Gpio_select(void);
//void InitSystem(void);
void delay_loop(long);

//###########################################################################
//						main code
//###########################################################################
int counter=0;

void main(void)
{
	// binary counter for digital output
	//InitSystem();	 	// Basic Core Initialization
	InitSysCtrl();      // función agegada

	DINT;				// Disable all interrupts

	Gpio_select();		// GPIO9,GPIO11,GPIO34 and GPIO49 as output (LEDs @ peripheral explorer)
	Setup_ePWM5B();

	InitPieCtrl();		//18
	InitPieVectTable(); //20  initialize the PIE-memory

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;


/*	InitCpuTimers();
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	ConfigCpuTimer(&CpuTimer0,150,1000000);
	IER |= 1;
	EINT;
	ERTM;
	CpuTimer0Regs.TCR.bit.TSS = 0;
*/
	while(1)
	{

	}
}

void delay_loop(long end)
{
	long i;
	for (i = 0; i < end; i++)
	{
		asm(" NOP");
	//	EALLOW;
	//	SysCtrlRegs.WDKEY = 0x55;
	//	SysCtrlRegs.WDKEY = 0xAA;
	//	EDIS;
	}
}

void Gpio_select(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
	GpioCtrlRegs.GPAMUX2.all = 0x00000000;	// GPIO31 ... GPIO16 = General Purpose I/O
	GpioCtrlRegs.GPBMUX1.all = 0x00000000;	// GPIO47 ... GPIO32 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0x00000000;	// GPIO63 ... GPIO48 = General Purpose I/O
	GpioCtrlRegs.GPCMUX1.all = 0x00000000;	// GPIO79 ... GPIO64 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0x00000000;	// GPIO87 ... GPIO80 = General Purpose I/O

	GpioCtrlRegs.GPADIR.all =  0x00000A00;
	// peripheral explorer: LED LD1 at GPIO9
	// peripheral explorer: LED LD2 at GPIO11
	GpioCtrlRegs.GPBDIR.all =  0x00000000;
	// peripheral explorer: LED LD3 at GPIO34
	// peripheral explorer: LED LD4 at GPIO49
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;


	EDIS;
}

void cpu_timer0_isr()

{


	counter++;
		  		(counter & 1)? (GpioDataRegs.GPASET.bit.GPIO9 =1) : (GpioDataRegs.GPACLEAR.bit.GPIO9 =1);
		  		(counter & 2)? (GpioDataRegs.GPASET.bit.GPIO11=1) : (GpioDataRegs.GPACLEAR.bit.GPIO11 =1);
		  		(counter & 4)? (GpioDataRegs.GPBSET.bit.GPIO34=1) : (GpioDataRegs.GPBCLEAR.bit.GPIO34 =1);
		  		(counter & 8)? (GpioDataRegs.GPBSET.bit.GPIO49=1) : (GpioDataRegs.GPBCLEAR.bit.GPIO49 =1);

		  		if (counter == 16)
		  			counter =0;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void Setup_ePWM5B()
{
	EPwm1Regs.TBCTL.bit.CLKDIV = 7;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 7;
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;  //up-down mode
	EPwm1Regs.TBPRD = 8454;
	EPwm1Regs.AQCTLA.all = 0x0006;

	//EPwm1Regs.TBCTL.bit.SYNCOSEL = 0;
	//EPwm1Regs.CMPB = 500;

	/*
	EPwm2Regs.TBCTL.bit.CLKDIV = 7;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 7;
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;
	EPwm2Regs.TBPRD = 8454;
	EPwm2Regs.AQCTLA.all = 0x0006;
	EPwm2Regs.TBCTL.bit.PHSEN = 1;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm2Regs.TBPHS.half.TBPHS = 2818;

	EPwm3Regs.TBCTL.bit.CLKDIV = 7;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 7;
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;
	EPwm3Regs.TBPRD = 8454;
	EPwm3Regs.AQCTLA.all = 0x0006;
	EPwm3Regs.TBCTL.bit.PHSEN = 1;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm3Regs.TBPHS.half.TBPHS = 5636;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;
	 */
}

/*
void InitSystem(void)
{
	EALLOW;
    SysCtrlRegs.WDCR = 0x00C2;
	SysCtrlRegs.SCSR = 0x0002;
	SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
	SysCtrlRegs.PLLCR.bit.DIV = 10;
	SysCtrlRegs.HISPCP.all = 1;
   	SysCtrlRegs.LOSPCP.all = 2;
	SysCtrlRegs.PCLKCR0.all = 0x0000;
	SysCtrlRegs.PCLKCR1.all = 0x0000;
	SysCtrlRegs.PCLKCR3.all = 0x0000;
	SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;
    EDIS;
}
*/
//===========================================================================
// End of SourceCode.
