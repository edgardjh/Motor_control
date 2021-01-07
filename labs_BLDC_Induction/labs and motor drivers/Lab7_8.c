#include "DSP2833x_Device.h"

// Prototype statements for functions found within this file.

interrupt void ePWM1_TZ_isr();
interrupt void cpu_timer0_isr(void);  //added function  part2
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitCpuTimers(void);
extern void InitPieVectTable(void);  //added number 19
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);  //added function part1

void Setup_ePWM1(void);
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
	Setup_ePWM1();

	InitPieCtrl();		//18
	InitPieVectTable(); //20  initialize the PIE-memory

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.EPWM1_TZINT=&ePWM1_TZ_isr;
	EDIS;


	InitCpuTimers();
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
	ConfigCpuTimer(&CpuTimer0,100,100000);
	IER |= 3;
	EINT;
	ERTM;
	CpuTimer0Regs.TCR.bit.TSS = 0;

	GpioDataRegs.GPASET.bit.GPIO9 = 1;
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
	GpioCtrlRegs.GPAMUX1.all = 0;
	GpioCtrlRegs.GPAMUX2.all = 0;	// GPIO31 ... GPIO16 = General Purpose I/O

	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;


	GpioCtrlRegs.GPAMUX2.bit.GPIO17 =3; // enable ZT6
	//GpioCtrlRegs.GPAPUD.bit.GPIO17 =0;

	//GpioCtrlRegs.GPBMUX1.all = 0;	// GPIO47 ... GPIO32 = General Purpose I/O
	//GpioCtrlRegs.GPBMUX2.all = 0;

	EDIS;
}

void cpu_timer0_isr()

{
	//GpioDataRegs.GPATOGGLE.bit.GPIO9 =1; //parpadeo de led

	counter++;
		  		/*
		  		counter & 1)? (GpioDataRegs.GPASET.bit.GPIO9 =1) : (GpioDataRegs.GPACLEAR.bit.GPIO9 =1);
		  		(counter & 2)? (GpioDataRegs.GPASET.bit.GPIO11=1) : (GpioDataRegs.GPACLEAR.bit.GPIO11 =1);
		  		(counter & 4)? (GpioDataRegs.GPBSET.bit.GPIO34=1) : (GpioDataRegs.GPBCLEAR.bit.GPIO34 =1);
		  		(counter & 8)? (GpioDataRegs.GPBSET.bit.GPIO49=1) : (GpioDataRegs.GPBCLEAR.bit.GPIO49 =1);

		  		if (counter == 16)
		  			counter =0;
*/
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void Setup_ePWM1()
{
	EPwm1Regs.TBCTL.bit.CLKDIV = 7;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 7;
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;
	EPwm1Regs.TBPRD = 8454;
	//EPwm1Regs.AQCTLA.all = 0x0000;
	EPwm1Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm1Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm1Regs.CMPA.half.CMPA = 4000;  //ancho del pulso
	//EPwm1Regs.AQCTLB.bit.CAD = 0x00001;
	//EPwm1Regs.AQCTLB.bit.CAU = 0x00002;
	EPwm1Regs.DBCTL.bit.POLSEL = 3;
	EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
	EPwm1Regs.DBCTL.bit.IN_MODE = 0;
	EPwm1Regs.DBRED = 1000;
	EPwm1Regs.DBFED = 1000;

	//ZT6 configure

	EALLOW;
	EPwm1Regs.TZCTL.bit.TZA = 2;
	EPwm1Regs.TZCTL.bit.TZB = 2;
	EPwm1Regs.TZSEL.bit.OSHT6 = 1;
	EPwm1Regs.TZEINT.all = 4;
	EDIS;


	EPwm2Regs.TBCTL.bit.CLKDIV = 7;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 7;
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;
	EPwm2Regs.TBPRD = 8454;
	EPwm2Regs.TBCTL.bit.PHSEN = 1;
	EPwm2Regs.TBPHS.half.TBPHS = 2818;
	EPwm2Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm2Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm2Regs.CMPA.half.CMPA = 4000;
	//EPwm2Regs.AQCTLB.bit.CAD = 0x00001;
	//EPwm2Regs.AQCTLB.bit.CAU = 0x00002;
	EALLOW;
	EPwm2Regs.TZCTL.bit.TZA = 2;
	EPwm2Regs.TZCTL.bit.TZB = 2;
	EPwm2Regs.TZSEL.bit.CBC6 = 1;
	EDIS;


	EPwm3Regs.TBCTL.bit.CLKDIV = 7;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 7;
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;
	EPwm3Regs.TBPRD = 8454;
	EPwm3Regs.TBCTL.bit.PHSEN = 1;
	EPwm3Regs.TBPHS.half.TBPHS = 5636;
	EPwm3Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm3Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm3Regs.CMPA.half.CMPA = 4000;
	//EPwm3Regs.AQCTLB.bit.CAD = 0x00001;
	//EPwm3Regs.AQCTLB.bit.CAU = 0x00002;
}

interrupt void ePWM1_TZ_isr()
{
	GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
	PieCtrlRegs.PIEACK.all = 2;

	EALLOW;
	SysCtrlRegs.WDKEY = 0x55;
	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm1Regs.TZCLR.bit.INT = 1;
	EDIS;
}
//===========================================================================
// End of SourceCode.
//===========================================================================
