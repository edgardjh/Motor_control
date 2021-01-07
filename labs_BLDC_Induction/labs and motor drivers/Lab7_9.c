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
#include "IQmathLib.h"
#include "DSP2833x_Device.h"

// Prototype statements for functions found within this file.


interrupt void ePWM1A_compare_isr(void);
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
#pragma DATA_SECTION(sine_table, "IQmathTables");
_iq30 sine_table[512];
int counter=0;


void main(void)
{
	// binary counter for digital output
	//InitSystem();	 	// Basic Core Initialization
	InitSysCtrl();      // funci�n agregada

	DINT;				// Disable all interrupts

	Gpio_select();		// GPIO9,GPIO11,GPIO34 and GPIO49 as output (LEDs @ peripheral explorer)
	Setup_ePWM1();

	InitPieCtrl();		//18
	InitPieVectTable(); //20  initialize the PIE-memory


	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.EPWM1_INT = &ePWM1A_compare_isr;
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;


	InitCpuTimers();
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	ConfigCpuTimer(&CpuTimer0,150,100);
	IER |= 5;
	EINT;
	ERTM;
	CpuTimer0Regs.TCR.bit.TSS = 0;

	while(1)
	{

	}
}

void Gpio_select(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX2.all = 0x00000000;	// GPIO31 ... GPIO16 = General Purpose I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
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
	EDIS;
}

void cpu_timer0_isr()

{
	//GpioDataRegs.GPATOGGLE.bit.GPIO9 =1; //parpadeo de led

	static unsigned int index = 0;
		EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD -_IQsat( _IQ30mpy((sine_table[index]+_IQ30(0.9999))/2, EPwm1Regs.TBPRD), EPwm1Regs.TBPRD,0);

		EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD -_IQsat( _IQ30mpy((sine_table[index]+_IQ30(0.9999))/2, EPwm2Regs.TBPRD), EPwm2Regs.TBPRD,0);

		EPwm3Regs.CMPA.half.CMPA = EPwm3Regs.TBPRD -_IQsat( _IQ30mpy((sine_table[index]+_IQ30(0.9999))/2, EPwm3Regs.TBPRD), EPwm3Regs.TBPRD,0);


		index +=1;
		if( index > 511) index = 0;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void Setup_ePWM1()
{
	EPwm1Regs.TBCTL.bit.CLKDIV = 2;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 2;
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;
	EPwm1Regs.TBPRD = 9375;
	EPwm1Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm1Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm1Regs.CMPA.half.CMPA = 4000;  //ancho del pulso
	//EPwm1Regs.AQCTLB.bit.CAD = 0x00001;
	//EPwm1Regs.AQCTLB.bit.CAU = 0x00002;
	//EPwm1Regs.DBCTL.bit.POLSEL = 3;
	//EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
	//EPwm1Regs.DBCTL.bit.IN_MODE = 0;
	EPwm1Regs.DBRED = 1000;
	EPwm1Regs.DBFED = 1000;
	EPwm1Regs.ETSEL.bit.INTSEL = 5;
	EPwm1Regs.ETSEL.bit.INTEN = 1;
	EPwm1Regs.ETPS.bit.INTPRD = 1;

	EPwm2Regs.TBCTL.bit.CLKDIV = 2;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 2;
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;
	EPwm2Regs.TBPRD = 9375;
	EPwm2Regs.TBCTL.bit.PHSEN = 1;
	EPwm2Regs.TBPHS.half.TBPHS = 3125;
	EPwm2Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm2Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm2Regs.CMPA.half.CMPA = 4000;
	EPwm2Regs.AQCTLB.bit.CAD = 0x00001;
	EPwm2Regs.AQCTLB.bit.CAU = 0x00002;

	EPwm3Regs.TBCTL.bit.CLKDIV = 2;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 2;
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;
	EPwm3Regs.TBPRD = 9375;
	EPwm3Regs.TBCTL.bit.PHSEN = 1;
	EPwm3Regs.TBPHS.half.TBPHS = 6250;
	EPwm3Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm3Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm3Regs.CMPA.half.CMPA = 4000;
	//EPwm3Regs.AQCTLB.bit.CAD = 0x00001;
	//EPwm3Regs.AQCTLB.bit.CAU = 0x00002;

}

interrupt void ePWM1A_compare_isr(void)
{
/*
	static unsigned int index = 0;
	EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD -_IQsat
	( _IQ30mpy((sine_table[index]+_IQ30(0.9999))/2, EPwm1Regs.TBPRD), EPwm1Regs.TBPRD,0);
	index +=1;
	if( index > 511) index = 0;
*/
	EPwm1Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = 4;
}
