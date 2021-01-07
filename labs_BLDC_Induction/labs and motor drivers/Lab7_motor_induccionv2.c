//###########################################################################
//
// FILE:	CONTROL PARA MOTOR DE INDUCCION 3-FASES
//
// TITLE:	DSP28335ControlCARD; Digital Output
//			Control for induction motor.
//###########################################################################
//  Ver | dd mmm yyyy | What  | Description of changes
// =====|=============|======|===============================================
//  1.0 | 09 Nov 2016 | Control | Lab5_1 for F28335;
//###########################################################################
#include "IQmathLib.h"
#include "DSP2833x_Device.h"

// Prototype statements for functions found within this file.

//interrupt void ePWM1A_compare_isr(void);

interrupt void cpu_timer0_isr(void);  //added function  part2
interrupt void adc_isr(void);

extern void InitAdc(void);
extern      displa_ADC(unsigned int);//
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitCpuTimers(void);
extern void InitPieVectTable(void);  //added number 19
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);  //added function part1

void Setup_ePWM(void);
void Gpio_select(void);
void delay_loop(long);

//###########################################################################
//						main code
//###########################################################################
#pragma DATA_SECTION(sine_table, "IQmathTables");

//global variables
float amp = .0005;
float tiempo = 1953;
float velocidad_VR1=32;
_iq30 sine_table[512];

void main(void)
{
	//InitSystem();	 	// Basic Core Initialization
	InitSysCtrl();      // función agregada

	DINT;				// Disable all interrupts

	Gpio_select();		// GPIO9,GPIO11,GPIO34 and GPIO49 as output (LEDs @ peripheral explorer)
	Setup_ePWM();
	InitPieCtrl();		//18
	InitPieVectTable(); //20  initialize the PIE-memory
	InitAdc();

	// REGISTER 1: ADCTRL1
	AdcRegs.ADCTRL1.all = 0;
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 0; // Dual Sequencer Mode
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0; // Single Run Mode
	AdcRegs.ADCTRL1.bit.ACQ_PS = 7; // 8 x ADC-Clock
	AdcRegs.ADCTRL1.bit.CPS =  0;  // divide by 1

	// REGISTER 2: ADCTRL2
	AdcRegs.ADCTRL2.all = 0;
	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1; // ePWM_SOCA trigger
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; // enable ADC int for seq1
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0; // interrupt after every EOS For register ADCTRL3:

	// REGISTER 3: ADCTRL3
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 6;  // set FCLK to 12.5 MHz

	// REGISTER MAXCONV
	AdcRegs.ADCMAXCONV.all = 1;  //2 conversion

	// REGISTER ADCCHSELSEG1
	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0; //1st channel ADCINA0
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1; // 2nd channel ADCINA1

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.ADCINT = &adc_isr;
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1; // duda con el 6
	//PieVectTable.EPWM1_INT = &ePWM1A_compare_isr;
	EDIS;

	//PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

	InitCpuTimers();
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	ConfigCpuTimer(&CpuTimer0,150,tiempo);
	IER |= 3;
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
	GpioCtrlRegs.GPAMUX1.all = 0x00000000; // duda
	GpioCtrlRegs.GPAMUX2.all = 0x00000000;	// GPIO31 ... GPIO16 = General Purpose I/O
	GpioCtrlRegs.GPADIR.all =  0x00000000;
	//GpioCtrlRegs.GPADIR.all =  0x00000A00;

	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;


	GpioCtrlRegs.GPBMUX1.all = 0x00000000;	// GPIO47 ... GPIO32 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0x00000000;	// GPIO63 ... GPIO48 = General Purpose I/O
	GpioCtrlRegs.GPBDIR.all =  0x00000000;

	GpioCtrlRegs.GPCMUX1.all = 0x00000000;	// GPIO79 ... GPIO64 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0x00000000;	// GPIO87 ... GPIO80 = General Purpose I/O
	GpioCtrlRegs.GPCDIR.all =  0x00000000;

	EDIS;
}

interrupt void cpu_timer0_isr(void)

{

	static unsigned int index_a = 0;
	static unsigned int index_b = 170;
	static unsigned int index_c = 340;
	static float v = 0;
	 // 9375/1953=4.88 entonces 4.88/9375= factor de 0 a 1
// (4.88/9375)(32.55:1953)
// mas facil 1/1953 = 0.000512

		v = (EPwm1Regs.TBPRD -_IQsat( _IQ30mpy(((amp*velocidad_VR1)*sine_table[index_a]+_IQ30(0.9999))/2, EPwm1Regs.TBPRD), EPwm1Regs.TBPRD,0));
		EPwm1Regs.CMPA.half.CMPA = (int)v;

		v = (EPwm2Regs.TBPRD -_IQsat( _IQ30mpy(((amp*velocidad_VR1)*sine_table[index_b]+_IQ30(0.9999))/2, EPwm2Regs.TBPRD), EPwm2Regs.TBPRD,0));
		EPwm2Regs.CMPA.half.CMPA = (int)v;

		v = (EPwm3Regs.TBPRD -_IQsat( _IQ30mpy(((amp*velocidad_VR1)*sine_table[index_c]+_IQ30(0.9999))/2, EPwm3Regs.TBPRD), EPwm3Regs.TBPRD,0));
		EPwm3Regs.CMPA.half.CMPA = (int)v;

		if(index_a++ > 511) index_a = 0;
		if(index_b++ > 511) index_b = 0;
		if(index_c++ > 511) index_c = 0;

	tiempo = 1985-velocidad_VR1;
	ConfigCpuTimer(&CpuTimer0,150,tiempo);  //1953+32.55=1985.55 para hacer la relacion 1 Hz = 1953us y 60 Hz = 32.55 us
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	CpuTimer0Regs.TCR.bit.TSS = 0;
}

void Setup_ePWM()
{
// sin banda muerta

	EPwm1Regs.TBCTL.bit.CLKDIV = 2;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 2;
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm1Regs.TBCTL.bit.PHSEN = 1;
	EPwm1Regs.TBPHS.half.TBPHS = 0;
	EPwm1Regs.TBPRD = 9375;
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm1Regs.CMPA.half.CMPA = 4000;  //ancho del pulso
	EPwm1Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm1Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm1Regs.AQCTLB.bit.CAD = 0x00001;
	EPwm1Regs.AQCTLB.bit.CAU = 0x00002;

	EPwm2Regs.TBCTL.bit.CLKDIV = 2;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 2;
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm2Regs.TBCTL.bit.PHSEN = 1;
	EPwm2Regs.TBPHS.half.TBPHS = 3125;
	EPwm2Regs.TBPRD = 9375;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm2Regs.CMPA.half.CMPA = 4000;
	EPwm2Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm2Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm2Regs.AQCTLB.bit.CAD = 0x00001;
	EPwm2Regs.AQCTLB.bit.CAU = 0x00002;
	//register ETSEL
	EPwm2Regs.ETSEL.bit.SOCAEN =1;
	EPwm2Regs.ETSEL.bit.SOCASEL =2;
	//Register ETPS
	EPwm2Regs.ETPS.all =0;
	EPwm2Regs.ETPS.bit.SOCAPRD =1;


	EPwm3Regs.TBCTL.bit.CLKDIV = 2;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 2;
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;
	EPwm3Regs.TBCTL.bit.PHSEN = 1;
	EPwm3Regs.TBPHS.half.TBPHS = 6250;
	EPwm3Regs.TBPRD = 9375;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm3Regs.CMPA.half.CMPA = 4000;
	EPwm3Regs.AQCTLA.bit.CAD = 0x00002;
	EPwm3Regs.AQCTLA.bit.CAU = 0x00001;
	EPwm3Regs.AQCTLB.bit.CAD = 0x00001;
	EPwm3Regs.AQCTLB.bit.CAU = 0x00002;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void adc_isr()
{
	// 1953.125 = 1 Hz  y  32.55 = 60 Hz
	// (1953.125-32.55)/4095 = 1920.575/4095 = 0.469
	//velocidad_VR1 = AdcMirror.ADCRESULT0;
	//velocidad_VR1 = (0.47*500)+32.55;
	velocidad_VR1 = _IQsat(AdcMirror.ADCRESULT0,1953,32); //funcion que sust. los 2 comandos anteriores

	EALLOW;
	SysCtrlRegs.WDKEY=0x0055;
	SysCtrlRegs.WDKEY=0x00AA;
	EDIS;
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; //reset ADC
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // clear interrut flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  // Acknowledge interrupt to PIE
}
