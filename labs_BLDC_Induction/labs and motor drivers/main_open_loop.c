#include "DSP2833x_Device.h"
#include <stdbool.h>

Uint16 PWM_DutyCycle = 100;

void Setup_ePWM(void);
void Gpio_select(void);
unsigned char Hall_YBG(void);

extern void InitPieVectTable(void);
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);

interrupt void gpio_hall_isr (void);

#define Y_GP 0x01
#define Y_GN 0x02
#define G_GP 0x04
#define G_GN 0x08
#define B_GP 0x10
#define B_GN 0x20

#define PWM_EN  0x050      //CAD = 1 & CAU 1
#define PWM_DS  0x090      //CAD = 2 & CAU 1

#define YPH_P   EPwm1Regs.AQCTLA.all
#define GPH_P   EPwm2Regs.AQCTLA.all
#define BPH_P   EPwm3Regs.AQCTLA.all

#define YPH_NE  GpioDataRegs.GPASET.bit.GPIO1
#define YPH_ND  GpioDataRegs.GPACLEAR.bit.GPIO1
#define GPH_NE  GpioDataRegs.GPASET.bit.GPIO3
#define GPH_ND  GpioDataRegs.GPACLEAR.bit.GPIO3
#define BPH_NE  GpioDataRegs.GPASET.bit.GPIO5
#define BPH_ND  GpioDataRegs.GPACLEAR.bit.GPIO5



void main(void){

	InitSysCtrl();

	DINT;				// Disable all interrupts

	Gpio_select();		// GPIO9,GPIO11,GPIO34 and GPIO49 as output (LEDs @ peripheral explorer)
	Setup_ePWM();
	InitPieCtrl();
	InitPieVectTable();

	EALLOW;
	PieVectTable.XINT1	= & gpio_hall_isr;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
	PieVectTable.XINT2	= & gpio_hall_isr;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
	PieVectTable.XINT3	= & gpio_hall_isr;
	PieCtrlRegs.PIEIER12.bit.INTx1 = 1;
	EDIS;

	IER |= M_INT1 | M_INT12;
	EINT;
	ERTM;
	CpuTimer0Regs.TCR.bit.TSS = 0;

	while(1);

}

void Gpio_select(void){
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all = 0;
	GpioCtrlRegs.GPAMUX2.all = 0;	// GPIO31 ... GPIO16 = General Purpose I/O
	GpioCtrlRegs.GPADIR.all =  0;

	GpioCtrlRegs.GPBMUX1.all = 0;	// GPIO47 ... GPIO32 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0;	// GPIO63 ... GPIO48 = General Purpose I/O
	GpioCtrlRegs.GPBDIR.all =  0;

	GpioCtrlRegs.GPCMUX1.all = 0;	// GPIO79 ... GPIO64 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0;	// GPIO87 ... GPIO80 = General Purpose I/O
	GpioCtrlRegs.GPCDIR.all =  0;

	GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0;    // Hall in, Yellow
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;    // Hall in, Green
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;    // Hall in, Blue

	// Asignacion de las interrupciones externas con los pines (XINTn)
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 8;
	GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 10;
	GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL = 34;

	//Interrupciones eternas en el flanco de subida/habilitacion (XINTn)
	XIntruptRegs.XINT1CR.bit.POLARITY = 1;
	XIntruptRegs.XINT1CR.bit.ENABLE   = 1;
	XIntruptRegs.XINT2CR.bit.POLARITY = 1;
	XIntruptRegs.XINT2CR.bit.ENABLE   = 1;
	XIntruptRegs.XINT3CR.bit.POLARITY = 1;
	XIntruptRegs.XINT3CR.bit.ENABLE   = 1;

	// Configuración de PWMs
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // GP_Y (PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0; // GN_Y (GPIO)
	GpioCtrlRegs.GPADIR.bit.GPIO1  = 1; // GN_Y out
	GpioDataRegs.GPACLEAR.bit.GPIO1 = 0;

	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // GP_G (PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0; // GN_G (GPIO)
	GpioCtrlRegs.GPADIR.bit.GPIO3  = 1;  // GN_G out
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 0;

	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // GP_B (PWM)
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0; // GN_B (GPIO)
	GpioCtrlRegs.GPADIR.bit.GPIO5  = 1; // GN_B out
	GpioDataRegs.GPACLEAR.bit.GPIO5 = 0;

	EDIS;
}

void Setup_ePWM(){
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm1Regs.TBCTL.bit.PHSEN = 0;
	EPwm1Regs.TBPRD = 3750;             // (1/2)(150MHz/20KHz(1)(1))
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm1Regs.CMPA.half.CMPA = PWM_DutyCycle;      //ancho del pulso
	EPwm1Regs.AQCTLA.all = PWM_DS;


	EPwm2Regs.TBCTL.bit.CLKDIV = 0;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm2Regs.TBCTL.bit.PHSEN = 0;
	EPwm2Regs.TBPRD = 3750;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm2Regs.CMPA.half.CMPA = PWM_DutyCycle;
	EPwm2Regs.AQCTLA.all = PWM_DS;

	EPwm3Regs.TBCTL.bit.CLKDIV = 0;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm3Regs.TBCTL.bit.PHSEN = 0;
	EPwm3Regs.TBPRD = 3750;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm3Regs.CMPA.half.CMPA = PWM_DutyCycle;
	EPwm3Regs.AQCTLA.all = PWM_DS;
}

interrupt void gpio_hall_isr (){
    unsigned char HALL_IN = 0;
	HALL_IN = Hall_YBG();
	switch (HALL_IN){
	case 6:
	    // B_GN rising edge; Y_GN falling edge
	    YPH_P = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        EPwm2Regs.AQCTLA.all = PWM_EN;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPASET.bit.GPIO5 = 1;
        break;

	case 4:
	    // Y_GP rising edge; G_GP falling edge
        EPwm1Regs.AQCTLA.all = PWM_EN;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPASET.bit.GPIO5 = 1;
        break;

	case 5:
	    // G_GN rising edge; B_GN falling edge
        EPwm1Regs.AQCTLA.all = PWM_EN;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPASET.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPASET.bit.GPIO5 = 1;
        break;
	case 1:
        // Y_GP rising edge; B_GP falling edge
        EPwm1Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPASET.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_EN;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        break;

	case 3:
        // Y_GP rising edge; G_GN falling edge
        EPwm1Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPASET.bit.GPIO1 = 1;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_EN;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        break;

	case 2:
        // G_GP rising edge; B_GP falling edge
	    EPwm1Regs.AQCTLA.all = PWM_DS;
	    GpioDataRegs.GPASET.bit.GPIO1 = 1;
	    EPwm2Regs.AQCTLA.all = PWM_EN;
	    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	    EPwm3Regs.AQCTLA.all = PWM_DS;
	    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
		break;
	default:
        // Todo Des-habilitado
        EPwm1Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        break;}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

unsigned char Hall_YBG(){
    unsigned char i = 0;
    i = (GpioDataRegs.GPADAT.bit.GPIO8 << 2) | (GpioDataRegs.GPADAT.bit.GPIO8 << 2)|(GpioDataRegs.GPADAT.bit.GPIO8);
    return   i;
}
