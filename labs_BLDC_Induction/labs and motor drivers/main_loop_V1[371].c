#include "DSP2833x_Device.h"
#include <stdbool.h>
#include "IQmathLib.h"

Uint16 PWM_DutyCycle = 100;
Uint16 PWM_FreqDes = 3750;
Uint32 periodo = 0;
float velocidad_VR1=0;

void Setup_ePWM(void);
void Setup_eCAP1(void);
void Gpio_select(void);
unsigned char Hall_YBG(void);

extern void InitAdc(void);
extern displa_ADC(unsigned int);
extern void InitPieVectTable(void);
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);

interrupt void eCAP1_isr(void);
interrupt void adc_isr(void);
interrupt void gpio_hall_isr (void);

#define Y_GP 0x01
#define Y_GN 0x02
#define G_GP 0x04
#define G_GN 0x08
#define B_GP 0x10
#define B_GN 0x20

#define PWM_EN  0x090      //CAD = 1 & CAU 1
#define PWM_DS  0x050      //CAD = 2 & CAU 1

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
	Setup_eCAP1();
	InitPieCtrl();
	InitPieVectTable();
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
	PieVectTable.ADCINT = &adc_isr;
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1; // duda con el 6

	PieVectTable.ECAP1_INT = & eCAP1_isr;  //ECAP
	PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
	PieVectTable.XINT1	= & gpio_hall_isr;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
	PieVectTable.XINT2	= & gpio_hall_isr;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
	PieVectTable.XINT3	= & gpio_hall_isr;
	PieCtrlRegs.PIEIER12.bit.INTx1 = 1;
	EDIS;

	IER |= M_INT1| M_INT4| M_INT12;

	GpioDataRegs.GPBCLEAR.bit.GPIO49 =1;

	while(1)
	{
	if (!GpioDataRegs.GPADAT.bit.GPIO17)
		{
		GpioDataRegs.GPBSET.bit.GPIO49 =1;
		EINT;
			ERTM;
		}
	}
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

	//LEDs: Para indicar secuencia
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;


	GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0;    // Hall in, Yellow
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;    // Hall in, Green
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;    // Hall in, Blue

	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0x00000000;


	// Asignacion de las interrupciones externas con los pines (XINTn)
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 8;
	GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 10;
	GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL = 60;

	//ECAP
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;

	//Interrupciones eternas en el flanco de subida/habilitacion (XINTn)
	XIntruptRegs.XINT1CR.bit.POLARITY = 3;
	XIntruptRegs.XINT1CR.bit.ENABLE   = 1;
	XIntruptRegs.XINT2CR.bit.POLARITY = 3;
	XIntruptRegs.XINT2CR.bit.ENABLE   = 1;
	XIntruptRegs.XINT3CR.bit.POLARITY = 3;
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
	EPwm1Regs.TBPRD = PWM_FreqDes;             // (1/2)(150MHz/20KHz(1)(1))
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm1Regs.CMPA.half.CMPA = PWM_DutyCycle;      //ancho del pulso
	EPwm1Regs.AQCTLA.all = PWM_DS;


	EPwm2Regs.TBCTL.bit.CLKDIV = 0;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm2Regs.TBCTL.bit.PHSEN = 0;
	EPwm2Regs.TBPRD = PWM_FreqDes;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
	EPwm2Regs.CMPA.half.CMPA = PWM_DutyCycle;
	EPwm2Regs.AQCTLA.all = PWM_DS;

	//register ETSEL
	EPwm2Regs.ETSEL.bit.SOCAEN =1;
	EPwm2Regs.ETSEL.bit.SOCASEL =2;
	//Register ETPS
	EPwm2Regs.ETPS.all =0;
	EPwm2Regs.ETPS.bit.SOCAPRD =1;

	EPwm3Regs.TBCTL.bit.CLKDIV = 0;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm3Regs.TBCTL.bit.PHSEN = 0;
	EPwm3Regs.TBPRD = PWM_FreqDes;
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
	    EPwm3Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        EPwm2Regs.AQCTLA.all = PWM_EN;
        GpioDataRegs.GPASET.bit.GPIO5 = 1;

        //LEDS
        GpioDataRegs.GPACLEAR.bit.GPIO9 =1;
        GpioDataRegs.GPASET.bit.GPIO11 = 1;
        GpioDataRegs.GPBSET.bit.GPIO34 =1;


        break;

	case 4:
	    // Y_GP rising edge; G_GP falling edge
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        EPwm3Regs.AQCTLA.all = PWM_DS;
        EPwm1Regs.AQCTLA.all = PWM_EN;
        GpioDataRegs.GPASET.bit.GPIO5 = 1;

        //LEDS
        GpioDataRegs.GPACLEAR.bit.GPIO9 =1;
        GpioDataRegs.GPACLEAR.bit.GPIO11 =1;
        GpioDataRegs.GPBSET.bit.GPIO34 =1;

        break;

	case 5:
	    // G_GN rising edge; B_GN falling edge
        EPwm2Regs.AQCTLA.all = PWM_DS;
        EPwm3Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        GpioDataRegs.GPASET.bit.GPIO3 = 1;
        EPwm1Regs.AQCTLA.all = PWM_EN;

        //LEDS
        GpioDataRegs.GPASET.bit.GPIO9 =1;
        GpioDataRegs.GPACLEAR.bit.GPIO11 =1;
        GpioDataRegs.GPBSET.bit.GPIO34=1;

        break;
	case 1:
        // Y_GP rising edge; B_GP falling edge
        EPwm1Regs.AQCTLA.all = PWM_DS;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        GpioDataRegs.GPASET.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_EN;

        //LEDS
        GpioDataRegs.GPASET.bit.GPIO9 =1;
        GpioDataRegs.GPACLEAR.bit.GPIO11 =1;
        GpioDataRegs.GPBCLEAR.bit.GPIO34 =1;

        break;

	case 3:
        // Y_GP rising edge; G_GN falling edge
        EPwm1Regs.AQCTLA.all = PWM_DS;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        GpioDataRegs.GPASET.bit.GPIO1 = 1;
        EPwm3Regs.AQCTLA.all = PWM_EN;

        //LEDS
        GpioDataRegs.GPASET.bit.GPIO9 =1;
        GpioDataRegs.GPASET.bit.GPIO11 =1;
        GpioDataRegs.GPBCLEAR.bit.GPIO34 =1;

        break;

	case 2:
        // G_GP rising edge; B_GP falling edge
	    EPwm1Regs.AQCTLA.all = PWM_DS;
	    EPwm3Regs.AQCTLA.all = PWM_DS;
	    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
	    GpioDataRegs.GPASET.bit.GPIO1 = 1;
	    EPwm2Regs.AQCTLA.all = PWM_EN;


	    //LEDS
        GpioDataRegs.GPACLEAR.bit.GPIO9 =1;
        GpioDataRegs.GPASET.bit.GPIO11 =1;
        GpioDataRegs.GPBCLEAR.bit.GPIO34 =1;

		break;
	default:
        // Todo Des-habilitado
        EPwm1Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        EPwm2Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        EPwm3Regs.AQCTLA.all = PWM_DS;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO9 =1;
        GpioDataRegs.GPACLEAR.bit.GPIO11 =1;
        GpioDataRegs.GPBCLEAR.bit.GPIO34 =1;


        break;}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

unsigned char Hall_YBG(){
    unsigned char i = 0;
    i = (GpioDataRegs.GPADAT.bit.GPIO8 << 2) | (GpioDataRegs.GPADAT.bit.GPIO10 << 1)|(GpioDataRegs.GPBDAT.bit.GPIO60);
    return   i;
}


interrupt void adc_isr()
{
	velocidad_VR1 = _IQsat(AdcMirror.ADCRESULT0,1500,0); //funcion que sust. los 2 comandos anteriores

	EPwm1Regs.CMPA.half.CMPA = velocidad_VR1;
	EPwm2Regs.CMPA.half.CMPA = velocidad_VR1;
	EPwm3Regs.CMPA.half.CMPA = velocidad_VR1;

	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; //reset ADC
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // clear interrut flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  // Acknowledge interrupt to PIE
}

void  Setup_eCAP1()
{
	ECap1Regs.ECCTL1.bit.CAP1POL = 0;
//	ECap1Regs.ECCTL1.bit.CTRRST1 = 1;
//	ECap1Regs.ECCTL1.bit.CAP2POL = 0;
	ECap1Regs.ECCTL1.bit.CTRRST1 = 1;
	ECap1Regs.ECCTL1.bit.FREE_SOFT = 0;
	ECap1Regs.ECCTL1.bit.PRESCALE = 0;
	ECap1Regs.ECCTL1.bit.CAPLDEN =1;
	ECap1Regs.ECCTL1.bit.CTRRST1;
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;
	ECap1Regs.ECCTL2.bit.STOP_WRAP = 1;
	ECap1Regs.ECCTL2.bit.REARM = 0;
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 2;
	ECap1Regs.ECEINT.bit.CEVT1 = 1;
}

interrupt void eCAP1_isr(void)
{
	periodo = (int32) ECap1Regs.CAP2-(int32) ECap1Regs.CAP1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
