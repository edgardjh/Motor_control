/*************************************************************************
 *	PI controller for a DC motor, using a mono-phase rectifier with
 *	SCRs,
 *	 *
 *	Conetions:
 *		PF_4		Zero cross signal, SW1
 *
 *		PF_1		Firing signal for SCR's, (Red Led)
 *		PD_6		Encoder input, QEI
 *
 *
 *		BLUE_LED, indica que se ha entrado al controlador PID
 *		PUSH2,	para entrar a la rutina del PID
 *
 *	!!!!NOTA¡¡¡¡ DEHABILITAR LA RESISTENCIA PULL-UP, EN EL CRUCE POR CERO
 *	a 80 MHz el periodo es de 12.5 nS asi que 12.5nS*80 = 1uS
 *
 ************************************************************************
 ************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/flash.h"
#include "driverlib/rom.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "utils/uartstdio.h"

// Definiciones para Interruptores y led's
#define RED_LED		GPIO_PIN_1
#define	BLUE_LED	GPIO_PIN_2
#define GREEN_LED	GPIO_PIN_3
#define SW1			GPIO_PIN_4
#define	SW2			GPIO_PIN_0


// Variables para el PWM
#define PWM_PERIOD      4000            // 4000*12.5nS = 50uS = 20KHz
volatile uint16_t PulseWidth = 2000;     // Iniciamos un ciclo de trabajo muy pequeño,mayor DbRISE y DbFALL nos aseguramos que el motor no arranque.

// With this variables we can control the firing angle
// Variables para el control, directo, del angulo.
// Nota: min y max alpa no son los valores reales, usar alphaRe para obtenerla.
#define alphaRe(alpha) (590000 - alpha) // Fórmula para calcular el alpha real
#define minAlpha	0					// Tiempo minimo, nos da un voltaje de ~0V (0 RPM)
#define maxAlpha	209160				// Tiempo maximo, nos da un voltaje de ~90V (~2224 RPM)
volatile int32_t alpha = alphaRe(0);	// Nos aseguramos que el motor este apagado o muy lento.

// Varables to be used with the peripheral QEI
#define QEI_fs  0x0A			// Tiempo de muestreo para el QEI (1/QEI_fs)
volatile float rpm = 0;			// Variable para almacenar la velocidad

// Variables for the PI controller
#define eqT2v(time) (0.0138*(time) - 645.77)		// Calculamos las RPM con un tiempo dado
#define eqV2t(vel) (72.416*(vel) + 46831)			// Calculamos el tiempo con las RPM; NO DA EL TIEMPO REAL, PARA ELLO OCUPAR alphaRe
// para un segundo volatile float Kp = 115.5699, Ki =91.7679;			// Constantes del PI
const float Kp = 8.9844, Ki = .8;   //Kp = 8.9844, Ki = 0.8;
volatile float ei = 0, ed = 0, e[2];				// Error Integral, Error Diferencial,
float rfAlpha = eqV2t(1000);					// Se establece una velocidad de referencia, y se calcula el tiempo equivalente.

// Variables auxiliares para leds y otros
uint8_t state = 0x00;				// Variables auxiliar para usar leds
uint8_t state2 = 0x02;
uint8_t state4 = 0x04;
bool init_pi = false;			// PI, Disabled.
uint16_t tiempo = 0;

// Declaracion de funciones.
void PID(void);

void config(){
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);		// Reloj principal a 80MHz
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);										// Reloj del puerto F, Led's e interruptores
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);										// Reloj del puerto D, QEI
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                                        // Puerto de PWM0 (primeros dos modulos)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Configuracion del PWM
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);       									// M0PWM 0 a 3
    ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    ROM_PWMGenConfigure(PWM0_BASE,  PWM_GEN_0, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM0_BASE,  PWM_GEN_0, PWM_PERIOD);         						// Se establece la frecuencia de los PWM'1 a 20KHz
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PulseWidth);         						// Se establecen los anchos de los pulso
    ROM_PWMOutputInvert(PWM0_BASE,PWM_OUT_0_BIT, true);
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

    // Configuracion del puerto UART
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);							// Configure the pins for the receiver and transmitter using GPIOPinConfigure
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000); 													// Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);

	// Configuracion del puerto digital GPIOF
		// Se desbloquea el GPIOF_0, para poder ocupar SW2
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW1 | SW2);										// Se declara pin de entrada, SW1 y SW2
	//ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, SW1 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);	// Se configura la resistencia pull-up PF_4
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, SW2 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);		// Se configura la resistencia pull-up PF_0

	// CONFIGURACIONES DEL QEI
		// Se desbloquea le pin GPIOD7, para usarlo con el QEI
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);												// Se habilita el periferioco QEI
	GPIOPinConfigure(GPIO_PD6_PHA0);														// Pin GPIOD6 como PHA0 // GPIOPinConfigure(0x00031806);  //0x00031806 =>GPIO_PD6_PHA0
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6);											// Pin GPIOD6 como PHA0
	QEIConfigure(QEI0_BASE,(QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP), 1725);  // Configuracion del enconder de cuadratura
	QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, (SysCtlClockGet()/QEI_fs)); 				// Divide by clock speed to get counts/sec
	QEIVelocityEnable(QEI0_BASE);															// QEI velocidad activada

	// CONFIGURACIONES DEL TIMER
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
	TimerLoadSet(TIMER0_BASE, TIMER_A, minAlpha + alpha);

	// CONFIGURACION DE INTERRUPCIONES
	IntPrioritySet(INT_GPIOF,0x00);						// Prioridad 0, para el cruce por cero.
	IntEnable(INT_GPIOF);								// Habilitada la interrupcion GPIOF PF_4
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);		// Habilitada en el pin 4, SW1

	IntPrioritySet(INT_TIMER0A, 0x20);					// Prioridad 1, del TIMER0 activado en el cruce por cero
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);	// Interrupcion TIMER0, activada

	IntPrioritySet(INT_QEI0, 0x40);						// Prioridad  2, de la interrupcion QEI
	IntEnable(INT_QEI0);								// Habilitada interrupcion de QEI
	QEIIntEnable(QEI0_BASE,QEI_INTTIMER);				// Habilitada la interrupcion QEI

	// Activamos las todas las interrupciones
	IntMasterEnable();

	uint32_t addflahs = 0x0006000;
	FlashErase(addflahs);

	// ACTIVACION DE PERIFERICOS
	TimerEnable(TIMER0_BASE, TIMER_A);
	FPUEnable();										// Se habilitan las FPU
	QEIEnable(QEI0_BASE);
	FPULazyStackingEnable();
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0); // 0s                       3*16.65uS = 49.95uS

}


void main(void){
	config();
	UARTprintf("-Control PI- \n");
	while((GPIOPinRead(GPIO_PORTF_BASE,SW2)));
	init_pi = true;
	UARTprintf("PI activo: \n");
	while(1){
	   UARTprintf("RPM: %i \n", (int)rpm);
	    SysCtlDelay(1000000);
		}
}


// SCR's firing signal
void Timer0IntHandler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);			// Clear the timer interrupt
	IntMasterDisable();										// Se deshabilitan las interrupciones
	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0x02);
	ROM_SysCtlDelay(5333);									// (3 ciclos de reloj)(12.5ns)(5333) ~ 200us
	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0x00);
	IntMasterEnable();										// Habilitamos las interrupciones
}

// Get the speed from the encoder using QEI
void QEI0IntHandler(void){
	float velocidad;
	QEIIntClear(QEI0_BASE,QEI_INTTIMER);
	GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, state);
	state = ~ state;
	velocidad = QEIVelocityGet(QEI0_BASE);
	rpm = (velocidad/1024)*60*QEI_fs;					// Calculo de las revoluciones
	PWM0_0_CMPA_R = (int)(rpm * 1.772);
	if(init_pi)
	PID();
	if(tiempo > 33)
	    rfAlpha = eqV2t(500);
}

// Zero cross detection
void GPIOFIntHandler(void){
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
	GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, state4);
	state4 = ~state4;
	TimerLoadSet(TIMER0_BASE, TIMER_A, minAlpha + alpha);
	TimerEnable(TIMER0_BASE, TIMER_A);
}

// PI contorller
void PID(void){
	float alphapi = 0;
	e[0] = rfAlpha - eqV2t(rpm);		// Obtener el error proporcional
	ei += e[0];							// Obtener el error integral
	alphapi = Kp*e[0] + Ki*ei;			// Calcular el valor de control del PI, Obtenemos un tiempo proporcional no real
	tiempo++;
	if((maxAlpha > alphapi) & (alphapi > minAlpha )){
		alpha = alphaRe(alphapi);
	}
	else{
		if(alphapi > maxAlpha)
			alphapi = maxAlpha;
		else
			alphapi = minAlpha;
		alpha = alphaRe(alphapi);
	}
}

