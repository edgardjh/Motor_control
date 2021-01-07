/*       CONTROL DE VELOCIDAD DE UN MOTOR DE INDUCCION (INVERSOR TRIFASICO)
 *
 *          PWM Configurado a 5KHz (200uS)
 *          La frecuencia del seno(codificado en el PWM) variara de 1Hz hasta 500Hz
 *          F/V se tendra que mantener constante para poder variar la velocidad adecuadamente
 *
 *          El contador del PWM se actualiza cada 25nS
 *          PB6, PWM de la señal con phi cero
 *          PB4, PWM de la señal con phi de 120°
 *          PE4, PWM de la señal con phi de 240°
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#define RED_LED     GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED   GPIO_PIN_3

#define PWM_PERIOD      8000            // 8000*(12.5nS*2) = 0.2mS = 5KHz
#define MAX_FRQ         60             // Frecuencia maxima para el seno
#define MIN_FRQ           1             // Frecuencia minima para el seno
#define PHASE_1           10            // Ubicacion del seno con fase +2pi/3
#define PHASE_2           20            // Ubicacion del seno con fase +4pi/3
#define DbRISE            1            // 2*12.5nS*DbRISE = 500nS; Tiempo muerto, flanco de subida
#define DbFALL            1            // 2*12.5nS*DbFALL = 500nS; Tiempo muerto, flando de bajada

#define FRQ2MAG_SIN(fc) (0.01*(fc))     // Calcula el valor del voltaje para mantener F/V constante
#define FRQ_TMR_SIN(fs) (1/((fs)*0.0000003875))  // Da lo intervalos para actualizar el ciclo de trabajo dando como resultado una frecuencia para el seno.
                                        // (31 valores del seno)*(12.5nS, (tiempo de conteo timer0)) = 387.5ns;
                                        // 1/fs*387.5nS = valor para cargar en el timer0.
volatile float FRQ_SIN = 1;             // Variable para controlar la frecuencia del seno
volatile uint8_t idx = 0;               // Contador que se usara en sin_pwm.
volatile uint16_t PulseWidth = 4000;     // Iniciamos un ciclo de trabajo muy pequeño.
volatile uint8_t BlinkLED = 0x02;
bool giro = true;

float sin1, sin2, sin3;

const int16_t sin_pwm[30] = {      // Constantes para generar el seno
        50,              // PWM0, con phi = 0°    [0]
        810,
        1586,
        2292,
        2898,
        3377,
        3709,
        3878,
        3878,
        3709,
        3377,            // PWM2, con phi = 120° [4]
        2898,
        2292,
        1586,
        810,
        50,
        -810,
        -1586,
        -2292,
        -2898,
        -3377,           // PWM3, con phi = 240° [8]
        -3709,
        -3878,
        -3878,
        -3709,
        -3377,
        -2898,
        -2292,
        -1586,
        -810
};



void config(void){
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);           // Reloj principal a 80MHz
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_8);                         // El contador del PWM va a (12.5nS * 2)

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                // Puerto de PWM0 (primeros dos modulos)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                // Puerto de PWM0 (tercer modulo)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                // Puerto de Leds y switch
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);                 // Habilitamos el modulo PWM0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);               // Hablitamos el modulo del TIMER0

    // Desbloqueamos el GPIO_F4
            HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
            HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
    ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);                           // Configuramos los interruptores
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // Pull-up en los interruptores

    //******* CONFIGURACION DE LOS PWM's *******//
    // Configurando los pines de los generadores en PWM0
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7  |  GPIO_PIN_4|GPIO_PIN_5);       // M0PWM 0 a 3
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);
    ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    ROM_GPIOPinConfigure(GPIO_PB7_M0PWM1);
    ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2);
    ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3);
    ROM_GPIOPinConfigure(GPIO_PE4_M0PWM4);
    ROM_GPIOPinConfigure(GPIO_PE5_M0PWM5);

    // Configuracion de los generadores, periodos y frecuencias en PWM0
        // Se configuran los generadores en conteo descendente y sincronizados
        //(para especificar cuando el contador y el comparador son actualizados)
    ROM_PWMGenConfigure(PWM0_BASE,  PWM_GEN_0, PWM_GEN_MODE_DOWN);
    ROM_PWMGenConfigure(PWM0_BASE,  PWM_GEN_1, PWM_GEN_MODE_DOWN);
    ROM_PWMGenConfigure(PWM0_BASE,  PWM_GEN_2, PWM_GEN_MODE_DOWN);

    ROM_PWMGenPeriodSet(PWM0_BASE,  PWM_GEN_0, PWM_PERIOD);         // Se establece la frecuencia de los PWM'1 a 20KHz
    ROM_PWMGenPeriodSet(PWM0_BASE,  PWM_GEN_1, PWM_PERIOD);         //
    ROM_PWMGenPeriodSet(PWM0_BASE,  PWM_GEN_2, PWM_PERIOD);         //

    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PulseWidth);         // Se establecen los anchos de los pulso
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PulseWidth);         //
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PulseWidth);         //


    //PWMOutputInvert(PWM0_BASE,PWM_OUT_1|PWM_OUT_3|PWM_OUT_5, true);
    // Activamos las salidas del PWM
    ROM_PWMDeadBandEnable(PWM0_BASE,PWM_GEN_0, DbRISE, DbFALL);     // Activamos los tiempos muertos en los tres generadores
    ROM_PWMDeadBandEnable(PWM0_BASE,PWM_GEN_1, DbRISE, DbFALL);     // Activamos los tiempos muertos en los tres generadores
    ROM_PWMDeadBandEnable(PWM0_BASE,PWM_GEN_2, DbRISE, DbFALL);     // Activamos los tiempos muertos en los tres generadores
    //ROM_PWMOutputInvert(PWM0_BASE,);
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT  |  PWM_OUT_2_BIT | PWM_OUT_3_BIT  |  PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
    //***************************************************************************************************************************//
    //***************************************************************************************************************************//
    //***************************************************************************************************************************//

    //******* CONFIGURACION DEL TIMER *******//
        // Configuramos el timer como periódico
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, FRQ_TMR_SIN(FRQ_SIN));       // Se configura el seno con una frecuencia FRQ_SIN
    IntPrioritySet(INT_TIMER0A, 0x20);                              // Prioridad 1.
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                // Interrupcion TIMER0, activada

    // Habilitamos los tres generadores con un retardo para generar un defase de 120°; (1778*3)(12.5nS) = 66.675uS con PWM@5KHz(200uS)
    ROM_PWMGenEnable(PWM0_BASE,PWM_GEN_0); // 0s                       3* 66.675uS = 200.025
    ROM_SysCtlDelay(444);                   // 66.675uS
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    ROM_SysCtlDelay(444);                   // 2*66.675uS = 133.35uS
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    ROM_IntMasterEnable();                  // Habilitacion de las interrupciones
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);  // Habilitamos el timer
}

int main(void){
    config();
    while(1){
       SysCtlDelay(4000000);
       if((!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)) & (!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0))){
           FRQ_SIN = MIN_FRQ;
           SysCtlDelay(100000000);
           giro = false;
           GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0x04);

       }
       if(!giro){
           if(FRQ_SIN < MAX_FRQ){
               FRQ_SIN ++;
           }
           else{
               GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0x00);
               //giro = true;
           }
       }

       if((!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)) & (FRQ_SIN < MAX_FRQ - 1)){
           FRQ_SIN++;

       }
        if((!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)) & (FRQ_SIN > MIN_FRQ)){
            FRQ_SIN-- ;
        }
     }
}


void Timer0IntHandler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       // Se limpia la interrupcion deo timer
    uint8_t idy = idx + PHASE_1, idz = idx + PHASE_2;     // Asignacion de los valores para los indices, para generar los senos defasados.
        if(idx > 9){
            idz = idx - 10;
            if(idx > 19){
                idy = idx - 20;
            }
        }

    // Cálculo de los valores para la magnitud del seno y frecuencia.
    TimerLoadSet(TIMER0_BASE, TIMER_A, FRQ_TMR_SIN(FRQ_SIN));           // Se carga el valor al timer.
    if(giro){
        PWM0_0_CMPA_R = (sin_pwm[idx]*FRQ2MAG_SIN(FRQ_SIN)) + 4000;         // Se calculan las magnitudes para los senos
        PWM0_1_CMPA_R = (sin_pwm[idy]*FRQ2MAG_SIN(FRQ_SIN)) + 4000;
        PWM0_2_CMPA_R = (sin_pwm[idz]*FRQ2MAG_SIN(FRQ_SIN)) + 4000;

    }
       else{
           PWM0_0_CMPA_R = (sin_pwm[idx]*FRQ2MAG_SIN(FRQ_SIN)) + 4000;         // Se calculan las magnitudes para los senos
           PWM0_1_CMPA_R = (sin_pwm[idz]*FRQ2MAG_SIN(FRQ_SIN)) + 4000;
           PWM0_2_CMPA_R = (sin_pwm[idy]*FRQ2MAG_SIN(FRQ_SIN)) + 4000;
       }


    if (++idx > 29){
        idx = 0;
        BlinkLED = ~BlinkLED;
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, BlinkLED);
    }
  }


