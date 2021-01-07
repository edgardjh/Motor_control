/*
 * config_perifericos.h
 *
 *  Created on: 01/09/2016
 *      Author: Edgar
 */

#ifndef CONFIG_PERIFERICOS_H_
#define CONFIG_PERIFERICOS_H_

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

#endif /* CONFIG_PERIFERICOS_H_ */
