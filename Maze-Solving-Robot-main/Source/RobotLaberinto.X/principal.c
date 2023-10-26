/*
// DATOS DEL PROGRAMA //

TITULO: Robot solucionador laberinto
MICRO:PIC16F15244
ESTUDIANTES: Andres Juan Duran Valencia 2420191020 // Juan Sebastian Mosquera Cifuentes 2420191031
Profesor: Harold F MURCIA
FECHA: Mayo 2021
      ___           ___           ___           ___           ___     
     /  /\         /  /\         /__/\         /__/\         /  /\    
    /  /:/_       /  /::\       |  |::\       |  |::\       /  /::\   
   /  /:/ /\     /  /:/\:\      |  |:|:\      |  |:|:\     /  /:/\:\  
  /  /:/_/::\   /  /:/~/::\   __|__|:|\:\   __|__|:|\:\   /  /:/~/::\ 
 /__/:/__\/\:\ /__/:/ /:/\:\ /__/::::| \:\ /__/::::| \:\ /__/:/ /:/\:\
 \  \:\ /~~/:/ \  \:\/:/__\/ \  \:\~~\__\/ \  \:\~~\__\/ \  \:\/:/__\/
  \  \:\  /:/   \  \::/       \  \:\        \  \:\        \  \::/     
   \  \:\/:/     \  \:\        \  \:\        \  \:\        \  \:\     
    \  \::/       \  \:\        \  \:\        \  \:\        \  \:\    
     \__\/         \__\/         \__\/         \__\/         \__\/    

*/
// CONFIGURACION del MCU //

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <conio.h>
#include <pic16f15244.h>

#pragma config FEXTOSC = OFF                                                    // External Oscillator Mode Selection bits->Oscillator not enabled
#pragma config RSTOSC = HFINTOSC_1MHZ                                           // Power-up Default Value for COSC bits->HFINTOSC (1 MHz)
#pragma config CLKOUTEN = OFF                                                   // Clock Out Enable bit->CLKOUT function is disabled; I/O function on RA4
#pragma config VDDAR = HI                                                       // VDD Range Analog Calibration Selection bit->Internal analog systems are calibrated for operation between VDD = 2.3V - 5.5V

// CONFIG2
#pragma config MCLRE = EXTMCLR                                                  // Master Clear Enable bit->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR
#pragma config PWRTS = PWRT_OFF                                                 // Power-up Timer Selection bits->PWRT is disabled
#pragma config WDTE = OFF                                                       // WDT Operating Mode bits->WDT disabled; SEN is ignored
#pragma config BOREN = ON                                                       // Brown-out Reset Enable bits->Brown-out Reset Enabled, SBOREN bit is ignored
#pragma config BORV = LO                                                        // Brown-out Reset Voltage Selection bit->Brown-out Reset Voltage (VBOR) set to 1.9V
#pragma config PPS1WAY = ON                                                     // PPSLOCKED One-Way Set Enable bit->The PPSLOCKED bit can be cleared and set only once in software
#pragma config STVREN = ON                                                      // Stack Overflow/Underflow Reset Enable bit->Stack Overflow or Underflow will cause a reset

// CONFIG4
#pragma config BBSIZE = BB512                                                   // Boot Block Size Selection bits->512 words boot block size
#pragma config BBEN = OFF                                                       // Boot Block Enable bit->Boot Block is disabled
#pragma config SAFEN = OFF                                                      // SAF Enable bit->SAF is disabled
#pragma config WRTAPP = OFF                                                     // Application Block Write Protection bit->Application Block is not write-protected
#pragma config WRTB = OFF                                                       // Boot Block Write Protection bit->Boot Block is not write-protected
#pragma config WRTC = OFF                                                       // Configuration Registers Write Protection bit->Configuration Registers are not write-protected
#pragma config WRTSAF = OFF                                                     // Storage Area Flash (SAF) Write Protection bit->SAF is not write-protected
#pragma config LVP = ON                                                         // Low Voltage Programming Enable bit->Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored.

// CONFIG5
#pragma config CP = OFF                                                         // User Program Flash Memory Code Protection bit->User Program Flash Memory code protection is disabled

// DEFINICION PARA USO DE PUERTOS

#define _XTAL_FREQ 1000000
#define ledizq LATAbits.LATA4                                                   //Salida led izquierdo por pin A4
#define ledder LATCbits.LATC1                                                   //Salida led derecho por pin C1
#define motorizqA LATAbits.LATA5                                                //Salida motor izquierdo avanzar por pin A5
#define motorizqR LATBbits.LATB5                                                //Salida motor izquierdo retroceder por pin B5
#define motorderA LATCbits.LATC4                                                //Salida motor derecho avanzar por pin C4
#define motorderR LATCbits.LATC5                                                //Salida motor derecho retroceder por pin C5
#define leddetenido LATAbits.LATA2                                              //Salida led detenerse por pin A2
#define TRIGGERSCEN_SetHigh() do { LATCbits.LATC7 = 1; } while(0)
#define TRIGGERSCEN_SetLow() do { LATCbits.LATC7 = 0; } while(0)
#define TRIGGERSIZQ_SetHigh() do { LATCbits.LATC0 = 1; } while(0)
#define TRIGGERSIZQ_SetLow() do { LATCbits.LATC0 = 0; } while(0)
#define TRIGGERSDER_SetHigh() do { LATBbits.LATB6 = 1; } while(0)
#define TRIGGERSDER_SetLow() do { LATBbits.LATB6 = 0; } while(0)
#define ECHOSCEN_GetValue() PORTCbits.RC6
#define ECHOSIZQ_GetValue() PORTBbits.RB7
#define ECHOSDER_GetValue() PORTBbits.RB4

// DECLARACION DE VARIABLES //

uint16_t dutyCycleIzquierda = 500;                                              //Ciclo util base para el motor izquierdo
uint16_t dutyCycleDerecha = 485;                                                //Ciclo util base para el motor derecho
int distanciafrontal;                                                           //Distancia entre el sensor centro y el obstaculo mas cercano
int distanciaizquierda;                                                         //Distancia entre el sensor izquierdo y el obstaculo mas cercano
int distanciaderecha;                                                           //Distancia entre el sensor derecho y el obstaculo mas cercano
bool scen;                                                                      //Variable que indica si un obstaculo se encuentra a menos de 3cm por delante
bool sizq;                                                                      //Variable que indica si un obstaculo se encuentra a menos de 3cm por la izquierda
bool sder;                                                                      //Variable que indica si un obstaculo se encuentra a menos de 3cm por la derecha
unsigned int conteo_cen = 0;
unsigned int conteo_izq = 0;
unsigned int conteo_der = 0;
volatile uint16_t timer1ReloadVal;

// DECLARACION DE FUNCIONES Y PROCEDIMIENTOS //

void InicializacionPuertos()                                                    //Se indican las caracteristicas de cada pin (entradas, salidas, digitales o analogicas)
{
    // LATx registers
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;

    // TRISx registers
    TRISA = 0x09;                                                               //Entradas en´pines A0 y A3
    TRISB = 0x90;                                                               //Entradas en pines B4 y B7
    TRISC = 0x40;                                                               //Entrada en pin C7                                                               

    // ANSELx registers
    ANSELC = 0x00;
    ANSELB = 0x00;
    ANSELA = 0x01;

    // WPUx registers
    WPUB = 0x00;
    WPUA = 0x00;
    WPUC = 0x00;

    // ODx registers
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x00;

    // SLRCONx registers
    SLRCONA = 0x37;
    SLRCONB = 0xF0;
    SLRCONC = 0xFF;

    // INLVLx registers
    INLVLA = 0x3F;
    INLVLB = 0xF0;
    INLVLC = 0xFF;
    
    // TRISx registers
    //SALIDAS
    TRISA1 = 0;                                                                 //PWM MOTOR DERECHO
    TRISA2 = 0;                                                                 //LED INDICADOR
    TRISA4 = 0;                                                                 //LED IZQUIERDO
    TRISA5 = 0;                                                                 //MOTOR IZQUIERDO AVANZAR
    TRISB6 = 0;                                                                 //TRIGGER SENSOR DERECHO
    TRISB5 = 0;                                                                 //MOTOR IZQUIERDO RETROCEDER
    TRISC0 = 0;                                                                 //TRIGGER SENSOR IZQUIERDO
    TRISC1 = 0;                                                                 //LED DERECHO
    TRISC3 = 0;                                                                 //PWM MOTOR IZQUIERDO
    TRISC4 = 0;                                                                 //MOTOR DERECHO AVANZAR
    TRISC5 = 0;                                                                 //MOTOR DERECHO RETROCEDER
    TRISC7 = 0;                                                                 //TRIGGER SENSOR CENTRO
    //ENTRADAS
    TRISC6 = 1;                                                                 //ECHO CENTRO
    TRISB4 = 1;                                                                 //ECHO DERECHO
    TRISB7 = 1;                                                                 //ECHO IZQUIERDO
    
    RA1PPS = 0x04;                                                              //RA1 = PWM4 = MOTOR DERECHO
    RC3PPS = 0x03;                                                              //RC3 = PWM3 = MOTOR IZQUIERDO
}

void TMR1_Initialize(void)
{
    T1GCON = 0x00;
    T1GATE = 0x00;
    T1CLK = 0x03;
    TMR1H = 0x00;
    TMR1L = 0x01;
    PIR1bits.TMR1IF = 0;
    timer1ReloadVal=(uint16_t)((TMR1H << 8) | TMR1L);
    T1CON = 0x25;
}

void OSCILLATOR_Initialize(void)
{
    OSCEN = 0x00;
    OSCFRQ = 0x00;
    OSCTUNE = 0x00;
}

void sensor_centro(void)
{
    TMR1H = 0x00;
    TMR1L = 0x00;
    conteo_cen = 0;
    TRIGGERSCEN_SetHigh();
    __delay_us(10);
    TRIGGERSCEN_SetLow();
    while(!ECHOSCEN_GetValue());
    TMR1ON = 1;
    while (ECHOSCEN_GetValue() && !TMR1IF);
    TMR1ON = 0;
    if(!TMR1IF)
    {
        conteo_cen |= TMR1H << 8;
        conteo_cen |= TMR1L; 
    }
    else
    {
        conteo_cen = 0;
        TMR1IF = 0;
    }
    distanciafrontal = (conteo_cen * 0.8)  / 48;
    if(distanciafrontal <= 1)
    {
        scen = true;
    }
    else
    {
        scen = false;
    }
}

void sensor_izquierdo(void)
{
    TMR1H = 0x00;
    TMR1L = 0x00;
    conteo_izq = 0;
    TRIGGERSIZQ_SetHigh();
    __delay_us(10);
    TRIGGERSIZQ_SetLow();
    while(!ECHOSIZQ_GetValue());
    TMR1ON = 1;
    while (ECHOSIZQ_GetValue() && !TMR1IF);
    TMR1ON = 0;
    if(!TMR1IF)
    {
        conteo_izq |= TMR1H << 8;
        conteo_izq |= TMR1L; 
    }
    else
    {
        conteo_izq = 0;
        TMR1IF = 0;
    }
    distanciaizquierda = (conteo_izq * 0.8)  / 48;
    if(distanciaizquierda <= 1)
    {
        sizq = true;
    }
    else
    {
        sizq = false;
    }
}

void sensor_derecho(void)
{
    TMR1H = 0x00;
    TMR1L = 0x00;
    conteo_der = 0;
    TRIGGERSDER_SetHigh();
    __delay_us(10);
    TRIGGERSDER_SetLow();
    while(!ECHOSDER_GetValue());
    TMR1ON = 1;
    while (ECHOSDER_GetValue() && !TMR1IF);
    TMR1ON = 0;
    if(!TMR1IF)
    {
        conteo_der |= TMR1H << 8;
        conteo_der |= TMR1L; 
    }
    else
    {
        conteo_der = 0;
        TMR1IF = 0;
    }
    distanciaderecha = (conteo_der * 0.8)  / 48;
    if(distanciaderecha <= 1)
    {
        sder = true;
    }
    else
    {
        sder = false;
    }
}

void InicializacionTimer2()                                                     //Se configura el TIMER2 para que funcione el PWM
{
    T2CLKCON = 0x01;
    T2HLT = 0x00;
    T2RST = 0x00;
    T2PR = 249;
    T2TMR = 0x00;
    PIR1bits.TMR2IF = 0;
    T2CON = 0b10000000;
}

void InicializacionPWM3(void)                                                   //Se confiigura el PWM3
{
    PWM3CON = 0x90;
    PWM3DCH = 0x3E;
    PWM3DCL = 0x40;
}

void InicializacionPWM4(void)                                                   //Se configura el PWM4
{
    PWM4CON = 0x90;
    PWM4DCH = 0x3E;
    PWM4DCL = 0x40;
}

void PWM3_LoadDutyValue(uint16_t dutyValue)
{
    PWM3DCH = (dutyValue & 0x03FC) >> 2;
    PWM3DCL = (dutyValue & 0x0003) << 6;
}

void PWM4_LoadDutyValue(uint16_t dutyValue)
{
    PWM4DCH = (dutyValue & 0x03FC) >> 2;
    PWM4DCL = (dutyValue & 0x0003) << 6;
}

void avanzar()                                                                  //Salidas para que el robot avance
{
    uint16_t dutyCycleIzquierda = 0;                                            //Ciclo util para que el robot avance recto
    uint16_t dutyCycleDerecha = 25;                                             //Ciclo util para que el robot avance recto
    while(scen != 1)
        {
        motorizqA = 1;
        motorderA = 1;
        motorizqR = 0;
        motorderR = 0;
        ledder = 0;
        ledizq = 0;
        leddetenido = 0;
        while(distanciaderecha > distanciaizquierda)                            //Nivelacion de las distancias derecha e izquierda mediante PWM
        {
        dutyCycleIzquierda = dutyCycleIzquierda + 1;
        dutyCycleDerecha = dutyCycleDerecha - 1;
        PWM3_LoadDutyValue(dutyCycleIzquierda);
        PWM4_LoadDutyValue(dutyCycleDerecha);
        }
        while(distanciaderecha < distanciaizquierda)            
        {
        dutyCycleIzquierda = dutyCycleIzquierda - 1;
        dutyCycleDerecha = dutyCycleDerecha + 1;
        PWM3_LoadDutyValue(dutyCycleIzquierda);
        PWM4_LoadDutyValue(dutyCycleDerecha);
        }
        while(distanciaderecha == distanciaizquierda)
        {
        dutyCycleIzquierda = dutyCycleIzquierda;
        dutyCycleDerecha = dutyCycleDerecha;
        PWM3_LoadDutyValue(dutyCycleIzquierda);
        PWM4_LoadDutyValue(dutyCycleDerecha);
        }   
    }
}

void giro_derecha()
{
    while(scen != 1)
    {
        uint16_t dutyCycleIzquierda = 0;
        uint16_t dutyCycleDerecha = 15;
        motorizqA = 1;
        motorderA = 0;
        motorizqR = 0;
        motorderR = 1;
        ledder = 1;
        ledizq = 0;
        leddetenido = 0;
        PWM3_LoadDutyValue(dutyCycleIzquierda);
        PWM4_LoadDutyValue(dutyCycleDerecha);
    }
}

void giro_izquierda()
{
    while(scen != 1)
    {
        uint16_t dutyCycleIzquierda = 0;
        uint16_t dutyCycleDerecha = 15;
        motorizqA = 0;
        motorderA = 1;
        motorizqR = 1;
        motorderR = 0;
        ledder = 0;
        ledizq = 1;
        leddetenido = 0;
        PWM3_LoadDutyValue(dutyCycleIzquierda);
        PWM4_LoadDutyValue(dutyCycleDerecha);
    }
}

void detenerse()
{
    motorizqA = 0;
    motorderA = 0;
    motorizqR = 0;
    motorderR = 0;
    ledder = 0;
    ledizq = 0;
    leddetenido = 1;
}

void movimientos()
{
    if(sizq == 0 && scen == 0 && sder == 0)
    {
        detenerse();
    }
    else if(sizq == 0 && scen == 0 && sder == 1)
    {
        giro_izquierda();
    }
    else if(sizq == 0 && scen == 1 && sder == 0)
    {
        giro_derecha();
    }
    else if(sizq == 0 && scen == 1 && sder == 1)
    {
        giro_izquierda();
    }
    else if(sizq == 1 && scen == 0 && sder == 0)
    {
        avanzar();
    }
    else if(sizq == 1 && scen == 0 && sder == 1)
    {
        avanzar();
    }
    else if(sizq == 1 && scen == 1 && sder == 0)
    {
        giro_derecha();
    }
    else if(sizq == 1 && scen == 1 && sder == 1)
    {
        giro_derecha();
    }
}

// INICIO DEL PROGRAMA PRINCIPAL //

void main()
{
    InicializacionPuertos();
    OSCILLATOR_Initialize();
    InicializacionTimer2();
    InicializacionPWM3();
    InicializacionPWM4();
    while(1)
    {
        sensor_centro();
        __delay_ms(100);
        sensor_izquierdo();
        __delay_ms(100);
        sensor_derecho();
        __delay_ms(100);
        movimientos();
    }
}