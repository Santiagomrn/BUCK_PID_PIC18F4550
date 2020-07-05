/*
 * File:   main_PID_BUCK.c
 */
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pic18f4550.h>


// CONFIG1L
#pragma config PLLDIV = 5       // PLL Prescaler Selection bits (Divide by 5 (20 MHz oscillator input))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HSPLL_HS  // Oscillator Selection bits (HS oscillator, PLL enabled (HSPLL))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)


#ifndef _XTAL_FREQ
#define _XTAL_FREQ 48000000UL                           //Defining the oscillator frequency
#endif


void timer_ms(unsigned int n)
{
    unsigned int i;
    T0CON = 0x82;                                       //Enable timer with prescale value 8
    for(i=0;i<n;i++)
    {
        TMR0 = 65535 - (_XTAL_FREQ/(4*8*1000));         //Preload value for 1 ms
        while(!TMR0IF);                                 //Check for timer overflow
        TMR0IF=0;                                       //Clear timer flag
    }
    T0CON = 0x00;                                       //Disable Timer 0
}
void ADC_Init()
{    
    TRISA = 0xFF;	/* Set as input port */
    ADCON1 = 0x0E;	/* Ref vtg is VDD and Configure pin as analog pin */
    ADCON2 = 0x92;	/* Left Justified, 4Tad and Fosc/32. */
    ADRESH=0;		/* Flush ADC output Register */
    ADRESL=0;       /* Flush ADC output Register */
} 
int ADC_Read(int channel)
{
    int digital;

    /* Channel 0 is selected i.e.(CHS3CHS2CHS1CHS0=0000) & ADC is disabled */
    ADCON0 =(ADCON0 & 0b11000011)|((channel<<2) & 0b00111100);  
    ADCON2bits.ACQT = 0b111; //  Tiempo de Adquisici�n 20Tad.
    ADCON2bits.ADCS = 0b110; //  Tiempo de Conversi�n Fosc/64.
    ADCON0 |= ((1<<ADON)|(1<<GO));	/*Enable ADC and start conversion*/

    /* Wait for End of conversion i.e. Go/done'=0 conversion completed */
    while(ADCON0bits.GO_nDONE==1);

    digital = (ADRESH*256) | (ADRESL);	/*Combine 8-bit LSB and 2-bit MSB*/
    return(digital);
}
void PWM(int output){
    //int voltage = output;
    
    //OSCCON = 0x72;	/* Set internal clock to 8MHz */
    TRISC2 = 0;		/* Set CCP1 pin as output for PWM out */
    PR2 = 29;		/* Load period value */
    CCPR1L = (output>>2);	/* load duty cycle value */
    output=((output<<4)&0x0030)|0B00001100; 
    CCP1CON = output;	/* Set PWM mode and no decimal for PWM bit4 y 5 are of the duty cicle low bits */
    T2CON = 0B00000111;		/*pre-scalar 1:16, timer2 is off */
    TMR2 = 0;		/* Clear Timer2 initially */
    TMR2ON = 1;		/* Timer ON for start counting*/
}

void main()
{
    ADC_Init(); // Inicializar el ADC
    //PID
    
   
    float Kp=.02; //constante proporcional
    float Ki=0.7;//constante integrativa
    float Kd=.00006; //constante derivativo
    
    int input=0; //intrada del PID-salida del ADC
    int offset=512; //Referencia del ADC O valor esperado
    float Error =0;        //declaracion del error
    float Proporcional =0; 
    float Integral=0;
    float Derivativo=0;
    float T=0.054; //periodo de muestreo
    float Error_0=0;//error anterior
    int Control=0; //entrada al PWM salida del PID
    float MaxIntegralError=120;
     PWM(0);
   
    while(1){
        
        input=ADC_Read(0);//lee el valor del ADC EN EL PUERTO ZERO
        
        Error = offset - input;                 // Variable que guarda el error 
        Proporcional = Error * Kp;                 // Constante proporcional
        Integral = Integral + Error * Ki * T;       // Constante Integral
        Derivativo = (Error - Error_0) * Kd / T;    // Constante Derivativa
        Error_0 = Error;                            // Error

        Control =(int)(Proporcional + Integral + Derivativo);

        if(Control>120){   
            Control=120;
        }
        PWM(Control);
        timer_ms(50);
      
        
    }
 
}
