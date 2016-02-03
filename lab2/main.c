#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>

#include "lcd.h"
#include "led.h"

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  

int globalClock = 0; // ms

void main(){
	// LCD init
	__C30_UART=1;
	lcd_initialize();
	lcd_clear();
	lcd_locate(0,0);

	// ==== LED init
	led_initialize();
	CLEARLED(LED4_PORT);
	CLEARLED(LED1_PORT);
	CLEARLED(LED2_PORT);

	// ==== button 1/external int 1 init
	AD1PCFGHbits.PCFG20 = 1; 
	TRISEbits.TRISE8 = 1;
	INTCON2bits.INT1EP = 1; // polarity selection, '1' falling edge triggering of the interrupt.
	IPC5bits.INT1IP = 0x01;
	IFS1bits.INT1IF = 0;
	IEC1bits.INT1IE = 1;

	// ==== LPOSCEN init
	__builtin_write_OSCCONL(OSCCONL | 2);
	
	// ==== timer 1 init
	T1CONbits.TON = 0; //Disable Timer
	T1CONbits.TCS = 1; //Select external clock
	T1CONbits.TSYNC = 0; //Disable Synchronization
	T1CONbits.TCKPS = 0b00; //Select 1:1 Prescaler
	TMR1 = 0x00; // Clear timer register
	PR1 = 32767; // Load the period value, 1 s
	IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
	IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1; // Enable Timer1 interrupt
	T1CONbits.TON = 1; // Start Timer

	// ==== timer 2 init
	T2CONbits.TON = 0;
	T2CONbits.TCS = 0; // Select internal clock
	T2CONbits.TGATE = 0; // Disable Gated Timer mode
	T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
	TMR2 = 0x00; // Clear timer register
	PR2 = 400; // Load the period value, 2ms
	IPC0bits.T2IP = 0x01;
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1;
	T2CONbits.TON = 1;

	// ==== timer 3 init

	int count = 0;
	int min = 0;
	int sec = 0;
	int msec = 0;
 	while(1){
	    count++;

	    TOGGLELED(LED4_PORT);

	    if (count == 2000){
		min = globalClock/6e4;
		sec = (globalClock % 6e3)/1e3;
		msec = (globalClock % 6e3) % 1e3;

            	lcd_locate(0,0);
            	lcd_printf("%2d:%2d:%3d\n",min,sec,msec);
	    }
	}
}

void __attribute__ ((__interrupt__)) _T1Interrupt(void){
	IFS0bits.T1IF = 0; // clear the interrupt flag
	
	TOGGLELED(LED2_PORT);
}

void __attribute__ ((__interrupt__)) _T2Interrupt(void){
	IFS0bits.T2IF = 0; 

	TOGGLELED(LED1_PORT);
	globalClock += 2;
}

void __attribute__ ((__interrupt__)) _INT1Interrupt(void){
	IFS1bits.INT1IF = 0;

	globalClock = 0;
}
