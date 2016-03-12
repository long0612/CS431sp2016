#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>

#include "lcd.h"
#include "led.h"
#include "flextouch.h"

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  

int state = 0;

uint16_t getX();
uint16_t getY();
int median(int[] arr, int n);
int compare (const void * a, const void * b);

void main(){
	// LCD init
	__C30_UART=1;
	lcd_initialize();
	lcd_clear();
	lcd_locate(0,0);

	// ==== LED init
	led_initialize();
	CLEARLED(LED1_PORT);
	CLEARLED(LED2_PORT);

        // ADC init
        //disable ADC
        CLEARBIT(AD2CON1bits.ADON);
        //initialize PIN
        SETBIT(TRISBbits.TRISB4); //set TRISE RB4 to input
        SETBIT(TRISBbits.TRISB5); //set TRISE RB5 to input
        SETBIT(AD2PCFGLbits.PCFG4); // RB4 analog ADC2CH4
        SETBIT(AD2PCFGLbits.PCFG5); // RB5 analog ADC2CH5
        //Configure AD2CON1
        SETBIT(AD2CON1bits.AD12B); //set 12b Operation Mode
        AD2CON1bits.FORM = 0; //set integer output
        AD2CON1bits.SSRC = 0x7; //set automatic conversion
        //Configure AD1CON2
        AD2CON2 = 0; //not using scanning sampling
        //Configure AD1CON3
        CLEARBIT(AD2CON3bits.ADRC); //internal clock source
        AD2CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
        AD2CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
        //Leave AD1CON4 at its default value
        //enable ADC
        SETBIT(AD2CON1bits.ADON);

	// ==== button 1/external int 1 init
	AD1PCFGHbits.PCFG20 = 1; 
	TRISEbits.TRISE8 = 1;
	INTCON2bits.INT1EP = 1; // polarity selection, '1' falling edge triggering of the interrupt.
	IPC5bits.INT1IP = 0x01;
	IFS1bits.INT1IF = 0;
	IEC1bits.INT1IE = 1;
        PrevStat = PORTEbits.RE8;

        touch_init();

        uint16_t xVal = 0;
        uint16_t xValMin = 0;
        uint16_t xValMax = 0;

        uint16_t yVal = 0;
        uint16_t yValMin = 0;
        uint16_t yValMax = 0;


        while(1);

}

uint16_t getX(void){
    AD2CHS0bits.CH0SA = 0x004; //set ADC to Sample AN4 pin
    SETBIT(AD2CON1bits.SAMP); //start to sample
    while(!AD2CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD2CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC2BUF0; //return sample
}
uint16_t getY(void){
    AD2CHS0bits.CH0SA = 0x005; //set ADC to Sample AN5 pin
    SETBIT(AD2CON1bits.SAMP); //start to sample
    while(!AD2CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD2CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC2BUF0; //return sample
}

int median(int[] arr, int n){
	//  assume n is odd
	qsort(arr, n, sizeof(int), compare);
	return arr[n+1/2]; 
}

int compare (const void * a, const void * b){
	return ( *(int*)a - *(int*)b );
}
