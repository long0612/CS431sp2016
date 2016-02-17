#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>

#include "lcd.h"
#include "led.h"
#include "flexserial.h"
#include "crc16.h"

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  

int PrevStat = 0;
uint32_t globalClock = 0; // ms

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

	// ==== UART 2 init
	uart2_init(9600);

	uint32_t count = 0;
	uint32_t min = 0;
	uint32_t sec = 0;
	uint32_t msec = 0;
	uint32_t T3Start = 0;
	uint32_t elapsedTime = 0;

 	while(1){
            
	}
}

void uart2_init(uint16_t baud){
	// ==== Stop UART port
	CLEARBIT(U1MODEbits.UARTEN); //Disable UART for configuration
	// Disable Interrupts
	IEC0bits.U1RXIE = 0;
	IEC0bits.U1TXIE = 0;
	// Clear Interrupt flag bits
	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;
	// Set IO pins
	TRISFbits.TRISF2 = 1; //set as input UART1 RX pin
	TRISFbits.TRISF3 = 0; //set as output UART1 TX pin
	// baud rate
	// use the following equation to compute the proper
	// setting for a specific baud rate
	U1MODEbits.BRGH = 0; //Set low speed baud rate
	U1BRG = (uint32_t)800000 / 9600 -1; //Set the baudrate to be at 9600
	// Operation settings and start port
	U1MODE = 0; // 8-bit, no parity and, 1 stop bit
	U1MODEbits.RTSMD = 0; //select simplex mode
	U1MODEbits.UEN = 0; //select simplex mode
	U1MODE |= 0x00;
	U1MODEbits.UARTEN = 1; //enable UART
	U1STA = 0;
	U1STAbits.UTXEN = 1; //enable UART TX
}

void uart2_putc(uint8_t c){

}

uint8_t uart2_getc(){

}
