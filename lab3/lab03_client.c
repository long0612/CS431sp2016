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

bool isCorrupt = false;

void main(){
	// LCD init
	__C30_UART=1;
	lcd_initialize();
	lcd_clear();
	lcd_locate(0,0);

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

	// ==== UART 2 init
	uart2_init(9600);

	uint8_t c;	
	uint16_t rCRC; // remote CRC
	uint16_t lCRC = 0; // local CRC
	uint16_t i = 0;
	uint16_t N;

 	while(1){
		c = uart2_getc();
		if (c != MSG_START && lCRC == 0){
			continue;
		}else {
			lCRC = crc_update(lCRC,c);

			if (i == 0){
				T1CONbits.TON = 1; // Start Timer
			}else if (i == 1){
      			rCRC |= c << 8;
			}else if (i == 2){
 			    rCRC |= c;
			}else if (i == 3){
				N = c;
			}else if (i >= 4 && i < N+3){
				if (isCorrupt){
					// reset on corrupt data
					lCRC = 0;
					i = 0;
				}
			}else if (i == N+3){
				// reset on done
				if (rCRC == lCRC){
				}
				lCRC = 0;
				i = 0;
					
			}

			i += 1;
		}
	}
}

void __attribute__ ((__interrupt__)) _T1Interrupt(void){
	IFS0bits.T1IF = 0; // clear the interrupt flag
	
	isCorrupt = true;
}
