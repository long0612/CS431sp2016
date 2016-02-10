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

	// ==== button 1/external int 1 init
	AD1PCFGHbits.PCFG20 = 1; 
	TRISEbits.TRISE8 = 1;
	INTCON2bits.INT1EP = 1; // polarity selection, '1' falling edge triggering of the interrupt.
	IPC5bits.INT1IP = 0x01;
	IFS1bits.INT1IF = 0;
	IEC1bits.INT1IE = 1;
        PrevStat = PORTEbits.RE8;

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
	IPC1bits.T2IP = 0x01; //Not IPC0bits
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1;
	T2CONbits.TON = 1;

	// ==== timer 3 init
	T3CONbits.TON = 0;
	T3CONbits.TCS = 0; // Select internal clock
	T3CONbits.TGATE = 0; // Disable Gated Timer mode
	T3CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
	TMR3 = 0x00; // Clear timer register
	PR3 = 65535; // Load the period value
	T3CONbits.TON = 1;

	uint32_t count = 0;
	uint32_t min = 0;
	uint32_t sec = 0;
	uint32_t msec = 0;
	uint32_t T3Start = 0;
	uint32_t elapsedTime = 0;
        lcd_locate(0,0);
        lcd_printf("00:00:000\n");
 	while(1){
	    T3Start = TMR3;

	    count++;

	    TOGGLELED(LED4_PORT);

	    if (count == 2000){
		min = globalClock/60000;
		sec = (globalClock % 60000)/1000;
		msec = (globalClock % 60000) % 1000;/*
                sec = globalClock / 1000;
                min = sec /60;
                msec = globalClock;*/

            	lcd_locate(0,0);
            	lcd_printf("%02ld:%02ld:%3ld\n",min,sec,msec);
                //lcd_clear_row(1);
            	//lcd_locate(0,1);
            	//lcd_printf("%ld\n",globalClock);
                //SETLED(LED5_PORT);
                count = 0;
	    }
/*
	    if (TMR3 > T3Start){

	    	elapsedTime = TMR3-T3Start;
	    }else{
	    	elapsedTime = TMR3+65536-T3Start;
	    }
            if (count == 1000){
                lcd_locate(0,3);
                //lcd_printf("%ld\n %.4f\n",elapsedTime, elapsedTime/12800000);
                lcd_printf("%d\n",TMR3);
            }
 * */
            if (count == 1500){
                if (TMR3 > T3Start){
                     lcd_locate(0,3);
                    //lcd_printf("%ld\n %.4f\n",elapsedTime, elapsedTime/12800000);
                     //lcd_clear_row(3);
                    lcd_printf("%04ld\n",TMR3-T3Start);
                    lcd_printf("%04g\n",(TMR3-T3Start)/12800000.0);
                } else {
                     lcd_locate(0,3);
                     //lcd_clear_row(3);
                    //lcd_printf("%ld\n %.4f\n",elapsedTime, elapsedTime/12800000);
                    lcd_printf("%04ld\n",(65536-T3Start+TMR3));
                    lcd_printf("%04g\n",(65536-T3Start+TMR3)/12800000.0);
                }
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

//int DBNOcount = 0;

void __attribute__ ((__interrupt__)) _INT1Interrupt(void){
	IFS1bits.INT1IF = 0;
        uint32_t DBcount = 0;
        uint32_t Tcount = 0;

        /*if(Tcount ==  2500){
            if(DBcount > 250){
                                    // state changed declared
                //if(PrevStat != PORTEbits.RE8)
                DBNOcount++;
                PrevStat = PORTEbits.RE8;
                globalClock = 0;
            }
            Tcount = 0;
            DBcount = 0;
        } else if ((PORTEbits.RE8 == PrevStat) && (Tcount != 0)) {
            Tcount++;
        } else if (PORTEbits.RE8 != PrevStat) {
            DBcount++;
            Tcount++;
        }*/
        for (Tcount = 0; Tcount < 2500; Tcount ++ ){
            if (PORTEbits.RE8 != PrevStat)
                DBcount++;
        }


        if(DBcount > 250){
                                // state changed declared
            //if(PrevStat != PORTEbits.RE8)
            PrevStat = PORTEbits.RE8;
            globalClock = 0;
        }
        Tcount = 0;
        DBcount = 0;
}

