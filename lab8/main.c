#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL
#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>

#include "lcd.h"
#include "led.h"
#include "flextouch.h"
#include "flexmotor.h"

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT);

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);

int median(uint16_t* arr, int n);
int compare (const void * a, const void * b);
void delay(uint16_t delay);

int start = 0;
uint16_t xVal[] = {0,0,0,0,0};
uint16_t yVal[] = {0,0,0,0,0};
uint16_t idx = 0;
double dutyX = 0.9;
double dutyY = 0.9;

void main(){
	// LCD init
	__C30_UART=1;
	lcd_initialize();
	lcd_clear();

	// ==== LED init
	led_initialize();
	CLEARLED(LED1_PORT);
	CLEARLED(LED2_PORT);

        // ==== LPOSCEN init
	__builtin_write_OSCCONL(OSCCONL | 2);
	// ==== timer 1 init
	T1CONbits.TON = 0; //Disable Timer
	T1CONbits.TCS = 0; //Select external clock
	T1CONbits.TSYNC = 0; //Disable Synchronization
	T1CONbits.TCKPS = 0b10; //Select 1:64 Prescaler
	TMR1 = 0x00; // Clear timer register
	PR1 = 10000; // Load the period value, 50 ms
	IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
	IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1; // Enable Timer1 interrupt
	T1CONbits.TON = 1; // Start Timer

	// ==== touch init
        touch_init();

	// ==== motor init
	motor_init(0);
	motor_init(1);

	// initial y position
        motor_set_duty(0, 0.9);
	motor_set_duty(1, 0.9);

        SETLED(LED1_PORT);

        int xMedian = 0;
        int yMedian = 0;
        int count = 0;
        double prevErrX = 0;
        double prevErrY = 0;
        double errorX = 0;
        double errorY = 0;
        double derivativeX = 0;
        double derivativeY = 0;
        double integralX = 0;
        double integralY = 0;
        double outputX = 0;
        double outputY = 0;
        double dt = 0.05; // second
        double setPointX = (3135.0+265.0)/2.0;
        double setPointY = (2358.0+510.0)/2.0;
        double KpX = 1.8, KiX = 0.0, KdX = 0.9;
        double KpY = 1.8, KiY = 0.0, KdY = 0.7;

        while(1){
            while(!start);
            start = 0;

            // compute the median
            xMedian = median(xVal,5);
            yMedian = median(yVal,5);

            // perform PID computation
            errorX = setPointX - xMedian;
            integralX = integralX + errorX*dt;
            derivativeX = (errorX - prevErrX)/dt;
            outputX = KpX*errorX + KiX*integralX + KdX*derivativeX;
			outputX = cap(outputX,14000.0,-11000.0);
            dutyX = (outputX + 11000.0)/(14000.0+11000.0)*1.2 + 0.9;
            prevErrX = errorX;

            errorY = setPointY - yMedian;
            integralY = integralY + errorY*dt;
            derivativeY = (errorY - prevErrY)/dt;
            outputY = KpY*errorY + KiY*integralY + KdY*derivativeY;
			outputY = cap(outputY,9500.0,-7000.0);
            dutyY = (outputY + 7000.0)/(9500.0+7000.0)*1.2 + 0.9;
            prevErrY = errorY;

            //update display value
            if (count == 20){
                lcd_clear();
                /*
                lcd_locate(0,0);
                lcd_printf("PID: %.2f,%.2f,%.2f",KpX,KiX,KdX);
                lcd_locate(0,1);
                lcd_printf("xm: %4d, ym: %4d",xMedian,yMedian);
                lcd_locate(0,2);
                lcd_printf("int: %.2f,%.2f",integralX,integralY);
                */
                lcd_locate(0,3);
                lcd_printf("der: %.2f,%.2f",derivativeX,derivativeY);
                lcd_locate(0,4);
                lcd_printf("err: %.2f,%.2f",errorX,errorY);
                lcd_locate(0,5);
                lcd_printf("F_x: %.2f,%.2f",outputX,outputY);
                lcd_locate(0,6);
                lcd_printf("duty: %.3f,%.3f",dutyX, dutyY);
                //lcd_locate(0,7);
                //lcd_printf("SP: %.2f,%.2f", setPointX, setPointY);
                count = 0;
            }
            count++;
        }

}

void __attribute__ ((__interrupt__)) _T1Interrupt(void){
    int i = 0;

    IFS0bits.T1IF = 0;

    TOGGLELED(LED2_PORT);

    // read the ball position
    touch_select_dim(1); // x
    delay(100000);
	/*
    for (i = 0; i < 5; i++){
        xVal[i] = touch_adc(1);
    }
	*/
    xVal[idx] = touch_adc(1);

    touch_select_dim(2); // y
    delay(100000);
	/*
    for (i = 0; i < 5; i++){
        yVal[i] = touch_adc(2);
    }
	*/
    yVal[idx] = touch_adc(2);
	
	if (idx>=4){
		idx = 0;
	}else{
		idx++;
	}

    // Do control here
	dutyX = cap(dutyX,2.1,0.9);
    motor_set_duty(0, dutyX);
    
	dutyY = cap(dutyY,2.1,0.9);
    motor_set_duty(1, dutyY);
    //motor_set_duty(1, 2.1);
    /*if(duty>=0.9 && duty<=2.1){
        motor_set_duty(0, duty);
    }*/
    // allow PID computation
    start = 1;
}

double cap(double in, double up, double low){
	if (in > up){
		return up
	} elseif (in < low){
		return low;
	} else{
		return in;
	}
}

int median(uint16_t* arr, int n){
    //  assume n is odd
    qsort(arr, n, sizeof(uint16_t), compare);
    return arr[(n+1)/2];
}

int compare (const void * a, const void * b){
	return ( *(uint16_t*)a - *(uint16_t*)b );
}

void delay(uint16_t delay){
    uint16_t i;
    for (i = 0; i < delay; i++);
}