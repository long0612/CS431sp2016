#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include "lcd.h"

#include "types.h"

void motor_init(uint8_t chan){
    //setup Timer 2
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
    PR2 = 4000; // Set timer period 20ms: 4000= 20*10^-3 * 12.8*10^6 * 1/64

    if (chan == 0){ // x
        //setup OC8
        CLEARBIT(TRISDbits.TRISD7); /* Set OC8 as output */
    }else{ // y
        //setup OC7
        CLEARBIT(TRISDbits.TRISD6); /* Set OC7 as output */
    }
}

void motor_set_duty(uint8_t chan, uint16_t duty_us){
    double duty_ms = duty_us/1000.0;
    if (chan == 0){ // x
        OC8R = duty_ms*200; /* Set the initial duty cycle to 5ms*/
        //OC8RS = duty_ms*200; /* Load OCRS: next pwm duty cycle */
        OC8CON = 0x0006; /* Set OC8: PWM, no fault check, Timer2 */
        SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
    }else{
        OC7R = duty_ms*200; /* Set the initial duty cycle to 5ms*/
        OC7RS = duty_ms*200; /* Load OCRS: next pwm duty cycle */
        OC7CON = 0x0006; /* Set OC7: PWM, no fault check, Timer2 */
        SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
    }
}