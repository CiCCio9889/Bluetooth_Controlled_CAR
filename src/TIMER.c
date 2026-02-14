#include "TIMER.h"
#include "AEB.h"
#include "Costant.h"
#include "DIRECTION_HANDLER.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>


volatile uint32_t cont=0;

//TIMER1 IS USED TO CALCULATE THE DISTANCE FROM OBJECT FOR AEB
void set_TIMER1(){
    TCCR1A=0;                  
    TCCR1B=0;
    TCCR1B|=(1<<ICES1)|(1<<CS11);
    TIMSK1|=(1<<ICIE1);
    TCNT1=0;
}

//TIMER0 IS USED TO GENERATE PWM SIGNAL FOR DC MOTORS
void Set_PWM(){
    TCCR0A = (1 << COM0A1) | (1<<COM0B1) | (1 << WGM00)|(1<<WGM01);
    TCCR0B = (1<<CS00)|(1<<CS01); // Fast PWM 8-bit, prescaler 64
    TIMSK0 |= (1 << TOIE0);   // enable interrupt overflow Timer0
}

// FUNCTION THAT RETURN ACTUAL TIME
uint32_t cont_temp(){
    uint8_t sreg=SREG;
    uint32_t temp;
    cli();
    temp=cont;
    SREG=sreg;
    return temp;
}

// ISR TIMER0 OVERFLOW used to create pulse wave on PIN7 (PIN TRIG OF ULTRASONIC SENSOR)
ISR(TIMER0_OVF_vect){
    static uint8_t trig_state;
    static uint32_t cont_trig;
    cont++;
    cont_trig++;
    if (trig_state == 0) { 
     if (cont_trig >= 60) {           
         PORTD |= (1<<PORTD7);      
         trig_state = 1;
         cont_trig = 0;
        }
    }
    else if (trig_state == 1) {
     if (cont_trig >= 1) {            
         PORTD &= ~(1<<PORTD7);     
         trig_state = 0;
         cont_trig = 0;
        }
    }

}

//ISR TIMER1 used to calculate the duration of pulse on PIN trig of ultasonic sensor and so the distance from the OBJECT
ISR(TIMER1_CAPT_vect){
    static uint16_t start_time;
    static uint16_t stop_time;
    
    if (TCCR1B&(1<<ICES1)){
     start_time=ICR1;
     TCCR1B&=~(1<<ICES1);
    }
    else{
     stop_time=ICR1;
     TCCR1B|=(1<<ICES1);
     uint16_t distance=(uint16_t)((stop_time-start_time)/116);
     if ((distance<=Braking_distance()&&distance>0)&&(MODE_DRIVE==AEB_MODE)&&(FLAG_DIRECTION==FORWARD)){  //If the direction is forward and the distance is <= of braking distance the car must have to STOP
         OCR0A=PWM_MAX-10;
         OCR0B=PWM_MAX-10;
         FLAG_WARNING=1;
        }
    }


}




