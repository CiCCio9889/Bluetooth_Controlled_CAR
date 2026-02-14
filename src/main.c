#include "State_Machine.h"
#include "motion.h"
#include "line_tracking_sensor.h"
#include "Costant.h"
#include "AEB.h"
#include "USART.h"
#include "DIRECTION_HANDLER.h"
#include "steering_handler.h"
#include "Initial_PIN_Setting.h"
#include "TIMER.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

STATUS Current_Status=STATE1;
uint8_t actual_flag_direction=0;

int main(){
     sei();
     set_BAUDRATE(BAUDRATE);
     SET_PIN();
     Set_PWM();
     set_TIMER1();
     OCR0A=38;    //initial value to PWM
     OCR0B=38;
     while(1){
         switch (Current_Status){
             case STATE1:
                 delete_buffer();
                 set_direction();
                 _delay_ms(20);
                 if (FLAG_DIRECTION!=1&&FLAG_WARNING==0){     
                     Current_Status=STATE2;                 //I go in STATE2 when direction is setted 
                    }
       
                break;

             case STATE2:
                 ramp_velocity_FB();
                 if((OCR0A<=14&&FLAG_DIRECTION==66)||(OCR0A>=241&&FLAG_DIRECTION==70)||(OCR0A<=14&&FLAG_DIRECTION==82)||(OCR0A>=241&&FLAG_DIRECTION==76)){ // I come back to STATE1 when i press brake pedal and so velocity of car is 0 m/s
                     Current_Status=STATE1;
                     actual_flag_direction=FLAG_DIRECTION;
                    }
                break;
            }
        }
}
