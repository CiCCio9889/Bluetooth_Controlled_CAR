#include "line_tracking_sensor.h"
#include "Costant.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//The value returned by function is used to decided if the turn is needed or not
uint8_t line_sensor_tracking_status(){
     if((PINB & (1<<PINB4))&&(!(PINB & (1<<PINB5)))&&(!(PINB & (1<<PINB3)))){
         return LEFT_1;                                                                       
        }
     else if((PINB & (1<<PINB4))&&(PINB & (1<<PINB5))&&(!(PINB & (1<<PINB3)))){
         return LEFT_1; 
        }
     else if((!(PINB & (1<<PINB4)))&&(!(PINB & (1<<PINB5)))&&(PINB & (1<<PINB3))){
         return RIGHT_1;     
        }
     else if ((!(PINB & (1<<PINB4)))&&(PINB & (1<<PINB5))&&(PINB & (1<<PINB3))){
         return RIGHT_1;
        }
     else if ((PINB & (1<<PINB4))&&(PINB & (1<<PINB5))&&(PINB & (1<<PINB3))){
         return CENTERED;
        }
     else if (!(PINB & (1<<PINB4))&&(PINB & (1<<PINB5))&&!(PINB & (1<<PINB3))){
         return CENTERED;
        }
     else{
         return 0;
        }
 }
