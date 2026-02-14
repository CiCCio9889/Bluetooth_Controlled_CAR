#include "AEB.h"
#include "Costant.h"
#include "DIRECTION_HANDLER.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>




//SET the intervention distance of AEB based on different value of speed
uint8_t Braking_distance(void){     
    uint8_t distance;
    if(OCR0A>=(PWM_MAX-80)){
         distance=low_speed_distance;     
    }
    if(OCR0A<(PWM_MAX-80)&&OCR0A>(PWM_MAX-150)){
         distance=low_medium_speed_distance;    
    }
    if(OCR0A<=(PWM_MAX-150)&&OCR0A>(PWM_MAX-220)){
         distance=medium_speed_distance;   
    }
    if(OCR0A<=(PWM_MAX-220)){
         distance=fast_speed_distance;      
    }
    return (distance);
}






