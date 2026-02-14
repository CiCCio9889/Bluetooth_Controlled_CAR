#include "DIRECTION_HANDLER.h"
#include "Costant.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//The manage of the steering is done with the Differential Steering
//When the curve is finished a function to come back gradually to the initial velocity is used to avoid a direct change of PWM value
void function_curve_sx(uint8_t *val1,uint8_t *val2){  
     if(FLAG_DIRECTION==BACKWARD){
         if(*val1>PWM_UPPER_LIMIT&&*val2>=PWM_LOWER_LIMIT){
         *val2=*val2-PWM_STEP;
         OCR0B=*val2;
        }
        if(*val2<PWM_LOWER_LIMIT&&*val1<=PWM_UPPER_LIMIT){
         *val1=*val1+PWM_STEP;
         OCR0A=*val1;
        } 
        if(*val1<=PWM_UPPER_LIMIT&&*val2>=PWM_LOWER_LIMIT){
         *val1=*val1+PWM_STEP;
         *val2=*val2-PWM_STEP;
         OCR0A=*val1;
         OCR0B=*val2;
        }
    }
     if(FLAG_DIRECTION==FORWARD){
         if(*val1>PWM_UPPER_LIMIT&&*val2>=PWM_LOWER_LIMIT){
             *val2=*val2-PWM_STEP;
             OCR0B=PWM_MAX-*val2;
            }
         if(*val2<PWM_LOWER_LIMIT&&*val1<=PWM_UPPER_LIMIT){
             *val1=*val1+PWM_STEP;
             OCR0A=PWM_MAX-*val1;
            }
         if(*val1<=PWM_UPPER_LIMIT&&*val2>=PWM_LOWER_LIMIT){
             *val1=*val1+PWM_STEP;
             *val2=*val2-PWM_STEP;
             OCR0A=PWM_MAX-*val1;
             OCR0B=PWM_MAX-*val2;
            }

    }

}

void function_curve_dx(uint8_t *val1,uint8_t *val2){
     if(FLAG_DIRECTION==BACKWARD){
         if(*val2>PWM_UPPER_LIMIT&&*val1>=PWM_LOWER_LIMIT){
             *val1=*val1-PWM_STEP;
             OCR0A=*val1;
            }
         if(*val1<PWM_LOWER_LIMIT&&*val2<=PWM_UPPER_LIMIT){
             *val2=*val2+PWM_STEP;
             OCR0B=*val2;
            }
         if(*val2<=PWM_UPPER_LIMIT&&*val1>=PWM_LOWER_LIMIT){
             *val2=*val2+PWM_STEP;
             *val1=*val1-PWM_STEP;
             OCR0B=*val2;
             OCR0A=*val1;
            }

    }
     if (FLAG_DIRECTION==FORWARD){
         if(*val2>PWM_UPPER_LIMIT&&*val1>=PWM_LOWER_LIMIT){
             *val1=*val1-PWM_STEP;
             OCR0A=PWM_MAX-*val1;
            }
         if(*val1<PWM_LOWER_LIMIT&&*val2<=PWM_UPPER_LIMIT){
             *val2=*val2+PWM_STEP;
             OCR0B=PWM_MAX-*val2;
            }
         if(*val2<=PWM_UPPER_LIMIT&&*val1>=PWM_LOWER_LIMIT){
             *val2=*val2+PWM_STEP;
             *val1=*val1-PWM_STEP;
             OCR0B=PWM_MAX-*val2;
             OCR0A=PWM_MAX-*val1;
            }


    }
}

void after_curve_sx(uint8_t *val1,uint8_t *val2,uint8_t *val3){
     if(FLAG_DIRECTION==BACKWARD){
         if(*val1>*val3){
             *val1=*val1-BACKWARD;
            }
         else{
             *val1=*val3;
            }
         if(*val2<*val3){
             *val2=*val2+PWM_STEP;
            }
         else{
             *val2=*val3;
            }

    }
     if(FLAG_DIRECTION==FORWARD){
         if(*val1>(PWM_MAX-*val3)){
             *val1=*val1-PWM_STEP;
            }
         else{
             *val1=PWM_MAX-*val3;
            }
         if(*val2<(PWM_MAX-*val3)){
             *val2=*val2+PWM_STEP;
            }
         else{
             *val2=PWM_MAX-*val3;
            }

    }
}

void after_curve_dx(uint8_t *val1,uint8_t *val2,uint8_t *val3){
     if(FLAG_DIRECTION==BACKWARD){
         if(*val2>*val3){
             *val2=*val2-PWM_STEP;
            }
         else{
             *val2=*val3;
            }
         if(*val1<*val3){
             *val1=*val1+PWM_STEP;
            }
         else{
             *val1=*val3;
            }

    }

     if(FLAG_DIRECTION==FORWARD){
         if(*val2>(PWM_MAX-*val3)){
             *val2=*val2-PWM_STEP;
            }
         else{
             *val2=PWM_MAX-*val3;
            }
         if(*val1<(PWM_MAX-*val3)){
             *val1=*val1+PWM_STEP;
            }
         else{
             *val1=PWM_MAX-*val3;
            }
    }
    
}



