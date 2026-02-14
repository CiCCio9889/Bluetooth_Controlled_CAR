#include "DIRECTION_HANDLER.h"
#include "Costant.h"
#include "USART.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

volatile uint8_t FLAG_DIRECTION=0;
volatile uint8_t MODE_DRIVE=0;
volatile uint8_t FLAG_WARNING=0;
uint8_t MODE_DRIVE_LINE=0;
uint8_t FLAG_command_acceleration=0;


// Function to set direction 'F', 'B', 'L', 'R'
// Global variable FLAG_DIRECTION is used to store direction and so if it is 1 direction is needed to set
// The driver used "Keystudio 8833 Motor Driver Expansion Board" set invert PWM when direction is FORWARD so to set PWM_MIN (15) you need to set PWM_MAX-PWM_MIN (255-15) 
void set_direction(){
  uint8_t buffer=0;
  buffer=receiveByte1();
  if (buffer==stop){                                           //s in ASCII code
     FLAG_DIRECTION=1;
    }
  if (buffer==STOP){                                           // S in ASCII code
     FLAG_DIRECTION=1;
    }

  if (buffer==AEB_MODE){                                       //Y in ASCII code
     MODE_DRIVE=AEB_MODE;
     FLAG_DIRECTION=1;
    }
  if (buffer==NORMAL_MODE){                                    //t in ASCII code
     MODE_DRIVE=NORMAL_MODE;
     MODE_DRIVE_LINE=NORMAL_MODE;
     FLAG_WARNING=0;
     FLAG_DIRECTION=1;
    }
  if (buffer==LINE_TRACKING_MODE){                             //X in ASCII code
     MODE_DRIVE_LINE=LINE_TRACKING_MODE;  
     FLAG_DIRECTION=1; 
    }
  if (buffer==FORWARD){                                        //'F' in ASCII code
     PORTD|=(1<<PORTD4);  //PIN4 to HIGH
     PORTD|=(1<<PORTD2);  //PIN2 to HIGH
     OCR0A=PWM_MAX-PWM_MIN;
     OCR0B=OCR0A;
     FLAG_DIRECTION=FORWARD;
    }
  if (buffer==BACKWARD){                                       //'B' in ASCII code
     FLAG_WARNING=0;
     FLAG_DIRECTION=BACKWARD;
     PORTD&=~(1<<PORTD4);  // PIN 4 LOW
     PORTD&=~(1<<PORTD2);
     OCR0A=PWM_MIN;
     OCR0B=OCR0A;
     
    }
  if (buffer==LEFT){                                           //'L' in ASCII code
     FLAG_WARNING=0;
     FLAG_DIRECTION=LEFT;
     PORTD|=(1<<PORTD4);  //PIN4 to HIGH
     PORTD&=~(1<<PORTD2);  //PIN2 to LOW
     OCR0A=PWM_MAX-PWM_MIN;
     OCR0B=PWM_MIN;
     PORTB&=~(1<<PORTB0);
    }
  if (buffer==RIGHT){                                          //'R' in ASCII code
     FLAG_WARNING=0;
     FLAG_DIRECTION=RIGHT;
     PORTD&=~(1<<PORTD4);  // PIN4 to LOW
     PORTD|=(1<<PORTD2);  //PIN2 to HIGH
     OCR0A=PWM_MIN;
     OCR0B=PWM_MAX-PWM_MIN;
     PORTB&=~(1<<PORTB0);
    }

  if (buffer==ACCELERATOR_PEDAL){                              //'a' in ASCII code
     if (FLAG_WARNING==0){
         FLAG_command_acceleration=1;
        }
     FLAG_DIRECTION=actual_flag_direction;
     if (FLAG_DIRECTION==FORWARD){
         OCR0A=PWM_MAX-PWM_MIN;
         OCR0B=OCR0A;
        }
     if (FLAG_DIRECTION==BACKWARD){
         OCR0A=PWM_MIN;
         OCR0B=PWM_MIN;
        }
     if (FLAG_DIRECTION==LEFT){
         OCR0A=PWM_MAX-PWM_MIN;
         OCR0B=PWM_MIN;
        }
     if (FLAG_DIRECTION==RIGHT){
         OCR0A=PWM_MIN;
         OCR0B=PWM_MAX-PWM_MIN;
        }
    }
    if (buffer!=FORWARD&&buffer!=BACKWARD&&buffer!=LEFT&&buffer!=RIGHT&&(buffer==ACCELERATOR_PEDAL&&FLAG_DIRECTION==0)){
     FLAG_DIRECTION=1;
    }
}




