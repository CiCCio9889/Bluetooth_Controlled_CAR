#include "Initial_PIN_Setting.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

void SET_PIN(){
     DDRD|=(1<<DDD6);          //PIN6 (PWM) for motor connected to A port of DRIVER
     DDRD|=(1<<DDD5);          //PIN5 (PWM) for motor connected to B port of DRIVER
     DDRD|=(1<<DDD4);          //PIN4 as digital output for direction of motor connected to A port of DRIVER
     DDRD|=(1<<DDD2);          //PIN2 as digital output for direction of motore connected to B port of DRIVER
     DDRD|=(1<<DDD7);          //PIN7 as OUTPUT TRIG (ultrasonic sensor)
     DDRB&=~(1<<DDB0);         //PIN8 as input ECHO  (ultrasonic sensor)
     DDRB&=~(1<<DDB4);         //PIN12 as INPUT LEFT (line tracking sensor) --> 1 is black and 0 is white
     DDRB&=~(1<<DDB5);         //PIN13 as INPUT CENTER (line tracking sensor) --> 1 is black and 0 is white
     DDRB&=~(1<<DDB3);         //PIN11 as INPUT RIGHT (line tracking sensor) --> 1 is black and 0 is white
     PORTD&=~(1<<PORTD4);      //PIN 4 LOW
     PORTD&=~(1<<PORTD2);      //PIN 2 LOW
     PORTD&=~(1<<PORTD7);      //PIN 7 as low to avoid the default set to HIGH
     PORTB&=~(1<<PORTB0);      //PIN 8 as low to avoid the default set to HIGH
     PORTB&=~(1<<PORTB4);      //PIN 12 as low to avoid the default set to HIGH
     PORTB&=~(1<<PORTB5);      //PIN 13 as low to avoid the default set to HIGH
     PORTB&=~(1<<PORTB3);      //PIN 11 as low to avoid the default set to HIGH
}

