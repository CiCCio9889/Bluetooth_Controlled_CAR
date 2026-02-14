#include "motion.h"
#include "line_tracking_sensor.h"
#include "Costant.h"
#include "AEB.h"
#include "USART.h"
#include "DIRECTION_HANDLER.h"
#include "steering_handler.h"
#include "TIMER.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//HANDLE MOTION OF CAR 
void ramp_velocity_FB(){ 
  uint8_t buffer_2=0;
  uint8_t valueA=0;
  uint8_t valueB=0;
  uint8_t actual_duty=0;
  uint8_t FLAG_line_sensor=0;
  uint32_t time=0;
  valueA=OCR0A;
  valueB=OCR0B;
  actual_duty=OCR0A;
  if(FLAG_WARNING==0){                      
     if(FLAG_command_acceleration==0){
         buffer_2=receiveByte_NO_BLOCKING();
        }
     if (FLAG_command_acceleration==1){
         buffer_2=ACCELERATOR_PEDAL;
         FLAG_command_acceleration=0;
        }
  
     if (buffer_2==LEFT){                                 //HANDLE LEFT TURN WHILE I'M GOING FORWARD OR BACKWARD
         delete_buffer();
         if (FLAG_DIRECTION==BACKWARD){
             buffer_2=receiveByte(TIMEOUT_USART);
             while(buffer_2==0){
                 AEB_BREAK();
                 function_curve_sx(&valueA,&valueB);
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                 }
             while(valueA!=actual_duty||valueB!=actual_duty){
                 AEB_BREAK();
                 after_curve_sx(&valueA,&valueB,&actual_duty);
                 OCR0A=valueA;
                 OCR0B=valueB;
                 _delay_ms(1);
                }

            }
         if(FLAG_DIRECTION==FORWARD){
             buffer_2=receiveByte(TIMEOUT_USART);
             valueA=PWM_MAX-valueA;
             valueB=PWM_MAX-valueB;
             while(buffer_2==0){
                 AEB_BREAK();
                 function_curve_sx(&valueA,&valueB);
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }

             while(valueA!=(PWM_MAX-actual_duty)||valueB!=(PWM_MAX-actual_duty)){
                 AEB_BREAK();
                 after_curve_sx(&valueA,&valueB,&actual_duty);
                 OCR0A=PWM_MAX-valueA;
                 OCR0B=PWM_MAX-valueB;
                 _delay_ms(1);
                }
 
            }

        } 

     if(buffer_2==RIGHT){                            //HANDLE RIGHT TURN WHILE I'M GOING FORWARD OR BACKWARD
         delete_buffer();
         if(FLAG_DIRECTION==BACKWARD){
             buffer_2=receiveByte(TIMEOUT_USART);
             while(buffer_2==0){
                 AEB_BREAK();
                 function_curve_dx(&valueA,&valueB);
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
             while(valueA!=actual_duty||valueB!=actual_duty){
                 AEB_BREAK();
                 after_curve_dx(&valueA,&valueB,&actual_duty);
                 OCR0A=valueA;
                 OCR0B=valueB;
                 _delay_ms(1);
                }
            }
         if(FLAG_DIRECTION==FORWARD){
             buffer_2=receiveByte(TIMEOUT_USART);
             valueA=PWM_MAX-valueA;
             valueB=PWM_MAX-valueB;
             while(buffer_2==0){
                 AEB_BREAK();
                 function_curve_dx(&valueA,&valueB);
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
             while(valueA!=(PWM_MAX-actual_duty)||valueB!=(PWM_MAX-actual_duty)){
                 AEB_BREAK();
                 after_curve_dx(&valueA,&valueB,&actual_duty);
                 OCR0A=PWM_MAX-valueA;
                 OCR0B=PWM_MAX-valueB;
                 _delay_ms(1);
                }
            }
        }

     if(buffer_2==ACCELERATOR_PEDAL){        //HANDLE THE ACCELERATION OF CAR IN FORWARD - BACKWARD DIRECTION AND WHILE DRIFTING
         delete_buffer();
         if(FLAG_DIRECTION==BACKWARD){
             buffer_2=receiveByte(TIMEOUT_USART);
             while(buffer_2==0&&valueA<=PWM_UPPER_LIMIT){
                 AEB_BREAK();
                 valueA=valueA+PWM_STEP;
                 OCR0A=valueA;
                 OCR0B=valueA;
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
            }
         if (FLAG_DIRECTION==FORWARD){               
             buffer_2=receiveByte(TIMEOUT_USART);
             valueA=PWM_MAX-valueA;
             valueB=PWM_MAX-valueB;
             while(buffer_2==0&&valueA<=PWM_UPPER_LIMIT){
                 AEB_BREAK();
                 valueA=valueA+PWM_STEP;
                 valueB=valueB+PWM_STEP;
                 OCR0A=PWM_MAX-valueA;
                 OCR0B=PWM_MAX-valueA;
                 actual_duty=OCR0A;
                 if(MODE_DRIVE_LINE==LINE_TRACKING_MODE){                     //IF IN TRACKING MODE I'M ABLE TO FOLLOW THE LINE WHILE I'M GOING FORWARD AND I'M ACCELERATING
                     time=cont_temp();
                     while(line_sensor_tracking_status()!=CENTERED&&((cont_temp()-time)<=TIMEOUT_LINE_SENSOR)){
                         AEB_BREAK();
                         if(line_sensor_tracking_status()==LEFT_1){
                             function_curve_sx(&valueA,&valueB);
                             FLAG_line_sensor=LEFT_1;
                            }
                         if(line_sensor_tracking_status()==RIGHT_1){
                             function_curve_dx(&valueA,&valueB);
                             FLAG_line_sensor=RIGHT_1;
                            }
                        }
                     if(FLAG_line_sensor==LEFT_1){
                         while(valueA!=(PWM_MAX-actual_duty)||valueB!=(PWM_MAX-actual_duty)){
                             AEB_BREAK();
                             after_curve_sx(&valueA,&valueB,&actual_duty);
                             OCR0A=PWM_MAX-valueA;
                             OCR0B=PWM_MAX-valueB;
                             _delay_ms(1);
                            }
                         FLAG_line_sensor=0;
                        }
                     if(FLAG_line_sensor==RIGHT_1){
                         while(valueA!=(PWM_MAX-actual_duty)||valueB!=(PWM_MAX-actual_duty)){
                             AEB_BREAK();
                             after_curve_dx(&valueA,&valueB,&actual_duty);
                             OCR0A=PWM_MAX-valueA;
                             OCR0B=PWM_MAX-valueB;
                             _delay_ms(1);
                            }
                         FLAG_line_sensor=0;
                        }
                    }
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
            }
         if (FLAG_DIRECTION==RIGHT){                              //DRIFTING IN RIGHT DIRECTION  
             buffer_2=receiveByte(TIMEOUT_USART);
             while(buffer_2==0&&valueA<=PWM_UPPER_LIMIT_DRIFT){
                 AEB_BREAK();
                 valueA=valueA+PWM_STEP;
                 OCR0A=valueA;
                 OCR0B=PWM_MAX-valueA;
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
            }
         if (FLAG_DIRECTION==LEFT){                             //DRIFTING IN LEFT DIRECTION
             buffer_2=receiveByte(TIMEOUT_USART);
             valueA=PWM_MAX-valueA;
             while(buffer_2==0&&valueA<=PWM_UPPER_LIMIT_DRIFT){
                 AEB_BREAK();
                 valueA=valueA+PWM_STEP;
                 OCR0A=PWM_MAX-valueA;
                 OCR0B=valueA;
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
            }
        }
     if(buffer_2==BRAKE_PEDAL){                 //HANDLE THE ACCELERATION OF CAR IN FORWARD - BACKWARD DIRECTION AND WHILE DRIFTING
         delete_buffer();
         if (FLAG_DIRECTION==BACKWARD){
             buffer_2=receiveByte(TIMEOUT_USART);
             while(buffer_2==0&&valueA>=PWM_MIN){
                 AEB_BREAK();
                 valueA=valueA-PWM_STEP_DECELERATION;
                 OCR0A=valueA;
                 OCR0B=valueA;
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
            }
         if (FLAG_DIRECTION==FORWARD){   
             buffer_2=receiveByte(TIMEOUT_USART);
             valueA=PWM_MAX-valueA;
             while(buffer_2==0&&valueA>=PWM_MIN){
                 AEB_BREAK();
                 valueA=valueA-PWM_STEP_DECELERATION;
                 OCR0A=PWM_MAX-valueA;
                 OCR0B=PWM_MAX-valueA;
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
            }
         if (FLAG_DIRECTION==RIGHT){
             buffer_2=receiveByte(TIMEOUT_USART);
             while(buffer_2==0&&valueA>=PWM_MIN){
                 AEB_BREAK();
                 valueA=valueA-PWM_STEP_DECELERATION;
                 OCR0A=valueA;
                 OCR0B=PWM_MAX-valueA;
                 buffer_2=receiveByte(TIMEOUT_USART);
                 _delay_ms(1);
                }
            }
         if (FLAG_DIRECTION==LEFT){
             buffer_2=receiveByte(TIMEOUT_USART);
             valueA=PWM_MAX-valueA;
                 while(buffer_2==0&&valueA>=PWM_MIN){
                     AEB_BREAK();
                     valueA=valueA-PWM_STEP_DECELERATION;
                     OCR0A=PWM_MAX-valueA;
                     OCR0B=valueA;
                     buffer_2=receiveByte(TIMEOUT_USART);
                     _delay_ms(1);
                    }
                }
        }
      

     if (MODE_DRIVE_LINE==LINE_TRACKING_MODE&&FLAG_DIRECTION==FORWARD){    //IF LINE_TRACKING IS ACTIVE AND I'M GOING FORWARD I'M ABLE TO FOLLOW THE LINE
         AEB_BREAK();
         if (line_sensor_tracking_status()==LEFT_1){ 
             OCR0A=PWM_MAX-20;
             OCR0B=PWM_MAX-20;
             PORTD|=(1<<PORTD4);
             PORTD&=~(1<<PORTD2);
             FLAG_line_sensor=LEFT_1;
             time=cont_temp();
             while(line_sensor_tracking_status()!=CENTERED&&(cont_temp()-time<=TIMEOUT_LINE_SENSOR)){
                 AEB_BREAK();
                 OCR0A=PWM_MAX-120;
                 OCR0B=120;
                }
            }
                 
         if (FLAG_line_sensor==LEFT_1){  
             OCR0A=PWM_MAX-20;
             OCR0B=20;
             PORTD|=(1<<PORTD4);  
             PORTD|=(1<<PORTD2);  
             OCR0A=actual_duty;
             OCR0B=actual_duty;
             FLAG_line_sensor=0;
            }

         if (line_sensor_tracking_status()==RIGHT_1){ 
             OCR0A=PWM_MAX-20;    
             OCR0B=PWM_MAX-20;
             PORTD|=(1<<PORTD2);
             PORTD&=~(1<<PORTD4);
             FLAG_line_sensor=RIGHT_1;
             time=cont_temp();
             while (line_sensor_tracking_status()!=CENTERED&&(cont_temp()-time<=TIMEOUT_LINE_SENSOR)){
                 AEB_BREAK();
                 OCR0A=120;
                 OCR0B=PWM_MAX-120;
                }

            }

         if(FLAG_line_sensor==RIGHT_1){ 
             OCR0A=20;
             OCR0B=PWM_MAX-20;
             PORTD|=(1<<PORTD4);  
             PORTD|=(1<<PORTD2);  
             OCR0A=actual_duty;
             OCR0B=actual_duty;
             FLAG_line_sensor=0;
            }         
            
        }

    }

}

