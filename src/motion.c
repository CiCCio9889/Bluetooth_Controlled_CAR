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

//function to handle forward and backward direction ____ TO USE FOR NORMAL MODE NATURALLY IT HAVE AEB IMPLEMENTED
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
  
     if (buffer_2==LEFT){
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

     if(buffer_2==RIGHT){
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

     if(buffer_2==ACCELERATOR_PEDAL){  //97 is 'a'
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
         if (FLAG_DIRECTION==FORWARD){      //mettere qui la cosa del controllo linea perchè appunto è quando sto andando avanti ed accelerando
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
                 if(MODE_DRIVE_LINE==LINE_TRACKING_MODE){
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
         if (FLAG_DIRECTION==RIGHT){
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
         if (FLAG_DIRECTION==LEFT){
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
     if(buffer_2==BRAKE_PEDAL){  //100 is 'd'
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
      

     if (MODE_DRIVE_LINE==LINE_TRACKING_MODE&&FLAG_DIRECTION==FORWARD){
         AEB_BREAK();
         if (line_sensor_tracking_status()==LEFT_1){ //aggiunto
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
                 
         if (FLAG_line_sensor==LEFT_1){  //aggiunto
             OCR0A=PWM_MAX-20;
             OCR0B=20;
             PORTD|=(1<<PORTD4);  //PIN4 to HIGH
             PORTD|=(1<<PORTD2);  //PIN2 to HIGH
             OCR0A=actual_duty;
             OCR0B=actual_duty;
             FLAG_line_sensor=0;
            }

         if (line_sensor_tracking_status()==RIGHT_1){ //aggiunto
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

         if(FLAG_line_sensor==RIGHT_1){ //aggiunto
             OCR0A=20;
             OCR0B=PWM_MAX-20;
             PORTD|=(1<<PORTD4);  //PIN4 to HIGH
             PORTD|=(1<<PORTD2);  //PIN2 to HIGH
             OCR0A=actual_duty;
             OCR0B=actual_duty;
             FLAG_line_sensor=0;
            }         
            
        }

    }

}

