#ifndef COSTANT_H
#define COSTANT_H
#include <stdint.h>

//VALUE OF PWM
#define PWM_MAX 255
#define PWM_MIN 15
#define PWM_UPPER_LIMIT 247
#define PWM_LOWER_LIMIT 40
#define PWM_UPPER_LIMIT_DRIFT 115
#define PWM_STEP 3
#define PWM_STEP_DECELERATION 7


//BAUDRATE USART
#define BAUDRATE 9600

//WAIT TIME USART [ms]
#define TIMEOUT_USART 30

//BRAKE DISTANCE [mm]
#define low_speed_distance 12
#define low_medium_speed_distance 22
#define medium_speed_distance 25
#define fast_speed_distance 28

//DRIVING MODE
#define AEB_MODE 89                  //'Y'
#define LINE_TRACKING_MODE 88        //'X'
#define NORMAL_MODE 116              //'t'

//DIRECTION OF MOTION
#define FORWARD 70                   //'F'
#define BACKWARD 66                  //'B'
#define RIGHT 82                     //'R'
#define LEFT 76                      //'L'
#define STOP 83                      //'S' --> data sent trought USART when button pressed on the application on the smart-phone is released
#define stop 115                     //'s' --> data sent trought USART when button pressed on the appication on the smart-phone is released

//PEDAL PRESSED
#define ACCELERATOR_PEDAL 97         //'a'
#define BRAKE_PEDAL 100              //'d'

//LINE SENSOR STATUS                 //Digital output of line-sensor
#define LEFT_1 75                    //L=1 C=0 R=0  OR  L=1 C=1 R=0
#define RIGHT_1 81                   //L=0 C=0 R=1  OR  L=0 C=1 R=1
#define CENTERED 70                  //L=1 C=1 R=1    OR   L=0 C=1 R=0



//TIME OUT LINE SENSOR
#define TIMEOUT_LINE_SENSOR 40

#endif

