#ifndef DIRECTION_HANDLER_H
#define DIRECTION_HANDLER_H
#include <stdint.h>

extern volatile uint8_t FLAG_DIRECTION;
extern volatile uint8_t MODE_DRIVE;
extern volatile uint8_t FLAG_WARNING;
extern uint8_t MODE_DRIVE_LINE;
extern uint8_t FLAG_command_acceleration;
extern uint8_t actual_flag_direction;


//Description comment of the functios below in the related .c
void set_direction(void);

#endif