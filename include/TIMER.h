#ifndef TIMER_H
#define TIMER_H
#include <stdint.h>

extern volatile uint32_t cont;


//Description comment of the functios below in the related .c
void set_TIMER1(void);
void Set_PWM(void);
uint32_t cont_temp(void);

#endif

