#ifndef AEB_H
#define AEB_H
#include <stdint.h>



//Description comment of the functios below in the related .c
#define AEB_BREAK() if (FLAG_WARNING==1) return; //This MACRO is used to exit from while loop when the distance intervention of AEB is reached
uint8_t Braking_distance(void);

#endif