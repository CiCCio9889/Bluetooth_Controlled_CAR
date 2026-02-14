#ifndef USART_H
#define USART_H
#include <stdint.h>

//Description comment of the functios below in the related .c
void set_BAUDRATE(uint16_t baud);
uint8_t receiveByte1(void);
uint8_t receiveByte_NO_BLOCKING(void);
uint8_t receiveByte(uint16_t const_temp);
void delete_buffer(void);

#endif

