#include "USART.h"
#include "TIMER.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//Set register of USART module
void set_BAUDRATE(uint16_t baud){
    uint16_t ubrr=16000000UL/(8UL*baud) - 1;
    UBRR0H=ubrr>>8;
    UBRR0L=ubrr;
    UCSR0A|=(1<<U2X0);
    UCSR0B=(1<<RXEN0)|(1<<TXEN0);      //enable Rx and Tx of UART module
    UCSR0C=(1<<UCSZ01)|(1<<UCSZ00);    //dimension of transmission data is 8 bit 
}

//Function to receive data from UART without time interrupt
uint8_t receiveByte1(void){
    while (!(UCSR0A&(1<<RXC0)));       //Wait until the buffer UART is filled
    return UDR0;
}

//Function to receive data from UART
uint8_t receiveByte_NO_BLOCKING(void){
    if ((UCSR0A&(1<<RXC0))){          //No wait until buffer UART is filled
     return UDR0;
    }
    return 0;
}

//Function to receive data from UART with time interrupt
uint8_t receiveByte(uint16_t const_temp){
    uint32_t actual_time=cont_temp();
    while (!(UCSR0A&(1<<RXC0))){      //Wait until the buffer UART is filled and maximum for 30 ms
     if ((cont_temp()-actual_time)>const_temp){
         return 0;
        }
    }
    return UDR0;
}

//Function for delete buffer UART
void delete_buffer(void){
    volatile uint8_t buffer_1;
    while((UCSR0A&(1<<RXC0))){
        buffer_1=UDR0;
    }
}
