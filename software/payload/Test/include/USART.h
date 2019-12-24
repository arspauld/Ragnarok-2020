#ifndef USART_H
#define USART_H_

#include "system.h"

typedef struct USART_options
{
    USART_TypeDef* port;
    uint32_t baud;
    uint8_t clock;
    uint8_t interrupts;
} USART_options_t;

void usart_init(USART_options_t* options);
void usart_write_byte(USART_TypeDef* port, uint8_t datum);
uint8_t usart_read_byte(USART_TypeDef* port);
void usart_write(USART_TypeDef* port, uint8_t* data);
void usart_write_string(USART_TypeDef* port, char* str);

#endif /* USART_H_ */