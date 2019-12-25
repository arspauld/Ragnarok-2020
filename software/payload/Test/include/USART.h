#ifndef USART_H
#define USART_H_

#include "system.h"

// Interrupt Selection Flags
#define     TXE_IF          (1U << 0) // Transmission Register Empty
#define     CTS_IF          (1U << 1) // Clear to Send
#define     TC_IF           (1U << 2) // Tranmission Complete
#define     RXNE_IF         (1U << 3) // Received Register Not Empty
#define     IDLE_IF         (1U << 4) // Idle 
#define     PE_IF           (1U << 5) // Parity Error
#define     LBD_IF          (1U << 6) // LIN Break Detected
#define     E_IF            (1U << 7) // Error

// Type Declaration for USART initialization options (UNFINISHED)
typedef struct USART_options
{
    USART_TypeDef* port;
    uint32_t baud;
    uint8_t clock;
    uint8_t interrupts;
} USART_options_t;


// Functions
void usart_init(USART_options_t* options); // Initializes the USART port seected
void enable_interrupt(USART_TypeDef* port, uint8_t interrupt);
void disable_interrupt(USART_TypeDef* port, uint8_t interrupt);

void usart_write_byte(USART_TypeDef* port, uint8_t datum); // Writes a singular byte to the data register selected, unprotected
uint8_t usart_read_byte(USART_TypeDef* port); // Reads a singular byte from the data register, unprotected
void usart_write(USART_TypeDef* port, uint8_t* data); // Writes an array of bytes to the data register, protected
void usart_write_string(USART_TypeDef* port, char* str); // Writes a string to the data resgister, protected

#endif /* USART_H_ */