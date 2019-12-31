#ifndef USART_H
#define USART_H_

#include "system.h"

/* USART.h is the header for a file that allows for the initialization of USART functions without RTS or CTS features
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */


// Initialization Options
// Clock Polarity
#define     No_Clock            (0U)
#define     Clock_Steady_Low    (1U << 0)
#define     Clock_Steady_High   (1U << 1)

// Interrupt Selection Flags
#define     TXE_IF              (1U << 0) // Transmission Register Empty
#define     CTS_IF              (1U << 1) // Clear to Send
#define     TC_IF               (1U << 2) // Tranmission Complete
#define     RXNE_IF             (1U << 3) // Received Register Not Empty
#define     IDLE_IF             (1U << 4) // Idle 
#define     PE_IF               (1U << 5) // Parity Error
#define     LBD_IF              (1U << 6) // LIN Break Detected
#define     E_IF                (1U << 7) // Error

// Direction Definitions
#define     No_Comm             (0U)
#define     RX_Only             (1U << 0)
#define     TX_Only             (1U << 1)
#define     RX_TX               (RX_Only | TX_Only)

// Data Width Selections
#define     Eight_Data_Bits     (0U)
#define     Nine_Data_Bits      (1U)

// Stop Bits
#define     Stop_Bits_0_5       (1U)
#define     Stop_Bits_1_0       (0U)
#define     Stop_Bits_1_5       (3U)
#define     Stop_Bits_2_0       (2U)

// Parity Control
#define     No_Parity           (0U)
#define     Even_Parity         (1U << 0)
#define     Odd_Parity          (1U << 1)

// Oversampling
#define     Over_Sampling_8     (8U)
#define     Over_Sampling_16    (16U)


// Type Declaration for USART initialization options (UNFINISHED)
typedef struct USART_options
{
    // initiliazation options for USART
    USART_TypeDef*  port;
    uint32_t        baud;
    uint8_t         clock;
    uint8_t         interrupts;
    uint8_t         direction;
    uint8_t         data_width;
    uint8_t         stop_bits;
    uint8_t         parity;
    uint8_t         oversampling;
    uint8_t         pin_set;
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