#include "USART.h"
#include <string.h>

void usart_init(USART_options_t* options)
{
    USART_TypeDef*  USARTx  =   options->port; // USART port name
    uint32_t        baud    =   options->baud; // USART baud rate value
    uint8_t         clock   =   options->clock; // Boolean of if the clock is to be on (USARt vs. UART)
    uint8_t         ints    =   options->interrupts; // Interrupt options
    uint8_t         dir     =   options->direction; // Direction Options
    uint8_t         data_w  =   options->data_width; // Data Width Options
    uint8_t         s_bits  =   options->stop_bits; // Stop Bit Options
    uint8_t         parity  =   options->parity; // Parity Control
    uint8_t         ovrsmp  =   options->oversampling; // Oversampling Selection

    if(IS_UART_INSTANCE(USARTx))
    {
        if(USARTx == USART2)
        {
            enable_peripheral(GPIOA_EN); // Enables the clock clock to the GPIOA peripheral to enable it
            enable_peripheral(USART2_EN); // Enables USART2

            GPIOA->MODER &= ~((0b11 << (GPIO_PIN_3_Pos * 2)) | (0b11 << (GPIO_PIN_2_Pos * 2))); // Resets Pins 2 and 3 to Input
            GPIOA->MODER |= (0b10 << (GPIO_PIN_3_Pos * 2)) | (0b10 << (GPIO_PIN_2_Pos * 2)); // Sets Pins 2 and 3 to Alternate Function

            GPIOA->OSPEEDR |= (0b11 << (GPIO_PIN_3_Pos * 2)) | (0b11 << (GPIO_PIN_2_Pos * 2)); // Sets pins 2 and 3 to high speed
            GPIOA->AFR[0] |= (AF7 << (GPIO_PIN_3_Pos * 4)) | (AF7 << (GPIO_PIN_2_Pos * 4)); // Enables USART2 Alternate Function for pins 2 and 3

            if(clock)
            {
                GPIOA->MODER &= ~(0b11 << (GPIO_PIN_4_Pos * 2)); // Sets pin 4 to alternative mode
                GPIOA->MODER |= (0b10 << (GPIO_PIN_4_Pos * 2));

                GPIOA->OSPEEDR |= (0b11 << (GPIO_PIN_4_Pos * 2)); // Sets pin 4 to high speed and alternate function 7
                GPIOA->AFR[0] |= (AF7 << (GPIO_PIN_4_Pos * 4));
            }

            if(ints) NVIC_EnableIRQ(USART2_IRQn); // Enables interrupts
        }

        USARTx->BRR |= (APB1_Clock / (ovrsmp * baud) << USART_BRR_DIV_Mantissa_Pos) | (APB1_Clock / baud) % ovrsmp; // Sets a baud rate divider of 325.5 (9600 baud)
        
        if(dir & RX_Only) USARTx->CR1 |= USART_CR1_RE; 
        if(dir & TX_Only) USARTx->CR1 |= USART_CR1_TE;
        
        if(clock) USARTx->CR2 |= USART_CR2_CLKEN; // Enables the clock signal

        if(data_w) USARTx->CR1 |= USART_CR1_M; // Sets the data width to 9 bits

        USARTx->CR2 |= ((s_bits & 0x03) << USART_CR2_STOP_Pos);

        if(parity)
        {
            USARTx->CR1 |= USART_CR1_PCE;
            if(parity & Odd_Parity) USARTx->CR1 |= USART_CR1_PS;
        }
        
        enable_interrupt(USARTx, ints); // Enables Any Interrupts Set
        USARTx->CR1 |= USART_CR1_UE; // Enables USART
        
        while(!(USARTx->SR & USART_SR_TC)); // Clear TC Flag
    }
}

void usart_write_byte(USART_TypeDef* port, uint8_t datum)
{
    port->DR = datum; // Places byte into data register
}

uint8_t usart_read_byte(USART_TypeDef* port)
{
    return port->DR; // Retrieves byte from data register
}

void usart_write(USART_TypeDef* port, uint8_t* data)
{
    for(uint8_t i = 0; i < sizeof(data); i++)
    {
        usart_write_byte(port, data[i]); // Increments through array of unsigned bytes
        while(!(port->SR & USART_SR_TC));
    }
}

void usart_write_string(USART_TypeDef* port, char* str)
{
    for(uint8_t i = 0; i < strlen(str); i++)
    {
        usart_write_byte(port, (uint8_t) str[i]); // Increments through string
        while(!(port->SR & USART_SR_TC));
    }
}

void enable_interrupt(USART_TypeDef* port, uint8_t interrupt)
{
    if(interrupt & TXE_IF)       port->CR1 |= USART_CR1_TXEIE;     // Enalbes the Tranmission Register Empty Interrupt
    if(interrupt & CTS_IF)       port->CR3 |= USART_CR3_CTSIE;     // Enables the Clear To Send Interrupt
    if(interrupt & TC_IF)        port->CR1 |= USART_CR1_TCIE;      // Enables the Transmission Complete Interrupt
    if(interrupt & RXNE_IF)      port->CR1 |= USART_CR1_RXNEIE;    // Enables the Receiver Register Not Empty Intterupt 
    if(interrupt & IDLE_IF)      port->CR1 |= USART_CR1_IDLEIE;    // Enables the Idle Interrupt
    if(interrupt & PE_IF)        port->CR1 |= USART_CR1_PEIE;      // Enables the Parity Error Interrupt
    if(interrupt & LBD_IF)       port->CR2 |= USART_CR2_LBDIE;     // Enables the LIN Break Detected Flag Interrupt
    if(interrupt & E_IF)         port->CR3 |= USART_CR3_EIE;       // Enables the Error Interrupt
}

void disable_interrupt(USART_TypeDef* port, uint8_t interrupt)
{
    if(interrupt & TXE_IF)       port->CR1 &= ~USART_CR1_TXEIE;    // Enalbes the Tranmission Register Empty Interrupt
    if(interrupt & CTS_IF)       port->CR3 &= ~USART_CR3_CTSIE;    // Enables the Clear To Send Interrupt
    if(interrupt & TC_IF)        port->CR1 &= ~USART_CR1_TCIE;     // Enables the Transmission Complete Interrupt
    if(interrupt & RXNE_IF)      port->CR1 &= ~USART_CR1_RXNEIE;   // Enables the Receiver Register Not Empty Intterupt 
    if(interrupt & IDLE_IF)      port->CR1 &= ~USART_CR1_IDLEIE;   // Enables the Idle Interrupt
    if(interrupt & PE_IF)        port->CR1 &= ~USART_CR1_PEIE;     // Enables the Parity Error Interrupt
    if(interrupt & LBD_IF)       port->CR2 &= ~USART_CR2_LBDIE;    // Enables the LIN Break Detected Flag Interrupt
    if(interrupt & E_IF)         port->CR3 &= ~USART_CR3_EIE;      // Enables the Error Interrupt
}