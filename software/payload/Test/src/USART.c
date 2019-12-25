#include "USART.h"
#include <string.h>

void usart_init(USART_options_t* options)
{
    USART_TypeDef* port = options->port; // USART port name
    uint32_t baud = options->baud; // USART baud rate value
    uint8_t clock = options->clock; // Boolean of if the clock is to be on (USARt vs. UART)
    uint8_t ints = options->interrupts; // Interrupt options

    if(port == USART2)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enables the clock clock to the GPIOA peripheral to enable it
        GPIOA->MODER &= ~((0b11 << (GPIO_PIN_3_Pos * 2)) | (0b11 << (GPIO_PIN_2_Pos * 2))); // Resets Pins 2 and 3 to Input
        GPIOA->MODER |= (0b10 << (GPIO_PIN_3_Pos * 2)) | (0b10 << (GPIO_PIN_2_Pos * 2)); // Sets Pins 2 and 3 to Alternate Function

        GPIOA->OSPEEDR |= (0b11 << (GPIO_PIN_3_Pos * 2)) | (0b11 << (GPIO_PIN_2_Pos * 2)); // Sets pins 2 and 3 to high speed
        GPIOA->AFR[0] |= (AF7 << (GPIO_PIN_3_Pos * 4)) | (AF7 << (GPIO_PIN_2_Pos * 4)); // Enables USART2 Alternate Function for pins 2 and 3

        RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enables USART2

        USART2->BRR |= (SystemCoreClock / 2 / (16 * options->baud) << USART_BRR_DIV_Mantissa_Pos) | (SystemCoreClock / 2 / options->baud) % 16; // Sets a baud rate divider of 325.5 (9600 baud)
    
        USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; // Enables RX, TX, and USART


        if(clock)
        {
            GPIOA->MODER &= ~(0b11 << (GPIO_PIN_4_Pos * 2)); // Sets pin 4 to alternative mode
            GPIOA->MODER |= (0b10 << (GPIO_PIN_4_Pos * 2));

            GPIOA->OSPEEDR |= (0b11 << (GPIO_PIN_4_Pos * 2)); // Sets pin 4 to high speed and alternate function 7
            GPIOA->AFR[0] |= (AF7 << (GPIO_PIN_4_Pos * 4));

            USART2->CR2 |= USART_CR2_CLKEN; // Enables the clock signal
        }


        if(ints)
        {
            enable_interrupt(port, ints);
            NVIC_EnableIRQ(USART2_IRQn); // Enables interrupts
        }


        while(!(USART2->SR & USART_SR_TC)); // Clear TC Flag
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