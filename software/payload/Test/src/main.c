#include "system.h"
#include "ADC.h"
#include "EXTI.h"
#include "USART.h"
#include "I2C.h"
#include "RingBuffer.h"
#include "flash.h"
#include <string.h>
#include <stdio.h>

// PLL constant definitions
#define PLL_P 2
#define PLL_N 200
#define PLL_M 16

volatile uint8_t write_flag = 0;
volatile uint8_t adc_ready = 0;
//volatile uint32_t* const val = (volatile uint32_t*) 0x0800C000; // Sets address of variable to a a spot in the Flash

void serial_uart_init(uint32_t baud);
void pwm_init(uint8_t duty);
void delay(volatile uint32_t s);
void reset(void);

void TIM5_IRQHandler(void)
{
    TIM5->SR &= ~TIM_SR_UIF; // Clears the Interrupt Flag first so the interrupts isn't called again
    GPIOA->ODR ^= GPIO_PIN_5; // Toggles LED on A5
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI->PR & EXTI_PR_PR13) // Checks if the intterrupt is for Channel 13
    {
        EXTI->PR |= EXTI_PR_PR13; // Clear Interrupt bit
        write_flag = 1; // Enables the write flag for the bit
        GPIOA->ODR ^= GPIO_PIN_5; // Toggles LED
    }
}

void USART2_IRQHandler(void)
{
    if(USART2->SR & USART_SR_RXNE)
    {
        //USART2->SR &= ~USART_SR_RXNE;
        USART2->DR = USART2->DR;
        while(!(USART2->SR & USART_SR_TXE)); // Protects from overloading the Data Register
    }
    if(USART2->SR & USART_SR_TC)
    {
        USART2->SR &= ~USART_SR_TC;
    }
}

void ADC_IRQHandler(void)
{    
    if(ADC1->SR & ADC_SR_EOC)
    {
        ADC1->SR &= ~ADC_SR_EOC; // Drops flag
        adc_ready = 1; // Sets Software flag
    }
}

int main(void)
{
    system_clock_init();
    flash_init();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; // Enables GPIO Ports A, B, and C
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_EXTITEN; // Enables the System Configuration Capability and the External Interrupt/Event Controller
    serial_uart_init(115200); // Initializes USART2
    pwm_init(10); // Outputs a PWM on A0
    //i2c_init();
    adc_init();
    user_button_init();

    /* Interrupt Driven LED Flash
     * RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Sets Bit 0 of the Reset and Clock Control AHB1 Peripheral Clock Enable Register (RCC_AHB1ENR) to enable GPIOA
     * RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Sets Bit 4 of the Reset and Clock Control APB1 Peripheral Clock Enable Register (RCC_APB1ENR) to enable TIM5
     * 
     * GPIOA->MODER &= ~(0b11 << 10); // Resets Pin 5 to Input
     * GPIOA->MODER |=  (0b01 << 10); // Sets Pin 5 to Output
     * 
     * GPIOA->ODR |= GPIO_PIN_5; // Turns on LED on A5
     * 
     * TIM5->PSC = 4999; // Prescaler that results in a 20 kHz timer clock
     * TIM5->ARR = 2000; // Automatic Reset value of timer, set to change at 10 Hz
     * TIM5->DIER |= TIM_DIER_UIE; // Enables the Interrupt flag for the timer
     * NVIC_EnableIRQ(TIM5_IRQn); // Enables global interrupts, (Built in to M4 header)
     * TIM5->CR1 |= TIM_CR1_CEN; // Enables the timer clock
     */

    char mess[20];
    uint8_t count = 0;

    //uint32_t temp = *val;
    //if(temp) flash_write32(val,temp/2);
    //else flash_erase(3);
    //sprintf(mess, "%lu -> %lu\n", temp, *val);
    //uart_write(mess);

    while(1)
    {        
        // Do Stuff
        if(write_flag)
        {
            count++;
            write_flag = 0;
            //uart_write("RESET\n");
            reset();
            //i2c_write(0x00,mess);
        }
        if(adc_ready) sprintf(mess, "Reading : %4u\n", adc_read());
        usart_write_string(USART2, mess); // Writes the message buffer
        delay(1000000);
    }

    return 0;
}

void serial_uart_init(uint32_t baud)
{
    USART_options_t USART_serial = {
		.port       =   USART2, // declares the port
        .baud	    =	baud, // passes along the baud rate
		.clock      =   FALSE, // No clock signal
		.interrupts =	RXNE_IF // Enables  an interrupt for Received Bytes
	};
    usart_init(&USART_serial);
}

void pwm_init(uint8_t duty)
{
    // Enables a PWM waveform on pin A0
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enables Timer Counter 5

    GPIOA->MODER &= ~(0b11 << 10); // Resets Pin 5 to Input (User LED)
    GPIOA->MODER |=  (0b01 << 10); // Sets Pin 5 to Output

    GPIOA->MODER &= ~(0b11 << 0); // Resets Pin 0 to Input (PWM)
    GPIOA->MODER |=  (0b10 << 0); // Sets Pin 0 to Alternate Function

    GPIOA->AFR[0] |= (2 << 0); // Sets Pin 0 to alternative function mode

    TIM5->PSC = 199; // Prescaler that results in a 500 kHz timer clock
    TIM5->ARR = 10000; // Automatic Reset value of timer, set to change at 50 Hz (a typical servo pwm frequency)
    TIM5->CCR1 = (TIM5->ARR * duty) / 100; // Sets the capture point

    TIM5->CCMR1 |= (0b110 << 4); // Sets the Timer to PWM mode
    TIM5->CCER |= TIM_CCER_CC1E; // Enable Compare and Capture 1

    TIM5->DIER |= TIM_DIER_UIE; // Sets Timer update flag
    //NVIC_EnableIRQ(TIM5_IRQn); // Attaches interrupt
    TIM5->CR1 |= TIM_CR1_CEN; // Enables the timer clock
}

void delay(volatile uint32_t s)
{
    for(s; s>0; s--);
}

void reset(void)
{
    //flash_erase(3);
    //flash_erase(4);
    NVIC_SystemReset(); // Built in function that performs a software reset to the system
}

/*
 * {
 *   HAL_Init();
 *   
 *   LED_GPIO_CLK_ENABLE();
 *   
 *   GPIO_InitTypeDef GPIO_InitStruct;
 *   
 *   GPIO_InitStruct.Pin = LED_PIN;
 *   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 *   GPIO_InitStruct.Pull = GPIO_PULLUP;
 *   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
 *   HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct); 
 * 
 *   while (1)
 *   {
 *     HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
 *     
 *     HAL_Delay(100);
 *   }
 * }
 * 
 * void SysTick_Handler(void)
 * {
 *   HAL_IncTick();
 * }
 * 
 * void NMI_Handler(void)
 * {
 * }
 * 
 * void HardFault_Handler(void)
 * {
 *   while (1) {}
 * }
 * 
 * 
 * void MemManage_Handler(void)
 * {
 *   while (1) {}
 * }
 * 
 * void BusFault_Handler(void)
 * {
 *   while (1) {}
 * }
 * 
 * void UsageFault_Handler(void)
 * {
 *   while (1) {}
 * }
 * 
 * void SVC_Handler(void)
 * {
 * }
 * 
 * 
 * void DebugMon_Handler(void)
 * {
 * }
 * 
 * void PendSV_Handler(void)
 * {
 * }
 */