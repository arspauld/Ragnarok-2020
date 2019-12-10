#include "stm32f410rx.h"
#include <string.h>

// PLL constant definitions
#define PLL_P 2
#define PLL_N 200
#define PLL_M 16

#define GPIO_PIN_5                 ((uint16_t) 1U<<5 )  /* Pin 5  selected    */

void system_clock_init(void);
void serial_uart_init(uint32_t baud);
void pwm_init(uint8_t duty);
void delay(volatile uint32_t s);

void TIM5_IRQHandler(void)
{
    TIM5->SR &= ~TIM_SR_UIF; // Clears the Interrupt Flag first so the interrupts isn't called again
    GPIOA->ODR ^= GPIO_PIN_5; // Toggles LED on A5
}

int main(void)
{
    system_clock_init();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enables GPIO Port A
    serial_uart_init(9600);
    pwm_init(25);

    /* Interrupt Driven LED Flash
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Sets Bit 0 of the Reset and Clock Control AHB1 Peripheral Clock Enable Register (RCC_AHB1ENR) to enable GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Sets Bit 4 of the Reset and Clock Control APB1 Peripheral Clock Enable Register (RCC_APB1ENR) to enable TIM5

    GPIOA->MODER &= ~(0b11 << 10); // Resets Pin 5 to Input
    GPIOA->MODER |=  (0b01 << 10); // Sets Pin 5 to Output

    GPIOA->ODR |= GPIO_PIN_5; // Turns on LED on A5

    TIM5->PSC = 4999; // Prescaler that results in a 20 kHz timer clock
    TIM5->ARR = 2000; // Automatic Reset value of timer, set to change at 10 Hz
    TIM5->DIER |= TIM_DIER_UIE; // Enables the Interrupt flag for the timer
    NVIC_EnableIRQ(TIM5_IRQn); // Enables global interrupts, (Built in to M4 header)
    TIM5->CR1 |= TIM_CR1_CEN; // Enables the timer clock
    */

    char mess[] = "Pat Rocks\n";
    while(1)
    {
        // Do Stuff
        for(uint8_t i = 0; i < strlen(mess); i++)
        {
            USART2->DR = mess[i];
            while(!(USART2->SR & USART_SR_TC));
        }

        delay(1000000);
    }

    return 0;
}

void system_clock_init(void)
{
    RCC->CR |= RCC_CR_HSION; // Verifies that the High Speed Internal Clock is on
    while(!(RCC->CR & RCC_CR_HSIRDY)); // Waits until HSI is on

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS_1 | PWR_CR_VOS_0; // Scales the Power 

    FLASH->ACR = FLASH_ACR_LATENCY_3WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN; // Sets Latency to account for new clock speed

    // Sets PLL to 100 MHz
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI | (((PLL_P / 2) - 1) << RCC_PLLCFGR_PLLP_Pos) | (PLL_N << RCC_PLLCFGR_PLLN_Pos) | (PLL_M << RCC_PLLCFGR_PLLM_Pos); // Sets PLL frequency using P, N, M
    RCC->CR |= RCC_CR_PLLON; // Enables PLL
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_SW_PLL; // Divides AHB clock and switches SYSCLK to a PLL source

    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); // Waits until the PLL is the System Clock

    SystemCoreClockUpdate();
}

void serial_uart_init(uint32_t baud)
{
    // Initializes USART2
    GPIOA->MODER &= ~((0b11 << 8) | (0b11 << 6) | (0b11 << 4)); // Resets Pins 2, 3, and 4 to Input
    GPIOA->MODER |= (0b10 << 8) | (0b10 << 6) | (0b10 << 4); // Sets Pins 2,  3, and 4 to Alternate Function

    GPIOA->OSPEEDR |= (0b11 << 8) | (0b11 << 6) | (0b11 << 4); // Sets pins 2, 3, and 4 to high speed
    GPIOA->AFR[0] |= (7 << 16) | (7 << 12) | (7 << 8); // Enabls USART2 Alternate Function for pins 2, 3, and 4
    
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enables USART2 peripheral

    USART2->BRR |= (SystemCoreClock / 2 / (16 * baud) << USART_BRR_DIV_Mantissa_Pos) | (SystemCoreClock / 2 / baud) % 16; // Sets a baud rate divider of 325.5 (9600 baud)
    
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; // Enables RX, TX, RX Interrupt, and USART
    USART2->CR2 |= USART_CR2_CLKEN; // Enables the clock signal
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

    TIM5->PSC = 4999; // Prescaler that results in a 20 kHz timer clock
    TIM5->ARR = 20000; // Automatic Reset value of timer, set to change at 2 Hz
    TIM5->CCR1 = (TIM5->ARR * duty) / 100; // Sets the capture point

    TIM5->CCMR1 |= (0b110 << 4); // Sets the Timer to PWM mode
    TIM5->CCER |= TIM_CCER_CC1E; // Enable Compare and Capture 1

    TIM5->DIER |= TIM_DIER_UIE; // Sets Timer update flag
    NVIC_EnableIRQ(TIM5_IRQn); // Attaches interrupt
    TIM5->CR1 |= TIM_CR1_CEN; // Enables the timer clock
}

void delay(volatile uint32_t s)
{
    for(s; s>0; s--){
        // Do nothing
    }
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