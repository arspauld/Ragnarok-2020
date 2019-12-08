#include "stm32f410rx.h"

#define GPIO_PIN_5                 ((uint16_t) 1U<<5 )  /* Pin 5  selected    */

void delay(volatile uint32_t s);

void TIM6_DAC_IRQHandler(void)
{
    TIM6->SR &= (uint16_t)(~(1U << 0)); // Clears the Interrupt Flag first so the interrupts isn't called again
    GPIOA->ODR ^= GPIO_PIN_5; // Toggles LED on A5
}

int main(void)
{
    SystemInit();
    RCC->AHB1ENR |= 1U << 0; // Sets Bit 0 of the Reset and Clock Control AHB1 Peripheral Clock Enable Register (RCC_AHB1ENR) to enable GPIOA
    RCC->APB1ENR |= 1U << 4; // Sets Bit 4 of the Reset and Clock Control APB1 Peripheral Clock Enable Register (RCC_APB1ENR) to enable TIM6

    GPIOA->MODER &= ~(0b11 << 10); // Resets Pin 5 to Input
    GPIOA->MODER |=  (0b01 << 10); // Sets Pin 5 to Output

    GPIOA->ODR |= 0; // Turns on LED on A5

    TIM6->PSC = 3999; // Prescaler that results in a 20 kHz timer clock
    TIM6->ARR = 4000; // Automatic Reset value of timer
    TIM6->DIER |= (1U << 0); // Enables the Interrupt flag for the timer
    NVIC_EnableIRQ(TIM6_DAC_IRQn); // Enables global interrupts, (Built in to M4 header)
    TIM6->CR1 |= (1U << 0); // Enables the timer clock

    while (1)
    {
        /*
        delay(50000);
        GPIOA->ODR ^= GPIO_PIN_5;
        */
    }

    return 0;
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