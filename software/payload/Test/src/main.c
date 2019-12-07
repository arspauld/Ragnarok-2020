#include "stm32f410rx.h"

#define GPIO_PIN_0                 ((uint16_t) 1U<<0 )  /* Pin 0  selected    */
#define GPIO_PIN_1                 ((uint16_t) 1U<<1 )  /* Pin 1  selected    */
#define GPIO_PIN_2                 ((uint16_t) 1U<<2 )  /* Pin 2  selected    */
#define GPIO_PIN_3                 ((uint16_t) 1U<<3 )  /* Pin 3  selected    */
#define GPIO_PIN_4                 ((uint16_t) 1U<<4 )  /* Pin 4  selected    */
#define GPIO_PIN_5                 ((uint16_t) 1U<<5 )  /* Pin 5  selected    */
#define GPIO_PIN_6                 ((uint16_t) 1U<<6 )  /* Pin 6  selected    */
#define GPIO_PIN_7                 ((uint16_t) 1U<<7 )  /* Pin 7  selected    */
#define GPIO_PIN_8                 ((uint16_t) 1U<<8 )  /* Pin 8  selected    */
#define GPIO_PIN_9                 ((uint16_t) 1U<<9 )  /* Pin 9  selected    */
#define GPIO_PIN_10                ((uint16_t) 1U<<10)  /* Pin 10 selected    */
#define GPIO_PIN_11                ((uint16_t) 1U<<11)  /* Pin 11 selected    */
#define GPIO_PIN_12                ((uint16_t) 1U<<12)  /* Pin 12 selected    */
#define GPIO_PIN_13                ((uint16_t) 1U<<13)  /* Pin 13 selected    */
#define GPIO_PIN_14                ((uint16_t) 1U<<14)  /* Pin 14 selected    */
#define GPIO_PIN_15                ((uint16_t) 1U<<15)  /* Pin 15 selected    */

void delay(volatile uint32_t s);


int main(void)
{
    RCC->AHB1ENR |= 1 << 0; // Sets Bit 0 of the Reset and Clock Control AHB1 Peripheral Clock Enable Register (RCC_AHB1ENR) to enable GPIOA

    GPIOA->MODER &= ~(0b11 << 10); // Resets Pin 5 to Input
    GPIOA->MODER |=  (0b01 << 10); // Sets Pin 5 to Output

    GPIOA->ODR |= GPIO_PIN_5;

    while (1)
    {
        delay(1000000);
        GPIOA->ODR ^= GPIO_PIN_5;
    }
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