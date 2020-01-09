#include "PWM.h"

void pwm_init(uint8_t duty)
{
    // Enables a PWM waveform on pin A0
    enable_peripheral(TIM5_EN);  // Enables Timer Counter 5

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