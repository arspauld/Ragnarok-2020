#include "ADC.h"

void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enables ADC (voltage range of 0 - 1.7 V)

    GPIOA->MODER &= ~(0b11 << (2*1)); // Resets Pin 1 to Input (User LED)
    GPIOA->MODER |=  (0b11 << (2*1)); // Sets Pin 1 to Analog

    ADC1->SMPR2 |= (ADC_SMPR2_SMP1 & 0b111); // Sets the sampling of ADC1 channel 0 to 480 cycles
    ADC1->SQR3 |= (ADC_SQR3_SQ1 & 1); // Sets the first ADC sequence to channel 0
    ADC1->SQR1 |= ((1 - 1) & ADC_SQR1_L); // Sets the number of sequences to 1
    ADC->CCR |= (ADC_CCR_ADCPRE & ((8/2)-1)); // Sets the ADC clock to have a prescaler of 8
    ADC1->CR1 |= ADC_CR1_EOCIE; // Enables an End of Conversion Interrupt
    ADC1->CR2 |= ADC_CR2_ADON; // Turns on ADC
    ADC1->CR2 |= ADC_CR2_SWSTART; // Starts ADC Conversions

    NVIC_EnableIRQ(ADC_IRQn); // Starts the ADC Interrupt
}


uint16_t adc_read(void)
{
    // Reads ADC Data Register and restarts conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
    return (uint16_t) ADC1->DR;
}