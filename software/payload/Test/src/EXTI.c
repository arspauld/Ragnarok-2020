#include "EXTI.h"

void user_button_init(void)
{
    // Enables An interrupt that is connected to the USer Button (Blue)
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // Connects the 13th channel of EXTI to Port C (Port C, Pin 13)
    EXTI->IMR |= EXTI_IMR_MR13; // Enables the Interrupt of Channel 13
    EXTI->FTSR |= EXTI_FTSR_TR13; // Enables Falling Edge Detection on Channel 13
    NVIC_EnableIRQ(EXTI15_10_IRQn); // Enables the EXTI Interrupt for Channels 10 through 15
}