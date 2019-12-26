#include "system.h"

uint32_t AHB1_Clock = 16000000; // Default Clock Speed
uint32_t APB1_Clock = 16000000; // Default Clock Speed
uint32_t APB2_Clock = 16000000; // Default Clock Speed

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
    AHB1_Clock = SystemCoreClock;
    APB1_Clock = SystemCoreClock / 2;
    APB2_Clock = SystemCoreClock;
}

void enable_peripheral(uint8_t peripheral_flag)
{
    switch(peripheral_flag)
    {
        // AHB1 Peripherals
        case GPIOA_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
            break;
        case GPIOB_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
            break;
        case GPIOC_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
            break;
        case GPIOH_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
            break;
        case CRC_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
            break;
        case DMA1_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
            break;
        case DMA2_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
            break;
        case RNG_EN:
            RCC->AHB1ENR |= RCC_AHB1ENR_RNGEN;
            break;

        // APB1 Peripherals
        case TIM5_EN:
            RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
            break;
        case TIM6_EN:
            RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
            break;
        case LPTIM1_EN:
            RCC->APB1ENR |= RCC_APB1ENR_LPTIM1EN;
            break;
        case RTCAPB_EN:
            RCC->APB1ENR |= RCC_APB1ENR_RTCAPBEN;
            break;
        case WWDG_EN:
            RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
            break;
        case SPI2_EN:
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
            break;
        case USART2_EN:
            RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
            break;
        case I2C1_EN:
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
            break;
        case I2C2_EN:
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
            break;
        case PWR_EN:
            RCC->APB1ENR |= RCC_APB1ENR_PWREN;
            break;
        case DAC_EN:
            RCC->APB1ENR |= RCC_APB1ENR_DACEN;
            break;

        // APB2 Peripherals
        case TIM1_EN:
            RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
            break;
        case USART1_EN:
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
            break;
        case USART6_EN:
            RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
            break;
        case ADC1_EN:
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
            break;
        case SPI1_EN:
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
            break;
        case SYSCFG_EN:
            RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
            break;
        case EXTI_EN:
            RCC->APB2ENR |= RCC_APB2ENR_EXTITEN;
            break;
        case TIM9_EN:
            RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
            break;
        case TIM11_EN:
            RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
            break;
        case SPI5_EN:
            RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
            break;
    }
}