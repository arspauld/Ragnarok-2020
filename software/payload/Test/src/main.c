#include "stm32f410rx.h"
#include <string.h>
#include <stdio.h>

// PLL constant definitions
#define PLL_P 2
#define PLL_N 200
#define PLL_M 16

#define GPIO_PIN_5                 ((uint16_t) 1U<<5 )  /* Pin 5  selected    */
#define GPIO_PIN_13                ((uint16_t) 1U<<13)  /* Pin 5  selected    */

volatile uint8_t write_flag = 0;
volatile uint8_t adc_ready = 0;
volatile uint32_t* const val = (volatile uint32_t*) 0x0800C000; // Sets address of variable to a a spot in the Flash

void system_clock_init(void);
void serial_uart_init(uint32_t baud);
void pwm_init(uint8_t duty);
void user_button_init(void);
void i2c_init(void);
void adc_init(void);
void flash_init(void);
void delay(volatile uint32_t s);
void uart_write(char* str);
void i2c_write(uint8_t addr, uint8_t* data);
void i2c_read(uint8_t addr, uint8_t* buf, uint8_t numbytes);
uint16_t adc_read(void);
void reset(void);
void flash_write8(volatile uint8_t* addr, uint8_t val);
void flash_write16(volatile uint16_t* addr, uint16_t val);
void flash_write32(volatile uint32_t* addr, uint32_t val);
void flash_erase(uint8_t sector);
void flash_overwrite(volatile uint32_t* addresses[], uint32_t values[], uint8_t sector);

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
        USART2->SR &= ~USART_SR_RXNE;
        //buff[buffpos] = USART2->DR;
        //buffpos++;
        //buffpos %= sizeof(buff);
        //while(!(USART2->SR & USART_SR_TC));
        USART2->DR = USART2->DR;
        //while(!(USART2->SR & USART_SR_TC));
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

    uint32_t temp = *val;
    if(temp) flash_write32(val,temp/2);
    else flash_erase(3);
    sprintf(mess, "%lu -> %lu\n", temp, *val);
    uart_write(mess);

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
        if(adc_ready) sprintf(mess, "%lu -> %lu\n", temp, *val);//"%lu\n", adc_read());
        //uart_write(mess); // Writes the message buffer
        delay(10000000);
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
    
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE |  USART_CR1_UE; // Enables RX, TX, RX Interrupt, and USART
    //USART2->CR2 |= USART_CR2_CLKEN; // Enables the clock signal

    NVIC_EnableIRQ(USART2_IRQn);

    while(!(USART2->SR & USART_SR_TC)); // Clear TC Flag
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

    TIM5->PSC = 199; // Prescaler that results in a 40 kHz timer clock
    TIM5->ARR = 10000; // Automatic Reset value of timer, set to change at 2 Hz
    TIM5->CCR1 = (TIM5->ARR * duty) / 100; // Sets the capture point

    TIM5->CCMR1 |= (0b110 << 4); // Sets the Timer to PWM mode
    TIM5->CCER |= TIM_CCER_CC1E; // Enable Compare and Capture 1

    TIM5->DIER |= TIM_DIER_UIE; // Sets Timer update flag
    //NVIC_EnableIRQ(TIM5_IRQn); // Attaches interrupt
    TIM5->CR1 |= TIM_CR1_CEN; // Enables the timer clock
}

void user_button_init(void)
{
    // Enables An interrupt that is connected to the USer Button (Blue)
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // Connects the 13th channel of EXTI to Port C (Port C, Pin 13)
    EXTI->IMR |= EXTI_IMR_MR13; // Enables the Interrupt of Channel 13
    EXTI->FTSR |= EXTI_FTSR_TR13; // Enables Falling Edge Detection on Channel 13
    NVIC_EnableIRQ(EXTI15_10_IRQn); // Enables the EXTI Interrupt for Channels 10 through 15
}

void i2c_init(void)
{
    /*The following is the required sequence in master mode.
     * • Program the peripheral input clock in I2C_CR2 Register in order to generate correct
     * timings
     * • Configure the clock control registers
     * • Configure the rise time register
     * • Program the I2C_CR1 register to enable the peripheral
     * • Set the START bit in the I2C_CR1 register to generate a Start condition
     */

    
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enables the I2C1 periphery (pins B6 and B7)
    I2C1->CR2 |= (50) << I2C_CR2_FREQ_Pos; // Sets the frequeny to 50 MHz
    I2C1->CCR |= I2C_CCR_FS | I2C_CCR_DUTY | (25) << I2C_CCR_CCR_Pos; // Sets the clock control such that fast mode is enabled with DUTY set to 1 and the time prescaler set to 25
    I2C1->TRISE |= (16) << I2C_TRISE_TRISE_Pos; // Programs a factor of 16 to relate the maximum rise time in Fast mode to the input clock
    I2C1->CR1 |= I2C_CR1_PE; // Enables I2C bus
}

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

void flash_init(void)
{
    // Unlocks Flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR |= FLASH_CR_PSIZE_1; // Sets 32 bit parallelism (writes 32 bytes at a time)
}

void delay(volatile uint32_t s)
{
    for(s; s>0; s--);
}

void uart_write(char* str)
{
    // Loops through the input character string and transmits it via USART
    for(uint8_t i = 0; i < strlen(str); i++)
    {
        USART2->DR = str[i];
        while(!(USART2->SR & USART_SR_TC)); // Waits until data byte is transmitted
    }
}

void i2c_write(uint8_t addr, uint8_t* data)
{
    /* Recieving
     * Set Start bit
     * Set ADDR
     * Set DR
     * Use RxNE bit to continuously recieve
     * Set Stop bit
     */

    uint8_t read = 0;
    I2C1->CR1 |= I2C_CR1_START; // Sends the start condition
    if(I2C1->SR1 & I2C_SR1_SB)
    {
        I2C1->DR = addr | read; // Places the address into the data register to send
        if(I2C1->SR1 & I2C_SR1_ADDR) // Checks SR1 register
        {
            if(I2C1->SR2 & I2C_SR2_TRA) // Checks SR2 register
            {
                for(uint8_t i = 0; i < sizeof(data); i++)
                {
                    I2C1->DR = data[i]; // Places each byte of data
                    if(i == sizeof(data) - 1) // Conducts process 
                    {
                        I2C1->CR1 |= I2C_CR1_STOP; // Creates stop condition after last byte is transmitted
                    }
                    while(!(I2C1->SR1 & I2C_SR1_TXE)); // Waits until the data was sent
                }
            }
        }
    }
}

void i2c_read(uint8_t addr, uint8_t* buf, uint8_t numbytes)
{
    /* Transmission
     * Set Start bit
     * Set ADDR
     * Set DR
     * Use TxE bit to continuously communicate
     * Set Stop bit to end
     */

    uint8_t read = 1;
    I2C1->CR1 |= I2C_CR1_START | I2C_CR1_ACK; // Sends the start condition and enables acknowledge return
    if(I2C1->SR1 & I2C_SR1_SB)
    {
        I2C1->DR = addr | read; // Places the address into the data register to send
        if(I2C1->SR1 & I2C_SR1_ADDR) // Checks SR1 register
        {
            if(!(I2C1->SR2 & I2C_SR2_TRA)) // Checks SR2 register
            {
                for(uint8_t i = 0; i < numbytes; i++)
                {
                    while(!(I2C1->SR1 & I2C_SR1_RXNE)); // Waits until the data was recieved
                    buf[i] = I2C1->DR; // Places each byte of data
                    if(i == numbytes - 2) // Conducts the below process after reading the second to last byte
                    {
                        I2C1->CR1 &= ~I2C_CR1_ACK; // Clears ACK bit
                        I2C1->CR1 |= I2C_CR1_STOP; // Creates stop condition after last byte is transmitted
                    }
                }
            }
        }
    }
}

uint16_t adc_read(void)
{
    // Reads ADC Data Register and restarts conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
    return (uint16_t) ADC1->DR;
}

void reset(void)
{
    flash_erase(3);
    //flash_erase(4);
    NVIC_SystemReset(); // Built in function that performs a software reset to the system
}

void flash_write8(volatile uint8_t* addr, uint8_t val) 
{
    // Writes value to Flash without checking memory and for 8 bit parallelism
    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR |= FLASH_CR_PG; // Enables Programming
    *addr = val; // Programs value (only bits from 1->0 is program only)
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PG; // Disables programming
}

void flash_write16(volatile uint16_t* addr, uint16_t val)
{
    // Writes value to Flash without checking memory and for 16 bit parallelism
    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR |= FLASH_CR_PG; // Enables Programming
    *addr = val; // Programs value (only bits from 1->0 is program only)
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PG; // Disables programming
}

void flash_write32(volatile uint32_t* addr, uint32_t val)
{
    // Writes value to Flash without checking memory and for 32 bit parallelism
    // Will most likely always be used with our voltage input
    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR |= FLASH_CR_PG; // Enables Programming
    *addr = val; // Programs value (only bits from 1->0 is program only)
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PG; // Disables programming
}

void flash_erase(uint8_t sector)
{
    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR &= ~FLASH_CR_SNB; // Clears Previous Sector Selection
    FLASH->CR |= ((sector & 0xF) << FLASH_CR_SNB_Pos) | FLASH_CR_SER; // Sector erase and the Sector Erase Bit (Sector address is limited to 4 bits)
    FLASH->CR |= FLASH_CR_STRT; // Starts the erase
    while(FLASH->SR & FLASH_SR_BSY);
}


void flash_overwrite(volatile uint32_t* addresses[], uint32_t values[], uint8_t sector) // Not tested
{
    // Erase
    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR &= ~FLASH_CR_SNB; // Clears Previous Sector Selection
    FLASH->CR |= ((sector & 0xF) << FLASH_CR_SNB_Pos) | FLASH_CR_SER; // Sector erase and the Sector Erase Bit (Sector address is limited to 4 bits)
    FLASH->CR |= FLASH_CR_STRT; // Starts the erase
    while(FLASH->SR & FLASH_SR_BSY);

    // Program
    FLASH->CR |= FLASH_CR_PG; // Enables Programming
    for(uint8_t i = 0, i < (sizeof(values) / 4); i++)
    {
        *(addresses[i]) = values[i];
    }
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PG; // Disables programming
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