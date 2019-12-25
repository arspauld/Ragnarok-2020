#include "I2C.h"

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