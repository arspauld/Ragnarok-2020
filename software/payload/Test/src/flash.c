#include "flash.h"

void flash_init(void)
{
    // Unlocks Flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR |= FLASH_CR_PSIZE_1; // Sets 32 bit parallelism (writes 32 bytes at a time)
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


void flash_overwrite(volatile uint32_t* addresses[], uint32_t values[], uint16_t length, uint8_t sector) // Not tested
{
    // Erase
    while(FLASH->SR & FLASH_SR_BSY); // Waits until Flash is finished
    FLASH->CR &= ~FLASH_CR_SNB; // Clears Previous Sector Selection
    FLASH->CR |= ((sector & 0xF) << FLASH_CR_SNB_Pos) | FLASH_CR_SER; // Sector erase and the Sector Erase Bit (Sector address is limited to 4 bits)
    FLASH->CR |= FLASH_CR_STRT; // Starts the erase
    while(FLASH->SR & FLASH_SR_BSY);

    // Program
    FLASH->CR |= FLASH_CR_PG; // Enables Programming
    for(uint8_t i = 0; i < length; i++)
    {
        *(addresses[i]) = values[i];
    }
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PG; // Disables programming
}