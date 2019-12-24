#ifndef FLASH_H_
#define FLASH_H_

#include "system.h"

void flash_init(void);
void flash_write8(volatile uint8_t* addr, uint8_t val);
void flash_write16(volatile uint16_t* addr, uint16_t val);
void flash_write32(volatile uint32_t* addr, uint32_t val);
void flash_erase(uint8_t sector);
void flash_overwrite(volatile uint32_t* addresses[], uint32_t values[], uint16_t length, uint8_t sector);

#endif /* FLASH_H_ */