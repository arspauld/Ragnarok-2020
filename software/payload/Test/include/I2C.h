#ifndef I2C_H_
#define I2C_H_

#include "system.h"


void i2c_init(void);
void i2c_write(uint8_t addr, uint8_t* data);
void i2c_read(uint8_t addr, uint8_t* buf, uint8_t numbytes);

#endif