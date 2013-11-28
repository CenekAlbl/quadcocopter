/*
 * I2C.h
 *
 *  Created on: 22.5.2013
 *      Author: Vincek
 */


#ifndef I2C_H_
#define I2C_H_

#define I2C_PORT I2C1_MASTER_BASE



void setup_I2C();

int write_to_I2C(uint8_t device, uint8_t address, uint8_t data);

int read_from_I2C(uint8_t device, uint8_t address, uint8_t * buffer, int num_bytes);

#endif
