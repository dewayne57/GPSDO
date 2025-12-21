/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * I2C interface for bit-banged I2C on PIC18F27Q43. This code uses 
 * a bit-banged approach to implement I2C communication using
 * RC3 (SCL) and RC4 (SDA) pins configured for open-drain operation.
 * The choice of bit-banging allows for flexibility in timing and
 * control over the I2C bus without relying on dedicated hardware modules.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef I2C_H
#define    I2C_H
#include <xc.h>

/**
 * Define the function return codes 
 */
#define I2C_SUCCESS 0
#define I2C_ERROR   1
#define I2C_NACK    2
#define I2C_BUSY    3
#define I2C_TIMEOUT 4
#define I2C_INVALID_PARAM 5
#define I2C_BUS_ERROR 6
#define I2C_BUS_COLLISION 7

/**
 * @brief Write data to an I2C device
 * This function writes a buffer of data to the specified
 * I2C device address.
 * @param address The I2C device address
 * @param data Pointer to the data buffer to write
 * @param length The number of bytes to write
 * @return 0 on success, non-zero on failure
 */
uint8_t i2cWriteBuffer(uint8_t address, uint8_t *data, uint8_t length);

/**
 * @brief Write a single register on an I2C device
 * This function writes a single byte to a specific register
 * of the I2C device.
 * @param address The I2C device address
 * @param reg The register address to write to
 * @param data The data byte to write
 * @return 0 on success, non-zero on failure
 */
uint8_t i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t data);

/**
 * @brief Read data from an I2C device
 * This function reads a buffer of data from the specified
 * I2C device address.  Note, the dedicated i2c module requires that the 
 * read of a specific register be done using the sequence: 
 * 1. Start
 * 2. Address write + register
 * 3. Restart 
 * 4. Address read 
 * 5. Stop 
 * @param address The I2C device address
 * @param data Pointer to the data buffer to store the read data
 * @param length The number of bytes to read
 * @return 0 on success, non-zero on failure
 */
uint8_t i2cReadBuffer(uint8_t address, uint8_t *data, uint8_t length);

/**
 * @brief Read a single register from an I2C device
 * This function reads a single byte from a specific register
 * of the I2C device.
 * @param address The I2C device address
 * @param reg The register address to read from
 * @param data Pointer to store the read data byte
 * @return 0 on success, non-zero on failure
 */
uint8_t i2cReadRegister(uint8_t address, uint8_t reg, uint8_t *data);

/**
 * &brief Resets the state of the bus by clearing all error flags and assuming
 * an idle state. 
 * This function is useful for recovering from bus errors or
 * stuck conditions.
 * @return None
 */
void i2cResetBus(); 

#endif
