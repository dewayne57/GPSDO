/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Bit-banging I2C implementation for PIC18F27Q43
 * Uses RC3 (SCL) and RC4 (SDA) with open drain configuration   
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */
#include <xc.h>
#include "i2c.h"
#include "config.h"

/****************************************************************************
 * Bit-banging I2C implementation for PIC18F27Q43
 * Uses RC3 (SCL) and RC4 (SDA) with open drain configuration
 ****************************************************************************/

// I2C timing delays for ~100kHz operation
#define I2C_DELAY() __delay_us(5)      // 5us delay for ~100kHz
#define I2C_HALF_DELAY() __delay_us(2) // Half period delay

/****************************************************************************
 * forward define internal functions not exposed via i2c.h header file
 ****************************************************************************/
void _i2cStart(void);
void _i2cRestart(void);
void _i2cStop(void);
void _i2cSclHigh(void);
void _i2cSclLow(void);
void _i2cSdaHigh(void);
void _i2cSdaLow(void);
uint8_t _i2cSdaRead(void);
uint8_t _i2cWriteByte(uint8_t data);
uint8_t _i2cReadByte(uint8_t ack);

/****************************************************************************
 * PUBLIC API FUNCTIONS
 ****************************************************************************/

/****************************************************************************
 * Function: i2cWriteBuffer
 * Description: Write data to an I2C device
 * Parameters:
 *   address - The I2C device address
 *   data - Pointer to the data buffer to write
 *   length - The number of bytes to write
 * Returns: 0 on success, non-zero on failure
 * *************************************************************************/
uint8_t i2cWriteBuffer(uint8_t address, uint8_t *data, uint8_t length)
{
    uint8_t i;
    uint8_t result;

    // Send start condition
    _i2cStart();

    // Send address with write bit (0)
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x00));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return I2C_NACK;
    }

    // Send data bytes
    for (i = 0; i < length; i++)
    {
        result = _i2cWriteByte(data[i]);
        if (result != I2C_SUCCESS)
        {
            _i2cStop();
            return I2C_NACK;
        }
    }

    // Send stop condition
    _i2cStop();
    return I2C_SUCCESS;
}

/****************************************************************************
 * Function: i2cWriteRegister
 * Description: Write a single register on an I2C device.  
 * Parameters:
 *   address - The I2C device address
 *   reg - The register address to write to
 *   data - The data byte to write
 * Returns: 0 on success, non-zero on failure
 * *************************************************************************/
uint8_t i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2];

    buffer[0] = reg;
    buffer[1] = data;
    return i2cWriteBuffer(address, buffer, 2);
}

/****************************************************************************
 * Function: i2cReadBuffer
 * Description: Read data from an I2C device
 * Parameters:
 *   address - The I2C device address
 *   data - Pointer to the data buffer to store the read data
 *   length - The number of bytes to read
 * Returns: 0 on success, non-zero on failure
 * *************************************************************************/
uint8_t i2cReadBuffer(uint8_t address, uint8_t *data, uint8_t length)
{
    uint8_t i;
    uint8_t result;

    // Send start condition
    _i2cStart();

    // Send address with read bit (1)
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x01));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return I2C_NACK;
    }

    // Read data bytes
    for (i = 0; i < length; i++)
    {
        // ACK all bytes except the last one (NACK the last byte)
        data[i] = _i2cReadByte(i < (length - 1));
    }

    // Send stop condition
    _i2cStop();
    return I2C_SUCCESS;
}

/****************************************************************************
 * Function: i2cReadRegister
 * Description: Read a single register from an I2C device
 * Parameters:
 *   address - The I2C device address
 *   reg - The register address to read from
 *   data - Pointer to store the read data byte
 * Returns: 0 on success, non-zero on failure
 * *************************************************************************/
uint8_t i2cReadRegister(uint8_t address, uint8_t reg, uint8_t *data)
{
    uint8_t result;

    // Send start condition
    _i2cStart();

    // Send address with write bit to write register address
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x00));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return I2C_NACK;
    }

    // Send register address
    result = _i2cWriteByte(reg);
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return I2C_NACK;
    }

    // Restart condition (maintains bus ownership)
    _i2cRestart();

    // Send address with read bit
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x01));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return I2C_NACK;
    }

    // Read the data byte (NACK since it's the only/last byte)
    *data = _i2cReadByte(0);

    // Send stop condition
    _i2cStop();
    return I2C_SUCCESS;
}

/*************************************************************************
 * Function: i2cResetBus
 * Description: Resets the state of the bus by clearing all error flags and
 * assuming an idle state. This function is useful for recovering from bus
 * errors or stuck conditions.
 * Parameters: None
 * Returns: None
 * ***********************************************************************/
void i2cResetBus(void)
{
    // Reset bus to idle state (both lines high)
    _i2cSdaHigh();
    _i2cSclHigh();
    I2C_DELAY();
}

/****************************************************************************
 * PRIVATE/INTERNAL FUNCTIONS
 ****************************************************************************/

void _i2cSclHigh(void)
{
    TRISC |= SCL;       // Ensure SCL is input
    I2C_HALF_DELAY();
}

void _i2cSclLow(void)
{
    LATC &= ~SCL;       // Output low
    TRISC &= ~SCL;      // Set as output
    I2C_HALF_DELAY();
}

void _i2cSdaHigh(void)
{
    TRISC |= SDA;       // Ensure SDA is input
    I2C_HALF_DELAY();
}

void _i2cSdaLow(void)
{
    LATC &= ~SDA;        // Output low
    TRISC &= ~SDA;       // Set as output
    I2C_HALF_DELAY();
}

uint8_t _i2cSdaRead(void)
{
    TRISC |= SDA;       // Ensure SDA is input
    I2C_HALF_DELAY();
    return PORTC & SDA; // Read SDA state
}

void _i2cStart(void)
{
    // Start condition: SDA goes low while SCL is high
    _i2cSdaHigh(); // Ensure SDA is high
    _i2cSclHigh(); // Ensure SCL is high
    I2C_DELAY();
    _i2cSdaLow(); // SDA low while SCL high = START
    I2C_DELAY();
    _i2cSclLow(); // SCL low to complete start
}

void _i2cRestart(void)
{
    // Restart condition: SDA goes high, then low while SCL is high
    // (without sending a stop first - maintains bus ownership)
    _i2cSdaHigh(); // SDA high first
    I2C_DELAY();
    _i2cSclHigh(); // SCL high
    I2C_DELAY();
    _i2cSdaLow(); // SDA low while SCL high = RESTART
    I2C_DELAY();
    _i2cSclLow(); // SCL low to complete restart
}

void _i2cStop(void)
{
    // Stop condition: SDA goes high while SCL is high
    _i2cSdaLow(); // Ensure SDA is low
    I2C_DELAY();
    _i2cSclHigh(); // SCL high first
    I2C_DELAY();
    _i2cSdaHigh(); // SDA high while SCL high = STOP
    I2C_DELAY();
}

uint8_t _i2cWriteByte(uint8_t data)
{
    uint8_t i;

    // Send 8 bits, MSB first
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            _i2cSdaHigh();
        }
        else
        {
            _i2cSdaLow();
        }
        _i2cSclHigh(); // Clock the bit
        _i2cSclLow();
        data <<= 1;
    }

    // Check for ACK
    _i2cSdaHigh();                // Release SDA for ACK
    _i2cSclHigh();                // Clock the ACK bit
    uint8_t ack = !_i2cSdaRead(); // ACK is low
    _i2cSclLow();

    return ack ? I2C_SUCCESS : I2C_NACK;
}

uint8_t _i2cReadByte(uint8_t ack)
{
    uint8_t data = 0;
    uint8_t i;

    _i2cSdaHigh(); // Release SDA for reading

    // Read 8 bits, MSB first
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        _i2cSclHigh();
        if (_i2cSdaRead())
        {
            data |= 1;
        }
        _i2cSclLow();
    }

    // Send ACK/NACK
    if (ack)
    {
        _i2cSdaLow(); // ACK
    }
    else
    {
        _i2cSdaHigh(); // NACK
    }
    _i2cSclHigh();
    _i2cSclLow();

    return data;
}

/****************************************************************************
 * End of i2c.c
 * **************************************************************************/