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

// Maximum polling iterations while waiting for a line to release high
#define I2C_LINE_HIGH_TIMEOUT 2000

/****************************************************************************
 * forward define internal functions not exposed via i2c.h header file
 ****************************************************************************/
static uint8_t _i2cStart(void);
static uint8_t _i2cRestart(void);
static uint8_t _i2cStop(void);
static uint8_t _i2cSclHigh(void);
static void _i2cSclLow(void);
static uint8_t _i2cSdaHigh(void);
static void _i2cSdaLow(void);
static uint8_t _i2cSdaRead(void);
static uint8_t _i2cWriteByte(uint8_t data);
static uint8_t _i2cReadByte(uint8_t ack, uint8_t *value);
static uint8_t _i2cWaitLineHigh(uint8_t mask);

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
    result = _i2cStart();
    if (result != I2C_SUCCESS)
    {
        return result;
    }

    // Send address with write bit (0)
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x00));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return result;
    }

    // Send data bytes
    for (i = 0; i < length; i++)
    {
        result = _i2cWriteByte(data[i]);
        if (result != I2C_SUCCESS)
        {
            _i2cStop();
            return result;
        }
    }

    // Send stop condition
    return _i2cStop();
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
    if (length == 0)
    {
        return I2C_SUCCESS;
    }

    uint8_t i;
    uint8_t result;

    // Send start condition
    result = _i2cStart();
    if (result != I2C_SUCCESS)
    {
        return result;
    }

    // Send address with read bit (1)
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x01));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return result;
    }

    // Read data bytes
    for (i = 0; i < length; i++)
    {
        // ACK all bytes except the last one (NACK the last byte)
        result = _i2cReadByte(i < (length - 1), &data[i]);
        if (result != I2C_SUCCESS)
        {
            _i2cStop();
            return result;
        }
    }

    // Send stop condition
    return _i2cStop();
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
    result = _i2cStart();
    if (result != I2C_SUCCESS)
    {
        return result;
    }

    // Send address with write bit to write register address
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x00));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return result;
    }

    // Send register address
    result = _i2cWriteByte(reg);
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return result;
    }

    // Restart condition (maintains bus ownership)
    result = _i2cRestart();
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return result;
    }

    // Send address with read bit
    result = _i2cWriteByte((uint8_t)((address << 1) | 0x01));
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return result;
    }

    // Read the data byte (NACK since it's the only/last byte)
    result = _i2cReadByte(0, data);
    if (result != I2C_SUCCESS)
    {
        _i2cStop();
        return result;
    }

    // Send stop condition
    return _i2cStop();
}

/****************************************************************************
 * PRIVATE/INTERNAL FUNCTIONS
 ****************************************************************************/

uint8_t _i2cSclHigh(void)
{
    TRISC |= SCL; // Release SCL
    return _i2cWaitLineHigh(SCL);
}

void _i2cSclLow(void)
{
    LATC &= ~SCL;  // Output low
    TRISC &= ~SCL; // Set as output
    I2C_HALF_DELAY();
}

uint8_t _i2cSdaHigh(void)
{
    TRISC |= SDA; // Release SDA
    return _i2cWaitLineHigh(SDA);
}

void _i2cSdaLow(void)
{
    LATC &= ~SDA;  // Output low
    TRISC &= ~SDA; // Set as output
    I2C_HALF_DELAY();
}

uint8_t _i2cSdaRead(void)
{
    TRISC |= SDA; // Ensure SDA is input
    I2C_HALF_DELAY();
    return PORTC & SDA; // Read SDA state
}

uint8_t _i2cStart(void)
{
    uint8_t status;

    // Start condition: SDA goes low while SCL is high
    status = _i2cSdaHigh(); // Ensure SDA is high
    if (status != I2C_SUCCESS)
    {
        return status;
    }
    status = _i2cSclHigh(); // Ensure SCL is high
    if (status != I2C_SUCCESS)
    {
        return status;
    }
    I2C_DELAY();
    _i2cSdaLow(); // SDA low while SCL high = START
    I2C_DELAY();
    _i2cSclLow(); // SCL low to complete start
    return I2C_SUCCESS;
}

uint8_t _i2cRestart(void)
{
    uint8_t status;

    // Restart condition: SDA goes high, then low while SCL is high
    // (without sending a stop first - maintains bus ownership)
    status = _i2cSdaHigh(); // SDA high first
    if (status != I2C_SUCCESS)
    {
        return status;
    }
    I2C_DELAY();
    status = _i2cSclHigh(); // SCL high
    if (status != I2C_SUCCESS)
    {
        return status;
    }
    I2C_DELAY();
    _i2cSdaLow(); // SDA low while SCL high = RESTART
    I2C_DELAY();
    _i2cSclLow(); // SCL low to complete restart
    return I2C_SUCCESS;
}

uint8_t _i2cStop(void)
{
    uint8_t status;

    // Stop condition: SDA goes high while SCL is high
    _i2cSdaLow(); // Ensure SDA is low
    I2C_DELAY();
    status = _i2cSclHigh(); // SCL high first
    if (status != I2C_SUCCESS)
    {
        return status;
    }
    I2C_DELAY();
    status = _i2cSdaHigh(); // SDA high while SCL high = STOP
    if (status != I2C_SUCCESS)
    {
        return status;
    }
    I2C_DELAY();
    return I2C_SUCCESS;
}

uint8_t _i2cWriteByte(uint8_t data)
{
    uint8_t i;

    // Send 8 bits, MSB first
    for (i = 0; i < 8; i++)
    {
        uint8_t status;
        if (data & 0x80)
        {
            status = _i2cSdaHigh();
        }
        else
        {
            _i2cSdaLow();
            status = I2C_SUCCESS;
        }

        if (status != I2C_SUCCESS)
        {
            return status;
        }

        status = _i2cSclHigh(); // Clock the bit
        if (status != I2C_SUCCESS)
        {
            return status;
        }
        _i2cSclLow();
        data <<= 1;
    }

    // Check for ACK
    uint8_t status = _i2cSdaHigh(); // Release SDA for ACK
    if (status != I2C_SUCCESS)
    {
        return status;
    }

    status = _i2cSclHigh(); // Clock the ACK bit
    if (status != I2C_SUCCESS)
    {
        return status;
    }

    uint8_t ack = !_i2cSdaRead(); // ACK is low
    _i2cSclLow();

    return ack ? I2C_SUCCESS : I2C_NACK;
}

uint8_t _i2cReadByte(uint8_t ack, uint8_t *value)
{
    uint8_t data = 0;
    uint8_t i;

    uint8_t status = _i2cSdaHigh(); // Release SDA for reading
    if (status != I2C_SUCCESS)
    {
        return status;
    }

    // Read 8 bits, MSB first
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        status = _i2cSclHigh();
        if (status != I2C_SUCCESS)
        {
            return status;
        }
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
        status = _i2cSdaHigh(); // NACK
        if (status != I2C_SUCCESS)
        {
            return status;
        }
    }

    status = _i2cSclHigh();
    if (status != I2C_SUCCESS)
    {
        return status;
    }
    _i2cSclLow();

    *value = data;
    return I2C_SUCCESS;
}

static uint8_t _i2cWaitLineHigh(uint8_t mask)
{
    uint16_t timeout = I2C_LINE_HIGH_TIMEOUT;

    while ((PORTC & mask) == 0)
    {
        if (timeout-- == 0)
        {
            return I2C_TIMEOUT;
        }
        __delay_us(5);
    }

    I2C_HALF_DELAY();
    return I2C_SUCCESS;
}

/****************************************************************************
 * End of i2c.c
 * **************************************************************************/