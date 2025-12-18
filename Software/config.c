/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * This module contains the system initialization code for the GPSDO project.
 * It sets up the microcontroller's I/O ports, interrupt system, and peripheral
 * modules required for operation.  It is called once at system startup from
 * main().
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <xc.h>
#include <string.h>
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "menu.h"
#include "encoder.h"
#include "smt.h"
#include "dac.h"
#include "control.h"
#include "gps.h"

extern uint8_t buffer[16];

/****************************************************************************/
/*                                                                          */
/* Initialize the system                                                    */
/*                                                                          */
/****************************************************************************/
void initialize(void)
{
    // Set up interrupts
    INTCON0bits.GIEH = 0; // Turn off high priority interrupts
    INTCON0bits.GIEL = 0; // And low priority interrupts too

    // Clear all interrupt enables
    PIE0 = 0x00;
    PIE2 = 0x00;
    PIE3 = 0x00;
    PIE4 = 0x00;
    PIE5 = 0x00;
    PIE6 = 0x00;
    PIE7 = 0x00;
    PIE8 = 0x00;
    PIE9 = 0x00;
    PIE10 = 0x00;
    PIE11 = 0x00;
    PIE12 = 0x00;
    PIE13 = 0x00;
    PIE14 = 0x00;
    PIE15 = 0x00;

    // Clear all interrupt requests
    PIR0 = 0x00;
    PIR1 = 0x00;
    PIR2 = 0x00;
    PIR3 = 0x00;
    PIR4 = 0x00;
    PIR5 = 0x00;
    PIR6 = 0x00;
    PIR7 = 0x00;
    PIR8 = 0x00;
    PIR9 = 0x00;
    PIR10 = 0x00;
    PIR11 = 0x00;
    PIR12 = 0x00;
    PIR13 = 0x00;
    PIR14 = 0x00;
    PIR15 = 0x00;

    // Set up external interrupt 0 (active-high from GPS module)
    INTCON0bits.IPEN = 1;       // Use priority interrupts
    INTCON0bits.INT0EDG = 1;    // Rising edge triggers INT0
    PIR1bits.INT0IF = 0;        // Clear any pending INT0 flag
    IPR1bits.INT0IP = 1;        // Make INT0 high priority (vectored)
    PIE1bits.INT0IE = 1;        // Enable External Interrupt 0

    // Enable all peripheral modules that are required and disable
    // all others to save power
    PMD0bits.SYSCMD = 0;        // System clock network enabled
    PMD0bits.FVRMD = 1;         // Fixed voltage reference disabled
    PMD0bits.HLVDMD = 1;        // High/low voltage detect disabled
    PMD0bits.CRCMD = 1;         // CRC module disabled
    PMD0bits.SCANMD = 1;        // SCAN module disabled
    PMD0bits.CLKRMD = 0;        // Clock reference module disabled
    PMD0bits.IOCMD = 0;         // Interrupt on change module enabled
    PMD1bits.SMT1MD = 0;        // SMT1 module enabled (for RF counting)
    PMD1bits.TMR0MD = 1;        // Timer0 module disabled
    PMD1bits.TMR1MD = 1;        // Timer1 module disabled
    PMD1bits.TMR2MD = 1;        // Timer2 module disabled
    PMD1bits.TMR3MD = 1;        // Timer3 module disabled
    PMD1bits.TMR4MD = 0;        // Timer4 module enabled
    PMD1bits.TMR5MD = 1;        // Timer5 module disabled
    PMD1bits.TMR6MD = 1;        // Timer6 module disabled
    PMD3bits.ACTMD = 1;         // No active clock tuning needed
    PMD3bits.DAC1MD = 0;        // DAC1 module enabled
    PMD3bits.ADCMD = 0;         // ADC module enabled
    PMD3bits.CM2MD = 1;         // Comparator 2 module disabled
    PMD3bits.CM1MD = 1;         // Comparator 1 module disabled
    PMD3bits.ZCDMD = 1;         // Zero-Cross detector disabled
    PMD4bits.CWG1MD = 1;        // CWG1 module disabled
    PMD4bits.CWG2MD = 1;        // CWG2 module disabled
    PMD4bits.CWG3MD = 1;        // CWG3 module disabled
    PMD4bits.DSM1MD = 1;        // DSM1 module disabled
    PMD4bits.NCO1MD = 1;        // NCO1 module disabled
    PMD4bits.NCO2MD = 1;        // NCO2 module disabled
    PMD4bits.NCO3MD = 1;        // NCO3 module disabled
    PMD5bits.PWM1MD = 1;        // Pulse-Width Modulator 1 disabled
    PMD5bits.PWM2MD = 1;        // Pulse-Width Modulator 2 disabled
    PMD5bits.PWM3MD = 1;        // Pulse-Width Modulator 3 disabled
    PMD5bits.CCP1MD = 1;        // Capture-Compare 1 disabled
    PMD5bits.CCP2MD = 1;        // Capture-Compare 2 disabled
    PMD5bits.CCP3MD = 1;        // Capture-Compare 3 disabled
    PMD6bits.I2C1MD = 0;        // I2C1 module enabled
    PMD6bits.SPI1MD = 1;        // SPI1 module disabled
    PMD6bits.SPI2MD = 1;        // SPI2 module disabled
    PMD6bits.U1MD = 0;          // UART 1 enabled
    PMD6bits.U2MD = 1;          // UART 2 disabled
    PMD6bits.U3MD = 1;          // UART 3 disabled
    PMD6bits.U4MD = 1;          // UART 4 disabled
    PMD6bits.U5MD = 1;          // UART 5 disabled
    PMD7bits.CLC1MD = 1;        // Configurable Logic Cell 1 disabled
    PMD7bits.CLC2MD = 1;        // Configurable Logic Cell 2 disabled
    PMD7bits.CLC3MD = 1;        // Configurable Logic Cell 3 disabled
    PMD7bits.CLC4MD = 1;        // Configurable Logic Cell 4 disabled
    PMD7bits.CLC5MD = 1;        // Configurable Logic Cell 5 disabled
    PMD7bits.CLC6MD = 1;        // Configurable Logic Cell 6 disabled
    PMD7bits.CLC7MD = 1;        // Configurable Logic Cell 7 disabled
    PMD7bits.CLC8MD = 1;        // Configurable Logic Cell 8 disabled
    PMD8bits.DMA1MD = 1;        // DMA1 module disabled
    PMD8bits.DMA2MD = 1;        // DMA2 module disabled
    PMD8bits.DMA3MD = 1;        // DMA3 module disabled
    PMD8bits.DMA4MD = 1;        // DMA4 module disabled
    PMD8bits.DMA5MD = 1;        // DMA5 module disabled
    PMD8bits.DMA6MD = 1;        // DMA6 module disabled
    
    // Set up port A
    //
    // All port A pins (except RA6) are used as analog inputs and outputs and
    // are used to sample the VRef level and to supply a default VRef if the
    // OCXO does not.  RA6 is configured as a digital output and outputs the
    // system clock for diagnostic purposes.
    //
    // RA0 - VRef Feedback
    // RA1 - Internal VRef output
    // RA2 - MFINTOSC output for debugging (500kHz clock)
    // RA3 - Unused
    // RA4 - Unused
    // RA5 - Unused
    // RA6 - Digital clock output
    // RA7 - Unused
    TRISA = 0xFF - VREF_FB - INT_REF - CLOCK_OUT; // RA0,RA1,RA6 as outputs
    ANSELA = 0xFF - VREF_FB - INT_REF;            // RA0,RA1 as analog inputs, RA6 digital
    LATA = 0x00;
    ODCONA = 0x00;
    WPUA = 0x00;
    SLRCONA = 0x00;
    INLVLA = 0x00;

    // Set up port B
    //
    // TTL levels, no open drain, no analog, slew rate not limited, all pins
    // use Schmitt trigger inputs.
    //
    // RB0 - GPS_TX (output FROM GPS to MCU)
    // RB1 - GPS_RX (input TO GPS from MCU)
    // RB2 - Interrupt input (weak pull up enabled)
    // RB3 - Unused
    // RB4 - Unused
    // RB5 - Unused
    // RB6 - ICSP/Debug PGC
    // RB7 - ICSP/Debug PGD
    TRISB = 0xFF - GPS_RX - INT; // RB0, RB2 inputs; RB1 output
    ANSELB = 0x00;               // All digital
    LATB = 0x00;
    ODCONB = 0x00;
    WPUB = 0x04;
    SLRCONB = 0x00;
    INLVLB = 0x00;

    // Set up port C
    //
    // Port C is all digital and is used to interface with the I/O extender,
    // the VCO voltage generator DAC, the external EEPROM to store and fetch the
    // application configuration and VCO settings, the encoder, and the
    // digital RF and 1PPS signals from the OCXO and GPS respectively.
    //
    // RC0 - 1PPS signal from GPS
    // RC1 - 10 MHz signal from OCXO
    // RC2 - RESET_N output to reset I/O extender and GPS (pull ups enabled
    //       and open drain)
    // RC3 - I2C SCL (i2c pull ups enabled, open drain)
    // RC4 - I2C SDA (i2c pull ups enabled, open drain)
    // RC5 - Encoder Phase A (pull ups enabled)
    // RC6 - Encoder Phase B (pull ups enabled)
    // RC7 - Encoder Enter_N Switch (pull-ups enabled)
    TRISC = 0xFF - RESET_N; // All inputs - I2C pins MUST be inputs for proper operation
    ANSELC = 0x00;
    LATC = 0x00;
    // Enable weak pull-ups on I2C and encoder pins
    WPUC = SDA + SCL + PHASE_A + PHASE_B + ENTER_N;
    INLVLC = 0x00;

    // Open drain on I2C and encoder pins
    ODCONC = SDA + SCL + PHASE_A + PHASE_B + ENTER_N;

    RC3I2C = 0x00;         // Disable I2C thresholds on RC3 (SCL)
    RC4I2C = 0x00;         // Disable I2C thresholds on RC4 (SDA)
    SLRCONCbits.SLRC3 = 0; // maximum slew rate for bit-bang timing
    SLRCONCbits.SLRC4 = 0; // maximum slew rate for bit-bang timing

    // set up timer 4 for delays in bit-banging
    T4CLKCON = 0x06; // Use MFINTOSC for clock source
    T4HLT = 0x00;
    T4RST = 0x00;
    T4PR = 0x05; // Shorter period for bit-bang timing
    T4TMR = 0x00;
    PIR11bits.TMR4IF = 0; // Clear timer 4 interrupt flag
    T4CON = 0x84;

    // Set up PPS for interrupt
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // unlock

    // Map only the interrupt signal to INT0
    INT0PPS = 0x0A; // RB2 -> INT0 input

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // lock

    // Reset sequence for MCP23017
    PORTC &= 0xFF - RESET_N; // Hold MCP23017's in reset
    __delay_ms(100);         // Hold reset for proper duration
    PORTC |= RESET_N;        // Enable the MCP23017's
    __delay_ms(50);          // Allow MCP23017 to stabilize after reset

    // Initialize I2C bus to idle state (both lines high)
    LATC |= SCL + SDA;  // Drive both lines high (via pull-ups)
    TRISC |= SCL + SDA; // both lines as inputs = high
    __delay_ms(10);     // Allow bus to stabilize

    // Initialize MCP23017 #1 for sequential mode to initialize all the registers
    i2cWriteRegister(MCP23017_ADDRESS, 0x0A, 0x80); // banked, sequential, no interrupts

    // Initialize MCP23017 Port A (status LEDs active low)
    buffer[0] = IODIRA; // Write IODIRA register
    buffer[1] = 0x00;   // All pins are output
    buffer[2] = 0x00;   // pins are not inverted
    buffer[3] = 0x00;   // No IOC
    buffer[4] = 0x00;   // default compare register
    buffer[5] = 0x00;   // compare against previous value
    buffer[6] = 0x80;   // re-write iocon for now
    buffer[7] = 0x00;   // disable pullups
    buffer[8] = 0x00;   // Ignored
    buffer[9] = 0x00;   // not used
    buffer[10] = 0xFF;  // All pins high
    i2cWriteBuffer(MCP23017_ADDRESS, buffer, 11);

    // Initialize MCP23017 #1 Port B (LCD interface)
    buffer[0] = IODIRB; // Write IODIRB register
    buffer[1] = 0x00;   // All pins output
    buffer[2] = 0x00;   // pins are NOT inverted
    buffer[3] = 0x00;   // No IOC
    buffer[4] = 0x00;   // Default compare
    buffer[5] = 0x00;   // Compare against prev value
    buffer[6] = 0x80;   // Re-write IOCON for now
    buffer[7] = 0x00;   // Disable pullups
    buffer[8] = 0x00;   // Ignored
    buffer[9] = 0x00;   // not used
    buffer[10] = 0x00;  // All pins low
    i2cWriteBuffer(MCP23017_ADDRESS, buffer, 11);

    // Now, set the IO Configuration to byte mode
    i2cWriteRegister(MCP23017_ADDRESS, IOCON, 0xE4); // Change to byte mode

    // Clear any pending MCP23017 interrupt from initialization (read INTFA then GPIOA)
    i2cReadRegister(MCP23017_ADDRESS, INTFA, buffer);

    // Load persistent system configuration from EEPROM (falls back to defaults)
    config_load((system_config_t *)buffer); // buffer used as temp storage

    // Initialize encoder GPIOs and state
    // encoder_init();

    // Initialize menu state
    // menu_init();

    // Initialize SMT1 counting and DAC-based control loop
    // smt_init();
    // dac_init();
    // control_init();

    // Enable global interrupts
    INTCON0bits.GIEH = 1; // High priority enabled
    INTCON0bits.GIEL = 1; // Low priority enabled
}

/* ------------------------------------------------------------------------- */
/* Persistent configuration helpers                                          */
/* ------------------------------------------------------------------------- */

volatile system_config_t system_config;

/*
 * Compute simple 8-bit checksum
 */
static uint8_t checksum(const uint8_t *data, uint8_t len)
{
    uint8_t s = 0;
    for (uint8_t i = 0; i < len; ++i)
    {
        s += data[i];
    }
    return s;
}

/*
 * Read config blob from EEPROM into buffer
 */
static uint8_t readEEProm(uint8_t addr, uint8_t *buf, uint8_t len)
{
    uint8_t a = addr;
    if (i2cWriteBuffer(EEPROM_ADDRESS, &a, 1) != I2C_SUCCESS)
        return I2C_ERROR;
    if (i2cReadBuffer(EEPROM_ADDRESS, buf, len) != I2C_SUCCESS)
        return I2C_ERROR;
    return I2C_SUCCESS;
}

/*
 * Write config blob to EEPROM (single page assumed small)
 */
static uint8_t writeEEProm(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    uint8_t tmp[EEPROM_PAGE_SIZE + 1];
    uint8_t remaining = len;
    uint8_t offset = addr;

    while (remaining)
    {
        uint8_t page_offset = offset % EEPROM_PAGE_SIZE;
        uint8_t space = (uint8_t)(EEPROM_PAGE_SIZE - page_offset);
        uint8_t write_len = remaining < space ? remaining : space;

        if ((write_len + 1) > sizeof(tmp))
            return I2C_INVALID_PARAM;
        tmp[0] = offset & 0xFF;
        memcpy(&tmp[1], &buf[len - remaining], write_len);

        if (i2cWriteBuffer(EEPROM_ADDRESS, tmp, (uint8_t)(write_len + 1)) != I2C_SUCCESS)
            return I2C_ERROR;
        /* Wait for internal write cycle (typical <=5ms, be conservative) */
        __delay_ms(10);

        remaining -= write_len;
        offset = (uint8_t)(offset + write_len);
    }

    return I2C_SUCCESS;
}

/*
 * Initialize default configuration values
 */
void config_defaults(system_config_t *cfg)
{
    cfg->magic = CONFIG_MAGIC;
    cfg->version = CONFIG_VERSION;
    cfg->vref_source = VREF_INTERNAL;
    cfg->gps_baud_index = 1; /* default 9600 */
    cfg->gps_stop_bits = 1;  /* 1 stop bit */
    cfg->gps_parity = PARITY_N;
    cfg->gps_protocol = GPS_PROTOCOL_NMEA; /* default NMEA */
    cfg->vco_dac = (uint16_t)DAC_MIDPOINT;
    memset(cfg->reserved, 0, sizeof(cfg->reserved));
    cfg->crc = 0;
    cfg->crc = (uint8_t)(~checksum((uint8_t *)cfg, sizeof(system_config_t) - 1));
}

/*
 * Load persistent system configuration from EEPROM (falls back to defaults)
 */
void config_load(system_config_t *cfg)
{
    uint8_t buf[sizeof(system_config_t)];
    if (readEEProm(CONFIG_EEPROM_ADDR, buf, sizeof(buf)) != I2C_SUCCESS)
    {
        config_defaults(cfg);
        memcpy((void *)&system_config, cfg, sizeof(system_config_t));
        return;
    }

    memcpy(cfg, buf, sizeof(system_config_t));
    uint8_t c = checksum((uint8_t *)cfg, sizeof(system_config_t) - 1);
    if (cfg->magic != CONFIG_MAGIC || cfg->version != CONFIG_VERSION || (uint8_t)(~c) != cfg->crc)
    {
        config_defaults(cfg);
    }
    /* copy to global */
    memcpy((void *)&system_config, cfg, sizeof(system_config_t));
}

/*
 * Save system configuration to EEPROM
 */
void config_save(const system_config_t *cfg)
{
    system_config_t tmp;
    memcpy(&tmp, cfg, sizeof(system_config_t));
    tmp.crc = (uint8_t)(~checksum((uint8_t *)&tmp, sizeof(system_config_t) - 1));
    (void)writeEEProm(CONFIG_EEPROM_ADDR, (const uint8_t *)&tmp, sizeof(system_config_t));
    /* copy to global */
    memcpy((void *)&system_config, &tmp, sizeof(system_config_t));
}
