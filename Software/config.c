/*
 * Copyright (c) 2026, Dewayne L. Hafenstein.  All rights reserved.
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
#include "config.h"
#include "control.h"
#include "dac.h"
#include "encoder.h"
#include "faults.h"
#include "gps.h"
#include "i2c.h"
#include "lcd.h"
#include "led.h"
#include "mcp23x17.h"
#include "menu.h"
#include "serial.h"
#include "smt.h"
#include "usb.h"
#include <string.h>
#include <xc.h>

/**
 * Global variables and data areas.
 */
bool system_initialized = false;           // Flag indicating system initialization complete
unsigned char i2c_buffer[I2C_BUFFER_SIZE]; // General-purpose I2C buffer
shadowA_t shadowA = {.all = 0xF8};         // I/O expander Port A state shadow register
shadowB_t shadowB = {.all = 0x00};         // I/O expander Port B state shadow register
system_config_t system_config;             // System configuration data
volatile encoder_state_t encoder_state = {
    .position = 0, .last_state = 0, .button_raw = 1, .button_stable = 0, .debounce_cnt = 0}; // Rotary encoder state
extern volatile gps_data_t gps_data;     // Global GPS data (defined in gps.c)
extern volatile bool gps_data_available; // Flag for new GPS data available (defined in gps.c)

/*
 * forward definitions
 */
static void selfCheck(void);

/****************************************************************************/
/*                                                                          */
/* Initialize the system                                                    */
/*                                                                          */
/****************************************************************************/
void initialize(void) {
    system_initialized = false;

    /*
     * Disable interrupts
     */
    INTCON0bits.GIEH = 0; // Turn off high priority interrupts
    INTCON0bits.GIEL = 0; // And low priority interrupts too

    /*
     * Clear all interrupt enables
     */
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

    /*
     * Clear all interrupt requests
     */
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

    /*
     * Disable all peripheral modules until after PPS mapping and I/O setup is done,
     * then selectively enable the required modules to save power.
     */
    PMD0 = 0xFF;
    PMD1 = 0xFF;
    PMD3 = 0xFF;
    PMD4 = 0xFF;
    PMD5 = 0xFF;
    PMD6 = 0xFF;
    PMD7 = 0xFF;
    PMD8 = 0xFF;

    // Set up port A
    //
    // All port A pins (except RA6) are used as analog inputs and
    // outputs and are used to sample the VRef level and to supply
    // a default VRef if the OCXO does not. RA6 is configured as a
    // digital output and outputs the system clock for diagnostic
    // purposes.
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
    // RB0 - External TX (output FROM MCU to external device)
    // RB1 - External RX (input TO MCU from external device)
    // RB2 - USB TX (output FROM MCU to USB-serial bridge)
    // RB3 - USB RX (input TO MCU from USB-serial bridge)
    // RB4 - GPS_TX (output FROM GPS to MCU)
    // RB5 - GPS_RX (input TO GPS from MCU)
    // RB6 - ICSP/Debug PGC, active low to enable bootloader mode
    // RB7 - ICSP/Debug PGD
    TRISB = 0xFF - GPS_RX - USB_RX - EXT_RX; // All receive pins inputs
    ANSELB = 0x00;                           // All digital
    LATB = 0x00;
    ODCONB = 0x00;
    WPUB = 0x00;
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
    // RC2 - RESET_N output to reset I/O extender (pull ups enabled and open drain)
    // RC3 - I2C SCL (i2c pull ups enabled, open drain)
    // RC4 - I2C SDA (i2c pull ups enabled, open drain)
    // RC5 - Encoder Phase A (pull ups enabled)
    // RC6 - Encoder Phase B (pull ups enabled)
    // RC7 - Encoder Enter_N Switch (pull-ups enabled)
    TRISC = 0xFF - RESET_N; // All inputs except RESET_N (which is output)
    ANSELC = 0x00;
    LATC = 0x00;
    ODCONC = 0x00;                                  // No open-drain (RESET_N will be push-pull output)
    WPUC = SDA + SCL + PHASE_A + PHASE_B + ENTER_N; // Weak pull-ups on I2C pins and encoder pins
    INLVLC = 0x00;
    SLRCONC = 0x00;

    // For bit-banging I2C, we use regular GPIO instead of I2C module features
    // RC3I2C = 0x51; // Disabled for bit-banging - i2c fast mode, 2x pullups, i2c thresholds on RC3 (SCL)
    // RC4I2C = 0x51; // Disabled for bit-banging - i2c fast mode, 2x pullups, i2c thresholds on RC4 (SDA)

    /*
     * Set up PPS as needed
     */
    PPSLOCK = 0x55;            // unlock PPS
    PPSLOCK = 0xAA;            // unlock PPS
    PPSLOCKbits.PPSLOCKED = 0; // unlock

    INT0PPS = 0x05; // RA5 -> INT0 input

    U1RXPPS = 0x0C; // RB4 (EXT_RX) -> UART1 RX input (for bootloader)
    RB3PPS = 0x13;  // RB3 (EXT_TX) -> UART1 TX output (for data transmission)

    // Map 10MHz signal (RC1) to SMT1 signal input and 1PPS (RC0) to window
    SMT1SIGPPS = 0x11; // RC1 -> SMT1 signal input
    SMT1WINPPS = 0x10; // RC0 -> SMT1 window input (1PPS)

    PPSLOCK = 0x55;            // lock PPS
    PPSLOCK = 0xAA;            // lock PPS
    PPSLOCKbits.PPSLOCKED = 1; // lock

    /*
     * Reset sequence for MCP23017
     */
    LATC &= ~RESET_N;  // Hold MCP23017 in reset (drive low)
    TRISC &= ~RESET_N; // Make RESET_N pin an output
    __delay_ms(10);    // Hold reset for minimum duration
    LATC |= RESET_N;   // Release reset (drive high)
    __delay_ms(100);   // Allow MCP23017 to fully stabilize after reset

    /*
     * Initialize I2C bus to idle state (both lines high)
     * Ensure LATC bits are cleared for proper open-drain operation
     */
    LATC &= ~(SCL + SDA); // Clear LATC bits so when output, they will drive low
    TRISC |= SCL + SDA;   // Set both lines as inputs (pull-ups will drive high)
    __delay_ms(10);       // Allow bus to stabilize

    // Temporary I2C bus scan for debugging
    unsigned char found_devices[16];
    unsigned char device_count;
    unsigned char i;
    memset(found_devices, 0, sizeof(found_devices));

    device_count = i2cScanBus(found_devices, 16);

    /*
     * Initialize MCP23017 #1 for sequential mode to initialize all the registers
     */
    i2cWriteRegister(MCP23017_ADDRESS, 0x0A, 0x80); // Set BANK=0, SEQOP=1

    /*
     * Initialize MCP23017 Port A
     */
    i2c_buffer[0] = IODIRA; // Write IODIRA register
    i2c_buffer[1] = 0x00;   // All pins are output
    i2c_buffer[2] = 0x00;   // pins are not inverted
    i2c_buffer[3] = 0x00;   // No IOC
    i2c_buffer[4] = 0x00;   // default compare register
    i2c_buffer[5] = 0x00;   // compare against previous value
    i2c_buffer[6] = 0x80;   // re-write iocon for now
    i2c_buffer[7] = 0x00;   // disable pullups
    i2c_buffer[8] = 0x00;   // Ignored
    i2c_buffer[9] = 0x00;   // not used
    i2c_buffer[10] = 0x00;  // Initial value (will be set by LED functions)
    i2cWriteBuffer(MCP23017_ADDRESS, i2c_buffer, 11);

    /*
     * Initialize MCP23017 #1 Port B
     */
    i2c_buffer[0] = IODIRB; // Write IODIRB register
    i2c_buffer[1] = 0x00;   // All pins output
    i2c_buffer[2] = 0x00;   // pins are NOT inverted
    i2c_buffer[3] = 0x00;   // No IOC
    i2c_buffer[4] = 0x00;   // Default compare
    i2c_buffer[5] = 0x00;   // Compare against prev value
    i2c_buffer[6] = 0x80;   // Re-write IOCON for now
    i2c_buffer[7] = 0x00;   // Disable pullups
    i2c_buffer[8] = 0x00;   // Ignored
    i2c_buffer[9] = 0x00;   // not used
    i2c_buffer[10] = 0x00;  // All pins low
    i2cWriteBuffer(MCP23017_ADDRESS, i2c_buffer, 11);

    /*
     * Now, set the IO Configuration to byte mode
     */
    i2cWriteRegister(MCP23017_ADDRESS, IOCON, 0xE4); // BANK=0, SEQOP=0, HAEN=1, ODR=0, INTCC=0

    /*
     * Clear any pending MCP23017 interrupt from initialization (read INTFA then GPIOA)
     */
    i2cReadRegister(MCP23017_ADDRESS, INTFA, i2c_buffer);
    i2cReadRegister(MCP23017_ADDRESS, GPIOA, i2c_buffer);

    /*
     * Initialize the LED module (turn all LEDs off)
     */
    ledInitialize();

    /*
     * Initialize the fault management module
     */
    faultsInit();

    /*
     * Load persistent system configuration from EEPROM (falls back to defaults)
     */
    config_load((system_config_t*)i2c_buffer);

    /*
     * Initialize encoder GPIOs and state
     */
    encoder_init();

    /*
     * Initialize menu state
     */
    menu_init();

    /*
     * Initialize SMT1 counting and DAC-based control loop
     */
    smt_init();
    dac_init();
    control_init();

    /* Timer1 is not used for main-loop timing anymore. Ensure it is stopped
     * and its interrupt is disabled so it does not affect the main loop.
     */
    TMR1IF = 0;
    TMR1IE = 0;             // don't enable Timer1 interrupt
    TMR1IP = 0;             // low priority if ever enabled
    TMR1CONbits.TMR1ON = 0; // ensure Timer1 is stopped

    /*
     * Enable all peripheral modules that are required
     */
    PMD0bits.SYSCMD = 0; // System clock network enabled
    PMD0bits.CLKRMD = 0; // Clock reference module enabled
    PMD0bits.IOCMD = 0;  // Interrupt on change module enabled
    PMD1bits.SMT1MD = 0; // SMT1 module enabled (for RF counting)
    PMD1bits.TMR1MD = 0; // Timer1 module enabled
    PMD3bits.DAC1MD = 0; // DAC1 module enabled
    PMD3bits.ADCMD = 0;  // ADC module enabled
    PMD6bits.I2C1MD = 0; // I2C1 module enabled
    PMD6bits.U1MD = 0;   // UART 1 enabled
    PMD6bits.U2MD = 0;   // UART 2 enabled

    /*
     * Enable global interrupts
     */
    INTCON0bits.GIEH = 1; // High priority enabled
    INTCON0bits.GIEL = 1; // Low priority enabled

    /*
     * Initialize all other sub-systems
     */
    lcdSetBacklight(true);
    gps_init();
    serial_init();
    usb_init();
    selfCheck();

    system_initialized = true;
}

/*
 * Perform a self-check by cycling the LEDs on the I/O expander and by initializing
 * and displaying a test pattern on the front panel display LCD.  This provides a basic
 * verification that the I/O expander, I2C bus, and LCD are functioning correctly.
 *
 * @return     None
 */
static void selfCheck(void) {
    // Initialize LCD hardware and buffer system
    lcdInitialize();
    __delay_ms(100); // Give LCD time to initialize
    lcdBufferInit();
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100); // Ensure clean start

    lcdSelfTest();

    // Ensure completely clean state after self test
    __delay_ms(100); // Let self test complete
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100); // Allow buffer update to complete
    lcdReturnHome();
    __delay_ms(100); // Allow LCD to settle

    // Ensure backlight and display are fully on after self test
    lcdSetBacklight(true);
    lcdWriteInstruction(DISPLAY_ON);
    __delay_ms(50);

    // Test all LEDs
    ledTest();
}

/*
 * Compute CRC-16-CCITT (polynomial 0x1021, initial value 0xFFFF)
 * Standard CRC used in many communication protocols.
 *
 * @param data  Pointer to data buffer
 * @param len   Length of data buffer in bytes
 * @return      Computed CRC-16 value
 */
static unsigned int crc16(const unsigned char* data, unsigned int len) {
    unsigned int crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= ((unsigned int)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc & 0xFFFF;
}

/*
 * Read config blob from EEPROM into buffer.
 *
 * @param addr  Starting EEPROM address to read from
 * @param buf   Buffer to read data into
 * @param len   Number of bytes to read
 * @return      I2C_SUCCESS on success, error code otherwise
 */
static unsigned char readEEProm(unsigned char addr, unsigned char* buf, unsigned char len) {
    unsigned char a = addr;
    if (i2cWriteBuffer(EEPROM_ADDRESS, &a, 1) != I2C_SUCCESS)
        return I2C_ERROR;
    if (i2cReadBuffer(EEPROM_ADDRESS, buf, len) != I2C_SUCCESS)
        return I2C_ERROR;
    return I2C_SUCCESS;
}

/*
 * Write config blob to EEPROM a page at a time.
 *
 * @param addr  Starting EEPROM address to write to
 * @param buf   Buffer containing data to write
 * @param len   Number of bytes to write
 * @return      I2C_SUCCESS on success, error code otherwise
 */
static unsigned char writeEEProm(unsigned char addr, const unsigned char* buf, unsigned char len) {
    unsigned char tmp[EEPROM_PAGE_SIZE + 1];
    unsigned char remaining = len;
    unsigned char offset = addr;

    while (remaining) {
        unsigned char page_offset = offset % EEPROM_PAGE_SIZE;
        unsigned char space = EEPROM_PAGE_SIZE - page_offset;
        unsigned char write_len = remaining < space ? remaining : space;

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
 *
 * @param cfg  Pointer to configuration structure to initialize
 * @return     None
 */
void config_defaults(system_config_t* cfg) {
    cfg->magic = CONFIG_MAGIC;
    cfg->version = CONFIG_VERSION;
    cfg->vref_source = VREF_SRC_INTERNAL;
    cfg->gps_baud = DEFAULT_GPS_BAUD;
    cfg->gps_stop_bits = DEFAULT_GPS_STOP_BITS;
    cfg->gps_parity = DEFAULT_GPS_PARITY;
    cfg->gps_protocol = GPS_PROTOCOL_NMEA;
    cfg->ext_baud = DEFAULT_EXT_BAUD;
    cfg->ext_stop_bits = DEFAULT_EXT_STOP_BITS;
    cfg->ext_parity = DEFAULT_EXT_PARITY;
    cfg->vco_dac = (uint16_t)DAC_MIDPOINT;
    cfg->tz_mode = DEFAULT_TZ_MODE;
    cfg->tz_offset_min = DEFAULT_TZ_OFFSET_MIN;
    memset(cfg->reserved, 0, sizeof(cfg->reserved));
    cfg->crc = crc16((unsigned char*)cfg, sizeof(system_config_t) - sizeof(cfg->crc));
}

/*
 * Load persistent system configuration from EEPROM (falls back to defaults)
 *
 * @param cfg  Pointer to configuration structure to load into
 * @return     None
 */
void config_load(system_config_t* cfg) {
    uint8_t buf[sizeof(system_config_t)];
    if (readEEProm(0, buf, sizeof(buf)) != I2C_SUCCESS) {
        config_defaults(cfg);
        memcpy((void*)&system_config, cfg, sizeof(system_config_t));
        return;
    }

    memcpy(cfg, buf, sizeof(system_config_t));
    unsigned int c = crc16((uint8_t*)cfg, sizeof(system_config_t) - sizeof(cfg->crc));
    if (cfg->magic != CONFIG_MAGIC || cfg->version != CONFIG_VERSION || c != cfg->crc) {
        config_defaults(cfg);
    }
    /* copy to global */
    memcpy((void*)&system_config, cfg, sizeof(system_config_t));
}

/*
 * Save system configuration to EEPROM.
 *
 * @param cfg  Pointer to configuration structure to save
 * @return     None
 */
void config_save(const system_config_t* cfg) {
    system_config_t tmp;
    memcpy(&tmp, cfg, sizeof(system_config_t));
    tmp.crc = crc16((uint8_t*)&tmp, sizeof(system_config_t) - sizeof(tmp.crc));
    (void)writeEEProm(0, (const uint8_t*)&tmp, sizeof(system_config_t));
    /* copy to global */
    memcpy((void*)&system_config, &tmp, sizeof(system_config_t));
}
