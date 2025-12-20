/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * This module contains the system configuration definitions and
 * configuration storage functions for the GPSDO project.
 * It defines the system configuration structure, default values,
 * and functions to load and save the configuration to/from EEPROM.
 * It is used by other modules to access and modify system settings.
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
 *
 */
#ifndef CONFIG_H
#define CONFIG_H

#define _XTAL_FREQ 64000000UL // Define the operating frequency of the microcontroller (64 MHz)
#define MCP23X17_BANKED

// PIC 18F27Q43 Configuration Bit Settings

#pragma config FEXTOSC = OFF           // Dont use the external oscillator
#pragma config RSTOSC = HFINTOSC_64MHZ // Use internal 64mHz high frequency osc
#pragma config CSWEN = OFF               // No clock switching allowed
#pragma config FCMEN = OFF               // Fail-safe clock monitor is disabled
#pragma config PR1WAY = 0               // PRLOCKED Set/Cleared repeatedly
#pragma config CLKOUTEN = 0               // Clock out is enabled on RA6
#pragma config BOREN = 3               // Brown-out reset is enabled
#pragma config LPBOREN = OFF           // Low power brown-out reset is disabled
#pragma config IVT1WAY = 0               // IVTLOCK Set/cleared repeatedly
#pragma config MVECEN = 1               // Vectored interrupts enabled
#pragma config PWRTS = 2               // Power up timer at 64mS
#pragma config MCLRE = 1               // Master clear retains that function
#pragma config XINST = OFF               // No extended instruction set
#pragma config LVP = 1                   // Low voltage programming is enabled
#pragma config STVREN = ON               // Stack over/under flow causes reset
#pragma config PPS1WAY = 0               // PPSLOCK set/reset repeatedly
#pragma config ZCD = 1                   // Zero-cross detection is disabled
#pragma config BORV = 0                   // Brown-out voltage is set to 2.85V
#pragma config WDTE = OFF               // No watch dog timer
#pragma config SAFEN = OFF               // Storage area flash is disabled
#pragma config BBEN = OFF               // Boot block is disabled
#pragma config WRTAPP = OFF               // Application block is NOT write protected
#pragma config WRTSAF = OFF               // SAF area is not write protected
#pragma config WRTC = OFF               // Configuration registers are NOT write protected
#pragma config WRTB = OFF               // Boot block is not write protected
#pragma config WRTD = OFF               // Data EEPROM is not write protected
#pragma config CP = OFF                   // Code is not protected

/****************************************************************************/
/*                                                                          */
/* Device addresses on the I2C buss                                         */
/*                                                                          */
/****************************************************************************/
#define MCP23017_ADDRESS 0x20 // I/O Expander address
#define EEPROM_ADDRESS 0x50      // EEPROM address
#define DAC8571_ADDRESS 0x4C  // DAC address

// DAC characteristics
#define DAC_RESOLUTION 4096U // 12-bit
#define DAC_MIDPOINT (DAC_RESOLUTION / 2)

#define CONFIG_EEPROM_ADDR 0x00 // EEPROM address to store system configuration
#define CONFIG_MAGIC 0xA5        // Magic number to identify valid config
#define CONFIG_VERSION 0x01        // Configuration structure version
#define EEPROM_PAGE_SIZE 16        // EEPROM page size for writes

typedef enum
{
    VREF_INTERNAL = 0, // use internal DAC as reference
    VREF_EXTERNAL = 1  // OCXO provides its own reference
} vref_source_t;

/* GPS serial port options */
static const uint32_t gps_baud_rates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};

/* GPS serial port options */
typedef enum
{
    PARITY_N = 0,
    PARITY_E = 1,
    PARITY_O = 2
} parity_t;

/* System configuration structure, saved in the external EEPROM */
typedef struct
{
    uint8_t magic;
    uint8_t version;
    uint8_t vref_source;    /* vref_source_t */
    uint8_t gps_baud_index; /* index into gps_baud_rates[] */
    uint8_t gps_stop_bits;    /* 0,1,2 */
    uint8_t gps_parity;        /* parity_t */
    uint8_t gps_protocol;    /* gps_protocol_t */
    uint16_t vco_dac;        /* stored VCO DAC raw value (0..4095) */
    uint8_t reserved[8];
    uint8_t crc;
} system_config_t;

/* Global instance */
extern volatile system_config_t system_config;

/****************************************************************************/
/*                                                                          */
/* Pin definitions on the CPU                                               */
/*                                                                            */
/* The pin values are represented as the decimal equivalents of the bit     */
/* positions in the Port A, Port B, and Port C registers                    */
/*                                                                          */
/****************************************************************************/
// Port A
#define VREF_FB 1      // RA0: VREF feedback pin
#define INT_REF 2      // RA1: Internal reference enable pin
#define CLOCK_OUT 64      // RA6: Clock output pin
// Port B
#define GPS_TX 1          // RB0: GPS Transmit pin
#define GPS_RX 2          // RB1: GPS Receive pin
#define INT 4              // RB2: Interrupt pin from GPS module
// Port C
#define PPS 1             // RC0: 1PPS signal from GPS module
#define RF 2              // RC1: RF From OCXO
#define RESET_N 4         // RC2: Active low reset for GPS module
#define SCL 8             // RC3: I2C Clock
#define SDA 16            // RC4: I2C Data
#define PHASE_A 32        // RC5: Phase A input from encoder
#define PHASE_B 64        // RC6: Phase B input from encoder
#define ENTER_N 128       // RC7: Active low enter button

/****************************************************************************/
/*                                                                          */
/* Pin definitions on the MCP23017 I/O Extender Port A                      */
/*                                                                          */
/* The values of the pins are the decimal equivalents of the bit positions  */
/* in the MCP23017 I/O Expander Port A                                      */
/****************************************************************************/
#define POWER_LED_N 1      // A0: Power LED output pin
#define HIGH_LED_N 2      // A1: Tracking high LED output pin
#define LOW_LED_N 4      // A2: Tracking low LED output pin
#define LOCK_LED_N 8      // A3: Lock LED output pin
#define HOLDOVER_LED_N 16 // A4: Holdover LED output pin
#define FAULT_LED_N 32      // A5: Fault LED output pin
#define GPS_LED_N 64      // A6: GPS LED output pin

/****************************************************************************/
/*                                                                          */
/* Pin definitions on the MCP23017 I/O Extender Port B                      */
/*                                                                          */
/* The values of the pins are the decimal equivalents of the bit positions  */
/* in the MCP23017 I/O Expander Port B                                      */
/****************************************************************************/
#define LCD_RS 1            // B0: LCD Register Select pin
#define LCD_RW 2            // B1: LCD Read/Write pin
#define LCD_E 4             // B2: LCD Enable pin
#define LCD_BL 8            // B3: LCD Backlight control pin
#define LCD_D4 16           // B4: LCD Data pin 4
#define LCD_D5 32           // B5: LCD Data pin 5
#define LCD_D6 64           // B6: LCD Data pin 6
#define LCD_D7 128          // B7: LCD Data pin 7

/* Self-test timing (milliseconds) */
#ifndef SELFTEST_ON_MS
#define SELFTEST_ON_MS 500U
#endif

#ifndef SELFTEST_OFF_MS
#define SELFTEST_OFF_MS 500U
#endif

/****************************************************************************/
/*                                                                          */
/* Forward defines of configuration functions                               */
/*                                                                          */
/****************************************************************************/

void initialize(void);
void config_load(system_config_t *cfg);
void config_save(const system_config_t *cfg);
void config_defaults(system_config_t *cfg);

#endif // CONFIG_H