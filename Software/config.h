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

#include <stdbool.h>
#include <stdint.h>

/* Include mytypes.h for boolean type */
#ifndef MYTYPES_H
#include "mytypes.h"
#endif

#define _XTAL_FREQ 64000000UL // Define the operating frequency of the microcontroller (64 MHz)
#define MCP23X17_BANKED

// PIC 18F27Q43 Configuration Bit Settings

#pragma config FEXTOSC = OFF           // Dont use the external oscillator
#pragma config RSTOSC = HFINTOSC_64MHZ // Use internal 64mHz high frequency osc
#pragma config CSWEN = OFF             // No clock switching allowed
#pragma config FCMEN = OFF             // Fail-safe clock monitor is disabled
#pragma config PR1WAY = 0              // PRLOCKED Set/Cleared repeatedly
#pragma config CLKOUTEN = 0            // Clock out is enabled on RA6
#pragma config BOREN = 3               // Brown-out reset is enabled
#pragma config LPBOREN = OFF           // Low power brown-out reset is disabled
#pragma config IVT1WAY = 0             // IVTLOCK Set/cleared repeatedly
#pragma config MVECEN = 1              // Vectored interrupts enabled
#pragma config PWRTS = 2               // Power up timer at 64mS
#pragma config MCLRE = 1               // Master clear retains that function
#pragma config XINST = OFF             // No extended instruction set
#pragma config LVP = 1                 // Low voltage programming is enabled
#pragma config STVREN = ON             // Stack over/under flow causes reset
#pragma config PPS1WAY = 0             // PPSLOCK set/reset repeatedly
#pragma config ZCD = 1                 // Zero-cross detection is disabled
#pragma config BORV = 0                // Brown-out voltage is set to 2.85V
#pragma config WDTE = OFF              // No watch dog timer
#pragma config SAFEN = OFF             // Storage area flash is disabled
#pragma config BBEN = OFF              // Boot block is disabled
#pragma config WRTAPP = OFF            // Application block is NOT write protected
#pragma config WRTSAF = OFF            // SAF area is not write protected
#pragma config WRTC = OFF              // Configuration registers are NOT write protected
#pragma config WRTB = OFF              // Boot block is not write protected
#pragma config WRTD = OFF              // Data EEPROM is not write protected
#pragma config CP = OFF                // Code is not protected

/****************************************************************************/
/*                                                                          */
/* Device addresses on the I2C buss                                         */
/*                                                                          */
/****************************************************************************/
#define MCP23017_ADDRESS 0x20 // I/O Expander address
#define EEPROM_ADDRESS 0x50   // EEPROM address
#define DAC8571_ADDRESS 0x4C  // DAC address

// DAC characteristics
#define DAC_RESOLUTION 4096U // 12-bit
#define DAC_MIDPOINT (DAC_RESOLUTION / 2)

#define CONFIG_MAGIC 0xA5   // Magic number to identify valid config
#define CONFIG_VERSION 0x01 // Configuration structure version
#define EEPROM_PAGE_SIZE 16 // EEPROM page size for writes

/*
 * Voltage reference source selection for the Menu.
 */
typedef enum {
    VREF_INTERNAL = 0, // use internal DAC as reference
    VREF_EXTERNAL = 1  // OCXO provides its own reference
} vref_source_t;

/*
 * Serial communication constants (defined in config.c)
 */
#define BAUD_RATES_COUNT 8
extern const uint32_t baud_rates[];
extern const char* stop_options[];
extern const char* parity_options[];
extern const char* baud_options[];
extern const char* vref_options[];
extern const char* protocol_options[];

/*
 * GPS serial port options for the menus.
 */
typedef enum { PARITY_N = 0, PARITY_E = 1, PARITY_O = 2 } parity_t;

/*
 * System configuration structure, saved in the external EEPROM
 */
typedef struct {
    uint8_t magic;
    uint8_t version;
    uint8_t vref_source;    /* vref_source_t */
    uint8_t gps_baud_index; /* index into gps_baud_rates[] */
    uint8_t gps_stop_bits;  /* 0,1,2 */
    uint8_t gps_parity;     /* parity_t */
    uint8_t gps_protocol;   /* gps_protocol_t */
    uint16_t vco_dac;       /* stored VCO DAC raw value (0..4095) */
    uint8_t ext_baud_index; /* index into gps_baud_rates[] for external port */
    uint8_t ext_stop_bits;  /* 0,1,2 */
    uint8_t ext_parity;     /* parity_t */
    uint8_t reserved[5];    /* reserved for future use */
    uint8_t crc;
} system_config_t;

/*
 * Global instance
 */
extern system_config_t system_config;

/****************************************************************************/
/*                                                                          */
/* Pin definitions on the CPU                                               */
/*                                                                          */
/* The pin values are represented as the decimal equivalents of the bit     */
/* positions in the Port A, Port B, and Port C registers. This allows       */
/* bits to be manipulated by adding, subtracting, OR'ing, AND'ing,          */
/* and inverting the bit values.  For example, to set a bit, you can use:   */
/* PORTA |= VREF_FB;                                                        */
/* To clear a bit, you can use:                                             */
/* PORTA &= ~ VREF_FB;                                                      */
/* This works because the decimal value of CLOCK_OUT is 64 which is 1 << 6  */
/* or 0x40, and when used in an equation, PORTA &= ~0x40 clears the         */
/* bit, PORTA |= 0x40 sets the bit.                                         */
/* Using the decimal values makes it easier to understand the bit           */
/* manipulation.  These values can be combined as well, such as:            */
/* PORTA |= VREF_FB + CLOCK_OUT;                                            */
/****************************************************************************/

// Port A
#define VREF_FB 1    // RA0: VREF feedback pin
#define INT_REF 2    // RA1: Internal reference enable pin
#define CLOCK_OUT 64 // RA6: Clock output pin

// Port B
#define GPS_TX 1   // RB0: GPS Transmit pin
#define GPS_RX 2   // RB1: GPS Receive pin
#define INT 4      // RB2: Interrupt pin from GPS module
#define EXT_RX 8   // RB3: External RX input
#define EXT_TX 16  // RB4: External TX output
#define PROGRAM 32 // RB5: Program mode select (active high)

// Port C
#define PPS 1       // RC0: 1PPS signal from GPS module
#define RF 2        // RC1: RF From OCXO
#define RESET_N 4   // RC2: Active low reset for GPS module
#define SCL 8       // RC3: I2C Clock
#define SDA 16      // RC4: I2C Data
#define PHASE_A 32  // RC5: Phase A input from encoder
#define PHASE_B 64  // RC6: Phase B input from encoder
#define ENTER_N 128 // RC7: Active low enter button

/****************************************************************************/
/*                                                                          */
/* Pin definitions on the MCP23017 I/O Extender Port A                      */
/*                                                                          */
/* The pins for the MCP23017 I/O Expander Port A are defined as bits in a   */
/* 8-bit register. Port A has 4 pins dedicated to the LCD function and 4    */
/* to the front panel LED indicators.                                       */
/* This definition is a type definition used to instantiate the IOPortA_t   */
/* data area used to track the values present on the MCP23017 I/O Extender  */
/* Port A.                                                                  */
/* The Port B pins are used to transfer data and commands to the LCD        */
/* using the full 8-bit register.                                           */
/****************************************************************************/
typedef union {
    struct {
        uint8_t LCD_RS : 1;     // LCD Register Select pin
        uint8_t LCD_RW : 1;     // LCD Read/Write pin
        uint8_t LCD_E : 1;      // LCD Enable pin
        uint8_t LCD_BL : 1;     // LCD Backlight control pin
        uint8_t POWER_N : 1;    // Power LED (active low)
        uint8_t LOCK_N : 1;     // Lock status LED (active low)
        uint8_t HOLDOVER_N : 1; // Holdover status LED (active low)
        uint8_t GPS_N : 1;      // GPS lock status LED (active low)
    };
    uint8_t all;
} IOPortA_t;

/****************************************************************************/
/*                                                                          */
/* Forward defines of configuration functions                               */
/*                                                                          */
/****************************************************************************/

void initialize(void);
void config_load(system_config_t* cfg);
void config_save(const system_config_t* cfg);
void config_defaults(system_config_t* cfg);

#endif // CONFIG_H