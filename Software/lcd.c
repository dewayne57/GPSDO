/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * This code provides a simple interface to the LCD display using the MCP23017 I2C interface.
 * It includes functions to write to the LCD, read from the LCD, and initialize the
 * LCD. The LCD is assumed to be in 4-bit mode and connected to the MCP23017.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This code supports both 4-bit and 8-bit modes of the HD44780 LCD controller.  To
 * enable the 4-bit mode, define the LCD_4BIT_MODE macro before including this file.
 * Otherwise, the code defaults to 8-bit mode.
 */
#include "lcd.h"
#include "config.h"
#include "gps.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "smt.h"
#include <stdbool.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <xc.h>

#define LCD_BUSY_MAX_POLLS 5000

/* LCD Buffer System */
static char lcd_buffer[LCD_LINES][LCD_CHARS_PER_LINE + 1]; // +1 for null terminator
static bool lcd_line_dirty[LCD_LINES];
static bool lcd_buffer_initialized = false;

/*
 * Global variables and data areas.
 */
extern volatile bool smt_new_capture;
extern volatile uint32_t smt_last_count;
extern volatile int32_t smt_last_error;
extern system_config_t system_config;
extern IOPortA_t ioporta;
extern uint8_t buffer[128];
extern volatile encoder_state_t encoder_state;
extern bool system_initialized;
extern volatile gps_data_t gps_data;
extern volatile bool gps_data_available;

/*********************************************************************************/
/* LCD Functions                                                                 */
/*                                                                               */
/* LCD functions for writing to the display.                                     */
/*********************************************************************************/
static uint8_t _lcdWriteByte(bool inst, uint8_t byte);
static uint8_t _lcdReadByte(bool inst, uint8_t* value);
static uint8_t _lcdWaitReady(uint16_t maxPolls);

/**
 * Update the LCD display with the current system status.
 *
 * This function displays the current system state on the LCD and is updated
 * during the main loop processing.  The LCD used is a 4X20 character display.
 * It is organized using the 4 lines as follows:
 * - Line 0: Static title "GPSDO V1.0 - 2025 DLH"
 * - Line 1: Date and time (UTC or local based on configuration) from the GPS.
 *           If the system is being initialized it displays "Initializing...".
 *           Once initialized, if a valid GPS fix is not available, displays
 *           "Waiting on GPS...".  Once a valid fix is obtained, displays the
 *           date and time.
 * - Line 2: Latitude, longitude, and altitide from the GPS.  If a valid GPS
 *           fix is not available, displays a blank line.
 * - Line 3: The current frequence measurement from the SMT module, updated
 *           each second when a new capture is available.  If GPS lock has
 *           not been obtained and the 1PPS signal is not present, displays
 *           a blank line.
 */
void updateDisplay(void) {
    lcdBufferSetLine(0, "GPSDO V1.0 2025 DLH");
    if (!system_initialized) {
        lcdBufferSetLine(1, "Initializing... ");
        lcdBufferSetLine(2, " ");
        lcdBufferSetLine(3, " ");
        lcdBufferUpdate(); // Force immediate display update
        return;
    }

    if (!gps_has_valid_fix()) {
        lcdBufferSetLine(1, "Waiting on GPS...");
        lcdBufferSetLine(2, " ");
        lcdBufferSetLine(3, " ");
        lcdBufferUpdate(); // Force immediate display update
        return;
    }

    uint8_t gie_saved;
    uint8_t giel_saved;
    CRITICAL_SECTION_ENTER(gie_saved, giel_saved);
    gps_datetime_t dt = gps_data.datetime;  // snapshot volatile data atomically
    gps_position_t pos = gps_data.position; // snapshot volatile data atomically
    CRITICAL_SECTION_EXIT(gie_saved, giel_saved);

    if (dt.valid == GPS_VALID && pos.valid == GPS_VALID) {
        // Valid GPS data available
        char date_time_line[21];
        gps_format_date_time(date_time_line, sizeof(date_time_line), &dt);
        lcdBufferSetLine(1, date_time_line);

        char pos_line[21];
        gps_format_position(pos_line, sizeof(pos_line), &pos);
        lcdBufferSetLine(2, pos_line);

        if (smt_capture_available()) {
            uint32_t c = smt_get_last_count();
            char freq_line[21];
            snprintf(freq_line, sizeof(freq_line), "Freq:%lu", (unsigned long)c);
            lcdBufferSetLine(3, freq_line);
        } else {
            /* If no new capture, indicate waiting on line 3 */
            lcdBufferSetLine(3, " ");
        }

    } else {
        // No valid GPS data
        lcdBufferSetLine(1, "Waiting on GPS...");
        lcdBufferSetLine(2, " ");
    }

    lcdBufferUpdate(); // Force immediate display update
}

/**
 * This function sets the LCD backlight state that all the other functions use.
 */
void lcdSetBacklight(bool state) {
    ioporta.LCD_BL = state ? 1 : 0;
}

/**
 * This function writes a null-terminated string to the LCD at the specified address.
 *
 * @param address The DDRAM address to start writing to.
 * @param data The null-terminated string to write.
 */
void lcdWriteBuffer(uint8_t address, char* data) {
    lcdWriteInstruction(SET_DDRAM_ADDRESS | address);
    lcdWriteString(data);
}

/**
 * This function writes a null-terminated string to the LCD at the current cursor position.
 * Highly optimized version that sends all register writes in a single I2C transaction.
 *
 * @param data The null-terminated string to write.
 *
 */
void lcdWriteString(char* data) {
    if (!data || !*data)
        return; // Handle empty strings

    size_t len = strlen(data);
    if (len == 0) {
        return;
    }

    char* p = data;
    while (*p) {
        if (_lcdWriteByte(false, *p++) != I2C_SUCCESS) {
            return;
        }
        __delay_us(50);
    }
    
    (void)_lcdWaitReady(LCD_BUSY_MAX_POLLS);
}

/**
 * Returns the cursor to the home position (0,0) on the LCD.
 *
 */
void lcdReturnHome(void) {
    lcdWriteInstruction(RETURN_HOME);
}

/*********************************************************************************/
/* LCD Buffer System Functions                                                   */
/*                                                                               */
/* Buffered LCD functions for efficient display management.                      */
/*********************************************************************************/

/**
 * Initialize the LCD buffer system
 */
void lcdBufferInit(void) {
    lcdBufferClear();
    lcd_buffer_initialized = true;
}

/**
 * Clear the entire LCD buffer
 */
void lcdBufferClear(void) {
    for (uint8_t i = 0; i < LCD_LINES; i++) {
        memset(lcd_buffer[i], ' ', LCD_CHARS_PER_LINE);
        lcd_buffer[i][LCD_CHARS_PER_LINE] = '\0';
        lcd_line_dirty[i] = true;
    }
}

/**
 * Set a complete line in the LCD buffer
 *
 * @param line Line number (0-3)
 * @param text Text to display (will be truncated or padded to 20 chars)
 */
void lcdBufferSetLine(uint8_t line, const char* text) {
    if (line >= LCD_LINES || !lcd_buffer_initialized)
        return;

    // Check to see if it changed
    if (text != NULL) {
        if (strncmp(lcd_buffer[line], text, LCD_CHARS_PER_LINE) == 0) {
            return; // No change
        }
    } else {
        // If text is NULL, treat as empty string
        if (strncmp(lcd_buffer[line], "                    ", LCD_CHARS_PER_LINE) == 0) {
            return; // No change
        }
    }

    // Clear line first
    memset(lcd_buffer[line], ' ', LCD_CHARS_PER_LINE);

    // Copy text, truncating if too long
    if (text != NULL) {
        size_t len = strlen(text);
        if (len > LCD_CHARS_PER_LINE)
            len = LCD_CHARS_PER_LINE;
        memcpy(lcd_buffer[line], text, len);
    }

    lcd_buffer[line][LCD_CHARS_PER_LINE] = '\0';
    lcd_line_dirty[line] = true;
}

/**
 * Printf-style formatting to a line in the LCD buffer
 *
 * @param line Line number (0-3)
 * @param format Printf-style format string
 * @param ... Arguments for format string
 */
void lcdBufferPrintf(uint8_t line, const char* format, ...) {
    if (line >= LCD_LINES || !lcd_buffer_initialized)
        return;

    char temp_buffer[LCD_CHARS_PER_LINE + 1];
    va_list args;
    va_start(args, format);
    vsnprintf(temp_buffer, sizeof(temp_buffer), format, args);
    va_end(args);

    lcdBufferSetLine(line, temp_buffer);
}

/**
 * Update the LCD display from the buffer (only updates changed lines)
 */
void lcdBufferUpdate(void) {
    if (!lcd_buffer_initialized)
        return;

    static const uint8_t line_addresses[] = {LINE_0, LINE_1, LINE_2, LINE_3};

    for (uint8_t i = 0; i < LCD_LINES; i++) {
        if (lcd_line_dirty[i]) {
            lcdWriteBuffer(line_addresses[i], lcd_buffer[i]);
            lcd_line_dirty[i] = false;
        }
    }
}

/**
 * Write an instruction byte to the LCD.
 *
 * @param data The instruction byte to write.
 */
void lcdWriteInstruction(uint8_t data) {
    if (_lcdWriteByte(true, data) != I2C_SUCCESS) {
        return;
    }

    (void)_lcdWaitReady(LCD_BUSY_MAX_POLLS);
}

/**
 * Write a character to the LCD.
 *
 * @param data The byte to write.
 *
 */
void lcdWriteChar(uint8_t data) {
    if (_lcdWriteByte(false, data) != I2C_SUCCESS) {
        return;
    }

    (void)_lcdWaitReady(LCD_BUSY_MAX_POLLS);
}

/**
 * Read a data byte from the LCD at the current address.
 */
uint8_t lcdReadData(void) {
    uint8_t byte = 0;
    (void)_lcdReadByte(false, &byte);

    return byte;
}

/**
 * Read the busy flag from the LCD and return TRUE if busy, FALSE if ready.
 *
 * This function must write a command to the LCD with the RS =0 and RW=1 to read
 * the busy flag (D7).  In order to read the busy flag, we need to turn the PB4..PB7
 * pins of the MCP23017 to inputs temporarily.
 *
 * @return TRUE if the LCD is busy, FALSE if ready.
 *
 */
bool isLcdBusy(void) {
    uint8_t byte = 0;
    if (_lcdReadByte(true, &byte) != I2C_SUCCESS) {
        return true;
    }
    return (byte & 0x80) ? true : false;
}

static uint8_t _lcdWaitReady(uint16_t maxPolls) {
    while (maxPolls--) {
        uint8_t byte = 0;
        uint8_t status = _lcdReadByte(true, &byte);
        if (status != I2C_SUCCESS) {
            return status;
        }

        if ((byte & 0x80) == 0) {
            return I2C_SUCCESS;
        }

        __delay_us(40);
    }

    return I2C_ERROR;
}

/**
 * Initialize the LCD in the appropriate mode
 */
void lcdInitialize(void) {
    lcdBufferInit();

    // Initialize sequence for 8-bit mode per HD44780
    if (_lcdWriteByte(true, 0x03) != I2C_SUCCESS) {
        return;
    }
    __delay_ms(5);
    if (_lcdWriteByte(true, 0x03) != I2C_SUCCESS) {
        return;
    }
    __delay_us(150);
    if (_lcdWriteByte(true, 0x03) != I2C_SUCCESS) {
        return;
    }
    __delay_us(150);

    // Function set: 8-bit, 2 lines, 5x8 dots
    lcdWriteInstruction(FUNCTION_SET_8_2_5X8);

    // Display off
    lcdWriteInstruction(DISPLAY_OFF);
    // Clear display
    lcdWriteInstruction(CLEAR);
    __delay_ms(2);
    // Entry mode set: increment, no shift
    lcdWriteInstruction(CURSOR_INCREMENT);
    // Display on, cursor off
    lcdWriteInstruction(DISPLAY_ON);
}

/**
 * Perform a self-test of the LCD.
 */
void lcdSelfTest(void) {
    // Ensure LCD is in known state first
    lcdWriteInstruction(CURSOR_OFF);
    lcdWriteInstruction(CLEAR);
    __delay_ms(50);
    lcdReturnHome();
    __delay_ms(50);

    // Wait for LCD to be completely ready
    (void)_lcdWaitReady(LCD_BUSY_MAX_POLLS);

    // Initialize buffer system cleanly
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100);

    // Display test pattern one line at a time
    lcdBufferSetLine(0, "abcdefghijklmnop");
    lcdBufferUpdate();
    __delay_ms(1000);

    lcdBufferSetLine(1, "ABCDEFGHIJKLMNOP");
    lcdBufferUpdate();
    __delay_ms(1000);

    lcdBufferSetLine(2, "0123456789_!@#$%");
    lcdBufferUpdate();
    __delay_ms(1000);

    lcdBufferSetLine(3, ",./;'[]-=<>?:{}+");
    lcdBufferUpdate();
    __delay_ms(1000);

    // Clear display again
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100);

    // Final hardware reset and ensure display is fully on
    lcdReturnHome();
    __delay_ms(50);
    (void)_lcdWaitReady(LCD_BUSY_MAX_POLLS);

    // Explicitly turn on display with full brightness
    lcdWriteInstruction(DISPLAY_ON);
    __delay_ms(10);
    lcdWriteInstruction(CURSOR_OFF);
    __delay_ms(10);
}

/**
 * Write the full 8-bit byte to the LCD via an MCP23017 I/O expander.  The control
 * bits RS, RW, E, and BL are on PA0..PA3 and data bits D0..D7 are on PB0..PB7.
 */
static uint8_t _lcdWriteByte(bool inst, uint8_t byte) {
    uint8_t status;

    if (inst) {
        ioporta.LCD_RS = 0; // RS = instruction
    } else {
        ioporta.LCD_RS = 1; // RS = data
    }
    ioporta.LCD_RW = 0; // RW = write
    ioporta.LCD_E = 1;  // E = 1

    status = i2cWriteRegister(MCP23017_ADDRESS, GPIOB, byte);
    if (status != I2C_SUCCESS) {
        return status;
    }

    status = i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    if (status != I2C_SUCCESS) {
        return status;
    }
    __delay_us(2);

    ioporta.LCD_E = 0; // E = 0
    status = i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    if (status != I2C_SUCCESS) {
        return status;
    }
    __delay_us(40);

    return I2C_SUCCESS;
}

/**
 * Read a full 8-bit byte from the LCD via MCP23017 PB0..PB7 with
 * control bits on PA0..PA3.  Since the MCP23017 pins PB0..PB7 are normally
 * outputs, we need to change them to inputs temporarily to read data, then
 * restore them to outputs.
 *
 * @param inst TRUE if reading an instruction, FALSE for data
 */
static uint8_t _lcdReadByte(bool inst, uint8_t* value) {
    // We need to change PB0..PB7 to inputs temporarily
    uint8_t iodir;
    uint8_t status = i2cReadRegister(MCP23017_ADDRESS, IODIRB, &iodir);
    if (status != I2C_SUCCESS) {
        return status;
    }

    status = i2cWriteRegister(MCP23017_ADDRESS, IODIRB, (iodir | 0xFF)); // All inputs
    if (status != I2C_SUCCESS) {
        return status;
    }

    if (inst) {
        ioporta.LCD_RS = 0; // RS = instruction
    } else {
        ioporta.LCD_RS = 1; // RS = data
    }
    ioporta.LCD_RW = 1; // RW = read
    ioporta.LCD_E = 1;  // E = 1

    status = i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    if (status != I2C_SUCCESS) {
        return status;
    }
    __delay_us(2);

    status = i2cReadRegister(MCP23017_ADDRESS, GPIOB, value);
    if (status != I2C_SUCCESS) {
        return status;
    }

    ioporta.LCD_E = 0; // E = 0
    status = i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    if (status != I2C_SUCCESS) {
        return status;
    }
    __delay_us(1);

    // Restore PB0..PB7 to outputs
    status = i2cWriteRegister(MCP23017_ADDRESS, IODIRB, iodir); // Restore IODIR
    if (status != I2C_SUCCESS) {
        return status;
    }

    return I2C_SUCCESS;
}