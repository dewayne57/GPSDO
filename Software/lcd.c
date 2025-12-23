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
#include "i2c.h"
#include "mcp23x17.h"
#include "mytypes.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <xc.h>

extern IOPortA_t ioporta;

/* LCD Buffer System */
static char lcd_buffer[LCD_LINES][LCD_CHARS_PER_LINE + 1]; // +1 for null terminator
static boolean lcd_line_dirty[LCD_LINES];
static boolean lcd_buffer_initialized = FALSE;

/*********************************************************************************/
/* LCD Functions                                                                 */
/*                                                                               */
/* LCD functions for writing to the display.                                     */
/*********************************************************************************/
static void _lcdWrite8Bits(boolean inst, uint8_t byte);
static uint8_t _lcdRead8Bits(boolean inst);

/**
 * This function sets the LCD backlight state that all the other functions use.
 */
void lcdSetBacklight(boolean state) {
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
 *
 * @param data The null-terminated string to write.
 *
 */
void lcdWriteString(char* data) {
    char* p = data;
    while (*p) {
        lcdWriteChar(*p++);
    }
}

/**
 * Returns the cursor to the home position (0,0) on the LCD.
 *
 */
void lcdReturnHome(void) {
    lcdWriteInstruction(RETURN_HOME);
}

/**
 * Clears the LCD display.
 */
void lcdClearDisplay(void) {
    lcdWriteInstruction(CLEAR);
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
    lcd_buffer_initialized = TRUE;
}

/**
 * Clear the entire LCD buffer
 */
void lcdBufferClear(void) {
    for (uint8_t i = 0; i < LCD_LINES; i++) {
        memset(lcd_buffer[i], ' ', LCD_CHARS_PER_LINE);
        lcd_buffer[i][LCD_CHARS_PER_LINE] = '\0';
        lcd_line_dirty[i] = TRUE;
    }
}

/**
 * Set a complete line in the LCD buffer
 * @param line Line number (0-3)
 * @param text Text to display (will be truncated or padded to 20 chars)
 */
void lcdBufferSetLine(uint8_t line, const char* text) {
    if (line >= LCD_LINES || !lcd_buffer_initialized)
        return;

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
    lcd_line_dirty[line] = TRUE;
}

/**
 * Set a single character in the LCD buffer
 * @param line Line number (0-3)
 * @param col Column number (0-19)
 * @param c Character to set
 */
void lcdBufferSetChar(uint8_t line, uint8_t col, char c) {
    if (line >= LCD_LINES || col >= LCD_CHARS_PER_LINE || !lcd_buffer_initialized)
        return;

    lcd_buffer[line][col] = c;
    lcd_line_dirty[line] = TRUE;
}

/**
 * Printf-style formatting to a line in the LCD buffer
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
            lcd_line_dirty[i] = FALSE;
        }
    }
}

/**
 * Get a pointer to a line buffer (for direct manipulation)
 * @param line Line number (0-3)
 * @return Pointer to line buffer or NULL if invalid
 */
char* lcdBufferGetLine(uint8_t line) {
    if (line >= LCD_LINES || !lcd_buffer_initialized)
        return NULL;
    return lcd_buffer[line];
}

/**
 * Write an instruction byte to the LCD.
 *
 * @param data The instruction byte to write.
 */
void lcdWriteInstruction(uint8_t data) {
    // Send full byte
    _lcdWrite8Bits(TRUE, data);

    while (isLcdBusy()) {
    }
}

/**
 * Write a character to the LCD.
 *
 * @param data The byte to write.
 *
 */
void lcdWriteChar(uint8_t data) {
    _lcdWrite8Bits(FALSE, data);
}

/**
 * Read a data byte from the LCD at the current address.
 */
uint8_t lcdReadData(void) {
    uint8_t byte = _lcdRead8Bits(FALSE);

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
boolean isLcdBusy(void) {
    // To determin if the LCD is busy, we need to read the instruction byte and check
    // the busy flag (D7).
    uint8_t byte = _lcdRead8Bits(TRUE);
    return (byte & 0x80) ? TRUE : FALSE;
}

/**
 * Initialize the LCD in the appropriate mode
 */
void lcdInitialize(void) {
    // Initialize sequence for 8-bit mode per HD44780
    _lcdWrite8Bits(TRUE, 0x03);
    __delay_ms(5);
    _lcdWrite8Bits(TRUE, 0x03);
    __delay_us(150);
    _lcdWrite8Bits(TRUE, 0x03);
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
    while (isLcdBusy()) {
    }

    // Initialize buffer system cleanly
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100);

    // Display test pattern one line at a time, using shorter strings to avoid wrapping
    lcdBufferSetLine(0, "abcdefghijklmnop"); // 16 chars instead of 20
    lcdBufferUpdate();
    __delay_ms(1000);

    lcdBufferSetLine(1, "ABCDEFGHIJKLMNOP"); // 16 chars instead of 20
    lcdBufferUpdate();
    __delay_ms(1000);

    lcdBufferSetLine(2, "0123456789_!@#$%"); // 16 chars instead of 20
    lcdBufferUpdate();
    __delay_ms(1000);

    lcdBufferSetLine(3, ",./;'[]-=<>?:{}+"); // 16 chars instead of 20
    lcdBufferUpdate();
    __delay_ms(1000);

    // Complete cleanup - buffer only, no direct LCD calls
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100);

    // Final hardware reset and ensure display is fully on
    lcdReturnHome();
    __delay_ms(50);
    while (isLcdBusy()) {
    }
    // Explicitly turn on display with full brightness
    lcdWriteInstruction(DISPLAY_ON);
    __delay_ms(10);
    lcdWriteInstruction(CURSOR_OFF);
    __delay_ms(10);
}

/**
 * Write the full 8-bit byte to the LCD via an MCP23017 I/O expander.  The control
 * bits RS, RW, E are on PA0..PA3 and data bits D0..D7 are on PB0..PB7.
 */
static void _lcdWrite8Bits(boolean inst, uint8_t byte) {
    if (inst) {
        ioporta.LCD_RS = 0; // RS = 1
    } else {
        ioporta.LCD_RS = 1; // RS = 0
    }
    ioporta.LCD_RW = 0; // RW = 0
    ioporta.LCD_E = 1;  // E = 1

    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, byte);
    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    __delay_us(2);

    ioporta.LCD_E = 0; // E = 0
    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    __delay_us(40);
}

/**
 * Helper: read a full 8-bit byte from the LCD via MCP23017 PB0..PB7 with
 * control bits on PA0..PA3.  Since the MCP23017 pins PB0..PB7 are normally
 * outputs, we need to change them to inputs temporarily to read data, then
 * restore them to outputs.
 *
 * @param inst TRUE if reading an instruction, FALSE for data
 */
static uint8_t _lcdRead8Bits(boolean inst) {
    // We need to change PB0..PB7 to inputs temporarily
    uint8_t iodir;
    (void)i2cReadRegister(MCP23017_ADDRESS, IODIRB, &iodir);
    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, (iodir | 0xFF)); // All inputs

    if (inst) {
        ioporta.LCD_RS = 0; // RS = 1
    } else {
        ioporta.LCD_RS = 1; // RS = 0
    }
    ioporta.LCD_RW = 1; // RW = 1
    ioporta.LCD_E = 1;  // E = 1

    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    __delay_us(2);

    uint8_t byte;
    (void)i2cReadRegister(MCP23017_ADDRESS, GPIOB, &byte);

    ioporta.LCD_E = 0; // E = 0
    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    __delay_us(1);

    // Restore PB0..PB7 to outputs
    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, iodir); // Restore IODIR

    return byte;
}