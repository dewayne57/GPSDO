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
#include "types.h"
#include "lcd.h"
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include <xc.h>

#define LCD_4BIT_MODE

/*********************************************************************************/
/* LCD Functions                                                                 */
/*                                                                               */
/* LCD functions for writing to the display.                                     */
/* Dewayne Hafenstein                                                            */
/*********************************************************************************/
#ifdef LCD_4BIT_MODE
static void _lcdWrite4Bits(boolean inst, uint8_t nibble);
static void _lcdRead4Bits(boolean inst, uint8_t* nibble);
#else
static void _lcdWrite8Bits(boolean inst, uint8_t byte);
static void _lcdRead8Bits(boolean inst, uint8_t* byte);
#endif

/*
 * Note, the control byte is used for just the 4 control signals to the LCD when
 * operating in 8-bit mode.  In 8-bit mode, the control signals are assumed to
 * be mapped to the MCP23017 PORT B pins PB0-PB3.  The data in 8-bit mode is
 * assumed to be read/written using PORT A of the MCP23017.
 *
 * In 4-bit mode, the control signals are combined with the data nibbles as they
 * are sent.  In this mode, the data bits D4-D7 are assumed to be mapped to
 * MCP23017 PORT B pins PB4-PB7, with control signals on PB0-PB3.
 *
 * In all cases, the control signals are:
 * PB0 - RS: Register Select (Instruction/Data)
 * PB1 - RW: Read/Write
 * PB2 - E: Enable
 * PB3 - BL: Backlight (1=on, 0=off)
 */
static uint8_t control_byte = 0x00;     // Control byte for LCD control signals
static boolean backlight_on = FALSE;    // Initially set backlight off.

/**
 * This function sets the LCD backlight state that all the other functions use. 
 */
void lcdSetBacklight(boolean state) {
    backlight_on = state;
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
void lcdWriteString(char * data) {
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

/**
 * Write an instruction byte to the LCD.
 *
 * @param data The instruction byte to write.
 */
void lcdWriteInstruction(uint8_t data) {
#ifdef LCD_4BIT_MODE
    // Send high nibble then low nibble
    _lcdWrite4Bits(TRUE, (data >> 4) & 0x0F);
    _lcdWrite4Bits(TRUE, data & 0x0F);
#else
    // Send full byte
    _lcdWrite8Bits(TRUE, data);
#endif

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
#ifdef LCD_4BIT_MODE
    // Send high nibble then low nibble
    uint8_t tmp = data;
    _lcdWrite4Bits(FALSE, (tmp >> 4) & 0x0F);
    _lcdWrite4Bits(FALSE, data & 0x0F);
#else
    _lcdWrite8Bits(FALSE, data);
#endif
}

/**
 * Read a data byte from the LCD at the current address.
 */
uint8_t lcdReadData(void) {
    uint8_t byte;
#ifdef LCD_4BIT_MODE
    _lcdRead4Bits(FALSE, &byte);
    byte <<= 4;
    uint8_t lo;
    _lcdRead4Bits(FALSE, &lo);
    byte |= (lo & 0x0F);
    return byte;
#else
    _lcdRead8Bits(FALSE, &byte);
#endif
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
    
    __delay_ms(1); 
    return FALSE;

//    control_byte = 0x00;
//    control_byte |= LCD_RW;  // RW = 1
//    control_byte &= ~LCD_RS; // RS = 0
//    control_byte |= LCD_E;   // E = 1
//    control_byte |= LCD_BL;  // Backlight
//
//    // Read IODIR and save it so we can put it back later
//    uint8_t iodir;
//    (void)i2cReadRegister(MCP23017_ADDRESS, IODIRB, &iodir);
//    // Make PB4..PB7 inputs
//    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, (iodir | 0xF0));
//
//    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
//    __delay_us(2);
//
//    uint8_t st;
//    (void)i2cReadRegister(MCP23017_ADDRESS, GPIOA, &st);
//    control_byte &= ~LCD_E; // E = 0
//    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
//    __delay_us(2);
//
//    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, iodir); // Restore IODIR
//
//    // Busy flag is D7 (MSB)
//    return (st & 0x80) ? TRUE : FALSE;
}

/**
 * Initialize the LCD in the appropriate mode (4-bit or 8-bit).
 */
void lcdInitialize(void) {
    // Initialize sequence for 4-bit mode per HD44780
    _lcdWrite4Bits(FALSE, 0x03);
    __delay_ms(5);
    _lcdWrite4Bits(FALSE, 0x03);
    __delay_us(150);
    _lcdWrite4Bits(FALSE, 0x03);
    __delay_us(150);

#ifdef LCD_4BIT_MODE
    // Set 4-bit mode (This is only the first 4-bits of the FUNCTION_SET command)
    _lcdWrite4Bits(FALSE, 0x02);
    lcdWriteInstruction(FUNCTION_SET_4_2_5X8);
#else
    // Function set: 8-bit, 2 lines, 5x8 dots
    lcdWriteInstruction(FUNCTION_SET_8_2_5X8);
#endif

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
    lcdWriteInstruction(CURSOR_OFF);
    lcdWriteBuffer(LINE_0, "abcdefghijklmnopqrst");
    __delay_ms(500);
    lcdWriteBuffer(LINE_1, "ABCDEFGHIJKLMNOPQRST");
    __delay_ms(500);
    lcdWriteBuffer(LINE_2, "0123456789_!@#$%^&*/");
    __delay_ms(500);
    lcdWriteBuffer(LINE_3, ",./;'[]-=<>?:\"{}_+~`");
    __delay_ms(1000);
    lcdClearDisplay();
    lcdReturnHome();
    lcdWriteBuffer(LINE_0, "Ready!");
    while (isLcdBusy()) {
    }
}

#ifdef LCD_4BIT_MODE
/**
 * Helper: write a 4-bit nibble to the LCD via MCP23017 PB4..PB7 with
 * control bits on PB0..PB3
 *
 * @param inst TRUE if writing an instruction, FALSE for data
 * @param nibble The 4-bit nibble to write (in the low 4 bits)
 */
static void _lcdWrite4Bits(boolean inst, uint8_t nibble) {
    control_byte = 0x00;
    if (inst) {
        control_byte &= ~LCD_RS;    // RS = 0
    } else {
        control_byte |= LCD_RS;     // RS = 1
    }
    control_byte |= backlight_on ? LCD_BL : 0;  // Set backlight state
    control_byte &= ~LCD_RW;                    // RW = 0
    control_byte |= LCD_E;                      // E = 1
    control_byte |= (nibble << 4);              // Data bits D4-D7
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(2);
    control_byte &= ~LCD_E; // E = 0
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(40);
}

/**
 * Read a 4-bit nibble from the LCD via MCP23017 PB4..PB7 with control bits on PB0..PB3
 *
 * @param inst TRUE if reading an instruction, FALSE for data
 * @param nibble Pointer to store the read nibble in the low 4 bits
 */
static void _lcdRead4Bits(boolean inst, uint8_t* nibble) {

    // Read IODIR and save it so we can put it back later
    uint8_t iodir;
    (void)i2cReadRegister(MCP23017_ADDRESS, IODIRB, &iodir);
    // Make PB4..PB7 inputs
    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, (iodir | 0xF0));

    // Read high nibble
    control_byte = 0x00;
    if (inst) {
        control_byte &= ~LCD_RS;    // RS = 0
    } else {
        control_byte |= LCD_RS;     // RS = 1
    }
    control_byte |= backlight_on ? LCD_BL : 0; // Set backlight state
    control_byte |= LCD_RW;                    // RW = 1
    control_byte |= LCD_E;                     // E = 1

    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(2);

    uint8_t tmp;
    (void)i2cReadRegister(MCP23017_ADDRESS, GPIOB, &tmp);
    *nibble = (tmp & 0xF0) >> 4;

    control_byte &= ~LCD_E; // E = 0
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(2);

    // Restore IODIR
    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, iodir);
}

#else
/**
 * Write the full 8-bit byte to the LCD via an MCP23017 I/O expander.  The control
 * bits RS, RW, E are on PB0..PB2 and data bits D0..D7 are on PA0..PA7.
 */
static void _lcdWrite8Bits(boolean inst, uint8_t byte) {
    control_byte = 0x00;
    if (inst) {
        control_byte |= LCD_RS; // RS = 1
    } else {
        control_byte &= ~LCD_RS; // RS = 0
    }
    control_byte |= backlight_on ? LCD_BL : 0; // Set backlight state
    control_byte &= ~LCD_RW;                   // RW = 0
    control_byte |= LCD_E;                     // E = 1
    control_byte |= (nibble << 4);             // Data bits D4-D7

    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, byte);
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(2);

    control_byte &= ~LCD_E; // E = 0
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(40);
}

/**
 * Helper: read a full 8-bit byte from the LCD via MCP23017 PA0..PA7 with
 * control bits on PB0..PB3
 *
 * @param inst TRUE if reading an instruction, FALSE for data
 * @param byte Pointer to store the read byte
 *
 */
static void _lcdRead8Bits(boolean inst, uint8_t* byte) {
    control_byte = 0x00;
    if (inst) {
        control_byte |= LCD_RS; // RS = 1
    } else {
        control_byte &= ~LCD_RS; // RS = 0
    }
    control_byte |= backlight_on ? LCD_BL : 0; // Set backlight state
    control_byte |= LCD_RW;                    // RW = 1
    control_byte |= LCD_E;                     // E = 1
    control_byte |= (nibble << 4);             // Data bits D4-D7
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(2);

    (void)i2cReadRegister(MCP23017_ADDRESS, GPIOA, &byte);

    control_byte &= 0xFF - LCD_E; // E = 0
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, control_byte);
    __delay_us(1);
}
#endif
