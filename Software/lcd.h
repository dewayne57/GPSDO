/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * LCD functions for writing to the display.
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
 * This code provides a simple interface to the LCD display using a
 * Hitachi HD44780 compatible controller via the MCP23017 I2C I/O expander.
 * It supports both 4-bit and 8-bit modes.  To enable 4-bit mode, define
 * the LCD_4BIT_MODE macro before including this file.  Otherwise, it defaults
 * to 8-bit mode.
 */
 #ifndef LCD_H
#define    LCD_H
#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h> // use standard bool/type

#ifdef    __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * HD44780 Command Codes 
 */
#define CLEAR                   0b00000001
#define RETURN_HOME             0b00000010
#define CURSOR_INCREMENT        0b00000110
#define CURSOR_INCREMENT_SHIFT  0b00000111
#define CURSOR_DECREMENT        0b00000100
#define CURSOR_DECREMENT_SHIFT  0b00000101
#define DISPLAY_ON              0b00001100
#define CURSOR_ON               0b00001110
#define CURSOR_BLINK            0b00001111
#define CURSOR_SOLID            0b00001110
#define CURSOR_OFF              0b00001100
#define DISPLAY_OFF             0b00001000
#define CURSOR_SHIFT_RIGHT      0b00010100
#define CURSOR_SHIFT_LEFT       0b00010000
#define DISPLAY_SHIFT_RIGHT     0b00011100
#define DISPLAY_SHIFT_LEFT      0b00011000
#define FUNCTION_SET_8_2_5X8    0b00111000
#define FUNCTION_SET_4_2_5X8    0b00101000
#define SET_CGRAM_ADDRESS       0b01000000
#define SET_DDRAM_ADDRESS       0b10000000

/**
 * LCD Line Addresses
 */
#define LINE_0                  0x00
#define LINE_1                  0x40
#define LINE_2                  0x14 
#define LINE_3                  0x54

/**
 * LCD Buffer System Constants
 */
#define LCD_LINES 4
#define LCD_CHARS_PER_LINE 20

/*
 * Function Prototypes
 */
void updateDisplay(void); 
void lcdSetBacklight(bool state);
void lcdReturnHome(void); 
void lcdClearDisplay(void); 
void lcdWriteBuffer(uint8_t address, char* data);
void lcdWriteString(char * data);
void lcdWriteChar(uint8_t data);
void lcdWriteInstruction(uint8_t data);
uint8_t lcdReadData(void);
bool isLcdBusy(void);
void lcdInitialize(void);
void lcdSelfTest(void);

/* LCD Buffer System Functions */
void lcdBufferInit(void);
void lcdBufferClear(void);
void lcdBufferSetLine(uint8_t line, const char* text);
void lcdBufferPrintf(uint8_t line, const char* format, ...);
void lcdBufferUpdate(void);

#ifdef    __cplusplus
}
#endif /* __cplusplus */

#endif    /* LCD_H */

