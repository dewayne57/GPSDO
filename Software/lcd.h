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
 */
 #ifndef LCD_H
#define    LCD_H
#include <xc.h> // include processor files - each processor file is guarded.  

#ifdef    __cplusplus
extern "C" {
#endif /* __cplusplus */

/* LCD pin mapping on MCP23017 Port B
 * PB0 = RS
 * PB1 = RW
 * PB2 = E
 * PB3 = Backlight (1 = on)
 * PB4..PB7 = D4..D7 (4-bit data)
 */
typedef struct {
        unsigned char rs : 1;   // register select
        unsigned char rw : 1;   // read/*write 
        unsigned char e : 1;    // Enable
        unsigned char bl : 1;   // backlight 
        unsigned char data: 4;  // data bits D4-D7
    } CONTROL_BYTE; 

union {
    CONTROL_BYTE control;
    uint8_t byte;
} lcd;

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
#define SET_CGRAM_ADDRESS       0b01000000
#define SET_DDRAM_ADDRESS       0b10000000

/**
 * LCD Line Addresses
 */
#define LINE_0                  0x00
#define LINE_1                  0x40
#define LINE_2                  0x14 
#define LINE_3                  0x54

/*
 * Function Prototypes
 */
void lcdReturnHome(void); 
void lcdClearDisplay(void); 
void lcdWriteBuffer(uint8_t address, char* data);
void lcdWriteString(char * data);
void lcdWriteByte(uint8_t data);
void lcdWriteInstruction(uint8_t data);
uint8_t lcdReadInstruction(void);
uint8_t lcdReadData(void);
unsigned char isLcdBusy(void);
void lcdInitialize(void);
void lcdSelfTest(void); 

#ifdef    __cplusplus
}
#endif /* __cplusplus */

#endif    /* LCD_H */

