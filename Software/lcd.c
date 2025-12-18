/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 */
#include <xc.h>
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "lcd.h"

/*********************************************************************************/
/* LCD Functions                                                                 */
/*                                                                               */
/* LCD functions for writing to the display.                                     */
/* Dewayne Hafenstein                                                            */
/*********************************************************************************/

static void lcdWrite4Bits(uint8_t nibble);
static uint8_t lcdReadByte(void);

/**
 * The address to update.
 */
void lcdWriteBuffer(uint8_t address, char* data) {
    lcdWriteInstruction(SET_DDRAM_ADDRESS | address);
    char *p = data;
    while (*p) {
        lcdWriteByte(*p++);
    }
}

void lcdReturnHome(void) {
    lcdWriteInstruction(RETURN_HOME);
}

void lcdClearDisplay(void) {
    lcdWriteInstruction(CLEAR);
}

void lcdWriteInstruction(uint8_t data) {
    // Command write (RS=0)
    lcd.control.rw = 0;
    lcd.control.rs = 0;
    lcd.control.e = 0;
    // Send high nibble then low nibble
    lcdWrite4Bits((data >> 4) & 0x0F);
    lcdWrite4Bits(data & 0x0F);
    // Wait for completion
    while (isLcdBusy()) {
    }
}

uint8_t lcdReadInstruction(void) {
    lcd.control.rw = 1;
    lcd.control.rs = 0;
    return lcdReadByte();
}

void lcdWriteByte(uint8_t data) {
    // Data write (RS=1)
    lcd.control.rw = 0;
    lcd.control.rs = 1;
    lcd.control.e = 0;
    lcdWrite4Bits((data >> 4) & 0x0F);
    lcdWrite4Bits(data & 0x0F);
}

uint8_t lcdReadData(void) {
    lcd.control.rw = 1;
    lcd.control.rs = 1;
    return lcdReadByte();
}

unsigned char isLcdBusy(void) {
    uint8_t st = lcdReadInstruction();
    // Busy flag is D7 (MSB)
    return (st & 0x80) ? 1 : 0;
}

void lcdInitialize(void) {
    lcd.control.bl = 1; // turn on backlight
    __delay_ms(15);
    // Initialize sequence for 4-bit mode per HD44780
    lcd.control.rs = 0;
    lcd.control.rw = 0;
    // Send 0x03 three times
    lcdWrite4Bits(0x03);
    __delay_ms(5);
    lcdWrite4Bits(0x03);
    __delay_us(150);
    lcdWrite4Bits(0x03);
    __delay_us(150);
    // Set 4-bit mode
    lcdWrite4Bits(0x02);

    // Function set: 4-bit, 2 lines, 5x8 dots
    lcdWriteInstruction(0x28);
    // Display off
    lcdWriteInstruction(0x08);
    // Clear display
    lcdWriteInstruction(0x01);
    __delay_ms(2);
    // Entry mode set: increment, no shift
    lcdWriteInstruction(0x06);
    // Display on, cursor off
    lcdWriteInstruction(0x0C);
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

/* Helper: write a 4-bit nibble to the LCD via MCP23017 PB4..PB7 with control bits on PB0..PB3 */
static void lcdWrite4Bits(uint8_t nibble) {
    // place nibble into high nibble (PB4..PB7)
    lcd.control.data = (nibble & 0x0F);
    // E = 1
    lcd.control.e = 1;
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, lcd.byte);
    __delay_us(2);
    // E = 0
    lcd.control.e = 0;
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, lcd.byte);
    __delay_us(40);
}

/* Helper: read a full byte from LCD in 4-bit mode (RS and RW already set in lcd.control) */
static uint8_t lcdReadByte(void) {
    uint8_t iodir;
    (void)i2cReadRegister(MCP23017_ADDRESS, IODIRB, &iodir);
    // Make PB4..PB7 inputs
    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, (iodir | 0xF0));

    uint8_t hi, lo, tmp;

    // Read high nibble
    lcd.control.e = 1; // E high
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, lcd.byte);
    __delay_us(2);
    (void)i2cReadRegister(MCP23017_ADDRESS, GPIOB, &tmp);
    hi = tmp & 0xF0;
    lcd.control.e = 0; // E low
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, lcd.byte);
    __delay_us(1);

    // Read low nibble
    lcd.control.e = 1; // E high
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, lcd.byte);
    __delay_us(2);
    (void)i2cReadRegister(MCP23017_ADDRESS, GPIOB, &tmp);
    lo = (tmp & 0xF0) >> 4;
    lcd.control.e = 0; // E low
    i2cWriteRegister(MCP23017_ADDRESS, GPIOB, lcd.byte);

    // Restore IODIR
    i2cWriteRegister(MCP23017_ADDRESS, IODIRB, iodir);

    lcd.control.rw = 0;
    return (hi | lo);
}

