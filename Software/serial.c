/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * External serial communication module implementation.
 * Handles UART2 for RS-232 communication on RB3 (TxD) and RB4 (RxD).
 * Uses printf redirection for all serial output.
 *
 * Supports sending GPS data and future bootloader functionality.
 * Configurable baud rate, parity, and stop bits via system configuration.
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
 */

#include "serial.h"
#include "config.h"
#include "date.h"
#include <stdio.h>
#include <string.h>

/* UART2 receive buffer and state */
static char rx_buffer[SERIAL_BUFFER_SIZE];
static volatile uint8_t rx_head = 0; /* 0-255 index (buffer is 256 bytes) */
static volatile uint8_t rx_tail = 0;

/*
 * Send a single character via UART2 (static - only used internally)
 */
static void serial_send_char(char c) {
    // Wait for transmit buffer to be empty
    while (!U2FIFObits.TXBE) {
        // Wait
    }

    // Send character
    U2TXB = c;
}

/*
 * Set UART2 baud rate (static - only used internally)
 */
static void serial_set_baud_rate(uint8_t baud_index) {
    if (baud_index >= BAUD_RATES_COUNT) {
        return; // Invalid baud rate index
    }

    uint32_t baud_rate = baud_rate_from_index(baud_index);
    uint32_t baud_div = (_XTAL_FREQ / (4 * baud_rate)) - 1U;

    U2BRGL = (uint8_t)(baud_div & 0xFFU);
    U2BRGH = (uint8_t)((baud_div >> 8) & 0xFFU);
}

/*
 * Initialize external serial communication using UART2
 * Configures RB3 as TxD and RB4 as RxD for RS-232 interface
 */
void serial_init(void) {
    // Clear receive buffer
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_head = rx_tail = 0;

    // Configure UART2 registers
    U2CON0 = 0x00; // Reset UART2
    U2CON1 = 0x00; // Reset UART2
    U2CON2 = 0x04; // BRGS=1 for high-speed baud (4x clock in divisor)

    // Set initial baud rate to 9600 (index from system config ext_baud_index)
    uint8_t baud_index = baud_rate_index(system_config.ext_baud);
    if (baud_index >= BAUD_RATES_COUNT) {
        baud_index = SERIAL_BAUD_9600; // Default to 9600 if invalid
    }
    serial_set_baud_rate(baud_index);

    // Configure UART2 mode
    U2CON0bits.MODE = 0x0; // Asynchronous 8-bit UART mode
    U2CON0bits.RXEN = 1;   // Enable receiver (for future bootloader)
    U2CON0bits.TXEN = 1;   // Enable transmitter

    // Configure parity and stop bits based on system config (only N/E/O supported)
    uint8_t parity = system_config.ext_parity;
    if (parity >= PARITY_OPTIONS_COUNT) {
        parity = PARITY_N;
    }
    if (parity == PARITY_E) {
        U2CON0bits.MODE = 0x1; // 8-bit with even parity
    } else if (parity == PARITY_O) {
        U2CON0bits.MODE = 0x3; // 8-bit with odd parity
    } else {
        U2CON0bits.MODE = 0x0; // 8-bit no parity
    }

    // Note: Stop bits configuration not directly available in this UART module
    // The PIC18F27Q43 UART2 uses 1 stop bit by default in asynchronous mode

    // Enable UART2
    U2CON1bits.ON = 1; // Enable UART2

    // For now, don't enable RX interrupt (will be enabled when bootloader is needed)
    PIR8bits.U2RXIF = 0; // Clear interrupt flag
    PIE8bits.U2RXIE = 0; // Disable UART2 RX interrupt for now
}

/*
 * Reconfigure UART2 settings from system config
 * Call this when external serial port settings are changed
 */
void serial_reconfigure(void) {
    // Disable UART2 briefly to change settings
    U2CON1bits.ON = 0;

    // Set baud rate from config
    uint8_t baud_index = baud_rate_index(system_config.ext_baud);
    if (baud_index >= BAUD_RATES_COUNT) {
        baud_index = SERIAL_BAUD_9600; // Default to 9600 if invalid
    }
    serial_set_baud_rate(baud_index);

    // Configure parity from config (only N/E/O supported)
    uint8_t parity = system_config.ext_parity;
    if (parity >= PARITY_OPTIONS_COUNT) {
        parity = PARITY_N;
    }
    if (parity == PARITY_E) {
        U2CON0bits.MODE = 0x1; // 8-bit with even parity
    } else if (parity == PARITY_O) {
        U2CON0bits.MODE = 0x3; // 8-bit with odd parity
    } else {
        U2CON0bits.MODE = 0x0; // 8-bit no parity
    }

    // Re-enable UART2
    U2CON1bits.ON = 1;
}

/*
 * Put character into receive buffer (called from ISR)
 */
void serial_buffer_put_char(char c) {
    uint8_t next_head = (uint8_t)(rx_head + 1); // wraps at 256

    if (next_head != rx_tail) { // Buffer not full
        rx_buffer[rx_head] = c;
        rx_head = next_head;
    }
    // If buffer is full, discard character (could add overflow flag here)
}

/*
 * Printf redirection function
 * This function is called by the XC8 compiler's printf implementation
 * to output characters to UART2
 */
void putch(char c) {
    serial_send_char(c);
}