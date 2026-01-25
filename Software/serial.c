/*
 * Copyright (c) 2026, Dewayne L. Hafenstein.  All rights reserved.
 *
 * External serial communication module implementation.
 * Handles UART1 for RS-232 communication on RB3 (TxD) and RB4 (RxD).
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

/* UART1 receive buffer and state */
static char rx_buffer[SERIAL_BUFFER_SIZE];
static volatile unsigned char rx_head = 0; /* 0-255 index (buffer is 256 bytes) */
static volatile unsigned char rx_tail = 0;

/*
 * Send a single character via UART1 (static - only used internally)
 */
static void serial_send_char(char c) {
    // Wait for transmit buffer to be empty
    while (!U1FIFObits.TXBE) {
        // Wait
    }

    // Send character
    U1TXB = c;
}

/*
 * Initialize external serial communication using UART1
 * Configures RB3 as TxD and RB4 as RxD for RS-232 interface
 */
void serial_init(void) {
    // Clear receive buffer
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_head = rx_tail = 0;

    // Configure UART1 registers
    U1CON0 = 0x00; // Reset UART1
    U1CON1 = 0x00; // Reset UART1
    U1CON2 = 0x04; // BRGS=1 for high-speed baud (4x clock in divisor)

    // Set initial baud rate to 9600 (index from system config ext_baud_index)
    long baud_rate = system_config.ext_baud;
    long baud_div = _XTAL_FREQ / (16 * (baud_rate + 1U));

    U1BRGL = (unsigned char)(baud_div & 0xFFU);
    U1BRGH = (unsigned char)((baud_div >> 8) & 0xFFU);

    // Configure UART1 mode
    // Configure parity and stop bits based on system config
    switch (system_config.ext_parity) {
        case PARITY_NONE:
            U1CON0bits.MODE = 0x00; // 8-bit no parity
            break;
        case PARITY_EVEN:
            U1CON0bits.MODE = 0x03; // 8-bit with even parity
            break;
        case PARITY_ODD:
            U1CON0bits.MODE = 0x02; // 8-bit with odd parity
            break;
        default:
            U1CON0bits.MODE = 0x0; // 8-bit no parity
            break;
    }
    // Configure UART1 mode
    U1CON0bits.RXEN = 1; // Enable receiver
    U1CON0bits.TXEN = 1; // Enable transmitter

    // Note: Stop bits configuration not directly available in this UART module
    // The PIC18F27Q43 UART1 uses 1 stop bit by default in asynchronous mode

    // Enable UART1
    U1CON1bits.ON = 1; // Enable UART1

    // For now, don't enable RX interrupt (will be enabled when bootloader is needed)
    PIR4bits.U1RXIF = 0; // Clear interrupt flag
    PIE4bits.U1RXIE = 0; // Disable UART1 RX interrupt for now
}

/*
 * Reconfigure UART1 settings from system config
 * Call this when external serial port settings are changed
 */
void serial_reconfigure(void) {
    // Disable UART1 briefly to change settings
    U1CON1bits.ON = 0;
    serial_init();
}

/*
 * Put character into receive buffer (called from ISR)
 */
void serial_buffer_put_char(char c) {
    unsigned char next_head = (unsigned char)(rx_head + 1); // wraps at 256

    if (next_head != rx_tail) { // Buffer not full
        rx_buffer[rx_head] = c;
        rx_head = next_head;
    }
    // If buffer is full, discard character (could add overflow flag here)
}

/*
 * Printf redirection function
 * This function is called by the XC8 compiler's printf implementation
 * to output characters to UART1
 */
void putch(char c) {
    serial_send_char(c);
}