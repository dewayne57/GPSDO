/*
 * Copyright (c) 2026, Dewayne L. Hafenstein.  All rights reserved.
 *
 * This module handles USB serial communication using UART3.
 * It provides buffered serial I/O for USB interface on RB2 (TX) and RB3 (RX).
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

#include "usb.h"
#include "config.h"
#include <xc.h>

// Circular buffer for USB receive data
static volatile char usb_rx_buffer[USB_BUFFER_SIZE];
static volatile uint8_t usb_rx_head = 0;
static volatile uint8_t usb_rx_tail = 0;

/*
 * Initialize UART3 for USB communication at 115200 baud.
 * Pins: RB2 (USB_TX), RB3 (USB_RX)
 */
void usb_init(void) {
    // Configure PPS for UART3
    // RB3 -> UART3 RX input
    U3RXPPS = 0x0B; // RB3 = pin 11

    // RB2 -> UART3 TX output
    RB2PPS = 0x33; // UART3 TX function

    // Configure UART3 for 115200 baud at 64 MHz Fosc
    // UART3 clock source = Fosc (64 MHz)
    // BRG = (Fosc / (4 * baudrate)) - 1
    // BRG = (64000000 / (4 * 115200)) - 1 = 138.89 ≈ 139
    U3BRGL = 139;
    U3BRGH = 0;

    // Configure UART3 control registers
    U3CON0bits.MODE = 0b000; // 8-bit async mode
    U3CON0bits.BRGS = 0;     // High-speed mode (divide by 4)
    U3CON0bits.TXEN = 1;     // Enable transmitter
    U3CON0bits.RXEN = 1;     // Enable receiver

    U3CON1 = 0x00; // No auto-baud, normal operation
    U3CON2 = 0x00; // Normal operation

    // Enable UART3 receive interrupt
    PIE9bits.U3RXIE = 1; // Enable UART3 RX interrupt
    PIR9bits.U3RXIF = 0; // Clear interrupt flag

    // Enable UART3
    U3CON1bits.ON = 1;
}

/*
 * Reconfigure USB UART baud rate.
 */
void usb_reconfigure(uint32_t baudrate) {
    // Calculate BRG value for high-speed mode
    // BRG = (Fosc / (4 * baudrate)) - 1
    uint16_t brg = (uint16_t)((64000000UL / (4UL * baudrate)) - 1);

    // Temporarily disable UART3
    U3CON1bits.ON = 0;

    // Update baud rate
    U3BRGL = (uint8_t)(brg & 0xFF);
    U3BRGH = (uint8_t)((brg >> 8) & 0xFF);

    // Re-enable UART3
    U3CON1bits.ON = 1;
}

/*
 * Send a single character over USB (UART3).
 */
void usb_send_char(char c) {
    // Wait for transmit buffer to have space
    while (!U3FIFObits.TXBE)
        ;

    // Send the character
    U3TXB = c;
}

/*
 * Send a null-terminated string over USB (UART3).
 */
void usb_send_string(const char* str) {
    while (*str) {
        usb_send_char(*str++);
    }
}

/*
 * Put a character into the USB receive buffer (called from ISR).
 */
void usb_buffer_put_char(char c) {
    uint8_t next_head = (usb_rx_head + 1) % USB_BUFFER_SIZE;

    // Only store if buffer not full
    if (next_head != usb_rx_tail) {
        usb_rx_buffer[usb_rx_head] = c;
        usb_rx_head = next_head;
    }
}

/*
 * Get a character from the USB receive buffer.
 */
char usb_buffer_get_char(void) {
    // Check if buffer is empty
    if (usb_rx_head == usb_rx_tail) {
        return 0;
    }

    // Get character and update tail
    char c = usb_rx_buffer[usb_rx_tail];
    usb_rx_tail = (usb_rx_tail + 1) % USB_BUFFER_SIZE;

    return c;
}

/*
 * Check if there are characters available in the USB receive buffer.
 */
bool usb_buffer_has_data(void) {
    return (usb_rx_head != usb_rx_tail);
}

/*
 * Get the number of characters available in the USB receive buffer.
 */
unsigned char usb_buffer_count(void) {
    if (usb_rx_head >= usb_rx_tail) {
        return usb_rx_head - usb_rx_tail;
    } else {
        return (unsigned char)(USB_BUFFER_SIZE - (usb_rx_tail - usb_rx_head));
    }
}
