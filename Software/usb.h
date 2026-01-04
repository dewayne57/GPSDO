/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
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

#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <stdbool.h>

#define USB_BUFFER_SIZE 256

/*
 * Initialize UART3 for USB communication at 115200 baud.
 */
void usb_init(void);

/*
 * Reconfigure USB UART baud rate.
 * 
 * @param baudrate - The desired baud rate.
 */
void usb_reconfigure(uint32_t baudrate);

/*
 * Send a single character over USB (UART3).
 * This function will wait for transmit buffer space before sending.
 *
 * @param c - The character to send.
 */
void usb_send_char(char c);

/*
 * Send a null-terminated string over USB (UART3).
 *
 * @param str - Pointer to the string to send.
 */
void usb_send_string(const char *str);

/*
 * Put a character into the USB receive buffer (called from ISR).
 *
 * @param c - The character to buffer.
 */
void usb_buffer_put_char(char c);

/*
 * Get a character from the USB receive buffer.
 *
 * @return The character read, or 0 if buffer is empty.
 */
char usb_buffer_get_char(void);

/*
 * Check if there are characters available in the USB receive buffer.
 *
 * @return true if characters are available, false otherwise.
 */
bool usb_buffer_has_data(void);

/*
 * Get the number of characters available in the USB receive buffer.
 *
 * @return Number of characters in the buffer.
 */
unsigned char usb_buffer_count(void);

#endif // USB_H
