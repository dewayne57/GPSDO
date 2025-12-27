/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * External serial communication module for RS-232 interface.
 * This module handles UART2 communication on pins RB3 (TxD) and RB4 (RxD)
 * for transmitting GPS data and supporting future bootloader functionality.
 * All output now goes through printf redirection to UART2.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "gps.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Serial UART2 configuration */
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_BAUD_9600   5
#define SERIAL_BAUD_19200  6
#define SERIAL_BAUD_38400  7
#define SERIAL_BAUD_57600  8
#define SERIAL_BAUD_115200 9

/* Serial message format structure */
typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    float latitude;
    float longitude;
    float altitude;
    uint8_t satellites;
    bool valid;
} serial_gps_data_t;

/* Function prototypes */
void serial_init(void);
void serial_reconfigure(void);
void serial_send_gps_data(const gps_data_t *gps_data);

/* Bootloader support functions */
bool serial_data_available(void);
char serial_get_char(void);
void serial_enable_rx_interrupt(void);
void serial_disable_rx_interrupt(void);

/* Internal buffer management (for ISR) */
void serial_buffer_put_char(char c);

/* Printf redirection function */
void putch(char c);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H */