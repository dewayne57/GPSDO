/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * External serial communication module implementation.
 * Handles UART2 for RS-232 communication on RB3 (TxD) and RB4 (RxD).
 * Transmits GPS data every second and provides bootloader support.
 */

#include "serial.h"
#include "config.h"
#include "date.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* UART2 receive buffer and state */
static char rx_buffer[SERIAL_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* Baud rate lookup table defined in data.c */

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
    U2CON2 = 0x00; // Reset UART2

    // Set initial baud rate to 9600 (index from system config ext_baud_index)
    uint8_t baud_index = system_config.ext_baud_index;
    if (baud_index >= BAUD_RATES_COUNT) {
        baud_index = SERIAL_BAUD_9600; // Default to 9600 if invalid
    }
    serial_set_baud_rate(baud_index);

    // Configure UART2 mode
    U2CON0bits.MODE = 0x0; // Asynchronous 8-bit UART mode
    U2CON0bits.RXEN = 1;   // Enable receiver (for future bootloader)
    U2CON0bits.TXEN = 1;   // Enable transmitter

    // Configure parity and stop bits based on system config
    if (system_config.ext_parity == PARITY_E) {
        U2CON0bits.MODE = 0x1; // 8-bit with even parity
    } else if (system_config.ext_parity == PARITY_O) {
        U2CON0bits.MODE = 0x3; // 8-bit with odd parity
    } else {
        U2CON0bits.MODE = 0x0; // 8-bit no parity
    }

    // Note: Stop bits configuration not directly available in this UART module
    // The PIC18F27Q43 UART2 uses 1 stop bit by default in asynchronous mode

    // Enable UART2 (start disabled for now - will enable when needed)
    U2CON1bits.ON = 1; // Enable UART2

    // For now, don't enable RX interrupt (will be enabled when bootloader is needed)
    PIR8bits.U2RXIF = 0; // Clear interrupt flag
    PIE8bits.U2RXIE = 0; // Disable UART2 RX interrupt for now
}

/*
 * Set UART2 baud rate
 */
void serial_set_baud_rate(uint8_t baud_index) {
    if (baud_index >= BAUD_RATES_COUNT) {
        return; // Invalid baud rate index
    }

    uint32_t baud_rate = baud_rate_from_index(baud_index);
    uint32_t baud_div = (_XTAL_FREQ / (4 * baud_rate)) - 1U;

    U2BRGL = (uint8_t)(baud_div & 0xFFU);
    U2BRGH = (uint8_t)((baud_div >> 8) & 0xFFU);
}

/*
 * Reconfigure UART2 settings from system config
 * Call this when external serial port settings are changed
 */
void serial_reconfigure(void) {
    // Disable UART2 briefly to change settings
    U2CON1bits.ON = 0;

    // Set baud rate from config
    uint8_t baud_index = system_config.ext_baud_index;
    if (baud_index >= BAUD_RATES_COUNT) {
        baud_index = SERIAL_BAUD_9600; // Default to 9600 if invalid
    }
    serial_set_baud_rate(baud_index);

    // Configure parity from config
    if (system_config.ext_parity == PARITY_E) {
        U2CON0bits.MODE = 0x1; // 8-bit with even parity
    } else if (system_config.ext_parity == PARITY_O) {
        U2CON0bits.MODE = 0x3; // 8-bit with odd parity
    } else {
        U2CON0bits.MODE = 0x0; // 8-bit no parity
    }

    // Note: Stop bits configuration not directly available in this UART module
    // The PIC18F27Q43 UART2 uses 1 stop bit by default in asynchronous mode

    // Re-enable UART2
    U2CON1bits.ON = 1;
}

/*
 * Send a single character via UART2
 */
void serial_send_char(char c) {
    // Wait for transmit buffer to be empty
    while (!U2FIFObits.TXBE) {
        // Wait
    }

    // Send character
    U2TXB = c;
}

/*
 * Send a null-terminated string via UART2
 */
void serial_send_string(const char* str) {
    if (str == NULL) {
        return;
    }

    while (*str) {
        serial_send_char(*str++);
    }
}

/*
 * Format GPS data into a readable message string
 * Format: "DATE: YYYY-MM-DD TIME: HH:MM:SS LAT: +/-XX.XXXXXX LON: +/-XXX.XXXXXX ALT: XXXX.X SAT: XX\r\n"
 */
void serial_format_gps_message(char* buffer, const gps_data_t* gps_data) {
    if (buffer == NULL || gps_data == NULL) {
        return;
    }

    // Check if GPS data is valid
    if (!gps_data->datetime.valid || !gps_data->position.valid) {
        sprintf(buffer, "GPS: NO VALID DATA\r\n");
        return;
    }

    /* Respect timezone settings */
    char tzbuf[8] = {0};
    gps_datetime_t outdt;
    const gps_datetime_t* use_dt = &gps_data->datetime;
    if (system_config.tz_mode == 1) {
        /* Local: apply offset */
        date_apply_offset(&gps_data->datetime, &outdt, system_config.tz_offset_min);
        use_dt = &outdt;
        tz_offset_to_string(system_config.tz_offset_min, tzbuf);
    } else {
        /* UTC */
        strcpy(tzbuf, "UTC");
    }

    sprintf(buffer,
            "DATE: 20%02d-%02d-%02d TIME: %02d:%02d:%02d TZ: %s LAT: %+09.6f LON: %+010.6f ALT: %+07.1f SAT: %02d\r\n",
            use_dt->year, use_dt->month, use_dt->day, use_dt->hour, use_dt->minute, use_dt->second, tzbuf,
            gps_data->position.latitude, gps_data->position.longitude, gps_data->position.altitude,
            gps_data->position.satellites);
}

/*
 * Send GPS data via UART2 in a formatted message
 */
void serial_send_gps_data(const gps_data_t* gps_data) {
    char message_buffer[256];

    serial_format_gps_message(message_buffer, gps_data);
    serial_send_string(message_buffer);
}

/*
 * Check if data is available in the receive buffer
 */
bool serial_data_available(void) {
    return (rx_head != rx_tail);
}

/*
 * Get a character from the receive buffer
 */
char serial_get_char(void) {
    if (rx_head == rx_tail) {
        return 0; // No data available
    }

    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % SERIAL_BUFFER_SIZE;
    return c;
}

/*
 * Enable UART2 receive interrupt (for bootloader mode)
 */
void serial_enable_rx_interrupt(void) {
    PIR8bits.U2RXIF = 0; // Clear interrupt flag
    IPR8bits.U2RXIP = 0; // Low priority interrupt
    PIE8bits.U2RXIE = 1; // Enable UART2 RX interrupt
}

/*
 * Disable UART2 receive interrupt
 */
void serial_disable_rx_interrupt(void) {
    PIE8bits.U2RXIE = 0; // Disable UART2 RX interrupt
}

/*
 * Put character into receive buffer (called from ISR)
 */
void serial_buffer_put_char(char c) {
    uint16_t next_head = (rx_head + 1) % SERIAL_BUFFER_SIZE;

    if (next_head != rx_tail) { // Buffer not full
        rx_buffer[rx_head] = c;
        rx_head = next_head;
    }
    // If buffer is full, discard character (could add overflow flag here)
}

/*
 * Debug helper function for formatted output
 */
void serial_debug_printf(const char* format, ...) {
    char debug_buffer[128];
    va_list args;

    va_start(args, format);
    vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);

    serial_send_string(debug_buffer);
}

/*
 * Debug helper for integer values
 */
void serial_debug_int(const char* label, int32_t value) {
    char debug_buffer[64];
    sprintf(debug_buffer, "[DEBUG] %s: %ld\r\n", label, value);
    serial_send_string(debug_buffer);
}

/*
 * Debug helper for float values
 */
void serial_debug_float(const char* label, float value) {
    char debug_buffer[64];
    sprintf(debug_buffer, "[DEBUG] %s: %.6f\r\n", label, value);
    serial_send_string(debug_buffer);
}

/*
 * Debug helper for hexadecimal values
 */
void serial_debug_hex(const char* label, uint32_t value) {
    char debug_buffer[64];
    sprintf(debug_buffer, "[DEBUG] %s: 0x%08lX\r\n", label, value);
    serial_send_string(debug_buffer);
}

/*
 * Send CSV header for Data Visualizer plotting
 */
void serial_send_csv_header(void) {
    serial_send_string("Time,Value1,Value2,Value3\r\n");
}

/*
 * Send CSV formatted data for Data Visualizer plotting
 */
void serial_send_csv_data(float timestamp, float value1, float value2, float value3) {
    char csv_buffer[128];
    sprintf(csv_buffer, "%.3f,%.6f,%.6f,%.6f\r\n", timestamp, value1, value2, value3);
    serial_send_string(csv_buffer);
}