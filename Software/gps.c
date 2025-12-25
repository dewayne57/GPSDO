/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * GPS module implementation for ublox M8M GPS receiver.
 * Handles UART communication and NMEA message parsing.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gps.h"
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "date.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>


/* 
 * global data areas 
 */
extern system_config_t system_config;       // System configuration data
volatile gps_data_t gps_data;               // Global GPS data
volatile bool gps_data_available;           // Flag for new GPS data available

/* Forward declaration of internal functions */
static void gps_update_led(void);

/*
 *  NMEA Protocol Configuration Commands
 */
static const uint8_t ubx_cfg_nmea[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                                       0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00,
                                       0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};

/*
 * UBX Protocol Configuration Commands
 */
static const uint8_t ubx_cfg_ubx[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                                      0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00,
                                      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79};

/*
 * RTCM Protocol Configuration Commands
 */
static const uint8_t ubx_cfg_rtcm[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                                       0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x20, 0x00,
                                       0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB9, 0x42};

/* UART receive buffer and state */
static char rx_buffer[GPS_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static char sentence_buffer[GPS_MAX_SENTENCE];

/*
 * Initialize GPS UART communication
 */
void gps_init(void) {
    // Initialize GPS data structure
    memset((void*)&gps_data, 0, sizeof(gps_data_t));
    gps_data.current_protocol = (gps_protocol_t)system_config.gps_protocol;

    // Clear buffers
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(sentence_buffer, 0, sizeof(sentence_buffer));
    rx_head = rx_tail = 0;
    gps_data_available = false;

    // Configure UART1 for GPS communication
    // PPS configuration for UART1 RX/TX
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // unlock

    // RB0 (GPS_TX from GPS module) -> UART1 RX input
    U1RXPPS = 0x08; // RB0
    // RB1 (GPS_RX to GPS module) -> UART1 TX output
    RB1PPS = 0x13; // UART1 TX

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // lock

    // Configure UART1 registers
    U1CON0 = 0x00; // Reset UART1
    U1CON1 = 0x00; // Reset UART1
    U1CON2 = 0x00; // Reset UART1

    // Set baud rate based on system configuration
    uint32_t baud_rate = baud_rate_from_index(system_config.gps_baud_index);
    uint32_t baud_div = (_XTAL_FREQ / (4 * baud_rate)) - 1U;

    U1BRGL = (uint8_t)(baud_div & 0xFFU);
    U1BRGH = (uint8_t)((baud_div >> 8) & 0xFFU);

    // Configure UART1 mode
    U1CON0bits.MODE = 0x0; // Asynchronous 8-bit UART mode
    U1CON0bits.RXEN = 1;   // Enable receiver
    U1CON0bits.TXEN = 1;   // Enable transmitter

    // Configure parity and stop bits based on system config
    if (system_config.gps_parity == PARITY_E) {
        U1CON0bits.MODE = 0x1; // 8-bit with even parity
    } else if (system_config.gps_parity == PARITY_O) {
        U1CON0bits.MODE = 0x3; // 8-bit with odd parity
    } else {
        U1CON0bits.MODE = 0x0; // 8-bit no parity
    }

    // Note: Stop bits are typically handled by baud rate generator on this chip

    // Enable UART receive interrupt
    PIR4bits.U1RXIF = 0; // Clear interrupt flag
    IPR4bits.U1RXIP = 0; // Low priority interrupt
    PIE4bits.U1RXIE = 1; // Enable UART1 RX interrupt

    // Enable UART
    U1CON1bits.ON = 1;

    // Initialize GPS LED to off (GPS not locked)
    uint8_t gpioa = 0xFF;
    if (i2cReadRegister(MCP23017_ADDRESS, GPIOA, &gpioa) == I2C_SUCCESS) {
        ioporta.GPS_N = 1; // turn off GPS LED (active low)
        (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOA, gpioa);
    }

    // Small delay for GPS module to stabilize
    __delay_ms(100);

    // Configure GPS module protocol based on system configuration
    gps_set_protocol((gps_protocol_t)system_config.gps_protocol);
}

/*
 * Send a command to the GPS module
 */
void gps_send_command(const char* cmd) {
    const char* ptr = cmd;
    while (*ptr) {
        while (U1FIFObits.TXBF) // Wait for transmit buffer space
            ;                   // Wait for transmit buffer space
        U1TXB = *ptr++;
    }
}

/*
 * Update GPS data (call from main loop)
 */
void gps_update(void) {
    static uint8_t sentence_pos = 0;

    // Process any complete sentences in the receive buffer
    while (1) {
        uint16_t head;
        uint16_t tail;
        char c;
        uint8_t gie_saved;
        uint8_t giel_saved;

        // Take an atomic snapshot of the buffer indices and consume one byte if available
        CRITICAL_SECTION_ENTER(gie_saved, giel_saved);
        head = rx_head;
        tail = rx_tail;
        if (head == tail) {
            CRITICAL_SECTION_EXIT(gie_saved, giel_saved);
            break; // no data
        }
        c = rx_buffer[tail];
        rx_tail = (uint16_t)((tail + 1U) % GPS_BUFFER_SIZE);
        CRITICAL_SECTION_EXIT(gie_saved, giel_saved);

        if (c == '$') {
            // Start of new sentence
            sentence_pos = 0;
            sentence_buffer[sentence_pos++] = c;
        } else if (c == '\n' || c == '\r') {
            // End of sentence
            if (sentence_pos > 6) { // Minimum valid sentence length
                sentence_buffer[sentence_pos] = '\0';
                gps_parse_sentence(sentence_buffer);
            }
            sentence_pos = 0;
        } else if (sentence_pos < (GPS_MAX_SENTENCE - 1)) {
            // Add character to sentence
            sentence_buffer[sentence_pos++] = c;
        } else {
            // Sentence too long/noisy—drop it and wait for next '$'
            sentence_pos = 0;
        }
    }
}

/**
 * Check if GPS has a valid fix
 */
bool gps_has_valid_fix(void) {
    return (gps_data.position.valid == GPS_VALID);
}

/*
 * Check if new GPS data is available
 */
bool gps_has_new_data(void) {
    uint8_t gie_saved;
    uint8_t giel_saved;
    CRITICAL_SECTION_ENTER(gie_saved, giel_saved);
    bool result = gps_data_available;
    gps_data_available = false;
    CRITICAL_SECTION_EXIT(gie_saved, giel_saved);
    return result;
}

/*
 * Get current GPS data
 */
void gps_get_data(gps_data_t* data) {
    // Copy current GPS data (atomic read)
    uint8_t gie_saved;
    uint8_t giel_saved;
    CRITICAL_SECTION_ENTER(gie_saved, giel_saved);
    memcpy(data, (void*)&gps_data, sizeof(gps_data_t));
    CRITICAL_SECTION_EXIT(gie_saved, giel_saved);
}

/*
 * Parse a complete NMEA sentence
 */
void gps_parse_sentence(const char* sentence) {
    // Basic checksum validation
    const char* checksum_pos = strrchr(sentence, '*');
    if (!checksum_pos)
        return;

    // Verify checksum (XOR of chars between '$' and '*')
    if (checksum_pos[1] == '\0' || checksum_pos[2] == '\0') {
        return; // incomplete checksum
    }
    uint8_t calc = 0;
    for (const char* p = sentence + 1; p < checksum_pos; p++) {
        calc ^= (uint8_t)(*p);
    }
    char chkbuf[3];
    chkbuf[0] = checksum_pos[1];
    chkbuf[1] = checksum_pos[2];
    chkbuf[2] = '\0';
    uint8_t expected = (uint8_t)strtoul(chkbuf, NULL, 16);
    if (calc != expected) {
        return; // checksum mismatch
    }

    // Split sentence into fields
    const char* fields[20];
    uint8_t field_count = gps_split_sentence(sentence, fields, 20);

    if (field_count < 2)
        return;

    // Parse based on sentence type
    if (strncmp(fields[0], "$GPRMC", 6) == 0) {
        gps_parse_gprmc(fields, field_count);
    } else if (strncmp(fields[0], "$GPGGA", 6) == 0) {
        gps_parse_gpgga(fields, field_count);
    }
}

/*
 * Split NMEA sentence into fields
 */
uint8_t gps_split_sentence(const char* sentence, const char* fields[], uint8_t max_fields) {
    static char work_buffer[GPS_MAX_SENTENCE];
    strncpy(work_buffer, sentence, GPS_MAX_SENTENCE - 1);
    work_buffer[GPS_MAX_SENTENCE - 1] = '\0';

    uint8_t field_count = 0;
    char* ptr = work_buffer;

    fields[field_count++] = ptr;

    while (*ptr && field_count < max_fields) {
        if (*ptr == ',') {
            *ptr = '\0';
            fields[field_count++] = ptr + 1;
        } else if (*ptr == '*') {
            *ptr = '\0';
            break;
        }
        ptr++;
    }

    return field_count;
}

/*
 * Parse GPRMC sentence (Recommended Minimum Course)
 * $GPRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,var_dir*checksum
 */
void gps_parse_gprmc(const char* fields[], uint8_t field_count) {
    if (field_count < 10)
        return;

    // Check if data is valid
    if (fields[2][0] != 'A') {
        gps_data.datetime.valid = GPS_INVALID;
        return;
    }

    // Parse time (hhmmss.sss)
    if (strlen(fields[1]) >= 6) {
        char temp[3];
        temp[2] = '\0';

        temp[0] = fields[1][0];
        temp[1] = fields[1][1];
        gps_data.datetime.hour = (uint8_t)atoi(temp);

        temp[0] = fields[1][2];
        temp[1] = fields[1][3];
        gps_data.datetime.minute = (uint8_t)atoi(temp);

        temp[0] = fields[1][4];
        temp[1] = fields[1][5];
        gps_data.datetime.second = (uint8_t)atoi(temp);
    }

    // Parse date (ddmmyy)
    if (strlen(fields[9]) >= 6) {
        char temp[3];
        temp[2] = '\0';

        temp[0] = fields[9][0];
        temp[1] = fields[9][1];
        gps_data.datetime.day = (uint8_t)atoi(temp);

        temp[0] = fields[9][2];
        temp[1] = fields[9][3];
        gps_data.datetime.month = (uint8_t)atoi(temp);

        temp[0] = fields[9][4];
        temp[1] = fields[9][5];
        gps_data.datetime.year = (uint8_t)atoi(temp);
    }

    // Parse latitude
    if (strlen(fields[3]) > 0 && strlen(fields[4]) > 0) {
        float lat = (float)atof(fields[3]);
        // Convert from DDMM.MMMM to decimal degrees
        int degrees = (int)(lat / 100);
        float minutes = lat - (degrees * 100);
        gps_data.position.latitude = degrees + (minutes / 60.0f);

        if (fields[4][0] == 'S') {
            gps_data.position.latitude = -gps_data.position.latitude;
        }
    }

    // Parse longitude
    if (strlen(fields[5]) > 0 && strlen(fields[6]) > 0) {
        float lon = (float)atof(fields[5]);
        // Convert from DDDMM.MMMM to decimal degrees
        int degrees = (int)(lon / 100);
        float minutes = lon - (degrees * 100);
        gps_data.position.longitude = degrees + (minutes / 60.0f);

        if (fields[6][0] == 'W') {
            gps_data.position.longitude = -gps_data.position.longitude;
        }
    }

    gps_data.datetime.valid = GPS_VALID;
    gps_data.position.valid = GPS_VALID;
    gps_data_available = true;
}

/*
 * Parse GPGGA sentence (Global Positioning System Fix Data)
 * $GPGGA,time,lat,lat_dir,lon,lon_dir,quality,satellites,hdop,altitude,alt_units,geoid_height,geoid_units,dgps_time,dgps_id*checksum
 */
void gps_parse_gpgga(const char* fields[], uint8_t field_count) {
    if (field_count < 11)
        return;

    // Parse fix quality and satellites
    if (strlen(fields[6]) > 0) {
        uint8_t quality = (uint8_t)atoi(fields[6]);
        if (quality == 0) {
            gps_data.position.fix_type = GPS_NO_FIX;
        } else if (quality == 1 || quality == 2) {
            gps_data.position.fix_type = GPS_2D_FIX; // Assume 2D for simplicity
        } else {
            gps_data.position.fix_type = GPS_3D_FIX;
        }
    }

    if (strlen(fields[7]) > 0) {
        gps_data.position.satellites = (uint8_t)atoi(fields[7]);
    }

    // Parse altitude
    if (strlen(fields[9]) > 0) {
        gps_data.position.altitude = (float)atof(fields[9]);
    }

    // Update validity based on fix quality
    if (gps_data.position.fix_type != GPS_NO_FIX) {
        gps_data.position.valid = GPS_VALID;
        gps_data_available = true;
    }

    // Update GPS LED based on fix status
    gps_update_led();
}

/*
 * Update GPS LED based on fix status
 */
static void gps_update_led(void) {
    static gps_fix_t prev_fix = GPS_NO_FIX;

    if (gps_data.position.fix_type != prev_fix) {
        prev_fix = gps_data.position.fix_type;

        uint8_t gpioa = 0xFF;
        if (i2cReadRegister(MCP23017_ADDRESS, GPIOA, &gpioa) == I2C_SUCCESS) {
            if (gps_data.position.fix_type == GPS_3D_FIX) {
                ioporta.GPS_N = 0;
            } else {
                ioporta.GPS_N = 1;
            }
            (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOA, gpioa);
        }
    }
}

/*
 * Format position for display (Lat: XX.XXX Lon: XX.XXX Alt: XXXM)
 */
void gps_format_position(char* buffer, size_t len, const gps_position_t* pos) {
    if (buffer == NULL || len == 0) {
        return;
    }

    if (pos->valid == GPS_VALID) {
        (void)snprintf(buffer, len, "%.3f,%.3f,%.0fm", pos->latitude, pos->longitude, pos->altitude);
    } else {
        (void)snprintf(buffer, len, "No GPS Fix");
    }
}

/**
 * Format date and time for display (DD-MM-YY HH:MM+HH:MM)
 */
void gps_format_date_time(char* buffer, size_t len, const gps_datetime_t* dt) {

    if (dt == NULL || buffer == NULL || len == 0U) {
        return;
    }

    if (dt->valid == GPS_VALID) {
        char timebuf[8];
        char tzbuf[8];
        gps_datetime_t outdt;
        const gps_datetime_t* display_dt = dt;
        if (system_config.tz_mode == 1) {
            date_apply_offset(dt, &outdt, system_config.tz_offset_min);
            display_dt = &outdt;
            tz_offset_to_string(system_config.tz_offset_min, tzbuf);
        } else {
            strcpy(tzbuf, "UTC");
        }
        date_format_time_short(timebuf, display_dt);
        /* Format: "DD-MM-YY HH:MM+HH:MM" (20 chars max) */
        snprintf(buffer, len, "%02d-%02d-%02d %s%s", display_dt->day, display_dt->month, display_dt->year, timebuf, tzbuf);
    } else {
        (void)snprintf(buffer, len, "No GPS Fix");
    }
}

/*
 * Set GPS protocol (NMEA, UBX, or RTCM)
 */
void gps_set_protocol(gps_protocol_t protocol) {
    const uint8_t* cmd = NULL;
    uint8_t cmd_size = 0;

    switch (protocol) {
        case GPS_PROTOCOL_NMEA:
            cmd = ubx_cfg_nmea;
            cmd_size = sizeof(ubx_cfg_nmea);
            break;

        case GPS_PROTOCOL_UBX:
            cmd = ubx_cfg_ubx;
            cmd_size = sizeof(ubx_cfg_ubx);
            break;

        case GPS_PROTOCOL_RTCM:
            cmd = ubx_cfg_rtcm;
            cmd_size = sizeof(ubx_cfg_rtcm);
            break;

        default:
            return; // Invalid protocol
    }

    // Send configuration command via UART
    for (uint8_t i = 0; i < cmd_size; i++) {
        while (U1FIFObits.TXBE == 0)
            ; // Wait for transmit buffer to be empty
        U1TXB = cmd[i];
    }

    // Update current protocol in GPS data
    gps_data.current_protocol = protocol;
}

/*
 * Put character into GPS receive buffer (called from ISR)
 */
void gps_buffer_put_char(char c) {
    uint16_t next_head = (rx_head + 1) % GPS_BUFFER_SIZE;

    // Check for buffer overflow
    if (next_head != rx_tail) {
        rx_buffer[rx_head] = c;
        rx_head = next_head;
    }
    // If buffer is full, character is discarded
}
