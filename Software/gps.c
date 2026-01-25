/*
 * Copyright (c) 2026, Dewayne L. Hafenstein.  All rights reserved.
 *
 * GPS module implementation for ublox M8M GPS receiver.
 * Handles UART communication and NMEA, UBX, and RTCM message parsing.
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
#include "date.h"
#include "faults.h"
#include "i2c.h"
#include "led.h"
#include "mcp23x17.h"
#include "menu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

/*
 * global data areas
 */
extern system_config_t system_config;         // System configuration data
volatile gps_data_t gps_data;                 // Global GPS data
volatile bool gps_data_available;             // Flag for new GPS data available
volatile bool gps_sentence_available;         // Flag for new GPS sentence available
volatile char gps_sentence[GPS_MAX_SENTENCE]; // Current GPS sentence buffer
static volatile uint16_t gps_sentence_length; // Length of current GPS sentence

/* Forward declaration of internal functions */
static void gps_update_led(void);
static void reset_buffer(void);
static void snapshot_sentence(void);

/*
 *  Protocol Configuration Commands to change the M8M UBlox chip to NMEA protocol.
 */
static const unsigned char ubx_cfg_nmea[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                                             0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00,
                                             0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};

/*
 * Protocol Configuration Commands to change the M8M UBlox chip to UBX protocol.
 */
static const unsigned char ubx_cfg_ubx[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                                            0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00,
                                            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79};

/*
 * Protocol Configuration Commands to change the M8M UBlox chip to RTCM protocol.
 */
static const unsigned char ubx_cfg_rtcm[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                                             0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x20, 0x00,
                                             0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB9, 0x42};

/* UART receive buffer and state */
static char gps_rx_buffer[GPS_BUFFER_SIZE];
static volatile unsigned int gps_rx_head = 0;
static volatile unsigned int gps_rx_tail = 0;
static volatile bool gps_sentence_started = false; // True when we have seen start of sentence

/*
 * Initialize GPS UART communication
 */
void gps_init(void) {
    // Initialize GPS data structure
    memset((void*)&gps_data, 0, sizeof(gps_data_t));
    gps_data.current_protocol = (gps_protocol_t)system_config.gps_protocol;

    // Clear buffers
    memset(gps_rx_buffer, 0, sizeof(gps_rx_buffer));
    memset((void*)gps_sentence, 0, sizeof(gps_sentence));
    gps_rx_head = gps_rx_tail = 0;
    gps_data_available = false;
    gps_sentence_available = false;
    gps_sentence_started = false;
    gps_sentence_length = 0;

    // Configure UART2 for GPS communication
    // PPS configuration for UART2 RX/TX
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // unlock

    // RB4 (GPS_TX from GPS module) -> UART2 RX input
    U2RXPPS = 0x0C; // RB4
    // RB5 (GPS_RX to GPS module) -> UART2 TX output
    RB5PPS = 0x23; // UART2 TX

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // lock

    // Configure UART2 registers
    U2CON0 = 0x00; // Reset UART2
    U2CON1 = 0x00; // Reset UART2
    U2CON2 = 0x00; // Reset UART2

    // Set baud rate based on system configuration
    long baud_rate = system_config.gps_baud;
    long baud_div = _XTAL_FREQ / (16L * (baud_rate + 1L));

    U2BRGL = (unsigned char)(baud_div & 0xFFU);
    U2BRGH = (unsigned char)((baud_div >> 8) & 0xFFU);

    // Configure parity and stop bits based on system config
    switch (system_config.gps_parity) {
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

    switch (system_config.gps_stop_bits) {
        case STOPBITS_1:
            U2CON2bits.STP = 0; // 1 stop bit
            break;
        case STOPBITS_1_5:
            U2CON2bits.STP = 1; // 1.5 stop bits
            break;
        case STOPBITS_2:
            U2CON2bits.STP = 2; // 2 stop bits
            break;
        default:
            U2CON2bits.STP = 0; // 1 stop bit
            break;
    }

    // Enable UART receive interrupt
    PIR8bits.U2RXIF = 0; // Clear interrupt flag
    IPR8bits.U2RXIP = 0; // Low priority interrupt
    PIE8bits.U2RXIE = 1; // Enable UART2 RX interrupt

    // Enable UART
    U2CON1bits.ON = 1;

    // Initialize GPS LED to off (GPS not locked)
    gpsLED_Off();

    // Small delay for GPS module to stabilize
    __delay_ms(100);

    // Configure GPS module protocol based on system configuration
    gps_set_protocol((gps_protocol_t)system_config.gps_protocol);
}

/**
 * Check if GPS has a valid fix
 */
bool gps_has_valid_fix(void) {
    return (gps_data.position.valid == GPS_VALID);
}

/*
 * Parse a complete NMEA, UBX, or RTCM sentence
 *
 * This function handles all three GPS protocols:
 * - NMEA: Text-based sentences starting with '$', validated with XOR checksum
 * - UBX: Binary messages starting with 0xB5 0x62, validated with Fletcher-8 checksum
 * - RTCM: Binary correction messages starting with 0xD3, validated with CRC-24Q
 *
 * The protocol is auto-detected based on the first byte of the message.
 */
void gps_parse_sentence() {
    // Determine protocol based on first byte
    unsigned char first_byte = (unsigned char)gps_sentence[0];

    if (first_byte == '$') {
        // NMEA message - validate checksum
        const char* checksum_pos = strrchr((void*)gps_sentence, '*');
        if (!checksum_pos)
            return;

        // Verify checksum (XOR of chars between '$' and '*')
        if (checksum_pos[1] == '\0' || checksum_pos[2] == '\0') {
            return; // incomplete checksum
        }
        unsigned char calc = 0;
        for (char* p = (char*)(gps_sentence + 1); p < checksum_pos; p++) {
            calc ^= (unsigned char)(*p);
        }
        char chkbuf[3];
        chkbuf[0] = checksum_pos[1];
        chkbuf[1] = checksum_pos[2];
        chkbuf[2] = '\0';
        unsigned char expected = (unsigned char)strtoul(chkbuf, NULL, 16);
        if (calc != expected) {
            faultsAdd(FAULT_MSG_GPS_NMEA_CHECKSUM_MISMATCH, FAULT_SEVERITY_ERROR);
            return; // checksum mismatch
        }

        // Split sentence into fields
        char* fields[20];
        unsigned char field_count = gps_split_sentence((const char*)gps_sentence, (const char**)fields, 20);

        if (field_count < 2) {
            faultsAdd(FAULT_MSG_GPS_NMEA_SENTENCE_TOO_SHORT, FAULT_SEVERITY_ERROR);
            return;
        }

        // Parse based on sentence type
        if (strncmp(fields[0], "$GPRMC", 6) == 0) {
            gps_parse_gprmc((const char**)fields, field_count);
        } else if (strncmp(fields[0], "$GPGGA", 6) == 0) {
            gps_parse_gpgga((const char**)fields, field_count);
        }
    } else if (first_byte == 0xB5) {
        // UBX message - validate sync chars and checksum
        if (gps_sentence_length < 8) {
            faultsAdd(FAULT_MSG_GPS_UBX_SENTENCE_TOO_SHORT, FAULT_SEVERITY_ERROR);
            return; // Too short for UBX
        }

        if ((uint8_t)gps_sentence[1] != 0x62)
            return; // Invalid sync char

        // Calculate checksum (Fletcher-8 algorithm)
        unsigned char ck_a = 0;
        unsigned char ck_b = 0;
        for (unsigned int i = 2; i < gps_sentence_length - 2; i++) {
            ck_a = (unsigned char)(ck_a + (unsigned char)gps_sentence[i]);
            ck_b = (unsigned char)(ck_b + ck_a);
        }

        if (ck_a != (unsigned char)gps_sentence[gps_sentence_length - 2] ||
            ck_b != (unsigned char)gps_sentence[gps_sentence_length - 1]) {
            faultsAdd(FAULT_MSG_GPS_UBX_CRC_MISMATCH, FAULT_SEVERITY_ERROR);
            return; // Checksum mismatch
        }

        // Parse UBX message
        gps_parse_ubx_message((const unsigned char*)gps_sentence, gps_sentence_length);
    } else if (first_byte == 0xD3) {
        // RTCM message - validate and parse
        if (gps_sentence_length < 6) {
            faultsAdd(FAULT_MSG_GPS_RTCM_SENTENCE_TOO_SHORT, FAULT_SEVERITY_ERROR);
            return; // Too short for RTCM
        }

        // Calculate CRC-24Q checksum
        unsigned long crc = 0;
        for (unsigned int i = 0; i < gps_sentence_length - 3; i++) {
            crc = ((crc << 8) & 0xFFFFFF) ^ ((unsigned long)(unsigned char)gps_sentence[i] << 16);
            for (unsigned char j = 0; j < 8; j++) {
                crc = (crc << 1) ^ ((crc & 0x800000) ? 0x1864CFB : 0);
            }
        }
        crc &= 0xFFFFFF;

        unsigned long expected_crc = ((unsigned long)(unsigned char)gps_sentence[gps_sentence_length - 3] << 16) |
                                     ((unsigned long)(unsigned char)gps_sentence[gps_sentence_length - 2] << 8) |
                                     ((unsigned long)(unsigned char)gps_sentence[gps_sentence_length - 1]);

        if (crc != expected_crc) {
            faultsAdd(FAULT_MSG_GPS_RTCM_CRC_MISMATCH, FAULT_SEVERITY_ERROR);
            return; // CRC mismatch
        }

        // Parse RTCM message
        gps_parse_rtcm_message((const unsigned char*)gps_sentence, gps_sentence_length);
    } else {
        faultsAdd(FAULT_MSG_GPS_UNKNOWN_PROTOCOL, FAULT_SEVERITY_WARNING);
        return; // Unknown protocol
    }
}

/*
 * Split NMEA sentence into fields
 */
unsigned char gps_split_sentence(const char* sentence, const char* fields[], unsigned char max_fields) {
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
void gps_parse_gprmc(const char* fields[], unsigned char field_count) {
    if (field_count < 10)
        return;

    // Check if data is valid
    if (fields[2][0] != 'A') {
        gps_data.datetime.valid = GPS_INVALID;
        faultsAdd(FAULT_MSG_GPS_INVALID_DATE_TIME, FAULT_SEVERITY_WARNING);
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
void gps_parse_gpgga(const char* fields[], unsigned char field_count) {
    if (field_count < 11)
        return;

    // Parse fix quality and satellites
    if (strlen(fields[6]) > 0) {
        unsigned char quality = (unsigned char)atoi(fields[6]);
        if (quality == 0) {
            gps_data.position.fix_type = GPS_NO_FIX;
        } else if (quality == 1 || quality == 2) {
            gps_data.position.fix_type = GPS_2D_FIX; // Assume 2D for simplicity
        } else {
            gps_data.position.fix_type = GPS_3D_FIX;
        }
    }

    if (strlen(fields[7]) > 0) {
        gps_data.position.satellites = (unsigned char)atoi(fields[7]);
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
        if (gps_data.position.fix_type == GPS_3D_FIX) {
            gpsLED_On();
        } else {
            gpsLED_Off();
        }
    }
}

/*
 * Format position for display (Lat: XX.XXX Lon: XX.XXX Alt: XXXM)
 */
void gps_format_position(char* buffer, int len, const gps_position_t* pos) {
    if (buffer == NULL || len == 0) {
        faultsAdd(FAULT_MSG_GPS_INVALID_POSITION, FAULT_SEVERITY_WARNING);
        return;
    }

    if (pos->valid == GPS_VALID) {
        (void)snprintf(buffer, (size_t)len, "%.3f,%.3f,%.0fm", pos->latitude, pos->longitude, pos->altitude);
    } else {
        (void)snprintf(buffer, (size_t)len, "No GPS Fix");
    }
}

/**
 * Format date and time for display (DD-MM-YY HH:MM+HH:MM)
 */
void gps_format_date_time(char* buffer, int len, const gps_datetime_t* dt) {

    if (dt == NULL || buffer == NULL || len == 0U) {
        faultsAdd(FAULT_MSG_GPS_INVALID_DATE_TIME, FAULT_SEVERITY_WARNING);
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
        snprintf(buffer, (size_t)len, "%02d-%02d-%02d %s%s", display_dt->day, display_dt->month, display_dt->year,
                 timebuf, tzbuf);
    } else {
        (void)snprintf(buffer, (size_t)len, "No GPS Fix");
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
            faultsAdd(FAULT_MSG_GPS_UNKNOWN_PROTOCOL, FAULT_SEVERITY_ERROR);
            return; // Invalid protocol
    }

    // Send configuration command via UART
    for (unsigned char i = 0; i < cmd_size; i++) {
        while (U1FIFObits.TXBE == 0)
            ; // Wait for transmit buffer to be empty
        U1TXB = cmd[i];
    }

    // Update current protocol in GPS data
    gps_data.current_protocol = protocol;
}

/*
 * Put character into GPS receive buffer (called from ISR).
 *
 * This function handles different protocols and manages buffer overflow.
 * It supports NMEA, UBX, and RTCM protocols, which have different start
 * and end sequences.  Since the routine only "sees" one character at a
 * time, it has to accumulate enough data to determine when a complete
 * message has been received.  Once it recognizes a complete message, it
 * copies it to a separate buffer for processing and sets a flag that a
 * gps sentence is available.
 *
 * Processing of the GPS sentence is done in the main loop to avoid doing
 * complex parsing in the ISR context.  If the receive buffer overflows, the
 * buffer is reset and the routine enters a resync mode where it looks for
 * the start of the next valid message before resuming normal operation.
 *
 * An NMEA message starts with '$' and ends with CR/LF.
 * A UBX message starts with 0xB5 0x62 and has a length field.
 * An RTCM message starts with 0xD3 and has a length field.
 *
 * This code uses a circular buffer to store incoming characters.  The
 * start of any message detected is indicated by the rx_head position. The
 * rx_tail position indicates where the current end of the accumulated data
 * is located.  All new characters are added to the buffer at rx_tail and
 * rx_tail is incremented (or wrapped) until a stop sequence has been found.
 *
 * When a complete message is detected, it is copied from the circular buffer
 * (starting at rx_head up to but excluding rx_tail) to the gps_sentence
 * buffer for processing.  rx_head is then advanced to rx_tail to prepare
 * for the next message.
 */
void gps_buffer_put_char(char c) {
    unsigned int next_tail = (unsigned int)((gps_rx_tail + 1U) % GPS_BUFFER_SIZE);

    /*
     * If we have not found a start of a sentence yet, keep hunting until
     * we do moving head and tail in the buffer as we examine each
     * character.  If we find the start of a sentence, set the flag and
     * stop moving the head pointer so we can start accumulating the
     * sentence.
     */
    gps_rx_buffer[gps_rx_tail] = c; // Save the character at current tail position
    // We'll advance the tail after we figure out what we have
    if (!gps_sentence_started) {
        if (c == '$' || c == 0xB5 || c == 0xD3) {
            // Found start of sentence
            gps_sentence_started = true;
            gps_rx_head = gps_rx_tail; // Set the head to the start of the sentence
        }
    } else {
        /*
         * Now, make sure we havent wrapped around and overrun the head
         * pointer.  If we have, we have a buffer overflow, so reset
         * the buffer and start looking for the start of the next sentence.
         */
        if (next_tail == gps_rx_head) {
            // Buffer overflow
            reset_buffer();
            faultsAdd(FAULT_MSG_GPS_BUFFER_OVERFLOW, FAULT_SEVERITY_ERROR);
            return;
        }

        /*
         * Now look for an end-of-sentence based on protocol.  For NMEA this
         * is CR/LF, for UBX it is based on length field, and for RTCM it
         * is also based on length field.  So, for RTCM and UBX we have to
         * accumulate enough characters to determine the length of the
         * sentence, then wait until we have received that many characters.
         */
        int length = (int)gps_rx_tail - (int)gps_rx_head;
        if (length < 0) {
            length += GPS_BUFFER_SIZE; // It wrapped
        }
        /*
         * Check to see if we have accumulated enough data for the UBX or
         * RTCM message to have received it's length field.
         * For UBX, this is 6 bytes (sync chars + class + id + length low + length high)
         * For RTCM, this is 3 bytes (preamble + length high + length low)
         */
        switch (length) {
            case 3: // Possible RTCM length field
                if (gps_data.current_protocol == GPS_PROTOCOL_RTCM) {
                    // Extract length from bytes 1 and 2
                    unsigned int rtcm_length =
                        ((unsigned int)(unsigned char)gps_rx_buffer[(gps_rx_head + 1U) % GPS_BUFFER_SIZE] & 0x03U) << 8;
                    rtcm_length |= (unsigned char)gps_rx_buffer[(gps_rx_head + 2U) % GPS_BUFFER_SIZE];
                    // Total RTCM message length = header + length + CRC
                    gps_sentence_length = 6 + rtcm_length;
                }
                break;
            case 6: // Possible UBX length field
                if (gps_data.current_protocol == GPS_PROTOCOL_UBX) {
                    // Extract length from bytes 4 and 5
                    unsigned int ubx_length = (unsigned char)gps_rx_buffer[(gps_rx_head + 4U) % GPS_BUFFER_SIZE];
                    ubx_length |= ((unsigned int)(unsigned char)gps_rx_buffer[(gps_rx_head + 5U) % GPS_BUFFER_SIZE])
                                  << 8;
                    // Total UBX message length = header + length + checksum
                    gps_sentence_length = 8 + ubx_length;
                }
                break;
        }

        // Check for end of sentence based on protocol
        bool sentence_complete = false;
        if (gps_data.current_protocol == GPS_PROTOCOL_NMEA) {
            // NMEA ends with CR/LF
            if (c == '\n' && length >= 2 &&
                gps_rx_buffer[(gps_rx_tail - 1U + GPS_BUFFER_SIZE) % GPS_BUFFER_SIZE] == '\r') {
                sentence_complete = true;
                gps_sentence_length = (unsigned int)(length + 1);
            }
        } else if (gps_data.current_protocol == GPS_PROTOCOL_UBX || gps_data.current_protocol == GPS_PROTOCOL_RTCM) {
            if (gps_sentence_length > 0 && length + 1 >= gps_sentence_length) {
                sentence_complete = true;
            }
        }

        if (sentence_complete) {
            // Snapshot the complete sentence for processing
            snapshot_sentence();
            reset_buffer();
            return;
        }
    }

    // always advance the tail pointer
    gps_rx_tail = next_tail; // Advance tail pointer
}

/*
 * We need to reset the buffer and start over.
 */
static void reset_buffer() {
    gps_rx_head = gps_rx_tail = 0;
    gps_sentence_started = false;
    gps_sentence_length = 0;
}

/**
 * Snapshot the current sentence from the circular buffer to the
 * gps_sentence buffer for processing in the main loop.
 */
static void snapshot_sentence(void) {
    // Copy from circular buffer to gps_sentence
    unsigned int index = gps_rx_head;
    unsigned int count = 0;
    unsigned char gieh_save = 0;
    unsigned char giel_save = 0;

    CRITICAL_SECTION_ENTER(gieh_save, giel_save);
    while (index != gps_rx_tail && count < GPS_MAX_SENTENCE - 1) {
        gps_sentence[count++] = gps_rx_buffer[index];
        index = (index + 1) % GPS_BUFFER_SIZE;
    }
    gps_sentence[count] = '\0'; // Null-terminate
    gps_sentence_length = count;

    // Advance head to tail for next message
    gps_rx_head = gps_rx_tail;
    gps_sentence_available = true;
    CRITICAL_SECTION_EXIT(gieh_save, giel_save);
}

/*
 * Parse UBX message
 * UBX message format:
 * 0xB5 0x62 (sync chars) | Class | ID | Length (2 bytes) | Payload | CK_A | CK_B
 */
void gps_parse_ubx_message(const unsigned char* data, unsigned int length) {
    if (data == NULL || length < 8)
        return;

    unsigned char msg_class = data[2];
    unsigned char msg_id = data[3];
    unsigned int payload_length = data[4] | ((unsigned int)data[5] << 8);

    // Verify payload length matches message length
    if (length != 8 + payload_length)
        return;

    const unsigned char* payload = &data[6];

    // Parse based on message class and ID
    if (msg_class == 0x01) {  // NAV (Navigation) class
        if (msg_id == 0x07) { // UBX-NAV-PVT (Position Velocity Time)
            // This is a comprehensive message with time, position, and velocity
            if (payload_length >= 92) {
                // Extract time (bytes 4-9: year, month, day, hour, min, sec)
                unsigned int year = payload[4] | ((unsigned int)payload[5] << 8);
                gps_data.datetime.year = (unsigned char)(year - 2000);
                gps_data.datetime.month = payload[6];
                gps_data.datetime.day = payload[7];
                gps_data.datetime.hour = payload[8];
                gps_data.datetime.minute = payload[9];
                gps_data.datetime.second = payload[10];

                // Extract validity flags (byte 11)
                unsigned char valid = payload[11];
                gps_data.datetime.valid = (valid & 0x04) ? GPS_VALID : GPS_INVALID;

                // Extract fix type (byte 20)
                unsigned char fix_type = payload[20];
                if (fix_type == 0x00) {
                    gps_data.position.fix_type = GPS_NO_FIX;
                } else if (fix_type == 0x02) {
                    gps_data.position.fix_type = GPS_2D_FIX;
                } else if (fix_type == 0x03) {
                    gps_data.position.fix_type = GPS_3D_FIX;
                } else {
                    gps_data.position.fix_type = GPS_NO_FIX;
                }

                // Extract number of satellites (byte 23)
                gps_data.position.satellites = payload[23];

                // Extract longitude (bytes 24-27, scaled 1e-7 degrees)
                long lon = (long)(payload[24] | ((unsigned long)payload[25] << 8) | ((unsigned long)payload[26] << 16) |
                                  ((unsigned long)payload[27] << 24));
                gps_data.position.longitude = (float)lon * 1e-7f;

                // Extract latitude (bytes 28-31, scaled 1e-7 degrees)
                long lat = (long)(payload[28] | ((unsigned long)payload[29] << 8) | ((unsigned long)payload[30] << 16) |
                                  ((unsigned long)payload[31] << 24));
                gps_data.position.latitude = (float)lat * 1e-7f;

                // Extract altitude MSL (bytes 36-39, millimeters)
                long alt = (long)(payload[36] | ((unsigned long)payload[37] << 8) | ((unsigned long)payload[38] << 16) |
                                  ((unsigned long)payload[39] << 24));
                gps_data.position.altitude = (float)alt * 0.001f;

                // Update position validity based on fix type
                gps_data.position.valid = (gps_data.position.fix_type != GPS_NO_FIX) ? GPS_VALID : GPS_INVALID;

                gps_data_available = true;
                gps_update_led();
            }
        } else if (msg_id == 0x21) { // UBX-NAV-TIMEUTC (UTC Time Solution)
            if (payload_length >= 20) {
                // Extract time validity (byte 19)
                unsigned char valid = payload[19];
                if (valid & 0x04) { // UTC time is valid
                    unsigned int year = payload[12] | ((unsigned int)payload[13] << 8);
                    gps_data.datetime.year = (unsigned char)(year - 2000);
                    gps_data.datetime.month = payload[14];
                    gps_data.datetime.day = payload[15];
                    gps_data.datetime.hour = payload[16];
                    gps_data.datetime.minute = payload[17];
                    gps_data.datetime.second = payload[18];
                    gps_data.datetime.valid = GPS_VALID;
                    gps_data_available = true;
                }
            }
        }
    }
}

/*
 * Parse RTCM message
 * RTCM message format:
 * 0xD3 (preamble) | Reserved + Length (2 bytes) | Message Type + Payload | CRC (3 bytes)
 */
void gps_parse_rtcm_message(const unsigned char* data, unsigned int length) {
    if (data == NULL || length < 6)
        return;

    // Extract message length (10 bits from bytes 1-2)
    unsigned int msg_length = (((unsigned int)data[1] & 0x03) << 8) | data[2];

    // Verify message length matches
    if (length != 6 + msg_length)
        return;

    // Extract message type (12 bits from bytes 3-4)
    unsigned int msg_type = ((unsigned int)data[3] << 4) | ((data[4] >> 4) & 0x0F);

    // RTCM messages are typically differential corrections and don't contain
    // absolute position/time information. For a GPSDO application, RTCM is
    // mainly useful for improving position accuracy when used with a base station.
    // We'll implement basic parsing for common RTCM message types.

    const unsigned char* payload = &data[3];

    switch (msg_type) {
        case 1005: // Stationary RTK reference station ARP
            // This contains reference station position
            // For GPSDO, we typically wouldn't use this directly
            break;

        case 1077: // GPS MSM7 (Multi-Signal Message)
        case 1087: // GLONASS MSM7
        case 1097: // Galileo MSM7
        case 1127: // BeiDou MSM7
            // These are high-precision observations
            // For GPSDO, these improve timing accuracy but don't provide
            // direct position/time data to parse
            break;

        case 1230: // GLONASS code-phase biases
            // Used for high-precision positioning
            break;

        default:
            // Unknown or unsupported RTCM message type
            // For a GPSDO application, most RTCM messages are correction data
            // that the GPS receiver uses internally to improve accuracy
            break;
    }

    // Note: RTCM messages don't typically contain absolute time/position data
    // They are differential corrections applied by the GPS receiver.
    // The receiver will output corrected positions via NMEA or UBX messages.
}
