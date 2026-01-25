/*
 * Copyright (c) 2026, Dewayne L. Hafenstein.  All rights reserved.
 *
 * GPS module for reading and parsing data from ublox M8M GPS receiver.
 * This module handles UART communication and NMEA message parsing to
 * extract date/time, position, and other relevant GPS data.
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

#ifndef GPS_H
#define GPS_H

#include "config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <xc.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_BUFFER_SIZE 1024
#define GPS_MAX_SENTENCE 120

/* Complete GPS data structure */
typedef struct {
    gps_datetime_t datetime;
    gps_position_t position;
    bool pps_valid;                  // 1PPS signal is valid
    gps_protocol_t current_protocol; // Current active protocol
} gps_data_t;

/* Function prototypes */
// Initialize GPS UART communication
void gps_init(void);
// determines if a valid GPS fix is available
bool gps_has_valid_fix(void);
// Update GPS data for positioning information
void gps_format_position(char* buffer, int len, const gps_position_t* pos);
// Update GPS data for date/time information
void gps_format_date_time(char* buffer, int len, const gps_datetime_t* dt);
// Set GPS protocol (NMEA, UBX, or RTCM)
void gps_set_protocol(gps_protocol_t protocol);
// Parse a complete NMEA, UBX, or RTCM sentence
void gps_parse_sentence(void);

/* Internal functions (exposed for testing) */
// Parses NMEA GPRMC sentence
void gps_parse_gprmc(const char* fields[], unsigned char field_count);
// Parses NMEA GPGGA sentence
void gps_parse_gpgga(const char* fields[], unsigned char field_count);
//  Parses UBX message
void gps_parse_ubx_message(const unsigned char* data, unsigned int length);
//  Parses RTCM message
void gps_parse_rtcm_message(const unsigned char* data, unsigned int length);
// Splits a NMEA sentence into fields
unsigned char gps_split_sentence(const char* sentence, const char* fields[], unsigned char max_fields);
/* GPS buffer access functions for ISR */
void gps_buffer_put_char(char c);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */