/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
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

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

/* GPS protocol options */
typedef enum
{
    GPS_PROTOCOL_NMEA = 0,
    GPS_PROTOCOL_UBX = 1,
    GPS_PROTOCOL_RTCM = 2
} gps_protocol_t;

#ifdef __cplusplus
extern "C"
{
#endif

/* GPS UART configuration */
#define GPS_BUFFER_SIZE 256
#define GPS_MAX_SENTENCE 120

    /* GPS data validity states */
    typedef enum
    {
        GPS_INVALID = 0,
        GPS_VALID = 1
    } gps_validity_t;

    /* GPS fix types */
    typedef enum
    {
        GPS_NO_FIX = 0,
        GPS_2D_FIX = 1,
        GPS_3D_FIX = 2
    } gps_fix_t;

    /* GPS date/time structure */
    typedef struct
    {
        uint8_t year;   // Years since 2000 (e.g., 25 for 2025)
        uint8_t month;  // Month 1-12
        uint8_t day;    // Day 1-31
        uint8_t hour;   // Hour 0-23 (UTC)
        uint8_t minute; // Minute 0-59
        uint8_t second; // Second 0-59
        gps_validity_t valid;
    } gps_datetime_t;

    /* GPS position structure */
    typedef struct
    {
        float latitude;  // Decimal degrees, positive = North
        float longitude; // Decimal degrees, positive = East
        float altitude;  // Meters above sea level
        gps_fix_t fix_type;
        uint8_t satellites; // Number of satellites in view
        gps_validity_t valid;
    } gps_position_t;

    /* Complete GPS data structure */
    typedef struct
    {
        gps_datetime_t datetime;
        gps_position_t position;
        bool pps_valid;                  // 1PPS signal is valid
        gps_protocol_t current_protocol; // Current active protocol
    } gps_data_t;

    /* Function prototypes */
    void gps_init(void);
    void gps_update(void);
    bool gps_has_new_data(void);
    void gps_get_data(gps_data_t *data);
    void gps_format_position(char *buffer, const gps_position_t *pos);
    void gps_format_date_time(char *buffer, const gps_datetime_t *dt);
    void gps_set_protocol(gps_protocol_t protocol);

    /* Internal functions (exposed for testing) */
    void gps_parse_sentence(const char *sentence);
    void gps_parse_gprmc(const char *fields[], uint8_t field_count);
    void gps_parse_gpgga(const char *fields[], uint8_t field_count);
    void gps_parse_ubx_message(const uint8_t *data, uint16_t length);
    void gps_parse_rtcm_message(const uint8_t *data, uint16_t length);
    uint8_t gps_split_sentence(const char *sentence, const char *fields[], uint8_t max_fields);

    /* GPS buffer access functions for ISR */
    void gps_buffer_put_char(char c);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */