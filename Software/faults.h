/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * This module provides fault management functionality for the GPSDO project.
 * It maintains a circular buffer of the last 5 fault messages, allowing
 * the user to view recent system faults. New faults push older faults down
 * the stack, with the oldest fault being discarded when the buffer is full.
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
 *
 */
#ifndef FAULTS_H
#define FAULTS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Maximum length of a fault message string (including null terminator)
 */
#define FAULT_MSG_MAX_LEN 40

/*
 * Maximum number of faults to store in the fault stack
 */
#define FAULT_STACK_SIZE 5

/*
 * Defaine fault messages. 
 */
//                                                        1         2         3         4
//                                               ....+....0....+....0....+....0....+....0
#define FAULT_MSG_GPS_SIGNAL_LOST               "GPS signal lost"
#define FAULT_MSG_GPS_DATA_INVALID              "Invalid GPS data"
#define FAULT_MSG_GPS_UNKNOWN_PROTOCOL          "Unknown GPS protocol"
#define FAULT_MSG_GPS_RTCM_CRC_MISMATCH         "GPS RTCM CRC mismatch"
#define FAULT_MSG_GPS_RTCM_SENTENCE_TOO_SHORT   "GPS RTCM sentence too short"
#define FAULT_MSG_GPS_UBX_CRC_MISMATCH          "GPS UBX CRC mismatch"
#define FAULT_MSG_GPS_UBX_SENTENCE_TOO_SHORT    "GPS UBX sentence too short"
#define FAULT_MSG_GPS_NMEA_CHECKSUM_MISMATCH    "GPS NMEA checksum mismatch"
#define FAULT_MSG_GPS_NMEA_SENTENCE_TOO_SHORT   "GPS NMEA sentence too short"
#define FAULT_MSG_GPS_INVALID_DATE_TIME         "GPS invalid date/time"
#define FAULT_MSG_GPS_INVALID_POSITION          "GPS invalid position"
#define FAULT_MSG_GPS_BUFFER_OVERFLOW           "GPS buffer overflow"
#define FAULT_MSG_OCXO_UNLOCKED                 "OCXO unlocked"
#define FAULT_MSG_I2C_COMM_ERROR                "I2C communication error"

/*
 * Fault severity levels
 */
typedef enum {
    FAULT_SEVERITY_INFO = 0,
    FAULT_SEVERITY_WARNING = 1,
    FAULT_SEVERITY_ERROR = 2,
    FAULT_SEVERITY_CRITICAL = 3
} FaultSeverity_t;

/*
 * Structure to hold a single fault record
 */
typedef struct {
    char message[FAULT_MSG_MAX_LEN];    // Fault message text
    FaultSeverity_t severity;            // Severity level of the fault
    bool valid;                          // True if this fault entry is valid
} FaultRecord_t;

/*
 * Initialize the fault management system
 * This function should be called once at system startup
 */
void faultsInit(void);

/*
 * Add a new fault to the fault stack
 * If the stack is full, the oldest fault is discarded
 * 
 * Parameters:
 *   message - Fault message string (will be truncated if longer than FAULT_MSG_MAX_LEN-1)
 *   severity - Severity level of the fault
 */
void faultsAdd(const char *message, FaultSeverity_t severity);

/*
 * Get the number of valid faults currently in the stack
 * 
 * Returns:
 *   Number of valid faults (0 to FAULT_STACK_SIZE)
 */
uint8_t faultsGetCount(void);

/*
 * Retrieve a fault record by index
 * Index 0 is the most recent fault, index (count-1) is the oldest
 * 
 * Parameters:
 *   index - Index of the fault to retrieve (0 = most recent)
 *   fault - Pointer to FaultRecord_t structure to receive the fault data
 * 
 * Returns:
 *   true if the fault was successfully retrieved, false if index is invalid
 */
bool faultsGet(uint8_t index, FaultRecord_t *fault);

/*
 * Clear all faults from the stack
 */
void faultsClear(void);

/*
 * Get a pointer to a fault record by index (for direct read access)
 * Index 0 is the most recent fault, index (count-1) is the oldest
 * 
 * Parameters:
 *   index - Index of the fault to retrieve (0 = most recent)
 * 
 * Returns:
 *   Pointer to the fault record, or NULL if index is invalid
 */
const FaultRecord_t* faultsGetPtr(uint8_t index);

/*
 * Update the fault LED based on the current fault stack status
 * Turns the LED on if there are any faults, off if the stack is empty
 */
void faultsUpdateLED(void);

#endif // FAULTS_H
