/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Implementation of fault management functionality for the GPSDO project.
 * This module maintains a circular buffer of the last 5 fault messages,
 * allowing the user to view recent system faults.
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
#include "faults.h"
#include "led.h"
#include <string.h>

/*
 * Circular buffer to store fault records
 * The buffer is organized as a circular queue where the newest fault
 * is added at the head position and wraps around when it reaches the end
 */
static FaultRecord_t faultStack[FAULT_STACK_SIZE];

/*
 * Head index - points to where the next fault will be written
 */
static uint8_t headIndex = 0;

/*
 * Current number of valid faults in the stack
 */
static uint8_t faultCount = 0;

/*
 * Initialize the fault management system
 */
void faultsInit(void) {
    uint8_t i;
    
    // Clear all fault records
    for (i = 0; i < FAULT_STACK_SIZE; i++) {
        faultStack[i].valid = false;
        faultStack[i].message[0] = '\0';
        faultStack[i].severity = FAULT_SEVERITY_INFO;
    }
    
    headIndex = 0;
    faultCount = 0;
    
    // Turn off fault LED since stack is empty
    faultLED_Off();
}

/*
 * Add a new fault to the fault stack
 */
void faultsAdd(const char *message, FaultSeverity_t severity) {
    FaultRecord_t *fault;
    
    if (message == NULL) {
        return;
    }
    
    // Get pointer to the next fault record slot
    fault = &faultStack[headIndex];
    
    // Copy the fault message (with bounds checking)
    strncpy(fault->message, message, FAULT_MSG_MAX_LEN - 1);
    fault->message[FAULT_MSG_MAX_LEN - 1] = '\0';  // Ensure null termination
    
    // Set fault metadata
    fault->severity = severity;
    fault->valid = true;
    
    // Advance the head index (circular buffer)
    headIndex = (headIndex + 1) % FAULT_STACK_SIZE;
    
    // Update fault count (saturate at FAULT_STACK_SIZE)
    if (faultCount < FAULT_STACK_SIZE) {
        faultCount++;
    }
    
    // Turn on fault LED since we have at least one fault
    faultLED_On();
}

/*
 * Get the number of valid faults currently in the stack
 */
uint8_t faultsGetCount(void) {
    return faultCount;
}

/*
 * Retrieve a fault record by index
 * Index 0 is the most recent fault
 */
bool faultsGet(uint8_t index, FaultRecord_t *fault) {
    uint8_t actualIndex;
    
    if (fault == NULL || index >= faultCount) {
        return false;
    }
    
    // Calculate the actual index in the circular buffer
    // Most recent fault is at (headIndex - 1), next is at (headIndex - 2), etc.
    if (headIndex > index) {
        actualIndex = headIndex - index - 1;
    } else {
        actualIndex = FAULT_STACK_SIZE - (index - headIndex) - 1;
    }
    
    // Copy the fault record
    *fault = faultStack[actualIndex];
    
    return faultStack[actualIndex].valid;
}

/*
 * Get a pointer to a fault record by index (for direct read access)
 */
const FaultRecord_t* faultsGetPtr(uint8_t index) {
    uint8_t actualIndex;
    
    if (index >= faultCount) {
        return NULL;
    }
    
    // Calculate the actual index in the circular buffer
    if (headIndex > index) {
        actualIndex = headIndex - index - 1;
    } else {
        actualIndex = FAULT_STACK_SIZE - (index - headIndex) - 1;
    }
    
    if (!faultStack[actualIndex].valid) {
        return NULL;
    }
    
    return &faultStack[actualIndex];
}

/*
 * Clear all faults from the stack
 */
void faultsClear(void) {
    uint8_t i;
    
    for (i = 0; i < FAULT_STACK_SIZE; i++) {
        faultStack[i].valid = false;
        faultStack[i].message[0] = '\0';
    }
    
    headIndex = 0;
    faultCount = 0;
    
    // Turn off fault LED since stack is now empty
    faultLED_Off();
}

/*
 * Update the fault LED based on the current fault stack status
 */
void faultsUpdateLED(void) {
    if (faultCount > 0) {
        faultLED_On();
    } else {
        faultLED_Off();
    }
}
