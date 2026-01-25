/*
 * Copyright (c) 2026, Dewayne L. Hafenstein.  All rights reserved.
 *
 * LED control functions for the front panel LEDs.
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
 *
 * This module provides simple functions to control the front panel LEDs
 * connected to the MCP23017 I/O expander Port B. All LEDs are active low.
 */
#include "led.h"
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include <xc.h>

extern shadowB_t shadowB;

/**
 * Write the current LED state to the hardware
 */
static void ledUpdate(void) {
    (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOB, shadowB.all);
}

/**
 * Initialize the LED module - turn all LEDs off
 */
void ledInitialize(void) {
    ledAllOff();
    ledUpdate();
    ledTest();
    ledAllOff();
    powerLED_On();
}

/**
 * Power LED control (active low)
 */
void powerLED_On(void) {
    shadowB.POWER_N = 0;
    ledUpdate();
}

void powerLED_Off(void) {
    shadowB.POWER_N = 1;
    ledUpdate();
}

/**
 * GPS LED control (active low)
 */
void gpsLED_On(void) {
    shadowB.GPS_N = 0;
    ledUpdate();
}

void gpsLED_Off(void) {
    shadowB.GPS_N = 1;
    ledUpdate();
}

/**
 * Lock LED control (active low)
 */
void lockLED_On(void) {
    shadowB.LOCK_N = 0;
    ledUpdate();
}

void lockLED_Off(void) {
    shadowB.LOCK_N = 1;
    ledUpdate();
}

/**
 * Holdover LED control (active low)
 */
void holdoverLED_On(void) {
    shadowB.HOLDOVER_N = 0;
    ledUpdate();
}

void holdoverLED_Off(void) {
    shadowB.HOLDOVER_N = 1;
    ledUpdate();
}
/**
 * Fault LED control (active low)
 */
void faultLED_On(void) {
    shadowB.FAULT_N = 0;
    ledUpdate();
}

void faultLED_Off(void) {
    shadowB.FAULT_N = 1;
    ledUpdate();
}

/**
 * Turn all LEDs on
 */
void ledAllOn(void) {
    shadowB.POWER_N = 0;
    shadowB.GPS_N = 0;
    shadowB.LOCK_N = 0;
    shadowB.HOLDOVER_N = 0;
    shadowB.FAULT_N = 0;
    ledUpdate();
}

/**
 * Turn all LEDs off
 */
void ledAllOff(void) {
    shadowB.POWER_N = 1;
    shadowB.GPS_N = 1;
    shadowB.LOCK_N = 1;
    shadowB.HOLDOVER_N = 1;
    shadowB.FAULT_N = 1;
    ledUpdate();
}

/**
 * LED test sequence - turns all LEDs on, then off, then power LED on
 */
void ledTest(void) {
    // Turn ON all LEDs
    ledAllOn();
    __delay_ms(500);

    // Turn OFF all LEDs
    ledAllOff();
    __delay_ms(500);

    // Turn ON only the power LED
    powerLED_On();
    __delay_ms(500);
}
