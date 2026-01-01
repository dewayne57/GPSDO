/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
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

extern IOPortB_t ioportb;

/**
 * Write the current LED state to the hardware
 */
static void ledUpdate(void) {
    (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOB, ioportb.all);
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
    ioportb.POWER_N = 0;
    ledUpdate();
}

void powerLED_Off(void) {
    ioportb.POWER_N = 1;
    ledUpdate();
}

/**
 * GPS LED control (active low)
 */
void gpsLED_On(void) {
    ioportb.GPS_N = 0;
    ledUpdate();
}

void gpsLED_Off(void) {
    ioportb.GPS_N = 1;
    ledUpdate();
}

/**
 * Lock LED control (active low)
 */
void lockLED_On(void) {
    ioportb.LOCK_N = 0;
    ledUpdate();
}

void lockLED_Off(void) {
    ioportb.LOCK_N = 1;
    ledUpdate();
}

/**
 * Holdover LED control (active low)
 */
void holdoverLED_On(void) {
    ioportb.HOLDOVER_N = 0;
    ledUpdate();
}

void holdoverLED_Off(void) {
    ioportb.HOLDOVER_N = 1;
    ledUpdate();
}
/**
 * Fault LED control (active low)
 */
void faultLED_On(void) {
    ioportb.FAULT_N = 0;
    ledUpdate();
}

void faultLED_Off(void) {
    ioportb.FAULT_N = 1;
    ledUpdate();
}

/**
 * Turn all LEDs on
 */
void ledAllOn(void) {
    ioportb.POWER_N = 0;
    ioportb.GPS_N = 0;
    ioportb.LOCK_N = 0;
    ioportb.HOLDOVER_N = 0;
    ioportb.FAULT_N = 0;
    ledUpdate();
}

/**
 * Turn all LEDs off
 */
void ledAllOff(void) {
    ioportb.POWER_N = 1;
    ioportb.GPS_N = 1;
    ioportb.LOCK_N = 1;
    ioportb.HOLDOVER_N = 1;
    ioportb.FAULT_N = 1;
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
