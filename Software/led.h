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
#ifndef LED_H
#define LED_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the LED module
 */
void ledInitialize(void);

/**
 * Power LED control
 */
void powerLED_On(void);
void powerLED_Off(void);

/**
 * GPS LED control
 */
void gpsLED_On(void);
void gpsLED_Off(void);

/**
 * Lock LED control
 */
void lockLED_On(void);
void lockLED_Off(void);

/**
 * Holdover LED control
 */
void holdoverLED_On(void);
void holdoverLED_Off(void);

/**
 * Fault LED control
 */
void faultLED_On(void);
void faultLED_Off(void);

/**
 * Control all LEDs at once
 */
void ledAllOn(void);
void ledAllOff(void);

/**
 * LED test sequence (cycles through all LEDs)
 */
void ledTest(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
