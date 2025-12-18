/* 
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *  
 * Simple PID-like control loop for VCO tuning using DAC8571
 * - Discrete-time update called periodically (we use 1s sample by default)
 * - Fixed-point Q8 gains to avoid floating point arithmetic on the MCU
 * - Output mapped to 12-bit DAC value and rate-limited / low-pass filtered
 * 
 * Q8 gain means that the gain value is multiplied by 256.0 to represent
 * fractional values using integer math.  For example, a gain of 0.5 is
 * represented as 128 (0.5 * 256 = 128).  This allows for fractional gains
 * using only integer arithmetic which is more efficient on microcontrollers
 * without floating-point units.  It is known as Q8 fixed-point representation.
 * The PID calculation uses bit-shifting to divide by 256 (>> 8) to convert
 * back to normal scale after multiplication.
 *
 * For more information on the Q notation, see 
 * https://en.wikipedia.org/wiki/Q_(number_format)
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
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

void control_init(void);    /* call once at startup */
void control_update(int32_t error); /* call periodically with latest error */

#endif // CONTROL_H
