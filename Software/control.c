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
 * For more informaation on the Q notation, see
 * https://en.wikipedia.org/wiki/Q_(number_format)
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
 */

#include "control.h"
#include "config.h"
#include "dac.h"
#include <stdint.h>
#include <xc.h>

/* Tunable parameters for PI controller */
/* Gains are Q8 fixed point (actual_gain = GAIN_Q / 256.0) */
static const long KP_Q = 128; // 0.5 - Proportional gain
static const long KI_Q = 16;  // 0.0625 - Integral gain

/*
 * Scale factor to convert PI output (in error units) to DAC counts.
 * Larger values reduce how much DAC moves for a given error.
 */
static const long PI_TO_DAC_SCALE = 1000; // divide PI output by this to get DAC delta

/*
 * Rate limiting and smoothing
 */
static const long MAX_STEP = 64; // max DAC counts change per update

/*
 * Low-pass filter alpha (Q8). 64 -> 0.25, higher = faster response
 */
static const long LPF_ALPHA_Q = 64; // 0.25

static long integral = 0;
static unsigned int filtered_output = DAC_MIDPOINT;

/*************************************************************************************/
/* Control loop functions                                                            */
/*************************************************************************************/

/*
 * Initialize the control module.
 *
 * @return     None
 */
void control_init(void) {
    integral = 0;
    filtered_output = DAC_MIDPOINT;
    dac_set_raw(filtered_output);
}

/*
 * Update the control loop with a new error value.
 *
 * @param error  The current frequency error (measured - target) in Hz
 * @return       None
 */
void control_update(long error) {
    // PI controller terms (Q8 math)
    long p = (KP_Q * error) >> 8;

    integral += error; // discrete integral (sample period included implicitly)
    long i = (KI_Q * integral) >> 8;

    long pi_output = p + i; // PI controller output

    // Map PI output to DAC delta
    long delta = pi_output / PI_TO_DAC_SCALE;

    // Apply rate limiting
    if (delta > MAX_STEP)
        delta = (long)MAX_STEP;
    else if (delta < -MAX_STEP)
        delta = -MAX_STEP;

    // Compute raw new value
    long raw_new = (long)filtered_output + delta;

    // Clamp to DAC range
    if (raw_new < 0L)
        raw_new = 0L;
    if (raw_new > (long)(DAC_RESOLUTION - 1U))
        raw_new = (long)(DAC_RESOLUTION - 1U);

    // Low-pass filter: filtered = alpha*new + (1-alpha)*old, using Q8
    long alpha = LPF_ALPHA_Q;
    long old = (long)filtered_output;
    long filtered = ((alpha * raw_new) + ((256L - alpha) * old)) >> 8;

    // Final clamp and write
    if (filtered < 0L)
        filtered = 0L;
    if (filtered > (long)(DAC_RESOLUTION - 1U))
        filtered = (long)(DAC_RESOLUTION - 1U);

    filtered_output = (unsigned int)filtered;
    dac_set_raw(filtered_output);
}
