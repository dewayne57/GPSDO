/* 
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Simple rotary encoder handler.
 * - RC5 = Phase A
 * - RC6 = Phase B
 * - RC7 = Switch (active low, pulled to ground when closed)
 *
 * This implementation enables weak pull-ups on RC5/6/7, processes the pin changes
 * as part of an interrup-on-change (IOC) service routine, and updates a signed
 * 8-bit position counter. Button debouncing is handled externally.

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
#include <xc.h>
#include <stdint.h>
#include "config.h"
#include "encoder.h"
/* encoder_state is defined centrally in config.c */


/*
 * Initialize the encoder hardware and state.
 */
void encoder_init(void)
{
    // Ensure pins are inputs
    TRISC |= PHASE_A + PHASE_B + ENTER_N; // RC5, RC6, RC7

    // Enable weak pull-ups for RC5..RC7 (bits 5,6,7)
    WPUC |= PHASE_A + PHASE_B + ENTER_N;

    // Configure IOC for both edges on RC5/RC6/RC7
    IOCCP |= PHASE_A + PHASE_B + ENTER_N; // positive edge
    IOCCN |= PHASE_A + PHASE_B + ENTER_N; // negative edge

    // Clear any existing IOC flags for port C pins
    IOCCF &= (uint8_t)~(PHASE_A + PHASE_B + ENTER_N);

    // Enable IOC module (power it up) and enable IOC interrupt
    PMD0bits.IOCMD = 0; // enable IOC module
    PIE0bits.IOCIE = 1; // enable IOC interrupt

    // Read initial quadrature state
    uint8_t a = PORTC & PHASE_A ? 1 : 0;
    uint8_t b = PORTC & PHASE_B ? 1 : 0;
    encoder_state.last_state = (uint8_t)((a << 1) | b);

    encoder_state.button_raw = PORTC & ENTER_N ? 1 : 0; // idle high
    encoder_state.button_stable = (encoder_state.button_raw == 0) ? 1 : 0;
    encoder_state.debounce_cnt = 0;
}

/*
 * Get the current encoder position (8-bit, clamped 0..127).
 */
uint8_t encoder_get_position(void)
{
    return encoder_state.position;
}

/*
 * Get the current debounced button state. 1 = pressed, 0 = released.
 */
uint8_t encoder_button_state(void)
{
    return encoder_state.button_stable;
}

/*
 * Called from ISR when IOC event on RC5/RC6/RC7 detected. Performs a quick
 * sample of the pins and updates position and button raw state. Button
 * debouncing is handled externally. This function is safe to call from ISR
 * context (keeps it short).
 */
void encoder_handle_ioc(void)
{
    // Snapshot PORTC once to keep A/B/button sampling coherent during IOC handling
    uint8_t portc = PORTC;

    uint8_t a = (portc & PHASE_A) ? 1U : 0U;
    uint8_t b = (portc & PHASE_B) ? 1U : 0U;
    uint8_t cur = (uint8_t)((a << 1) | b);

    uint8_t last = encoder_state.last_state;
    if (cur != last)
    {
        if (((last + 1) & 3U) == cur)
        {
            if (encoder_state.position < 127U)
            {
                encoder_state.position = (uint8_t)(encoder_state.position + 1U);
            }
        }
        else if (((last + 3) & 3U) == cur)
        {
            if (encoder_state.position > 0U)
            {
                encoder_state.position = (uint8_t)(encoder_state.position - 1U);
            }
        }
        encoder_state.last_state = cur;
    }

    uint8_t braw = (portc & ENTER_N) ? 1U : 0U; // active high when released
    encoder_state.button_raw = braw;
    encoder_state.button_stable = braw; // no internal debounce

    // Clear IOC flags for the handled pins (write 0 to clear)
    IOCCF &= (uint8_t)~(PHASE_A + PHASE_B + ENTER_N);
}
