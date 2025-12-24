/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 * 
 * Encoder interface for rotary encoder on RC5 (PHASE_A), RC6 (PHASE_B),
 * and RC7 (ENTER_N)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*     
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Debounce polls required for stable button state */
#ifndef ENCODER_BUTTON_DEBOUNCE_POLL_COUNT
#define ENCODER_BUTTON_DEBOUNCE_POLL_COUNT 5
#endif

typedef struct {
    volatile uint8_t position;    // 8-bit encoder position (wraps)
    volatile uint8_t last_state;  // last quadrature state (bits: A<<1 | B)
    volatile uint8_t button_raw;  // raw 1/0 reading (active low)
    volatile uint8_t button_stable;// debounced logical state (1=pressed)
    volatile uint8_t debounce_cnt; // remaining debounce polls
} encoder_state_t;

extern volatile encoder_state_t encoder_state;

void encoder_init(void);            /* initialize encoder hardware and state */
void encoder_poll(void);             /* periodic housekeeping (debounce) */
uint8_t encoder_get_position(void);    /* get current encoder position (8-bit) */
uint8_t encoder_button_state(void); /* 1 = pressed, 0 = released (debounced) */

/* Called from the ISR when IOC detected on RC5/6/7 */
void encoder_handle_ioc(void);

#endif // ENCODER_H
