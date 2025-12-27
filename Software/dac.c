/* 
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Simple driver for a 12-bit I2C DAC (DAC8571 like device)
 * - Writes 12-bit values to the DAC via I2C
 * - Uses i2cWriteBuffer()
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
#include <xc.h>
#include <stdint.h>
#include "i2c.h"
#include "config.h"
#include "dac.h"

static uint16_t dac_current = 0;
extern system_config_t system_config;
extern uint8_t buffer[128];

/*
 * Initialize the DAC module.  Sets the DAC to a known state.
 */
void dac_init(void)
{
    // If the system config contains a stored DAC value, use it as startup
    // Otherwise fall back to midpoint
    uint16_t start = (uint16_t)DAC_MIDPOINT;
    if (system_config.magic == CONFIG_MAGIC && system_config.version == CONFIG_VERSION) {
        start = system_config.vco_dac;
        if (start >= DAC_RESOLUTION) start = (uint16_t)(DAC_RESOLUTION - 1);
    }
    dac_current = start;
    dac_set_raw(dac_current);
}

/* DAC8571 write: control byte followed by 16-bit data (big-endian).
 * Control 0x10 = write-and-update, power-up (per DAC8571 datasheet).
 */
void dac_set_raw(uint16_t value)
{
    if (value >= DAC_RESOLUTION) value = (uint16_t)(DAC_RESOLUTION - 1);

    buffer[0] = 0x10;                              // control: write + update DAC, power-up
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);    // D15..D8
    buffer[2] = (uint8_t)(value & 0xFF);           // D7..D0

    i2cWriteBuffer(DAC8571_ADDRESS, buffer, 3);
    dac_current = value;
}

/* 
 * Get the current raw DAC value 
 */
uint16_t dac_get_raw(void)
{
    return dac_current;
}
