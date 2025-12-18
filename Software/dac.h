/* 
 *Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
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

#ifndef DAC_H
#define DAC_H

#include <stdint.h>

void dac_init(void);
void dac_set_raw(uint16_t value); /* 12-bit value 0..4095 */
uint16_t dac_get_raw(void);

#endif // DAC_H
