
/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Date/time conversion helpers to support UTC/local display with an arbitrary
 * offset (minutes east of UTC). Converts between GPS-provided UTC `gps_datetime_t`
 * and a local `gps_datetime_t` with proper date rollovers.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DATE_H
#define DATE_H

#include <stdint.h>
#include "gps.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * Convert `utc` datetime to a new datetime adjusted by `offset_minutes`
 * (positive offsets are east of UTC). If `utc` is invalid, `out` is marked invalid.
 */
void date_apply_offset(const gps_datetime_t *utc, gps_datetime_t *out, int16_t offset_minutes);

/* 
 * Format time (HH:MM) for display. Buffer should be at least 6 chars long.
 * If invalid, returns "--:--".
 */
void date_format_time_short(char *buf, const gps_datetime_t *dt);

/* 
 * Create an offset string in the form "+HH:MM" or "-HH:MM" for non-zero offsets.
 * For zero offset returns "+00:00". Buffer should be at least 7 bytes.
 */
void tz_offset_to_string(int16_t offset_minutes, char *buf);

#ifdef __cplusplus
}
#endif

#endif /* DATE_H */
