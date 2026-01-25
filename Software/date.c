/**
 * Copyright (c) 2026 Dewayne VanHoozer. All rights reserved.
 *
 * Date and time utility functions.
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
#include "date.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

/*
 * Compute Julian Day Number from Gregorian date
 * Valid for Gregorian calendar dates.
 *
 * @param y  Year (e.g., 2025)
 * @param m  Month (1-12)
 * @param d  Day (1-31)
 * @return   Julian Day Number
 */
static long jdn_from_ymd(int y, int m, int d) {
    int a = (14 - m) / 12;
    int y2 = y + 4800 - a;
    int m2 = m + 12 * a - 3;
    long jdn = d + (153 * m2 + 2) / 5 + 365L * y2 + y2 / 4 - y2 / 100 + y2 / 400 - 32045;
    return jdn;
}

/*
 * Convert JDN back to y,m,d using Fliegel & Van Flandern algorithm
 *
 * @param jdn  Julian Day Number
 * @param y    Pointer to store year
 * @param m    Pointer to store month
 * @param d    Pointer to store day
 * @return     None
 */
static void ymd_from_jdn(long jdn, int* y, int* m, int* d) {
    long l = jdn + 68569;
    long n = (4 * l) / 146097;
    l = l - (146097 * n + 3) / 4;
    long i = (4000 * (l + 1)) / 1461001;
    l = l - (1461 * i) / 4 + 31;
    long j = (80 * l) / 2447;
    long day = l - (2447 * j) / 80;
    l = j / 11;
    long month = j + 2 - (12 * l);
    long year = 100 * (n - 49) + i + l;
    *y = (int)year;
    *m = (int)month;
    *d = (int)day;
}

/*
 * Apply a timezone offset to a UTC date/time.
 *
 * @param utc             Pointer to input UTC date/time
 * @param out             Pointer to output date/time with offset applied
 * @param offset_minutes  Timezone offset in minutes (e.g., -300 for UTC-05:00)
 * @return                None
 */
void date_apply_offset(const gps_datetime_t* utc, gps_datetime_t* out, int offset_minutes) {
    if (!utc || !out)
        return;
    if (utc->valid != GPS_VALID) {
        out->valid = GPS_INVALID;
        return;
    }

    int year = 2000 + utc->year;
    int month = utc->month;
    int day = utc->day;
    long jdn = jdn_from_ymd(year, month, day);

    long seconds = (long)utc->hour * 3600L + (long)utc->minute * 60L + (long)utc->second;
    long add = (long)offset_minutes * 60L;
    long total = seconds + add;

    /* Adjust days if total wraps around */
    while (total < 0) {
        total += 86400L;
        jdn -= 1;
    }
    while (total >= 86400L) {
        total -= 86400L;
        jdn += 1;
    }

    int y, m, d;
    ymd_from_jdn(jdn, &y, &m, &d);

    out->year = (uint8_t)(y - 2000);
    out->month = (uint8_t)m;
    out->day = (uint8_t)d;
    out->hour = (uint8_t)(total / 3600L);
    out->minute = (uint8_t)((total % 3600L) / 60L);
    out->second = (uint8_t)(total % 60L);
    out->valid = GPS_VALID;
}

/**
 * Format time (HH:MM) for display. Buffer should be at least 6 chars long.
 * If invalid, returns "--:--".
 *
 * @param buf  Pointer to output buffer
 * @param dt   Pointer to gps_datetime_t structure
 * @return     None
 */
void date_format_time_short(char* buf, const gps_datetime_t* dt) {
    if (!buf || !dt)
        return;
    if (dt->valid != GPS_VALID) {
        (void)snprintf(buf, 6, "--:--");
        return;
    }
    (void)snprintf(buf, 6, "%02d:%02d", dt->hour, dt->minute);
}

/**
 * Create an offset string in the form "+HH:MM" or "-HH:MM" for non-zero offsets.
 * For zero offset returns "+00:00". Buffer should be at least 7 bytes.
 *
 * @param offset_minutes  Timezone offset in minutes
 * @param buf             Pointer to output buffer
 * @return                None
 */
void tz_offset_to_string(int offset_minutes, char* buf) {
    if (!buf)
        return;
    char sign_char = (offset_minutes >= 0) ? '+' : '-';
    int absmin = (offset_minutes >= 0) ? (int)offset_minutes : -(int)offset_minutes;
    int h = absmin / 60;
    int m = absmin % 60;
    (void)snprintf(buf, 7, "%c%02d:%02d", sign_char, h, m);
}
