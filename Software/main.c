/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 */
#include <xc.h>
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "lcd.h"
#include "encoder.h"
#include "menu.h"
#include "smt.h"
#include "control.h"
#include "dac.h"
#include "gps.h"

/*
 * Function prototypes
 */
void selfCheck(void);
void startUp(void);

uint8_t buffer[16];

/************************************************************************
 * Main program loop
 *
 * This is the main program code for the GPSDO (GPS Disciplined Oscillator)
 * project. It initializes the system, performs a self-check by cycling the
 * LEDs connected to the I/O expander, and then enters an infinite loop
 * where it continually processes inputs from either the rotary encoder
 * (to set or view system configuration or properties), or from the GPS module
 * (to discipline the local oscillator). The program uses I2C communication
 * to interface with the I/O expander for front panel controls and indicators.
 *
 ************************************************************************/
void main(int argc, char **argv)
{

    // Initialize the system
    initialize();
    gps_init();
    selfCheck();
    startUp();

    // Main program loop: poll housekeeping (debounce) at ~10ms
    while (1)
    {
        encoder_poll();
        menu_process();
        // Update GPS data processing
        gps_update();

        // Update LCD display once per second
        static uint16_t tick = 0;
        if (++tick >= 100)
        {
            tick = 0;
            uint32_t c = smt_get_last_count();
            int32_t err = smt_get_last_error();
            control_update(err);

            // Get GPS data
            gps_data_t gps_data;
            gps_get_data(&gps_data);

            // Line 1: "GPS Disciplined Oscillator V1"
            lcdWriteBuffer(LINE_0, "GPS Disciplined Osc V1");

            // Line 2: Current date
            char date_line[21];
            gps_format_date(date_line, &gps_data.datetime);
            lcdWriteBuffer(LINE_1, date_line);

            // Line 3: Lat, Long, Altitude
            char pos_line[21];
            gps_format_position(pos_line, &gps_data.position);
            lcdWriteBuffer(LINE_2, pos_line);

            // Line 4: Current frequency count
            char freq_line[21];
            // simple unsigned to decimal conversion
            char tmp[12];
            int ti = 0;
            uint32_t temp_c = c;
            if (temp_c == 0)
                tmp[ti++] = '0';
            while (temp_c > 0 && ti < (int)sizeof(tmp))
            {
                tmp[ti++] = '0' + (temp_c % 10);
                temp_c /= 10;
            }
            int pos = 0;
            freq_line[pos++] = 'F';
            freq_line[pos++] = 'r';
            freq_line[pos++] = 'e';
            freq_line[pos++] = 'q';
            freq_line[pos++] = ':';
            // reverse digits into line
            for (int i = ti - 1; i >= 0; --i)
            {
                freq_line[pos++] = tmp[i];
            }
            freq_line[pos] = '\0';
            lcdWriteBuffer(LINE_3, freq_line);
            // Update LOCK LED if error is within +/-1 (active low)
            static int prev_locked = -1;
            int locked = (err >= -1 && err <= 1) ? 1 : 0;
            if (locked != prev_locked)
            {
                prev_locked = locked;
                uint8_t gpioa = 0xFF;
                if (i2cReadRegister(MCP23017_ADDRESS, GPIOA, &gpioa) == I2C_SUCCESS)
                {
                    if (locked)
                    {
                        gpioa &= (uint8_t)(~LOCK_LED_N); // active low -> clear bit to turn on
                    }
                    else
                    {
                        gpioa |= LOCK_LED_N; // turn off
                    }
                    (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOA, gpioa);
                }
            }
            /* If locked, persist the current DAC setting to EEPROM (only if it differs)
             * We avoid repeated writes by checking the stored config value first.
             */
            if (locked)
            {
                extern volatile system_config_t system_config;
                uint16_t cur = dac_get_raw();
                if (system_config.vco_dac != cur)
                {
                    system_config.vco_dac = cur;
                    config_save((const system_config_t *)&system_config);
                }
            }
        }
        __delay_ms(10);
    }

    return;
}

/*
 * Perform a self-check by cycling the LEDs on the I/O expander and by initializing
 * and displaying a test pattern on the front panel display LCD.
 */

void selfCheck(void)
{
    // Initialize and display self-test message on LCD
    lcdInitialize();
    lcdClearDisplay();
    lcdWriteBuffer(LINE_0, "Self-test...");
    lcdWriteBuffer(LINE_1, "LED test");

    // Turn ON all LEDs (active low): drive Port A low
    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, 0x00);
    __delay_ms(SELFTEST_ON_MS);

    // Turn OFF all LEDs (active low): drive Port A high
    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, 0xFF);
    __delay_ms(SELFTEST_OFF_MS);

    // Turn ON only the power LED (active low): clear POWER_LED_N bit
    uint8_t val = 0xFF & (uint8_t)(~POWER_LED_N);
    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, val);

    // Finish LCD self-test and show startup header
    startUp();
}

/* startUp: display the standard startup header on the LCD */
void startUp(void)
{
    lcdClearDisplay();
    lcdWriteBuffer(LINE_0, "GPS Disciplined Osc V1");
    lcdWriteBuffer(LINE_1, "Initializing...");
    lcdWriteBuffer(LINE_2, "Waiting for GPS...");
    lcdWriteBuffer(LINE_3, "Please wait...");
    __delay_ms(2000); // Show startup message for 2 seconds
}
