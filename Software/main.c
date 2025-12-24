/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Main program loop for the GPSDO (GPS Disciplined Oscillator) project.
 * The program initializes the system, performs a self-check by cycling the
 * LEDs connected to the I/O expander, and then enters an infinite loop
 * where it continually processes inputs from either the rotary encoder
 * (to set or view system configuration or properties), or from the GPS module
 * (to discipline the local oscillator). The program uses I2C communication
 * to interface with the I/O expander for front panel controls and indicators.
 * The program uses the LCD display to show system status and configuration,
 * the DAC to control the VCO (Voltage Controlled Oscillator) to control the
 * frequency of the local oscillator, the SMT (Synchronous Modulation Tracking)
 * algorithm to discipline the local oscillator to the GPS signal.
 *
 * System configuration and properties are managed through the LCD display
 * and the rotary encoder, and persisted in the EEPROM.
 *
 * The system uses several LED indicators on the front panel.  These include:
 * - POWER_N: Power on LED (active low)
 * - GPS_N: Indicates GPS signal lock (active low)
 * - HOLDOVER_N: Indicates holdover status (active low) (meaning the GPS signal is not available
 * - LOCK_N: Lock status LED (active low).  Indicates that the OCXO is locked to the GPS signal.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "config.h"
#include "control.h"
#include "dac.h"
#include "date.h"
#include "dv.h"
#include "encoder.h"
#include "gps.h"
#include "i2c.h"
#include "isr.h"
#include "lcd.h"
#include "mcp23x17.h"
#include "menu.h"
#include "serial.h"
#include "smt.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <xc.h>

/*
 * Function prototypes
 */
void selfCheck(void);
void startUp(void);
static void formatGpsDateTime(const gps_data_t* gps_data, uint8_t tz_mode, int16_t tz_offset_min);

/*
 * Global variables and data areas.
 */
#include "config.h" /* central place for shared globals */

/************************************************************************
 * Main program loop
 *
 * This is the main program code for the GPSDO (GPS Disciplined Oscillator)
 * project. It initializes the system, performs a self-check by cycling the
 * LEDs connected to the I/O expander, and then enters an infinite processing
 * loop.
 *
 * This processing loop continuously polls the rotary encoder, processes menu inputs,
 * and updates LCD display. This is to make sure that the user interface remains
 * responsive under all conditions.
 *
 * The SMT (Signal Measurement Timer) processing is only performed when a new
 * capture is available, ensuring that the frequency discipline logic is executed
 * in a timely manner without blocking the UI updates.  This capture occurs each
 * time the SMT module window event triggers, indicating a new frequency
 * measurement is available.  This is triggered by the 1PPS signal from the GPS
 * module.
 *
 * During the SMT capture processing, the program updates the DAC to adjust the VCO
 * frequency based on the measured error from the GPS signal. It also updates the
 * LCD display with the latest frequency measurement and manages the LOCK LED status.
 * If the system is locked to the GPS signal, it persists the current DAC setting
 * to EEPROM to maintain the frequency control across power cycles IF the setting
 * has changed.
 *
 * Also during the SMT capture processing, the program transmits the latest GPS data
 * over the serial interface for external monitoring or logging (about once per second).
 * This information includes the current date, time, and position as reported by the
 * GPS module.  The date and time are also displayed on the LCD.
 *
 * The user has the option to display the date/time in UTC or local time based on
 * their configuration settings.
 *
 ************************************************************************/
void main(int argc, char** argv) {

    // Initialize the system
    initialize();
    lcdSetBacklight(true);
    gps_init();
    serial_init();
    dv_init(); // Initialize Data Visualizer streaming
    selfCheck();
    startUp();
    dv_printf("GPSDO System Started - DVRT Active\r\n");
    serial_debug_printf("GPSDO System Started\r\n");

    /**
     * Main processing loop.
     */
    while (1) {
        encoder_poll();
        menu_process();
        gps_update();

        /*
         * Wait for the Timer1-driven 0.1s flag. While waiting, continue to poll
         * encoder, menu and GPS so the UI remains responsive.
         */

        gps_data_t gps_data;
        if (timer_wait_flag) {
            /* Clear timer flag so we await the next 0.1s interval */
            timer_wait_flag = false;
            // Send GPS data every cadence
            gps_get_data(&gps_data);
            serial_send_gps_data(&gps_data);
        }

        // Update LCD common lines
        lcdBufferSetLine(0, "GPSDO V1.0");

        char date_time_line[21];
        gps_format_date_time(date_time_line, &gps_data.datetime);
        lcdBufferSetLine(1, date_time_line);

        char pos_line[21];
        gps_format_position(pos_line, &gps_data.position);
        lcdBufferSetLine(2, pos_line);

        /*
         * Only perform SMT-based control processing when a new capture has
         * been recorded by the SMT ISR. Clear the capture flag after handling.
         */
        if (smt_capture_available()) {
            uint32_t c = smt_get_last_count();
            int32_t err = smt_get_last_error();
            control_update(err);

            // Send debug data to Data Visualizer
            dv_debug_int("SMT Count", (int32_t)c);
            dv_debug_int("Freq Error", err);
            dv_debug_int("DAC Output", (int32_t)dac_get_raw());
            dv_send_multi_graph_data(1, (float)err, (float)dac_get_raw(), (float)c);

            // Format and set frequency line using printf-style formatting
            lcdBufferPrintf(3, "Freq:%lu", (unsigned long)c);

            // Update LOCK LED if error is within +/-1 (active low)
            int locked = (err >= -1 && err <= 1) ? 1 : 0;
            if (locked) {
                ioporta.LOCK_N = 0; // active low -> clear bit to turn on
            } else {
                ioporta.LOCK_N = 1; // turn off
            }
            (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);

            /*
             * If locked, persist the current DAC setting to EEPROM (only if it differs)
             * We avoid repeated writes by checking the stored config value first.
             */
            if (locked) {
                uint16_t cur = dac_get_raw();
                if (system_config.vco_dac != cur) {
                    system_config.vco_dac = cur;
                    config_save((const system_config_t*)&system_config);
                }
            }

            /* Mark this capture as handled */
            smt_clear_capture();
        } else {
            /* If no new capture, indicate waiting on line 3 */
            lcdBufferSetLine(3, "Freq: waiting...");
        }
    }

    return;
}

/*
 * Perform a self-check by cycling the LEDs on the I/O expander and by initializing
 * and displaying a test pattern on the front panel display LCD.
 */
void selfCheck(void) {
    // Initialize LCD hardware and buffer system
    lcdInitialize();
    __delay_ms(100); // Give LCD time to initialize
    lcdBufferInit();
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100); // Ensure clean start

    lcdSelfTest();

    // Ensure completely clean state after self test
    __delay_ms(100); // Let self test complete
    lcdBufferClear();
    lcdBufferUpdate();
    __delay_ms(100); // Allow buffer update to complete
    lcdReturnHome();
    __delay_ms(100); // Allow LCD to settle

    // Ensure backlight and display are fully on after self test
    lcdSetBacklight(true);
    lcdWriteInstruction(DISPLAY_ON);
    __delay_ms(50);

    /*
     * Turn ON all LEDs (active low): drive Port A low
     */
    ioporta.POWER_N = 0;
    ioporta.GPS_N = 0;
    ioporta.HOLDOVER_N = 0;
    ioporta.LOCK_N = 0;

    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    __delay_ms(500);

    /*
     * Turn OFF all LEDs (active low): drive Port A high
     */
    ioporta.POWER_N = 1;
    ioporta.GPS_N = 1;
    ioporta.HOLDOVER_N = 1;
    ioporta.LOCK_N = 1;

    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    __delay_ms(500);

    /*
     * Turn ON only the power LED (active low)
     */
    ioporta.POWER_N = 0;
    i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
    __delay_ms(500);
}

/**
 * Display startup message on LCD for 2 seconds.
 */
void startUp(void) {
    lcdBufferClear();
    lcdBufferSetLine(0, "GPS Disciplined Osc V1");
    lcdBufferSetLine(1, "Initializing...");
    lcdBufferSetLine(2, "Waiting for GPS...");
    lcdBufferSetLine(3, "Please wait...");
    lcdBufferUpdate(); // Force immediate display update
    __delay_ms(2000);  // Show startup message for 2 seconds
}
