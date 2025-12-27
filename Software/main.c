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
#include <xc.h>

/*
 * Function prototypes
 */
void selfCheck(void);
void updateDisplay(void);

/*
 * Global variables and data areas.
 */
extern system_config_t system_config;
extern IOPortA_t ioporta;
extern uint8_t buffer[128];
extern volatile encoder_state_t encoder_state;
extern bool system_initialized;

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
    updateDisplay();
    printf("GPSDO System Started\r\n");

    /**
     * Main processing loop.
     */
    while (1) {
        menu_process();

        // Only process GPS data when complete sentences are available
        if (gps_sentence_ready()) {
            gps_update();
        }

        /*
         * Only perform SMT-based control processing when a new capture has
         * been recorded by the SMT ISR. Clear the capture flag after handling.
         */
        if (smt_capture_available()) {
            uint32_t c = smt_get_last_count();
            int32_t err = smt_get_last_error();
            control_update(err);

            // Send debug data over serial
            printf("[PRINTF] Count: %lu, Error: %ld, DAC: %d\r\n", (unsigned long)c, (long)err, (int)dac_get_raw());

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
        updateDisplay();
    }

    return;
}
