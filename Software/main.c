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
 * - POWER_LED_N: Power on LED (active low)
 * - GPS_N: Indicates GPS signal lock (active low)
 * - HOLDOVER_N: Indicates holdover status (active low) (meaning the GPS signal is not available
 * - HIGH_N: Indicates that the OCXO output frequency is above the target frequency (active low)
 * - LOW_N: Indicates that the OCXO output frequency is below the target frequency (active low)
 * - LOCK_LED_N: Lock status LED (active low).  Indicates that the OCXO is locked to the GPS signal.
 * - FAULT_N: Indicates a fault condition (active low) *
 *
 * licensed under the Apache License, Version 2.0 (the "License");
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
#include "encoder.h"
#include "gps.h"
#include "i2c.h"
#include "lcd.h"
#include "mcp23x17.h"
#include "menu.h"
#include "mytypes.h"
#include "serial.h"
#include "smt.h"
#include <xc.h>

/*
 * Function prototypes
 */
void selfCheck(void);
void startUp(void);

/*
 * Global variables and data areas.
 */
uint8_t buffer[16];
IOPortA_t ioporta;
system_config_t system_config;

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
void main(int argc, char** argv) {

    // Initialize the system
    initialize();
    lcdSetBacklight(TRUE);
    gps_init();
    serial_init();
    selfCheck();
    startUp();

    // Main program loop: poll housekeeping (debounce) at ~10ms
    while (1) {
        encoder_poll();
        menu_process();
        // Update GPS data processing
        gps_update();

        // Update LCD display once per second
        static uint16_t tick = 0;
        if (++tick >= 100) {
            tick = 0;
            uint32_t c = smt_get_last_count();
            int32_t err = smt_get_last_error();
            control_update(err);

            // Get GPS data
            gps_data_t gps_data;
            gps_get_data(&gps_data);

            // Send GPS data via serial port every second
            serial_send_gps_data(&gps_data);

            // Update LCD buffer with current data
            lcdBufferSetLine(0, "GPS Disciplined Osc V1");

            // Format and set date line
            char date_line[21];
            gps_format_date(date_line, &gps_data.datetime);
            lcdBufferSetLine(1, date_line);

            // Format and set position line
            char pos_line[21];
            gps_format_position(pos_line, &gps_data.position);
            lcdBufferSetLine(2, pos_line);

            // Format and set frequency line using printf-style formatting
            lcdBufferPrintf(3, "Freq:%lu", (unsigned long)c);

            // Update LOCK LED if error is within +/-1 (active low)
            static int prev_locked = -1;
            int locked = (err >= -1 && err <= 1) ? 1 : 0;
            if (locked != prev_locked) {
                prev_locked = locked;
                uint8_t gpioa = 0xFF;
                if (i2cReadRegister(MCP23017_ADDRESS, GPIOA, &gpioa) == I2C_SUCCESS) {
                    if (locked) {
                        ioporta.LOCK_N = 0; // active low -> clear bit to turn on
                    } else {
                        ioporta.LOCK_N = 1; // turn off
                    }
                    (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOA, ioporta.all);
                }
            }

            /* If locked, persist the current DAC setting to EEPROM (only if it differs)
             * We avoid repeated writes by checking the stored config value first.
             */
            if (locked) {
                uint16_t cur = dac_get_raw();
                if (system_config.vco_dac != cur) {
                    system_config.vco_dac = cur;
                    config_save((const system_config_t*)&system_config);
                }
            }
        }

        // Update LCD display from buffer
        lcdBufferUpdate();

        __delay_ms(10);
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
    lcdSetBacklight(TRUE);
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

void startUp(void) {
    lcdBufferClear();
    lcdBufferSetLine(0, "GPS Disciplined Osc V1");
    lcdBufferSetLine(1, "Initializing...");
    lcdBufferSetLine(2, "Waiting for GPS...");
    lcdBufferSetLine(3, "Please wait...");
    lcdBufferUpdate(); // Force immediate display update
    __delay_ms(2000);  // Show startup message for 2 seconds
}
