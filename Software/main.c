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
void main(int argc, char** argv) {

    // Initialize the system
    initialize();
    selfCheck();
    startUp();
    
    // Main program loop: poll housekeeping (debounce) at ~10ms
    while (1) {
        encoder_poll();
        menu_process();
        // Update SMT display once per second
        static uint16_t tick = 0;
        if (++tick >= 100) {
            tick = 0;
            uint32_t c = smt_get_last_count();
            int32_t err = smt_get_last_error();
            control_update(err);
            // Format as: CNT:12345678
            char line[21];
            // simple unsigned to decimal conversion
            char tmp[12];
            int ti = 0;
            if (c == 0) tmp[ti++] = '0';
            while (c > 0 && ti < (int)sizeof(tmp)) {
                tmp[ti++] = '0' + (c % 10);
                c /= 10;
            }
            int pos = 0;
            line[pos++] = 'C'; line[pos++] = 'N'; line[pos++] = 'T'; line[pos++] = ':';
            // reverse digits into line
            for (int i = ti - 1; i >= 0; --i) {
                line[pos++] = tmp[i];
            }
            line[pos] = '\0';
            lcdWriteBuffer(LINE_1, line);
            // Also show current DAC setting on line 0 as: DAC:2048
            uint16_t d = dac_get_raw();
            char line0[21];
            int dpi = 0;
            uint16_t dv = d;
            char dt[6];
            int dti = 0;
            if (dv == 0) dt[dti++] = '0';
            while (dv > 0 && dti < (int)sizeof(dt)) {
                dt[dti++] = '0' + (dv % 10);
                dv /= 10;
            }
            int p0 = 0;
            line0[p0++] = 'D'; line0[p0++] = 'A'; line0[p0++] = 'C'; line0[p0++] = ':';
            for (int i = dti - 1; i >= 0; --i) line0[p0++] = dt[i];
            line0[p0] = '\0';
            lcdWriteBuffer(LINE_0, line0);
            // Update LOCK LED if error is within +/-1 (active low)
            static int prev_locked = -1;
            int locked = (err >= -1 && err <= 1) ? 1 : 0;
            if (locked != prev_locked) {
                prev_locked = locked;
                uint8_t gpioa = 0xFF;
                if (i2cReadRegister(MCP23017_ADDRESS, GPIOA, &gpioa) == I2C_SUCCESS) {
                    if (locked) {
                        gpioa &= (uint8_t)(~LOCK_LED_N); // active low -> clear bit to turn on
                    } else {
                        gpioa |= LOCK_LED_N; // turn off
                    }
                    (void)i2cWriteRegister(MCP23017_ADDRESS, GPIOA, gpioa);
                }
            }
            /* If locked, persist the current DAC setting to EEPROM (only if it differs)
             * We avoid repeated writes by checking the stored config value first.
             */
            if (locked) {
                extern volatile system_config_t system_config;
                uint16_t cur = dac_get_raw();
                if (system_config.vco_dac != cur) {
                    system_config.vco_dac = cur;
                    config_save((const system_config_t*)&system_config);
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

void selfCheck(void) {
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
void startUp(void) {
    lcdClearDisplay();
    lcdWriteBuffer(LINE_0, "GPSDO V1.0");
}
