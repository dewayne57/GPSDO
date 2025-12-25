/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * This module contains the interrupt service routines (ISRs) for the
 * various interrupt sources used in the project.
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
 */
#include "isr.h"
#include "config.h"
#include "encoder.h"
#include "gps.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "serial.h"
#include "smt.h"
#include <xc.h>

/*
 * Dedicated low-priority ISR for pin-change / IOC events.
 * This keeps IOC handling separated from the default ISR and makes the
 * encoder handler appear as its own logical interrupt.
 */
void __interrupt(irq(0x07), low_priority) ioChangeIsr(void) {
    if (IOCIF) {
        encoder_handle_ioc();
        IOCCF = 0; // clear all IOC flags for port C
        IOCIF = 0; // clear global IOC interrupt flag
        return;
    }
}

/*
 * UART1 Receive ISR.
 *
 * UART1 is used for GPS module communication.  This interrupt reads incoming
 * characters and stores them in a circular buffer for processing in the main loop.
 */
void __interrupt(irq(U1RX), high_priority) uart1_rx_isr(void) {
    if (PIR4bits.U1RXIF) {
        // Clear overrun if present to keep receiver alive
        if (U1ERRIRbits.RXFOIF) {
            U1CON0bits.RXEN = 0;
            NOP();
            U1CON0bits.RXEN = 1;
            (void)U1RXB;            // Flush
            PIR4bits.U1RXIF = 0;    // Clear interrupt flag
            U1ERRIRbits.RXFOIF = 0; // Clear overrun flag
            return;
        }

        char c = U1RXB;         // Read character
        gps_buffer_put_char(c); // Store in circular buffer

        PIR4bits.U1RXIF = 0; // Clear interrupt flag
    }
}

/*
 * UART2 Receive ISR (for external serial/bootloader).
 *
 * UART2 is used for external serial communication, normally for the GPS output
 * data (date, time, position) and can also support the boot loader to load new
 * firmware (e.g., bootloader interface).
 * This interrupt reads incoming characters and stores them in a circular buffer
 * when not used by the bootloader  .
 */
void __interrupt(irq(U2RX), high_priority) uart2_rx_isr(void) {
    if (PIR8bits.U2RXIF) {
        // Clear overrun if present to keep receiver alive
        if (U2ERRIRbits.RXFOIF) {
            U2CON0bits.RXEN = 0;
            NOP();
            U2CON0bits.RXEN = 1;
            (void)U2RXB;            // Flush
            PIR8bits.U2RXIF = 0;    // Clear interrupt flag
            U2ERRIRbits.RXFOIF = 0; // Clear overrun flag
            return;
        }

        char c = U2RXB;            // Read character from UART2
        serial_buffer_put_char(c); // Store in circular buffer

        PIR8bits.U2RXIF = 0; // Clear interrupt flag
    }
}

/**
 * SMT1 Period Result (capture on 1PPS) ISR
 * The CMT (Signal Measurement Timer) captures the timer count on each rising edge
 * of the 1PPS signal from the GPS module. This ISR handles the capture event
 * and processes the frequency measurement.
 */
void __interrupt(irq(SMT1PRA), low_priority) smt1_isr(void) {
    if (SMT1PRAIF) {
        smt_handle_capture();
        SMT1PRAIF = 0; // clear capture interrupt flag
    }
}

/**
 * This ISR does nothing and does not clear any pending interrupt flags.  The
 * intention is that all other interrupts should have their own dedicated ISRs.
 * If an unexpected interrupt occurs, this default ISR will be invoked and
 * simply return, leaving the interrupt flag set.  This will allow debugging
 * to determine which interrupt source is misconfigured or unhandled as it
 * will continuously re-enter this ISR (loop of death) until the issue is resolved.
 */
void __interrupt(irq(default)) defaultIsr(void) {
    NOP();
}
