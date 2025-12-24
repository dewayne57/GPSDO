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
        IOCIF = 0;
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
        // Read character from UART
        char c = U1RXB;

        // Store in circular buffer
        gps_buffer_put_char(c);

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
        // Read character from UART2
        char c = U2RXB;

        // Store in circular buffer
        serial_buffer_put_char(c);

        PIR8bits.U2RXIF = 0; // Clear interrupt flag
    }
}

/*
 * Timer1 Overflow ISR 
 *
 * Timer1 is configured to overflow every ~10ms. This ISR counts 10 overflows
 * to create a 0.1s (100ms) timer flag for the main loop.   
 */
void __interrupt(irq(TMR1), low_priority) timer1_isr(void) {
    if (TMR1IF) {
        static uint8_t t1_cnt = 0;
        TMR1IF = 0; // clear interrupt flag

        /* Reload for next ~10ms interval (preload value for 20,000 ticks) */
        TMR1H = 0xB2;
        TMR1L = 0xA0;

        if (++t1_cnt >= 10) {
            t1_cnt = 0;
            timer_wait_flag = true;
        }
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
    }
}

/**
 * The default ISR now handles non-Timer1 and non-SMT interrupts
 */
void __interrupt(irq(default)) defaultIsr(void) {
    NOP();
}

