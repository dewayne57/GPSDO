/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 */
#include <xc.h>
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "encoder.h"
#include "smt.h"
#include "gps.h"
#include "isr.h"

/*
 * Dedicated low-priority ISR for pin-change / IOC events.
 * This keeps IOC handling separated from the default ISR and makes the
 * encoder handler appear as its own logical interrupt.
 */
void __interrupt(irq(0x07), low_priority) ioChangeIsr(void)
{
    if (IOCIF)
    {
        encoder_handle_ioc();
        IOCIF = 0;
        return;
    }

    // Fall through for other low-priority interrupts if needed
    NOP();
}

/*
 * UART1 Receive ISR
 */
void __interrupt(irq(U1RX), high_priority) uart1_rx_isr(void)
{
    if (PIR4bits.U1RXIF)
    {
        // Read character from UART
        char c = U1RXB;

        // Store in circular buffer
        gps_buffer_put_char(c);

        PIR4bits.U1RXIF = 0; // Clear interrupt flag
    }
}

/**
 * The default ISR simply clears all other interrupts and ignores them.
 */
void __interrupt(irq(default)) defaultIsr(void)
{
    // Keep this minimal to avoid masking other unexpected interrupt sources
    // SMT1 period result (capture on 1PPS)
    if (SMT1PRAIF)
    {
        smt_handle_capture();
        return;
    }

    NOP();
}
