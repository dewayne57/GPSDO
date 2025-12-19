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
#include <xc.h>
#include "config.h"
#include "i2c.h"
#include "mcp23x17.h"
#include "encoder.h"
#include "smt.h"

/*
 * Dedicated low-priority ISR for pin-change / IOC events.
 * This keeps IOC handling separated from the default ISR and makes the
 * encoder handler appear as its own logical interrupt.
 */
void __interrupt(irq(0x07), low_priority) ioChangeIsr(void)
{
    if (IOCIF) {
        encoder_handle_ioc();
        IOCIF = 0;
        return;
    }

    // Fall through for other low-priority interrupts if needed
    NOP();
}

/**
 * The default ISR simply clears all other interrupts and ignores them.
 */
void __interrupt(irq(default)) defaultIsr(void)
{
    // Keep this minimal to avoid masking other unexpected interrupt sources
    // SMT1 period result (capture on 1PPS)
    if (SMT1PRAIF) {
        smt_handle_capture();
        return;
    }

    NOP();
}
