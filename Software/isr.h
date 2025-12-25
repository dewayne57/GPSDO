/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Interrupt Service Routines header file
 */

#ifndef ISR_H
#define ISR_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function prototypes for ISRs */
void __interrupt(irq(0x07), low_priority) ioChangeIsr(void);
void __interrupt(irq(SMT1PRA), low_priority) smt1_isr(void);
void __interrupt(irq(default)) defaultIsr(void);
void __interrupt(irq(U1RX), base(0x0008)) uart1_rx_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* ISR_H */