/* SPDX-License-Identifier: Apache-2.0 */
/* SMT1-based 10 MHz pulse counter with 1 PPS capture
 * - SMT1SIGPPS -> RC1 (10 MHz source)
 * - SMT1WINPPS -> RC0 (1PPS trigger)
 * Capture result read from SMT1CPR (24-bit) when SMT1PRAIF is set.
 */

#include <xc.h>
#include <stdint.h>
#include "config.h"

volatile uint32_t smt_last_count = 0;
volatile int32_t  smt_last_error = 0;
volatile bool  smt_new_capture = false; /* set in ISR when capture completes */

/* Return true if a new capture has been recorded (callers should clear it when handled) */
bool smt_capture_available(void)
{
    return smt_new_capture;
}

/* Clear the capture-available flag (after handling) */
void smt_clear_capture(void)
{
    smt_new_capture = false;
}

void smt_init(void)
{
    // Enable SMT1 module in PMD
    PMD1bits.SMT1MD = 0;

    // Map 10MHz signal (RC1) to SMT1 signal input and 1PPS (RC0) to window
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // unlock

    SMT1SIGPPS = 0x11; // RC1 -> SMT1 signal input
    SMT1WINPPS = 0x10; // RC0 -> SMT1 window input (1PPS)

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // lock

    // Configure SMT1 to count pulses and capture on window event
    SMT1CON1bits.MODE = 0x2; // Counter mode (counts SMT input edges)
    SMT1CON0bits.STP = 0;    // don't stop
    SMT1CON0bits.EN = 1;     // enable module

    // Clear any pending flags
    SMT1PRAIF = 0;
    SMT1PWAIF = 0;
    SMT1IF = 0;

    // Enable period result interrupt (capture event)
    SMT1PRAIE = 1;
}

/* Called from ISR when a capture completes (SMT1PRAIF set) */
void smt_handle_capture(void)
{
    uint32_t v = 0;
    /* Read low byte first to latch upper bytes coherently */
    uint8_t l = SMT1CPRL;
    uint8_t h = SMT1CPRH;
    uint8_t u = SMT1CPRU;
    v = ((uint32_t)u << 16) | ((uint32_t)h << 8) | (uint32_t)l;
    smt_last_count = v;
    smt_last_error = (int32_t)((int32_t)v - (int32_t)10000000);

    /* Indicate to the rest of the system that a new capture is available */
    smt_new_capture = true;

    // Clear the capture flag
    SMT1PRAIF = 0;
}

uint32_t smt_get_last_count(void)
{
    /* Protect 32-bit read from tearing while ISR updates */
    uint8_t gie_saved;
    uint8_t giel_saved;
    CRITICAL_SECTION_ENTER(gie_saved, giel_saved);
    uint32_t v = smt_last_count;
    CRITICAL_SECTION_EXIT(gie_saved, giel_saved);
    return v;
}

int32_t smt_get_last_error(void)
{
    uint8_t gie_saved;
    uint8_t giel_saved;
    CRITICAL_SECTION_ENTER(gie_saved, giel_saved);
    int32_t v = smt_last_error;
    CRITICAL_SECTION_EXIT(gie_saved, giel_saved);
    return v;
}
