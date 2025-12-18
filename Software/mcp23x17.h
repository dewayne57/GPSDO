/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * MCP23x17 I2C I/O Expander register definitions.
 * - Supports both banked and sequential register modes.
 * - Default is sequential mode after power-on reset.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MCP23X17_H
#define MCP23X17_H
#include <xc.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

    /*
     * The MCP23x17 can operate in either banked registers or sequential
     * register addresses.  This is set by the configuration register, and
     * defaults to sequential addressing when powered on or after a reset.
     *
     * To change the code to operate in banked mode, define the symbol
     * MCP23X17_BANKED.
     */
#ifdef MCP23X17_BANKED

    /*
     * Controls the direction of data I/O.  Each bit corresponds to one pin of the
     * device I/O port. If a bit is high, the I/O pin is input.  When clear, the
     * pin is output. The pin values are represented in decimal as the value of
     * the pin when set to 1.
     */
#define IODIRA 0X00
#define IODIRB 0x10

    /*
     * Defines the input polarity of the port pins.  Each bit in this register
     * corresponds to the pin in the port.  When the bit is clear, the
     * corresponding pin will represent the current value of the pin.  When set
     * the value of the pin is inverted (1 -> 0, and 0 -> 1).
     */
#define IPOLA 0x01
#define IPOLB 0x11

    /*
     * Controls the interrupt-on-change mode for the I/O port.  Each bit in the
     * register corresponds to a bit in the port.  If clear, the pin does not
     * generate an interrupt when its value changes.  If set, a change detected
     * on the pin generates an interrupt based on the additional settings of the
     * DEFVAL and INTCON registers. If this is set, these additional registers
     * must be configured.
     */
#define GPINTENA 0x02
#define GPINTENB 0x12

    /*
     * Establishes the default value for pin comparisons on a port when the
     * interrupt-on-change feature is enabled.  If interrupt-on-change is
     * enabled, and the INTCON bit is also set, then the definition of "changed"
     * means the ports I/O bit value is different from the default value in this
     * register.  If the associated INTCON bit is clear, then the definition of
     * "changed" means the value is different from it's previous value.
     */
#define DEFVALA 0x03
#define DEFVALB 0x13

    /*
     * The INTCON register controls how the associated pin value is compared for
     * the interrupt-on-change feature. If a bit is set, the corresponding I/O
     * pin is compared against the associated bit in the DEFVAL register. If a
     * bit value is clear, the corresponding I/O pin is compared against the
     * previous value.
     */
#define INTCONA 0x04
#define INTCONB 0x14

    /*
     * The configuration register controls the operation of the MCP23x17 device.
     * This register appears at different addresses based on the power-on-reset
     * state and the banked configuration.  Additionally, there are two addresses
     * assigned to the same register in both modes.  This is denoted as a low
     * (_L) address and a high (_H) address.  If you refer to the register by
     * it's generic symbol, then the first address is used.
     *
     * Because these symbols define the register locations when in banked mode,
     * and it is essential to set these bits in the IOCON register after power
     * on reset (which is at a different address), there is a symbolic definition
     * of its address after POR as well (_POR).
     */
#define IOCON 0x05
#define IOCON_POR 0x0A
#define IOCON_L 0x05
#define IOCON_H 0x15

    /*
     * The GPPU register controls the pull-up resistors for the port pins. If a
     * bit is set and the corresponding pin is configured as an input, the
     * corresponding port pin is internally pulled up with a 100 k? resistor.
     */
#define GPPUA 0x06
#define GPPUB 0x16

    /*
     * The INTF register reflects the interrupt condition on the port pins of
     * any pin that is enabled for interrupts via the GPINTEN register. A 1 bit
     * indicates that the associated pin caused the interrupt.
     *
     * This register is 'read-only'. Writes to this register will be ignored.
     */
#define INTFA 0x07
#define INTFB 0x17

    /*
     * The INTCAP register captures the GPIO port value at the time the interrupt
     * occurred. The register is 'read-only' and is updated only when an interrupt
     * occurs. The register will remain unchanged until the interrupt is cleared
     * via a read of INTCAP or GPIO register.
     */
#define INTCAPA 0x08
#define INTCAPB 0x18

    /*
     * The GPIO register reflects the value on the port.  Reading from this
     * register reads the port. Writing to this register modifies the Output
     * Latch (OLAT) register.
     */
#define GPIOA 0x09
#define GPIOB 0x19

    /*
     * The OLAT register provides access to the output latches. A read from this
     * register results in a read of the OLAT and not the port itself. A write
     * to this register modifies the output latches that modifies the pins
     * configured as outputs.
     */
#define OLATA 0x0A
#define OLATB 0x1A

#else
/*
 * Controls the direction of data I/O.  Each bit corresponds to one pin of the
 * device I/O port. If a bit is high, the I/O pin is input.  When clear, the
 * pin is output. The pin values are represented in decimal as the value of
 * the pin when set to 1.
 */
#define IODIRA 0X00
#define IODIRB 0x01

/*
 * Defines the input polarity of the port pins.  Each bit in this register
 * corresponds to the pin in the port.  When the bit is clear, the
 * corresponding pin will represent the current value of the pin.  When set
 * the value of the pin is inverted (1 -> 0, and 0 -> 1).
 */
#define IPOLA 0x02
#define IPOLB 0x03

/*
 * Controls the interrupt-on-change mode for the I/O port.  Each bit in the
 * register corresponds to a bit in the port.  If clear, the pin does not
 * generate an interrupt when its value changes.  If set, a change detected
 * on the pin generates an interrupt based on the additional settings of the
 * DEFVAL and INTCON registers. If this is set, these additional registers
 * must be configured.
 */
#define GPINTENA 0x04
#define GPINTENB 0x05

/*
 * Establishes the default value for pin comparisons on a port when the
 * interrupt-on-change feature is enabled.  If interrupt-on-change is
 * enabled, and the INTCON bit is also set, then the definition of "changed"
 * means the ports I/O bit value is different from the default value in this
 * register.  If the associated INTCON bit is clear, then the definition of
 * "changed" means the value is different from it's previous value.
 */
#define DEFVALA 0x06
#define DEFVALB 0x07

/*
 * The INTCON register controls how the associated pin value is compared for
 * the interrupt-on-change feature. If a bit is set, the corresponding I/O
 * pin is compared against the associated bit in the DEFVAL register. If a
 * bit value is clear, the corresponding I/O pin is compared against the
 * previous value.
 */
#define INTCONA 0x08
#define INTCONB 0x09
/*
 * The configuration register controls the operation of the MCP23x17 device.
 * This register appears at different addresses based on the power-on-reset
 * state and the banked configuration.  Additionally, there are two addresses
 * assigned to the same register in both modes.  This is denoted as a low
 * (_L) address and a high (_H) address.  If you refer to the register by
 * it's generic symbol, then the first address is used.
 */
#define IOCON 0x0A
#define IOCON_L 0x0A
#define IOCON_H 0x0B

/*
 * The GPPU register controls the pull-up resistors for the port pins. If a
 * bit is set and the corresponding pin is configured as an input, the
 * corresponding port pin is internally pulled up with a 100 k? resistor.
 */
#define GPPUA 0x0C
#define GPPUB 0x0D

/*
 * The INTF register reflects the interrupt condition on the port pins of
 * any pin that is enabled for interrupts via the GPINTEN register. A 1 bit
 * indicates that the associated pin caused the interrupt.
 *
 * This register is 'read-only'. Writes to this register will be ignored.
 */
#define INTFA 0x0E
#define INTFB 0x0F

/*
 * The INTCAP register captures the GPIO port value at the time the interrupt
 * occurred. The register is 'read-only' and is updated only when an interrupt
 * occurs. The register will remain unchanged until the interrupt is cleared
 * via a read of INTCAP or GPIO register.
 */
#define INTCAPA 0x10
#define INTCAPB 0x11

/*
 * The GPIO register reflects the value on the port.  Reading from this
 * register reads the port. Writing to this register modifies the Output
 * Latch (OLAT) register.
 */
#define GPIOA 0x12
#define GPIOB 0x13

/*
 * The OLAT register provides access to the output latches. A read from this
 * register results in a read of the OLAT and not the port itself. A write
 * to this register modifies the output latches that modifies the pins
 * configured as outputs.
 */
#define OLATA 0x14
#define OLATB 0x15
#endif

    /*
     * I/O Direction registers.
     *
     * Controls the direction of data I/O.  Each bit corresponds to one pin of the
     * device I/O port. If a bit is high, the I/O pin is input.  When clear, the
     * pin is output. The pin values are represented in decimal as the value of
     * the pin when set to 1.
     */
#define IO7 128
#define IO6 64
#define IO5 32
#define IO4 16
#define IO3 8
#define IO2 4
#define IO1 2
#define IO0 1

    /*
     * Input Polarity Registers.
     *
     * Defines the input polarity of the port pins.  Each bit in this register
     * corresponds to the pin in the port.  When the bit is clear, the
     * corresponding pin will represent the current value of the pin.  When set
     * the value of the pin is inverted (1 -> 0, and 0 -> 1).
     */
#define IP7 128
#define IP6 64
#define IP5 32
#define IP4 16
#define IP3 8
#define IP2 4
#define IP1 2
#define IP0 1

    /*
     * Interrupt-on-change control registers.
     *
     * Controls the interrupt-on-change mode for the I/O port.  Each bit in the
     * register corresponds to a bit in the port.  If clear, the pin does not
     * generate an interrupt when its value changes.  If set, a change detected
     * on the pin generates an interrupt based on the additional settings of the
     * DEFVAL and INTCON registers. If this is set, these additional registers
     * must be configured.
     */
#define GPINT7 128
#define GPINT6 64
#define GPINT5 32
#define GPINT4 16
#define GPINT3 8
#define GPINT2 4
#define GPINT1 2
#define GPINT0 1

    /*
     * Default compare register for interrupt-on-change.
     *
     * Establishes the default value for pin comparisons on a port when the
     * interrupt-on-change feature is enabled.  If interrupt-on-change is
     * enabled, and the INTCON bit is also set, then the definition of "changed"
     * means the ports I/O bit value is different from the default value in this
     * register.  If the associated INTCON bit is clear, then the definition of
     * "changed" means the value is different from it's previous value.
     */
#define DEF7 128
#define DEF6 64
#define DEF5 32
#define DEF4 16
#define DEF3 8
#define DEF2 4
#define DEF1 2
#define DEF0 1

    /*
     * Interrupt Control Registers
     *
     * The INTCON register controls how the associated pin value is compared for
     * the interrupt-on-change feature. If a bit is set, the corresponding I/O
     * pin is compared against the associated bit in the DEFVAL register. If a
     * bit value is clear, the corresponding I/O pin is compared against the
     * previous value.
     */
#define IOC7 128
#define IOC6 64
#define IOC5 32
#define IOC4 16
#define IOC3 8
#define IOC2 4
#define IOC1 2
#define IOC0 1

    /*
     * Configuration control register.
     *
     * BANK - If 1, the registers are separated into banks.  Define the symbol
     *        MCP23X17_BANKED if banked support is desired.  Defaults to
     *        sequential.
     * MIRROR - If 1, the two interrupt output pins are connected and an interrupt
     *        generated from either port will activate both pins.  If 0, the
     *        interrupt pins are split and assigned to each port.
     * SEQOP - If 1, the register address pointer is not incremented by 1 after
     *        each read or write operation.  If 0, the register address is
     *        incremented by 1 after each read or write operation to a register.
     * DISSLW - If 1, slew rate control for the SDA pint is disabled.  If 0,
     *        slew rate control is enabled.
     * HAEN   - If 1, the address pins are enabled, and if 0, the address pins
     *        are disabled.  This only applies to the MCP23S17 (SPI version)
     *        and the MCP23017 (I2C version) address pins are always enabled.
     * ODR    - If 1, the interrupt output pin is configured as an open-drain
     *        output.  If 0, the pin is configured as totem-pole drive.
     * INTPOL - If 1, the interrupt output is assumed to be active when HIGH.
     *        If 0, the interrupt output is assumed to be active when LOW.
     */
#define BANK 128
#define MIRROR 64
#define SEQOP 32
#define DISSLW 16
#define HAEN 8
#define ODR 4
#define INTPOL 2

    /*
     * Pull-Up Configuration Register
     *
     * These pins correspond to the port input pins and control if pull-up
     * resistors are active or not.  If the pin is defined as an output, this
     * setting has no effect.
     */
#define PU7 128
#define PU6 64
#define PU5 32
#define PU4 16
#define PU3 8
#define PU2 4
#define PU1 2
#define PU0 1

    /*
     * Interrupt Flag Register
     *
     * This register is read-only, and writes to it are ignored.  These bits are
     * set (1) when the associated pin has generated an interrupt because it was
     * changed.  If unset (0), then no change has been observed on the pin.
     *
     * Note, to clear the interrup flag, either read the INTCAP register or the
     * GPIO port.
     */
#define INT7 128
#define INT6 64
#define INT5 32
#define INT4 16
#define INT3 8
#define INT2 4
#define INT1 2
#define INT0 1

    /*
     * Interrupt Capture Register
     *
     * These bits correspond to the port pins and are set when an interrupt has
     * been generated (captured).  They are only cleared by reading this port
     * or reading the port via the GPIO register.
     */
#define ICP7 128
#define ICP6 64
#define ICP5 32
#define ICP4 16
#define ICP3 8
#define ICP2 4
#define ICP1 2
#define ICP0 1

    /*
     * Port Register
     *
     * Each bit in this register corresponds to the pin of the port.
     */
#define GP7 128
#define GP6 64
#define GP5 32
#define GP4 16
#define GP3 8
#define GP2 4
#define GP1 2
#define GP0 1

    /*
     * Output Latch Register
     *
     * These bits correspond to the bits of the output latch register.
     */
#define OL7 128
#define OL6 64
#define OL5 32
#define OL4 16
#define OL3 8
#define OL2 4
#define OL1 2
#define OL0 1

    /*
     * The definition of the control byte that is used to address the MCP23x17
     * and define the operation as a register read or register write.
     *
     * The initial value is used to initialize the control structure by setting
     * the value. The address is then set in the control byte using the "address"
     * fields of the "bits" structure.  If the operation is a read, then set the
     * value of the "read" bit to 1, otherwise 0 for a write.
     *
     * For example,
     * control.value = CONTROL_INIT_VALUE;
     * control.bits.address = 3;
     * control.bits.read = 0;
     */
#define CONTROL_INIT_VALUE 0x40

    union
    {
        struct
        {
            unsigned : 4;
            unsigned address : 3;
            unsigned read : 1;
        } bits;

        unsigned char value;
    } MCP23X16_CTL;

    /*
     * Compute the control byte to address the MCP23x17 device.
     *
     * address -- The address to use to signal the appropriate device.  The
     *            address is a 3-bite value, and can range from 0 - 7 only.
     * read ----- True (non-zero) if the device is to be read, and False (zero)
     *            if the device is to be written.
     *
     * returns:   the control byte to send to the device.
     */
    unsigned char Mcp23x17GetControl(unsigned char address, unsigned char read);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MCP23X17_H */
