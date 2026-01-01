# UART3 USB Interface Configuration

## Overview
UART3 has been configured for USB serial communication on the PIC18F27Q43.

## Pin Assignments
- **RB2**: USB_TX (UART3 transmit output)
- **RB3**: USB_RX (UART3 receive input)

## Configuration Details
- **Baud Rate**: 115200 (configurable via `usb_reconfigure()`)
- **Data Format**: 8-bit, no parity, 1 stop bit
- **Buffer Size**: 256 bytes circular receive buffer
- **Clock Source**: System Fosc (64 MHz)
- **BRG Value**: 139 (for 115200 baud at 64 MHz)

## PPS (Peripheral Pin Select) Routing
```c
U3RXPPS = 0x0B;  // RB3 (pin 11) -> UART3 RX input
RB2PPS = 0x33;   // UART3 TX -> RB2 output
```

## API Functions (usb.h)
### Initialization
- `void usb_init(void)` - Initialize UART3 at 115200 baud
- `void usb_reconfigure(uint32_t baudrate)` - Change baud rate

### Transmit
- `void usb_send_char(char c)` - Send single character
- `void usb_send_string(const char *str)` - Send null-terminated string

### Receive
- `void usb_buffer_put_char(char c)` - Store received character (called from ISR)
- `char usb_buffer_get_char(void)` - Get character from buffer
- `bool usb_buffer_has_data(void)` - Check if data available
- `uint8_t usb_buffer_count(void)` - Get number of buffered characters

## Interrupt Service Routine
UART3 RX interrupt handler is defined in [isr.c](isr.c):
```c
void __interrupt(irq(U3RX), high_priority) uart3_rx_isr(void)
```

The ISR handles:
- Receive buffer overflow recovery
- Character buffering via circular buffer
- Interrupt flag clearing

## Files Modified/Created
1. **usb.h** - USB/UART3 interface header
2. **usb.c** - USB/UART3 implementation
3. **isr.c** - Added UART3 RX ISR
4. **config.c** - Added `usb_init()` call
5. **nbproject/Makefile-default.mk** - Added usb.c to build

## Current UART Usage Summary
- **UART1**: External serial/bootloader (RB3 TX, RB4 RX) - 115200 baud
- **UART2**: GPS communication (RB5 TX, RB4 RX) - 9600 baud
- **UART3**: USB interface (RB2 TX, RB3 RX) - 115200 baud

## Example Usage
```c
#include "usb.h"

// Send string to USB
usb_send_string("Hello USB!\r\n");

// Check for received data
if (usb_buffer_has_data()) {
    char c = usb_buffer_get_char();
    // Process received character
}

// Change baud rate
usb_reconfigure(9600);
```
