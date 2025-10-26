# GPSDO (GPS Disciplined Oscillator)

## What is a GPS disciplined oscillator?

A GPS disciplined oscillator (GPSDO) is an electronic timing device that combines a precise local oscillator (typically an OCXO or high-quality TCXO) with timing and frequency information derived from GPS satellites. The GPS receiver provides an accurate reference (usually a 1 pulse-per-second, 1 PPS, and timing messages), and the GPSDO continuously steers the local oscillator so its long-term frequency and phase match the GPS reference.

Key points:

- Components: a GPS receiver, a stable local oscillator (OCXO/TCXO), a disciplining/control circuit (often a microcontroller or PLL), and output drivers for standard reference signals.
- Outputs: common outputs are 10 MHz and 1 PPS; some units provide additional logic-level timing signals or frequency multiples.
- How it works: the system measures the difference between the local oscillator and the GPS reference and applies small corrections (frequency/phase adjustments) to the oscillator to remove long-term drift. The correction loop is designed to preserve the short-term stability of the local oscillator while achieving excellent long-term accuracy tied to GPS.
- Holdover: if GPS signals are lost (blocked or jammed), a good GPSDO enters "holdover" mode where the local oscillator continues to run using the last-applied corrections; performance in holdover depends on the quality of the local oscillator.

Why use one:

- Provides an accurate, traceable frequency/time reference for labs, communications equipment, test instrumentation, SDRs, and embedded systems.
- Improves long-term stability to GPS-level accuracy while keeping low short-term phase noise thanks to a high-quality local oscillator.

Typical performance: while exact specs vary, a well-built GPSDO will provide parts-per-billion (ppb) to parts-per-trillion (ppt) long-term frequency accuracy tied to GPS, and low phase noise inherited from the OCXO for short-term stability. Holdover accuracy depends on oscillator drift and ambient conditions.

## Hardware

This project contains the hardware (KiCad) and documentation for building a GPSDO.  The GPSDO design uses a
Microchip PIC 18F27Q43 microcontroller running at 64mHz.  Several different OCXO (Oven-controlled crystal 
oscillators) can be used.  The design allows for the use of the following OCXO units: 
- Isotemp 131 series 
- Isotemp 143 (Package A) Series 
- Isotemp 143 (Package N) Series 
- Vectron C4550 Series
- MTronPTI )XO512x series 

### Voltage Selection 
The Chosen OCXO may operate from 5V or 12V, which is slected by a jumper on the board.  No support has been 
added for other voltage operation.  

### Sine Output Support
The hardware includes a provision for a sine-to-square wave conversion circuit to allow the use of Sine
wave output OCXO units.  This conversion circuit can be bypassed by a jumper if a CMOS/TTL output unit 
is used.

### GPS Support 
This design is based on the use of a UBlox Neo M8 GPS module, either as a plugin module or by adding the 
UBlox M8 chip to the board along with the appropriate logic level converter. 
