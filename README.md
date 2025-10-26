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

This project contains the hardware (KiCad) and documentation for building a GPSDO. The GPSDO design uses a
Microchip PIC 18F27Q43 microcontroller running at 64mHz. Several different OCXO (Oven-controlled crystal
oscillators) can be used. The design allows for the use of the following OCXO units:

- Isotemp 131 series
- Isotemp 143 (Package A) Series
- Isotemp 143 (Package N) Series
- Vectron C4550 Series
- MTronPTI )XO512x series

### Voltage Selection

The Chosen OCXO may operate from 5V or 12V, which is slected by a jumper on the board. No support has been
added for other voltage operation.

### Sine Output Support

The hardware includes a provision for a sine-to-square wave conversion circuit to allow the use of Sine
wave output OCXO units. This conversion circuit can be bypassed by a jumper if a CMOS/TTL output unit
is used.

### GPS Support

This design is based on the use of a UBlox Neo M8 GPS module, either as a plugin module or by adding the
UBlox M8 chip to the board along with the appropriate logic level converter.

To ensure an acceptable GPS lock, the use of an external antenna is required. The unit will provide
an SMA connector on the rear panel to allow an external GPS antenna to be connected.

When a GPS signal has been acheived using 3 or more satellites, an LED on the front panel labeled "GPS" will be illuminated. This LED indicates that at least 3 satellites are being tacked and ensures the highest
accuracy.

If the GPS signal is lost for any reason, a "HOLDOVER" LED will be illuminated indicating that the GPSDO
will continue generating the 10mHz reference locally by using the last computed GPS adjustment value.

## Digital disciplining loop (PIC18F27Q43 + u‑blox M8)

This GPSDO implements the disciplining loop in the digital domain using the PIC18F27Q43's signal/timing peripherals together with the u‑blox M8 1PPS output. The microcontroller measures the timing relationship between the local oscillator and the GPS 1PPS and computes a digital correction which is converted to an analog control voltage for the VCO/OCXO.

### How the loop works (high level):

- Measurement: the PIC's signal measurement/timer peripheral captures the interval between successive 1PPS edges (or counts local oscillator cycles between PPS edges). That capture yields a digital count proportional to the local oscillator frequency/phase relative to the GPS 1PPS.
- Error calculation: the firmware compares the measured count to the expected count (the target number of local oscillator ticks per second). The difference (measured - expected) gives a frequency error (or fractional frequency error when normalized by the expected count).
- Filtering and averaging: raw 1PPS measurements include jitter from the GPS receiver and noise from the capture hardware. The firmware applies robust filtering (moving average, median/trim trimming, or exponential smoothing) to reduce the influence of single-pulse jitter before it reaches the controller. This filtering sets the loop's effective bandwidth and is configurable in firmware.
- Control law: a digital controller (typically a PI — proportional + integral — loop) converts the filtered frequency error into a control value. The integral term forces long-term frequency alignment to GPS, while the proportional term provides damping and faster response to changes.
- Output conversion: the computed control value is scaled and sent to the VCO control input via a DAC. The DAC output is limited and smoothed to avoid introducing spurious steps or wideband noise into the oscillator.

### Implementation notes / robustness features:

- Fixed-point math: the microcontroller uses fixed-point arithmetic to implement the controller (no floating-point needed), with explicit scaling factors documented in the firmware.
- Anti-windup and limits: integral accumulation is bounded and anti-windup logic prevents the integrator from saturating when the DAC is at its limit.
- Outlier rejection: measurements that differ sharply from recent history (GPS glitch, bad pulse) can be rejected or down-weighted so a single bad PPS does not drive a large correction.
- Timer overflow and wrap: capture differences handle timer wrap/overflow safely and use sufficiently wide counters to avoid ambiguity between pulses.
- Loop timing: the main correction update runs once per second (on the 1PPS boundary), while internal averaging may operate over many seconds to minutes depending on the desired loop bandwidth. This preserves the OCXO's superior short-term stability while locking its long-term frequency to GPS.

### Holdover behavior:

When 1PPS is lost, the firmware enters holdover: the last good control value is held in a non-volatile 
static ram module.  This also allows the GPSDO to continue in holdover mode across a power cycle. The quality of holdover depends entirely on the OCXO stability and any implemented drift compensation.


