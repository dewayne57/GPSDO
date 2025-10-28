# GPSDO (GPS Disciplined Oscillator)

## What is a GPS disciplined oscillator?

A GPS-disciplined oscillator (GPSDO) is a local frequency source (VCO/OCXO) whose long‑term frequency and phase are locked to GNSS timing signals (commonly the 1PPS output of a GPS receiver). The GPS provides an absolute timing reference; the local oscillator provides low short‑term noise and a usable output (typically 10 MHz and 1 PPS). A control loop continuously measures the difference between the local oscillator and GPS timing, computes a correction, and applies that correction to the oscillator control input so the local output inherits GPS long‑term accuracy while retaining the oscillator's short‑term stability.

## Typical components:

- Local reference oscillator (VCO, OCXO)
- GPS receiver with a disciplined 1PPS output (u‑blox M8 here)
- Measurement and control electronics (microcontroller + timer, DAC or VCO control)
- Optional holdover circuitry/logic for operation when GPS is lost

## Typical outputs and performance:

- 10 MHz square wave and a 1 PPS logic pulse
- Long‑term accuracy tied to GPS (10⁻⁹ to 10⁻¹³ depending on design and receiver)
- Short‑term stability determined by the oscillator and loop bandwidth (design dependent)

## Why use one:

- Provides an accurate, traceable frequency/time reference for labs, communications equipment, test instrumentation, SDRs (Software Defined Radio), and embedded systems
- Combines GPS long-term accuracy with OCXO short-term stability and low phase noise
- Enables holdover operation when GPS is temporarily unavailable
- Can be used as a NIST tracable frequency standard for calibration of counters, oscilloscopes, and 
other time measurement equipment. 

# Hardware

This project contains the hardware (KiCad) and documentation for building a GPSDO. The GPSDO design uses a
Microchip PIC 18F27Q43 microcontroller running at 64mHz.

## OCXO Selections

Several different OCXO (Oven-controlled crystal oscillators) can be used. The design was based on specific commonly available units, but others could be used as well with the appropriate pin connections. The
ability of the GPSDO to adjust the OCXO output using a VCO control voltage is absolutely required. An OCXO
must supply a VCO control input.

Many OCXO units provide an on-board voltage reference output. If the OCXO has its own voltage reference,
that reference is used as the reference for the VCO DAC. If a OCXO is used that does NOT have a voltage
reference, an on-board 4.096V reference can be used instead. Use of the OCXO voltage reference or 4.096
volt reference is set by jumper on the board.

The design is based on the following OCXO units:

- Isotemp 131 series
- Isotemp 143 (Package A) Series
- Isotemp 143 (Package N) Series
- Vectron C4550 Series
- MTronPTI XO512x series
- Oscilloquartz 8663 series
- Bliley N47 or NV47 series

## Voltage References

Many OCXO modules include an on-board voltage reference that can be used to bias the OCXO voltage
controlled oscillator to fine tune the frequency. Typical ranges for the VCO adjustment is +/-
20 ppm (parts per million). The VCO control voltage is usally mid-range of the reference voltage
to adjust the output frequency to the nominal value (10 MHz).

5V modules typically have a voltage reference of around 4 to 4.5 volts, where the center of the adjustment
range is about 2V. Giving an adjustment range of +/- 2V (centered about +2V), therefore 0 - 4v. 12V modules
typically have a reference between 8-10 volts, with a center of the adjustment range around 4-5 volts.

Since the DAC and ADC circuits operate on 5V, the use of 12V modules requires the reference to be scaled
by dividing it by 2. The VCO adjustment must also be scaled by multiplication by 2. Since the VCO
voltage is generated from a 12-bit DAC, the adjustment units are 1/4096 of the reference voltage. If the
OCXO reference is divided by 2, the adjustment steps are 1mv increments (assuming a 4.096V reference).
If the VCO adjustment is multiplied by 2, the adjustment increments are 2mV. When you consider that
most OCXO modules have a total adjustment range of just a few PPM (Parts Per Million), this gives an
extremely fine adjustment, even when multipled by 2.

For example, the Bliley N47/NV47 series offers an adjustment range of +/- .5 PPM. At 10mHz, that
is an adjustment of +/- 5 Hz. The ISOTEMP 143 series offers an adjustment range of about +/- 1 PPM,
or +/- 10 Hz. If you divide that range by 4096 steps, the adjustment step size for the Bliley is
.00244 Hz per step.

## Voltage Selection

The Chosen OCXO may operate from 5V or 12V, which is slected by a jumper on the board. No support has been
added for other voltage operation.

Even if the chosen OCXO does not provide a VRef output, the GPSDO unit can supply the reference voltage of
4.096 volts. Even if a 12V unit is used with a higher voltage adjustment, the VCO adjstment voltage can be
scaled as needed.

## Sine Output Support

The hardware includes a provision for a sine-to-square wave conversion circuit to allow the use of Sine
wave output OCXO units. This conversion circuit can be bypassed by a jumper if a CMOS/TTL output unit
is used.

The sine conversion circuit offsets the sine output by 2.5V then feeds that to a high speed comparator
that triggers at 2.5V. The output of the sine to square converter, as well as the output from any
square wave OCXO, is fed to a Schmitt trigger to square up the signal.

## GPS Support

This design is based on the use of a UBlox Neo M8 GPS module, either as a plugin module or by adding the
UBlox M8 chip to the board along with the appropriate logic level converter.

To ensure an acceptable GPS lock, the use of an external antenna is required. The unit will provide
an SMA connector on the rear panel to allow an external GPS antenna to be connected.

When a GPS signal has been acheived using 3 or more satellites, an LED on the front panel labeled "GPS" will be illuminated. This LED indicates that at least 3 satellites are being tracked and ensures the highest
accuracy.

If the GPS signal is lost for any reason, a "HOLDOVER" LED will be illuminated indicating that the GPSDO
will continue generating the 10mHz reference locally by using the last computed GPS adjustment value.

# Digital disciplining loop (PIC18F27Q43 + u‑blox M8)

## Overview

This design implements the disciplining loop entirely in firmware. The PIC18F27Q43 uses a signal‑measurement timer (input capture) to count local oscillator cycles between consecutive 1PPS edges from the u‑blox M8. The firmware converts those timer captures into a frequency/phase error, filters the measurements to reduce GPS jitter, and drives a DAC (or control voltage) to pull the VCO toward GPS time.

## Measurement

On each 1PPS edge the PIC captures the current timer count. The difference between successive captures (accounting for timer wrap) gives the number of local ticks per second, N.

- Expected count N₀ corresponds to the nominal oscillator frequency (N₀ ≈ f_local_nominal / 1 Hz)
- Fractional frequency error e = (N − N₀) / N₀ (signed). This is the primary loop input
- Phase errors can be derived from sub‑second timing offsets if needed

## Filtering and outlier rejection

A single 1PPS measurement contains GPS jitter; the firmware reduces noise before applying corrections:

- Average or decimate multiple seconds of error measurements (typical averaging window: 8–64 s depending on desired bandwidth)
- Use median filtering or simple outlier rejection to ignore grossly corrupted 1PPS samples (e.g., from receiver reacquisition)
- Implement a moving average or low‑pass IIR on the error signal before feeding the controller

## Controller (digital PI)

The firmware uses a PI controller: u[n] = Kp·e_filtered[n] + Ki·integral

- Tune Kp and Ki so loop bandwidth is low relative to GPS‑noise (slow loop, e.g., bandwidth ≪ 0.1 Hz)
- The integral term forces long-term frequency alignment to GPS; the proportional term provides damping and faster response to changes
- Implement integral anti‑windup: clamp the integrator when DAC output saturates and optionally back‑calculate to prevent overshoot
- Apply rate limits or slew limiting to the DAC command to avoid injecting fast disturbances into the VCO

## DAC scaling and calibration

- Map controller output to DAC codes with an offset and slope calibrated against the VCO tuning characteristic
- Store calibration constants in nonvolatile memory
- Provide a manual trim and a one‑point or two‑point calibration routine to set the control range and center

## Implementation details and robustness

- **Fixed‑point math**: The microcontroller uses fixed-point integer arithmetic (32‑bit accumulators) to implement the controller without floating point overhead; explicit scaling factors are documented in the firmware
- **Timer wrap handling**: Capture differences handle timer wrap/overflow safely using unsigned arithmetic and modulo subtraction
- **GPS status check**: Check u‑blox status flags (1PPS locked / time valid). While receiver is not locked, ignore measurements or switch to a safe startup mode
- **Outlier rejection**: Measurements that differ sharply from recent history (GPS glitch, bad pulse) are rejected or down‑weighted so a single bad PPS does not drive a large correction
- **Loop timing**: The main correction update runs once per second (on the 1PPS boundary), while internal averaging may operate over many seconds to minutes depending on the desired loop bandwidth. This preserves the OCXO's superior short‑term stability while locking its long‑term frequency to GPS
- **Logging**: Measurements and control outputs are logged periodically for debugging and tuning

## Holdover and recovery

When GPS is lost, the firmware enters holdover mode:

- The last good control value is held in a non-volatile static RAM module, allowing the GPSDO to continue in holdover mode across a power cycle
- Optional slow evolution of the control value using a drift model or temperature compensation maintains frequency until GPS returns
- On GPS reacquisition, a soft re‑engage (ramp integrator or temporarily reduce Ki) avoids large corrective transients
- Holdover quality depends on the OCXO stability and any implemented drift compensation

## Practical tuning hints

- Start with very small Ki (slow integral) and modest Kp; observe DAC steps and count variation
- Increase averaging window if the output is noisy; reduce it if loop reacts too slowly to real drift
- Verify limits and anti‑windup by commanding extremes (temperature chamber, manual DAC) to confirm predictable behavior

## Example high‑level loop (pseudo code)

1. Wait for 1PPS capture; read timer
2. Compute delta ticks (account for wrap)
3. Compute fractional error e = (delta − N₀) / N₀ in fixed point
4. Filter e (moving average / median)
5. Compute PI output; apply clamps and anti‑windup
6. Convert to DAC code using calibration; write DAC
7. Log values and repeat

# Front Panel

The front panel of the GPSDO unit contains three BNC connectors that output TTL level signals based on 5V logic plus a 50 ohm output suitable for use in instrumentation.

## Outputs

- A 10mHz square wave signal that is obtained from the OCXO, locked to GPS, and buffered at the selected TTL levels (not 50 ohm). This signal will be driven by a high speed TTL gate and level translator. The output is standard TTL.
- A 10mHz square wave signal also obtained from the OCXO and buffered. This output will be a standard 50 ohm instrument output signal.
- A 1 pulse-per-second (1 Hz) square wave with approximately 50% duty cycle. The signal rising edge represents the start of the 1PPS signal.

## LED Indicators

Several LEDs will be placed across the front panel. These are:

- GPS: turned on when the GPS receiver is tracking 3 or more satellites.
- HOLDOVER: Turned on when the GPS receiver is either not receiving any signal, or the receiver is tracking less than 3 satellites.
- LOW: Illuminated if the measured output frequency from the OCXO is below 10MHz minus the lock range (based on the accumulated signal measurement counter on the edge of the 1PPS clock).
- HIGH: Illuminated if the measured output frequency from the OCXO is above 10MHz (plus the lock range).
- LOCK: turned on whenever the error count in the digital control loop is less than the lock range. For
  example, if the lock range is 100 counts this represents an accuracy of .001%. The lock range can be adjusted using DIP switches on the board.
- POWER: Turned on whenever the GPSDO is running.
- FAULT: If the microcontroller detects any failures.

## LCD Display

The GPSDO will also use a 24X2 backlit LCD display that will show the current GMT and local date and time, plus any error or diagnostic messages.

The lines will be formatted as:
GMT: YYYY/MM/DD HH:MM:SS
LCL: YYYY/MM/DD HH:MM:SS

# Rear Panel

The rear panel will consist of the IEC power connector with the fuse for protection, as well as the GPS antenna connector.
