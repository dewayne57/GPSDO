# GPSDO (GPS Disciplined Oscillator)

This repository contains a hardware and firmware design for a GPS-disciplined oscillator (GPSDO). The system uses
a local oven-controlled crystal oscillator (OCXO) disciplined by a GPS reference (1PPS and time) via a digital control
loop implemented on a PIC microcontroller.

Purpose: provide a compact, buildable GPSDO that combines the short-term stability of an OCXO with the long-term
accuracy of GPS. The design favors through-hole assembly and flexible OCXO support so hobbyists and technicians can
adapt the board to available oscillators.

**Repository layout**
- `Hardware/`: KiCad schematics and PCB files, and OCXO compatibility notes.
- `Software/`: PIC firmware source (`main.c`, `isr.c`, config and peripheral drivers).

**Key features**
- Uses a PIC18F series microcontroller for the digital control loop and measurement counters.
- Supports 5 V and 12 V OCXO modules with jumper-selectable scaling.
- External DAC (12-bit) for fine VCO control; optional use of OCXO internal reference.
- Sine-to-square conversion option for sine-output OCXOs.
- Outputs: 10 MHz (square), 1PPS, and serial time/status (where supported).

Hardware notes
- OCXO compatibility: designed for OCXO modules that expose a VCO (frequency adjust) input. Modules without a VCO
	pin are not supported.
- Recommended OCXO families (examples): Bliley N47/NV47, IsoTemp 141 A/N, Vectron C4550, Oscilloquartz 8663.
- VCO adjustment: typical adjustment ranges are on the order of ±0.4 to ±1.0 ppm. For a 10 MHz OCXO, ±0.4 ppm
	corresponds to about ±4 Hz of pull range.
- Voltage reference: jumpers allow either the OCXO's onboard reference or the microcontroller-supplied reference to be
	used for VCO control circuitry.
- Power: supports 5 V and 12 V OCXO modules. The design omits 3.3 V OCXO variants to keep voltage-scaling circuitry
	simpler.

Firmware and control
- The PIC microcontroller implements the digital disciplining loop: it measures phase/frequency against GPS 1PPS and
	computes corrections applied through the DAC to the OCXO VCO input.
- Communication: firmware includes basic status reporting via serial (see `Software/` source files).

## Theory of Operation

Overview
- The system uses the PIC18F's high-speed signal acquisition timer/counter to measure the OCXO frequency against the GPS
	1PPS reference. The 10 MHz output from the OCXO (after conditioning through comparator or buffering as needed) is fed
	into the controller's counter. Each rising edge of the 1PPS signal triggers a capture of the counter value.

Measurement and error calculation
- The counter measurement produces the number of 10 MHz cycles counted during one 1-second interval. For an ideal
	10 MHz oscillator the measured count is 10,000,000. The firmware computes the frequency error as:

	`error = measured_count - 10,000,000`

- Control action sign: when `measured_count > 10,000,000` (oscillator fast) the firmware decreases the DAC output
	(reducing VCO control voltage) to lower the oscillator frequency. Conversely, when `measured_count < 10,000,000`
	(oscillator slow) the firmware increases the DAC output to raise the frequency.

Resolution and averaging
- A single 1-second count has 1 Hz quantization (one extra or missing cycle = ±1 Hz). Because the OCXO VCO pull range
	is often a few Hz, and long-term stability targets are well below 1 Hz, the firmware uses averaging and filtering
	across multiple 1‑second samples to resolve sub-Hz frequency offsets. Averaging N seconds reduces quantization noise by
	roughly `1/sqrt(N)` for uncorrelated noise and linearly for simple accumulation.

Digital control loop
- The control loop is implemented in firmware and is PID-like: it applies a proportional term (responds to the
	instantaneous frequency error), an integral term (eliminates steady-state offset over time) and optional damping
	(derivative-like or filtered difference) to stabilize transient responses.
- To avoid overshoot and instability, the loop includes:
	- limited update rate and maximum DAC-step per update,
	- anti-windup on the integrator when the DAC saturates,
	- configurable loop bandwidth (slow for best long-term stability, faster during initial acquisition).

Noise and spurious rejection
- Input conditioning: if the OCXO provides a sine output, it is converted to a clean logic edge via the comparator and
	Schmitt trigger chain (comparator `U401`, logic buffers), which reduces timing jitter and false counts.
- Outlier rejection: the firmware detects and ignores single-sample glitches (spikes) using median filters, clamp
	thresholds, or by discarding samples with implausible changes.
- Low-pass filtering: measurement samples are filtered (moving average or IIR) before being fed into the control law to
	reject short-term noise and spurious phase jitter from the GPS receiver or RF pickup.

DAC mapping and VCO sensitivity
- The external DAC (12-bit) produces the control voltage applied to the OCXO VCO pin (optionally scaled by jumper
	networks for 5/12 V OCXOs). The frequency change per DAC LSB depends on the OCXO sensitivity (Hz per volt). The
	firmware maps desired frequency correction to DAC steps and can be configured for the measured sensitivity of a
	particular OCXO.
- Because frequency pull is small (few Hz), the DAC mapping and loop gains are tuned to provide fine, stable control
	without hunting.

Holdover and failure modes
- If GPS is lost, the firmware stops updating the DAC with GPS-derived corrections and enters a holdover mode. During
	holdover the DAC will be left at its last value.
- GPS lock is indicated by a front-panel LED.  If the GPS lock is lost, the operator should take the appropriate 
    actions to reposition or repair the GPS antenna.

Practical notes
- One-count resolution at 10 MHz corresponds to 1 Hz. To achieve sub-Hz apparent resolution the firmware averages and
	filters multiple 1PPS-captured counts before making DAC adjustments.
- Avoid using OCXO modules without an external VCO adjust pin — they cannot be disciplined by this design.
- Proper grounding, shielding, and clean supply rails reduce spurious counts and improve loop stability.

Build and usage
- Hardware: assemble the board following the silkscreen and BOM in `Hardware/`. Populate only the sections required for
	your chosen OCXO (jumper options allow omitting comparator/driver or scaling components).
- Antenna: mount the GPS antenna with a clear sky view; use a low-loss cable and proper grounding/lightning protection.
- Initial setup:
	1. Install firmware from `Software/` using a PIC programmer supporting the PIC18F family.
	2. Connect GPS antenna and power the board.
	3. Wait for GPS lock and allow the disciplining loop to converge (can take minutes to hours depending on configuration).
- Holdover: if GPS signal is lost, the OCXO runs free; holdover performance depends on the OCXO quality.

Testing and calibration
- Verify 1PPS and 10 MHz outputs with a frequency counter or scope.
- Observe DAC control voltage and confirm it stays within expected range while disciplining.
- Perform a holdover test by disconnecting GPS and measuring drift over a chosen interval.

Design considerations and limitations
- GPS vulnerability: system depends on GPS; consider anti-spoofing/jamming mitigation for critical deployments.
- Environmental sensitivity: OCXO performance varies with temperature — good thermal control improves holdover.


Contributing
If you improve the firmware, PCB, or documentation, please open an issue or a pull request. Include a short description
of your changes and validation steps.

License and credits
- This entire repository (software and hardware design files) is licensed under the Apache License, Version 2.0. See
  `LICENSE` at the repository root for the full text.
- If you incorporate third-party components with separate license requirements, include their required notices and
  follow the terms of those licenses.

Files of interest
- `Hardware/GPSDO.kicad_sch`, `Hardware/GPSDO.kicad_pcb`: primary schematic and PCB.
- `Software/main.c`, `Software/isr.c`, `Software/config.h`: firmware entry points and configuration.

Contact
Open issues in this repository or contact the author via the repository metadata for questions and contribution guidance.

