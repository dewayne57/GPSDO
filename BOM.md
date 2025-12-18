Bill of Materials (BOM) — GPSDO Project

This file is a human-friendly summary of the key parts used in this GPSDO project. The full, board-level parts list and footprints are available as a CSV exported from the KiCad project:

- Full parts list: `Hardware/GPSDO.csv`

Key components (recommended starting points)

- Microcontroller:
  - Reference: `U311` — PIC18F27Q43 (PIC18F family microcontroller used for the digital control loop)

- DAC / VCO control:
  - Reference: `U301` — DAC8571 (12-bit external DAC recommended)

- GPS receiver:
  - Reference: `U302` — u-blox NEO-M8M (or compatible module); requires SMA antenna connector `J302`

- OCXO (examples supported):
  - `X401` — IsoTemp 131
  - `X402` — Bliley N47 / NV47
  - `X403` — IsoTemp 143N and 143A
  - `X404` — Vectron C4550A
  - `X406` — Oscilloquartz 8663
  - Note: board supports 5 V and 12 V OCXO variants; OCXOs must expose a VCO adjustment pin.

- Analog / amplifier:
  - `U306` — GALI-52 (RF amplifier for 10 MHz / output driver)
  - `U303`, `U304` — OP07 (precision op amps used in analog sections)
  - `U401` — ADCMP600 (comparator for sine-to-square conversion)

- Level translators / logic:
  - `U305` — TXS0108E (8-bit level translator)
  - `U307`, `U308`, `U402` — 74LVC1G17 (single buffer/driver)
  - `U309` — MCP23017 (I/O expander)
  - `U310` — 24AA02 (I2C EEPROM)

- Power / regulators:
  - `U201` — LM7912 (negative regulator)
  - `U202` — LM2574N-3.3 (switching regulator for 3.3 V)
  - `U203`, `U204` — LM2596 (5 V and 12 V switching regulators)

- RF / connectors and passive items:
  - GPS SMA connector: `J302`
  - RF SMA output: `J308` (50 Ω)
  - 1PPS and TTL outputs: `J310` etc.
  - Battery: `BT301` — CR2032 (backup battery for RTC or similar)

- Other notable items:
  - `U307`, `U308` — logic buffers
  - `Q301` — 2N3904 (general transistor)
  - Various axial/toroidal inductors and power transformers listed in `Hardware/GPSDO.csv`

How the CSV maps to the BOM

- `Hardware/GPSDO.csv` contains the full reference designators, quantities, values, footprints, datasheets, and (where available) manufacturer / Mouser part numbers and links. Use that CSV for procurement and PCB assembly.

