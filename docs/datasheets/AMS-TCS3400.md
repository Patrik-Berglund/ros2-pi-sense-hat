# TCS3400 Datasheet

**Color Light-to-Digital Converter**

*ams Datasheet [v1-06] 2017-Oct-10*

---

## General Description

The TCS3400 device provides color and IR (red, green, blue, clear and IR) light sensing. The color sensing provides for improved accuracy lux and color temperature measurements typically used to adjust the backlight intensity and correct the display color gamket. Additionally it can be used for light source type detection as it reports the IR content of the light.

---

## Key Benefits & Features

### Benefits
- Single Device Integrated Optical Solution
- Power Management Features
- Color Temperature and Ambient Light Sensing
- Equal Response to 360 degree Incident Light
- Ideal for Operation Behind Dark Glass
- Light Source Detection

### Features
- RGBC and ALS Support
- Programmable Gain & Integration Time
- 1000000:1 Dynamic Range
- Circular Segmented RGBC Photodiode
- Very High Sensitivity
- RGBC + IR sensor

---

## Applications

The TCS3400 applications include:
- Ambient light sensing
- Color temperature sensing
- Industrial process control
- Medical diagnostics

---

## Block Diagram

The functional blocks of this device include:
- UV & IR stopband filter
- Clear, Red, Blue, Green photodiodes
- IR passband filter
- Clear ADC, Red ADC, Green ADC, Blue ADC
- RGBC Control with Lower/Upper Limits
- I²C Interface
- Interrupt capability

**Connections:**
- INT (Interrupt)
- SCL (I²C Serial Clock)
- SDA (I²C Serial Data)
- VDD (Supply Voltage)

---

## Pin Assignment

### Pin Diagram (Top View)
Package FN Dual Flat No-Lead - 2mm x 2.4mm

### Pin Description

| Pin Number | Pin Name | Description |
|------------|----------|-------------|
| 1 | VDD | Supply voltage |
| 2 | SCL | I²C serial clock input terminal |
| 3 | GND | Power supply ground. All voltages are referenced to GND. |
| 4 | I/C | Internal connection, connect to ground or leave floating. |
| 5 | INT | Interrupt — open drain output (active low) |
| 6 | SDA | I²C serial data I/O terminal – open drain |

---

## Absolute Maximum Ratings

**Note:** Stresses beyond those listed under Absolute Maximum Ratings may cause permanent damage to the device.

| Parameter | Min | Max | Units | Comments |
|-----------|-----|-----|-------|----------|
| Supply voltage, VDD | — | 3.8 | V | All voltages are with respect to GND |
| Input terminal voltage | -0.5 | 3.8 | V | |
| Output terminal voltage | -0.5 | 3.8 | V | |
| Output terminal current (SDA, INT) | -1 | 20 | mA | |
| Storage temperature range, TSTRG | -40 | 85 | ºC | |
| Input current (latch up immunity) JEDEC JESD78D Nov 2011 | — | — | — | CLASS 1 |
| Electrostatic discharge HBM S-001-2014 | — | ±2000 | V | |
| Electrostatic discharge CDM JEDEC JESD22-C101F Oct 2013 | — | ±500 | V | |

---

## Electrical Characteristics

### Recommended Operating Conditions

| Symbol | Parameter | Min | Typ | Max | Units |
|--------|-----------|-----|-----|-----|-------|
| VDD | Supply voltage | 2.7 | 3 | 3.6 | V |
| TA | Operating free-air temperature | -40 | — | 70 | °C |

**Note:** While the device is operational across the temperature range, functionality will vary with temperature. Specifications are stated at 25°C unless otherwise noted.

### Operating Characteristics (VDD=3V, TA=25ºC)

| Symbol | Parameter | Conditions | Min | Typ | Max | Units |
|--------|-----------|------------|-----|-----|-----|-------|
| IDD | Supply current | Active | — | 235 | 330 | μA |
| | | Wait state | — | 60 | — | μA |
| | | Sleep state - no I²C activity | — | 1.0 | 10 | μA |
| VOL | INT, SDA output low voltage | 3 mA sink current | 0 | — | 0.4 | V |
| | | 6 mA sink current | 0 | — | 0.6 | V |
| ILEAK | Leakage current, SDA, SCL, INT pins | | -5 | — | 5 | μA |
| VIH | SCL, SDA input high voltage | TCS34001, TCS34005 | 0.7 VDD | — | — | V |
| | | TCS34003, TCS34007 | 1.26 | — | — | V |
| VIL | SCL, SDA input low voltage | TCS34001, TCS34005 | — | — | 0.3 VDD | V |
| | | TCS34003, TCS34007 | — | — | 0.54 | V |

### Optical Characteristics - Clear Channel (VDD = 3V, TA = 25°C, AGAIN = 16x, ATIME = 0xF6)

| Parameter | Test Conditions | Min | Typ | Max | Unit |
|-----------|----------------|-----|-----|-----|------|
| Re (Irradiance Responsivity - Clear Channel) | White LED, CCT = 2700K | 11.2 | 14.0 | 16.8 | counts/(μW/cm²) |
| | Blue LED, λD = 465 nm | 9.5 | 11.8 | 14.2 | counts/(μW/cm²) |
| | Green LED, λD = 525 nm | 11.6 | 14.5 | 17.4 | counts/(μW/cm²) |
| | Red LED, λD = 615 nm | 13.6 | 17.0 | 20.4 | counts/(μW/cm²) |

### Optical Characteristics - IR Channel (VDD = 3V, TA = 25°C, AGAIN = 16x, ATIME = 0xF6)

| Parameter | Test Condition | Min | Typ | Max | Unit |
|-----------|---------------|-----|-----|-----|------|
| Re (Irradiance Responsivity - IR Channel) | λP = 850 nm | 10.0 | 13.3 | 16.6 | counts/(μW/cm²) |

### Color ADC Count Value Ratio (VDD=3V, TA=25ºC)

| Test Conditions | Red/Clear | Green/Clear | Blue/Clear | IR/Clear |
|----------------|-----------|-------------|------------|----------|
| | Min-Max | Min-Max | Min-Max | Min-Max |
| λD = 465 nm | 0%-13% | 10%-38% | 70%-91% | — |
| λD = 525 nm | 3%-22% | 59%-86% | 10%-40% | — |
| λD = 615 nm | 80%-110% | 0%-15% | 3%-26% | 0%-5% |
| λP = 850 nm | — | — | — | 667% |

### RGBC Characteristics (VDD = 3V, TA = 25ºC, AGAIN = 16x, AEN = 1)

| Parameter | Conditions | Min | Typ | Max | Units |
|-----------|------------|-----|-----|-----|-------|
| Dark ADC count value (Clear and RGB Channels) | Ee = 0, AGAIN = 64x, ATIME = 0xB8 (200ms) | 0 | 1 | 4 | counts |
| Dark ADC count value (IR Channel) | | 0 | 1 | 6 | counts |
| Integration time step size | | 2.65 | 2.78 | 2.93 | ms |
| Number of integration steps | | 1 | — | 256 | steps |
| ADC count value | ATIME = 0xFF (2.78ms) to 0xC1 (175ms) | 0 | — | 1024 | counts/step |
| | ATIME = 0xC0 (178ms) to 0x00 (712ms) | 0 | — | 65535 | counts |
| Gain scaling, relative to 16× | 1x: AGAIN = 00 | 0.936 | 0.985 | 1.065 | × |
| | 4x: AGAIN = 01 | 3.66 | 3.85 | 4.16 | × |
| | 16x: AGAIN = 10 | — | 16.0 | — | × |
| | 64x: AGAIN = 11 | 59.6 | 62.7 | 67.8 | × |

### Wait Characteristics (VDD = 3V, TA = 25ºC, WEN = 1)

| Parameter | Conditions | Min | Typ | Max | Units |
|-----------|------------|-----|-----|-----|-------|
| Wait step size | WTIME = 0xFF | — | 2.78 | — | ms |

---

## Timing Characteristics

### AC Electrical Characteristics (VDD = 3V, TA = 25°C)

| Parameter | Conditions | Min | Max | Unit |
|-----------|------------|-----|-----|------|
| fSCL | Clock frequency (I²C only) | 0 | 400 | kHz |
| tBUF | Bus free time between start and stop condition | 1.3 | — | μs |
| tHD;STA | Hold time after (repeated) start condition | 0.6 | — | μs |
| tSU;STA | Repeated start condition setup time | 0.6 | — | μs |
| tSU;STO | Stop condition setup time | 0.6 | — | μs |
| tHD;DAT | Data hold time | 60 | — | ns |
| tSU;DAT | Data setup time | 100 | — | ns |
| tLOW | SCL clock low period | 1.3 | — | μs |
| tHIGH | SCL clock high period | 0.6 | — | μs |
| tF | Clock/data fall time | — | 300 | ns |
| tR | Clock/data rise time | — | 300 | ns |
| Ci | Input pin capacitance | — | 10 | pF |

---

## Typical Operating Characteristics

### Spectral Responsivity

The device shows normalized responsivity across wavelengths from 300nm to 1100nm for:
- Clear channel
- Red channel
- Green channel
- Blue channel
- IR channel

### Normalized Responsivity vs. Angular Displacement

The device shows uniform responsivity across ±90° angular displacement for both optical and mechanical axes.

### Responsivity Temperature Coefficient

| Wavelength | Temperature Coefficient |
|------------|------------------------|
| 400 – 670nm | 250 ppm/°C |
| 850nm | 2500 ppm/°C |
| 950nm | 5500 ppm/°C |

---

## Functional Description

The TCS3400 device provides ambient light sensing and color temperature sensing. The internal state machine manages the operation of the device, controlling ALS functionality and power down modes. Average power consumption is managed via control of variable endurance low power wait cycles.

### Interrupt Feature

The interrupt feature improves system efficiency by eliminating the need to poll the sensor. Two interrupt sources (ALS, ALS saturation) can activate the open drain output pin. Each interrupt source is enabled independently. ALS interrupts appear when upper or lower thresholds are exceeded for a consecutive number of sample readings.

### Color Sensing Architecture

The advanced digital color light sensor contains a segmented circular photodiode array used for color measurements. This architecture provides stable color sensing independent of the incident angle of light. Four integrating analog-to-digital converters (ADCs) integrate light energy from photodiodes simultaneously.

### Communication Interface

Communication is accomplished through a fast (up to 400 kHz) two wire I²C serial bus. The device typically draws only 235μA in color operation and 1μA during power down.

### State Machine

The ALS state machine transitions between:
- **Sleep**: PON = 0
- **RGBC**: Active measurement state (PON = 1, AEN = 1)
- **Wait**: Low-power wait state (PON = 1, WEN = 1)

---

## Register Description

### Register Map

| Address | Register Name | R/W | Register Function | Reset Value |
|---------|--------------|-----|-------------------|-------------|
| 0x80 | ENABLE | R/W | Enables states and interrupts | 0x00 |
| 0x81 | ATIME | R/W | RGBC integration time | 0xFF |
| 0x83 | WTIME | R/W | Wait time | 0xFF |
| 0x84 | AILTL | R/W | Clear interrupt low threshold low byte | 0x00 |
| 0x85 | AILTH | R/W | Clear interrupt low threshold high byte | 0x00 |
| 0x86 | AIHTL | R/W | Clear interrupt high threshold low byte | 0x00 |
| 0x87 | AIHTH | R/W | Clear interrupt high threshold high byte | 0x00 |
| 0x8C | PERS | R/W | Interrupt persistence filter | 0x00 |
| 0x8D | CONFIG | R/W | Configuration | 0x40 |
| 0x8F | CONTROL | R/W | Gain control register | 0x00 |
| 0x90 | AUX | R/W | Auxiliary control register | 0x00 |
| 0x91 | REVID | R | Revision ID | Rev |
| 0x92 | ID | R | Device ID | ID |
| 0x93 | STATUS | R | Device status | 0x00 |
| 0x94 | CDATAL | R | Clear / IR channel low data register | 0x00 |
| 0x95 | CDATAH | R | Clear / IR channel high data register | 0x00 |
| 0x96 | RDATAL | R | Red ADC low data register | 0x00 |
| 0x97 | RDATAH | R | Red ADC high data register | 0x00 |
| 0x98 | GDATAL | R | Green ADC low data register | 0x00 |
| 0x99 | GDATAH | R | Green ADC high data register | 0x00 |
| 0x9A | BDATAL | R | Blue ADC low data register | 0x00 |
| 0x9B | BDATAH | R | Blue ADC high data register | 0x00 |
| 0xC0 | IR | R/W | Access IR Channel | 0x00 |
| 0xE4 | IFORCE | W | Force Interrupt | 0x00 |
| 0xE6 | CICLEAR | W | Clear channel interrupt clear | 0x00 |
| 0xE7 | AICLEAR | W | Clear all interrupts | 0x00 |

---

### Enable Register (0x80)

| Bit | Field | Description |
|-----|-------|-------------|
| 7 | Reserved | Reserved. Write as 0. |
| 6 | SAI | Sleep After Interrupt. When asserted, the device will power down at the end of a RGBC cycle if an interrupt is generated. |
| 5 | Reserved | Reserved. Write as 0. |
| 4 | AIEN | ALS Interrupt Enable. When asserted permits ALS interrupts to be generated, subject to the persist filter. |
| 3 | WEN | Wait Enable. This bit activates the wait feature. Writing a 1 activates the wait timer. |
| 2 | Reserved | Reserved. Write as 0. |
| 1 | AEN | ADC Enable. This bit activates the four-channel (RGBC) ADC. Writing a 1 enables the ADC. |
| 0 | PON | Power ON. This bit activates the internal oscillator to permit the timers and ADC channels to operate. |

---

### RGBC Integration Time Register (0x81)

The ATIME register controls the internal integration time of the RGBC channel ADCs.

Maximum count value = min[CYCLES × 1024, 65535]

| Value | Cycles | Time | Max Count |
|-------|--------|------|-----------|
| 0xFF | 1 | 2.78 ms | 1024 |
| 0xF6 | 10 | 27.8 ms | 10240 |
| 0xDB | 37 | 103 ms | 37888 |
| 0xC0 | 64 | 178 ms | 65535 |
| 0x00 | 256 | 712 ms | 65535 |

---

### Wait Time Register (0x83)

The WTIME controls the amount of time in a low power mode. It is set in 2.78 ms increments unless the WLONG bit is asserted (12× longer).

| Register Value | Wait Time | Time (WLONG=0) | Time (WLONG=1) |
|----------------|-----------|----------------|----------------|
| 0xFF | 1 | 2.78 ms | 0.03 s |
| 0xAB | 85 | 236 ms | 2.84 s |
| 0x00 | 256 | 712 ms | 8.54 s |

**Note:** The wait time register should be configured before AEN is asserted.

---

### Clear Channel Interrupt Threshold Registers (0x84 - 0x87)

These registers provide 16-bit values for high and low thresholds for comparison to CDATA values.

| Register | Address | Description |
|----------|---------|-------------|
| AILTL | 0x84 | Clear Channel low threshold lower byte |
| AILTH | 0x85 | Clear Channel low threshold upper byte |
| AIHTL | 0x86 | Clear Channel high threshold lower byte |
| AIHTH | 0x87 | Clear Channel high threshold upper byte |

---

### Interrupt Persistence Register (0x8C)

Controls the interrupt persistence filter.

| Bit 7-4 | Bit 3-0 (APERS) |
|---------|-----------------|
| Reserved | Clear Interrupt Persistence |

**APERS Field Values:**

| Value | Persistence |
|-------|-------------|
| 0000 | Every RGBC cycle generates an interrupt |
| 0001 | Any value outside of threshold range |
| 0010 | 2 consecutive values out of range |
| 0011 | 3 consecutive values out of range |
| 0100 | 5 consecutive values out of range |
| 0101 | 10 consecutive values out of range |
| 0110 | 15 consecutive values out of range |
| 0111 | 20 consecutive values out of range |
| 1000 | 25 consecutive values out of range |
| 1001 | 30 consecutive values out of range |
| 1010 | 35 consecutive values out of range |
| 1011 | 40 consecutive values out of range |
| 1100 | 45 consecutive values out of range |
| 1101 | 50 consecutive values out of range |
| 1110 | 55 consecutive values out of range |
| 1111 | 60 consecutive values out of range |

---

### Configuration Register (0x8D)

| Bit | Field | Description |
|-----|-------|-------------|
| 7 | Reserved | Reserved. Write as 0. |
| 6 | Reserved | Reserved. Write as 1. |
| 5-2 | Reserved | Reserved. Write all as 0. |
| 1 | WLONG | Wait Long. When asserted, the wait cycles are increased by a factor 12×. |
| 0 | Reserved | Reserved. Write as 0. |

---

### Control Register (0x8F)

| Bit | Field | Description |
|-----|-------|-------------|
| 7-2 | Reserved | Reserved. Write all as 0. |
| 1-0 | AGAIN | RGBC Gain Control |

**AGAIN Field Values:**

| Value | RGBC Gain |
|-------|-----------|
| 00 | 1× Gain |
| 01 | 4× Gain |
| 10 | 16× Gain |
| 11 | 64× Gain |

---

### Auxiliary Register (0x90)

| Bit | Field | Description |
|-----|-------|-------------|
| 7-6 | Reserved | Reserved. Write all as 0. |
| 5 | ASIEN | ALS Saturation Interrupt Enable (0 = disabled, 1 = enabled) |
| 4-0 | Reserved | Reserved. |

---

### Revision ID Register (0x91)

| Bit | Field | Description |
|-----|-------|-------------|
| 7-4 | Reserved | Reserved. |
| 3-0 | RevID | Wafer die revision level |

---

### ID Register (0x92)

| Bit | Field | Description |
|-----|-------|-------------|
| 7-2 | ID | Device Identification = 100100 |
| 1-0 | VID | 00b for TCS34001 & TCS34005<br>11b for TCS34003 & TCS34007 |

---

### Status Register (0x93)

| Bit | Field | Description |
|-----|-------|-------------|
| 7 | ASAT | ALS Saturation. Indicates analog sensor at upper end of dynamic range. |
| 6-5 | Reserved | Reserved. |
| 4 | AINT | ALS Interrupt. Indicates an ALS event met programmed thresholds and persistence. |
| 3-1 | Reserved | Reserved. |
| 0 | AVALID | RGBC Valid. Indicates that the RGBC cycle has completed since AEN was asserted. |

---

### RGBC Data Registers (0x94 - 0x9B)

Clear, red, green, and blue data is stored as 16-bit values.

| Register | Address | Description |
|----------|---------|-------------|
| CDATAL | 0x94 | Clear / IR data low byte |
| CDATAH | 0x95 | Clear / IR data high byte |
| RDATAL | 0x96 | Red data low byte |
| RDATAH | 0x97 | Red data high byte |
| GDATAL | 0x98 | Green data low byte |
| GDATAH | 0x99 | Green data high byte |
| BDATAL | 0x9A | Blue data low byte |
| BDATAH | 0x9B | Blue data high byte |

**Note:** The read sequence must read byte pairs (low followed by high) starting on an even address boundary.

---

### IR Register (0xC0)

| Bit | Field | Description |
|-----|-------|-------------|
| 7 | IR | IR Sensor access. If set, the clear channel reports measurement from the IR sensor. |
| 6-0 | Reserved | Reserved. Always write as 0. |

---

### Clear Interrupt Registers

| Register | Address | Description |
|----------|---------|-------------|
| IFORCE | 0xE4 | Forces an interrupt (any value) |
| CICLEAR | 0xE6 | Clear channel interrupt clear (any value) |
| AICLEAR | 0xE7 | Clears all interrupts (any value) |

---

## Power Supply Considerations

### Typical Application Hardware Circuit

- Place a 1-μF low-ESR decoupling capacitor as close as possible to the VDD pin
- I²C signals and Interrupt are open-drain outputs requiring pull-up resistors
- Recommended pull-up resistor values:
  - I²C signals: 1.5 kΩ (for 400 kbit/s operation)
  - Interrupt line: 10 kΩ

**Note:** VBUS refers to the I²C bus voltage which is either VDD or 1.8V depending on device variant.

---

## PCB Pad Layout

Suggested PCB pad layout guidelines for the surface mount module. Flash Gold is recommended as a surface finish for the landing pads.

The layout is based on IPC-7351B Generic Requirements for Surface Mount Design and Land Pattern Standard (2010) for the small outline no-lead (SON) package.

---

## Package Drawings & Markings

### Package: FN Dual Flat No-Lead
- Dimensions: 2.0mm × 2.4mm
- 6 pins
- Photodiode active area centered within package (tolerance ±75 μm)
- Top surface molded with electrically non-conductive clear plastic compound (refractive index 1.55)
- Contact finish: Copper Alloy A194 with pre-plated NiPdAu
- Lead-free (Pb-free)
- RoHS compliant

---

## Soldering & Storage Information

### Solder Reflow Profile

| Parameter | Reference Device |
|-----------|------------------|
| Average temperature gradient in preheating | 2.5 ºC/s |
| Soak time (tsoak) | 2 to 3 minutes |
| Time above 217 ºC (T1, t1) | Max 60 s |
| Time above 230 ºC (T2, t2) | Max 50 s |
| Time above Tpeak - 10 ºC (T3, t3) | Max 10 s |
| Peak temperature in reflow (Tpeak) | 260 ºC |
| Temperature gradient in cooling | Max -5 ºC/s |

**Note:** Components should be limited to a maximum of three passes through the solder reflow profile.

---

### Moisture Sensitivity

**Moisture Sensitivity Level:** MSL 3

**Shelf Life:** 12 months in unopened moisture barrier bag
- Ambient Temperature: < 40°C
- Relative Humidity: < 90%

**Floor Life:** 168 hours after opening moisture barrier bag
- Ambient Temperature: < 30°C
- Relative Humidity: < 60%

**Rebaking Instructions:** When shelf life or floor life limits have been exceeded, rebake at 50°C for 12 hours.

---

## Ordering Information

### Ordering Codes

| Ordering Code | Address | Interface | Delivery Form | Delivery Quantity |
|---------------|---------|-----------|---------------|-------------------|
| TCS34001FN | 0x39 | I²C VBUS = VDD | FN-6 | 12000 pcs/reel |
| TCS34001FNM | 0x39 | I²C VBUS = VDD | FN-6 | 500 pcs/reel |
| TCS34003FN | 0x39 | I²C bus = 1.8V | FN-6 | 12000 pcs/reel |
| TCS34003FNM | 0x39 | I²C bus = 1.8V | FN-6 | 500 pcs/reel |
| TCS34005FN | 0x29 | I²C VBUS = VDD | FN-6 | 12000 pcs/reel |
| TCS34005FNM | 0x29 | I²C VBUS = VDD | FN-6 | 500 pcs/reel |
| TCS34007FN | 0x29 | I²C bus = 1.8V | FN-6 | 12000 pcs/reel |
| TCS34007FNM | 0x29 | I²C bus = 1.8V | FN-6 | 500 pcs/reel |

---

## Contact Information

**Buy products or get free samples:**  
www.ams.com/ICdirect

**Technical Support:**  
www.ams.com/Technical-Support

**Provide feedback:**  
www.ams.com/Document-Feedback

**Email:**  
ams_sales@ams.com

**Sales offices, distributors and representatives:**  
www.ams.com/contact

### Headquarters
ams AG  
Tobelbader Strasse 30  
8141 Premstaetten  
Austria, Europe  
Tel: +43 (0) 3136 500 0  
Website: www.ams.com

---

## RoHS Compliant & ams Green Statement

**RoHS Compliant:** ams AG products fully comply with current RoHS directives. Products do not contain any chemicals for all 6 substance categories, including the requirement that lead not exceed 0.1% by weight in homogeneous materials.

**ams Green:** In addition to RoHS compliance, products are free of Bromine (Br) and Antimony (Sb) based flame retardants (Br or Sb do not exceed 0.1% by weight in homogeneous material).

---

## Document Status

| Document Status | Product Status | Definition |
|-----------------|----------------|------------|
| Product Preview | Pre-Development | Information based on product ideas in planning phase |
| Preliminary Datasheet | Pre-Production | Information based on products in design/validation/qualification phase |
| **Datasheet** | **Production** | **Information based on products in ramp-up or full production** |
| Datasheet (discontinued) | Discontinued | Products superseded and should not be used for new designs |

---

## Revision Information

**Changes from 1-05 (2016-Aug-11) to current revision 1-06 (2017-Oct-10):**
- Updated ordering information

---

## Copyright & Disclaimer

Copyright ams AG, Tobelbader Strasse 30, 8141 Premstaetten, Austria-Europe. Trademarks Registered. All rights reserved.

This product is provided by ams AG "AS IS" and any express or implied warranties, including, but not limited to the implied warranties of merchantability and fitness for a particular purpose are disclaimed.

ams AG shall not be liable to recipient or any third party for any damages, including but not limited to personal injury, property damage, loss of profits, loss of use, interruption of business or indirect, special, incidental or consequential damages, of any kind, in connection with or arising out of the furnishing, performance or use of the technical data herein.

---

*Document v1-06, 2017-Oct-10*