# LPS25H MEMS Pressure Sensor Datasheet

## Overview

The LPS25H is a high-resolution digital output barometer packaged in a HCLGA holed package. It features:

- **Pressure Range**: 260 to 1260 hPa
- **Resolution**: 0.01 hPa (equivalent to ~10 cm altitude)
- **Supply Voltage**: 1.7V to 3.6V
- **Low Power**: 4.5 μA typical at 1 Hz with embedded filter
- **Package**: HCLGA-10L (2.5 × 2.5 × 1.0 mm)

## Key Features

- Ultra-compact package
- Embedded 32-level FIFO buffer
- I²C and SPI digital interfaces
- Interrupt generation for pressure thresholds
- Factory calibrated (no user calibration required)
- Hardware digital filter for noise reduction
- AutoZero function for differential pressure measurements

## Absolute Maximum Ratings

| Parameter | Symbol | Min | Max | Unit |
|-----------|--------|-----|-----|------|
| Supply voltage | VDD | -0.3 | 4.8 | V |
| I/O supply voltage | VDD_IO | -0.3 | 4.8 | V |
| Input voltage | Vin | -0.3 | VDD_IO + 0.3 | V |
| Overpressure | P | - | 2 | MPa |
| Storage temperature | TSTG | -40 | +125 | °C |
| ESD protection | ESD | 2 (HBM) | - | kV |

**Warning**: This is a mechanical shock sensitive device. Improper handling can cause permanent damage.

## Functional Description

### Sensing Element

The LPS25H uses a proprietary ST process to create a mono-silicon micro-sized membrane based on a piezoresistive Wheatstone bridge. When pressure is applied, membrane deflection creates an imbalance in the bridge, which is converted to a digital signal by the IC interface.

Intrinsic mechanical stoppers prevent breakage during pressure overstress, ensuring measurement repeatability.

### IC Interface

The measurement chain consists of:
1. Low-noise amplifier converting resistance changes to analog voltage
2. Analog-to-digital converter for digital output
3. I²C/SPI interface for microcontroller communication
4. Data-Ready signal for synchronization

### Factory Calibration

The device is factory calibrated at three temperatures and two pressures. Trimming values are stored in non-volatile memory and automatically loaded on power-up. **No user calibration is required.**

### FIFO Buffer

The 32-slot FIFO stores pressure measurements, reducing the need for continuous polling by the host processor. This improves system power efficiency.

**FIFO Modes:**
- **Bypass Mode** (000): FIFO disabled
- **FIFO Mode** (001): Fills until full, then stops
- **Stream Mode** (010): Continuously fills, discarding oldest data when full
- **FIFO Mean Mode** (110): Hardware averaging of stored samples

**Watermark Interrupt**: Programmable threshold to generate interrupts when FIFO reaches specified level.

## Electrical Connection

```
VDD ──┬── [4.7μF] ── GND
      └── [100nF] ── GND

VDD_IO ── Device I/O Supply

For I²C mode: CS must be tied to VDD_IO
```

Power supply decoupling capacitors (100 nF and 4.7 μF) should be placed as close as possible to the device supply pins.

## Digital Interfaces

### I²C Interface (CS = High)

**Slave Address**: `1011110xb` where x = SA0 pin state
- SA0 = 0: Address `0xB8` (write) / `0xB9` (read)
- SA0 = 1: Address `0xBA` (write) / `0xBB` (read)

**Speed**: Supports Fast Mode (400 kHz) and Standard Mode (100 kHz)

**Protocol**:
1. START condition
2. Slave address + R/W bit
3. Sub-address (register address)
4. Data byte(s)
5. STOP condition

**Multi-byte Read/Write**: Set MSB of sub-address to 1 for auto-increment.

### SPI Interface (CS = Low)

**Modes**: 4-wire (default) or 3-wire (set SIM bit in CTRL_REG1)

**Protocol**:
- Bit 0: R/W (0 = write, 1 = read)
- Bit 1: MS (0 = no auto-increment, 1 = auto-increment)
- Bits 2-7: Register address
- Bits 8-15: Data

**Pin Mapping**:
- CS: Chip select (active low)
- SPC: Serial clock
- SDI: Serial data input
- SDO: Serial data output (also used for I²C address LSB)

## Register Map Summary

| Address | Register | Type | Default | Description |
|---------|----------|------|---------|-------------|
| 0x08-0x0A | REF_P_XL/L/H | R/W | 0x00 | Reference pressure (24-bit) |
| 0x0F | WHO_AM_I | R | 0xBD | Device ID |
| 0x10 | RES_CONF | R/W | 0x05 | Resolution configuration |
| 0x20 | CTRL_REG1 | R/W | 0x00 | Control register 1 |
| 0x21 | CTRL_REG2 | R/W | 0x00 | Control register 2 |
| 0x22 | CTRL_REG3 | R/W | 0x00 | Interrupt control |
| 0x23 | CTRL_REG4 | R/W | 0x00 | Interrupt configuration |
| 0x24 | INT_CFG | R/W | 0x00 | Interrupt differential config |
| 0x25 | INT_SOURCE | R | 0x00 | Interrupt source |
| 0x27 | STATUS_REG | R | 0x00 | Status register |
| 0x28-0x2A | PRESS_OUT_XL/L/H | R | - | Pressure output (24-bit) |
| 0x2B-0x2C | TEMP_OUT_L/H | R | - | Temperature output (16-bit) |
| 0x2E | FIFO_CTRL | R/W | 0x00 | FIFO control |
| 0x2F | FIFO_STATUS | R | 0x00 | FIFO status |
| 0x30-0x31 | THS_P_L/H | R/W | 0x00 | Pressure threshold (16-bit) |
| 0x39-0x3A | RPDS_L/H | R/W | 0x38/0x00 | Pressure offset (16-bit) |

## Key Registers

### CTRL_REG1 (0x20) - Control Register 1

| Bit | Name | Description |
|-----|------|-------------|
| 7 | PD | Power down control (0: power-down, 1: active) |
| 6:4 | ODR[2:0] | Output data rate selection |
| 3 | DIFF_EN | Interrupt circuit enable |
| 2 | BDU | Block data update |
| 1 | RESET_AZ | Reset AutoZero function |
| 0 | SIM | SPI mode (0: 4-wire, 1: 3-wire) |

**Output Data Rate Options**:

| ODR[2:0] | Pressure Rate | Temperature Rate |
|----------|---------------|------------------|
| 000 | One-shot | One-shot |
| 001 | 1 Hz | 1 Hz |
| 010 | 7 Hz | 7 Hz |
| 011 | 12.5 Hz | 12.5 Hz |
| 100 | 25 Hz | 25 Hz |

### CTRL_REG2 (0x21) - Control Register 2

| Bit | Name | Description |
|-----|------|-------------|
| 7 | BOOT | Reboot memory content |
| 6 | FIFO_EN | FIFO enable |
| 5 | WTM_EN | Watermark level enable |
| 4 | FIFO_MEAN_DEC | Enable 1Hz ODR decimation |
| 2 | SWRESET | Software reset |
| 1 | AUTO_ZERO | AutoZero enable |
| 0 | ONE_SHOT | Trigger one-shot measurement |

### RES_CONF (0x10) - Resolution Configuration

**Pressure Internal Average**:

| AVGP[1:0] | Samples |
|-----------|---------|
| 00 | 8 |
| 01 | 32 |
| 10 | 128 |
| 11 | 512 |

**Temperature Internal Average**:

| AVGT[1:0] | Samples |
|-----------|---------|
| 00 | 8 |
| 01 | 16 |
| 10 | 32 |
| 11 | 64 |

## Data Conversion Formulas

### Pressure Output

```
Pressure (hPa) = PRESS_OUT / 4096
```

**Example**: `PRESS_OUT = 0x3ED000` (4116480 LSB)
- Pressure = 4116480 / 4096 = 1005 hPa

**Default value**: `0x2F800` = 760 hPa

### Temperature Output

```
Temperature (°C) = 42.5 + (TEMP_OUT / 480)
```

**Example**: `TEMP_OUT = 0` → Temperature = 42.5°C

### Pressure Threshold

```
Threshold (hPa) = THS_P / 16
```

## Hardware Digital Filter

The embedded digital filter reduces pressure noise to **0.01 hPa RMS** (1 Pa at 1σ).

**Recommended Low-Power Configuration** (1 Hz, 4.5 μA typical):

```
RES_CONF (0x10) = 0x05
FIFO_CTRL (0x2E) = 0xC0
CTRL_REG2 (0x21) = 0x40
```

This configuration uses FIFO Mean Mode with 32-sample averaging.

**FIFO Mean Mode Sample Sizes**:

| WTM_POINT[4:0] | Samples Averaged |
|----------------|------------------|
| 00001 | 2 |
| 00011 | 4 |
| 00111 | 8 |
| 01111 | 16 |
| 11111 | 32 |

## Interrupt Generation

The device features a programmable interrupt pin (INT1) that can be configured for:

- **Data Ready** (DRDY): New measurement available
- **FIFO Watermark**: FIFO reached threshold level
- **FIFO Full**: FIFO buffer full
- **FIFO Empty**: FIFO buffer empty
- **Pressure High**: Pressure exceeds threshold
- **Pressure Low**: Pressure below threshold

**Interrupt Configuration** (CTRL_REG3):

| Bit | Name | Description |
|-----|------|-------------|
| 7 | INT_H_L | Active high/low (0: high, 1: low) |
| 6 | PP_OD | Push-pull/open-drain (0: push-pull, 1: open-drain) |
| 1:0 | INT1_S[2:1] | Interrupt source selection |

**Interrupt Sources**:

| INT1_S[2:1] | Function |
|-------------|----------|
| 00 | Data signal (DRDY, watermark, etc.) |
| 01 | Pressure high |
| 10 | Pressure low |
| 11 | Pressure low OR high |

## AutoZero Function

The AutoZero feature enables differential pressure measurements by storing a reference pressure value.

**To use AutoZero**:
1. Set `AUTO_ZERO` bit in CTRL_REG2
2. Current pressure is copied to REF_P registers
3. PRESS_OUT becomes the difference between current and reference pressure

**To reset AutoZero**:
- Set `RESET_AZ` bit in CTRL_REG1
- REF_P registers reset to RPDS default value

## Application Notes

### Soldering

The HCLGA package is ECOPACK® compliant and qualified for soldering heat resistance per JEDEC J-STD-020.

### Pin Configuration (HCLGA-10L)

| Pin | Name | Function |
|-----|------|----------|
| 1 | VDD_IO | I/O supply voltage |
| 2 | SCL/SPC | I²C clock / SPI clock |
| 3 | GND | Ground |
| 4 | GND | Ground |
| 5 | GND | Ground |
| 6 | VDD | Supply voltage |
| 7 | RES | Reserved |
| 8 | SDA/SDI/SDO | I²C data / SPI data |
| 9 | SDO/SA0 | SPI data out / I²C address LSB |
| 10 | CS | Chip select / I²C mode select |

### Power Consumption Examples

| Configuration | ODR | Current (typical) | Noise |
|---------------|-----|-------------------|-------|
| Standard | 1 Hz | 25 μA | 0.01 hPa RMS |
| Low-power with filter | 1 Hz | 4.5 μA | 0.01 hPa RMS |
| One-shot mode | - | 1 μA | - |

## Quick Start Guide

### Basic Initialization (I²C)

1. **Power up device**: Set PD bit in CTRL_REG1
2. **Configure resolution**: Write to RES_CONF (default 0x05 is fine)
3. **Set data rate**: Configure ODR bits in CTRL_REG1
4. **Enable BDU**: Set BDU bit to prevent reading inconsistent data

**Example Configuration** (1 Hz continuous mode):
```
CTRL_REG1 = 0x84  // PD=1, ODR=001 (1Hz), BDU=1
```

### Reading Pressure

1. Check STATUS_REG bit P_DA (pressure data available)
2. Read PRESS_OUT_H, PRESS_OUT_L, PRESS_OUT_XL (0x2A, 0x29, 0x28)
3. Combine into 24-bit signed value
4. Convert: Pressure (hPa) = value / 4096

### Reading Temperature

1. Check STATUS_REG bit T_DA (temperature data available)
2. Read TEMP_OUT_H, TEMP_OUT_L (0x2C, 0x2B)
3. Combine into 16-bit signed value
4. Convert: Temperature (°C) = 42.5 + (value / 480)

### One-Shot Mode

For ultra-low power applications:

1. Set ODR[2:0] = 000 in CTRL_REG1
2. Set ONE_SHOT bit in CTRL_REG2 to trigger measurement
3. Wait for P_DA bit in STATUS_REG
4. Read pressure and temperature
5. ONE_SHOT bit auto-clears after measurement

## Document Information

- **Document ID**: DocID023722 Rev 6
- **Revision Date**: October 29, 2019
- **Device ID**: 0xBD (WHO_AM_I register)

---

*© 2018 STMicroelectronics – All rights reserved*
