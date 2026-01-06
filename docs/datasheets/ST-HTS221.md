# HTS221 Capacitive Digital Humidity and Temperature Sensor

## Overview

The HTS221 is an ultracompact sensor for relative humidity and temperature. It includes a sensing element and a mixed signal ASIC to provide measurement information through digital serial interfaces.

- **Humidity Range**: 0 to 100% relative humidity
- **Temperature Range**: -40°C to +120°C
- **Supply Voltage**: 1.7V to 3.6V
- **Low Power**: 2 μA @ 1 Hz ODR
- **Package**: HLGA-6L (2 × 2 × 0.9 mm)

**Note**: This device is not recommended for new designs. The candidate replacement is SHT40-AD1B from Sensirion.

## Key Features

- 0 to 100% relative humidity range
- High rH sensitivity: 0.004% rH/LSB
- Humidity accuracy: ± 3.5% rH (20 to +80% rH)
- Temperature accuracy: ± 0.5°C (15 to +40°C)
- Embedded 16-bit ADC
- 16-bit humidity and temperature output data
- Selectable ODR from 1 Hz to 12.5 Hz
- SPI and I²C interfaces
- Factory calibrated
- Internal heating element for condensation recovery
- ECOPACK compliant

## Absolute Maximum Ratings

| Parameter | Symbol | Min | Max | Unit |
|-----------|--------|-----|-----|------|
| Supply voltage | VDD | 1.7 | 3.6 | V |
| Operating temperature | TOP | -40 | +120 | °C |
| Storage temperature | TSTG | -40 | +125 | °C |
| Relative humidity | RH | 0 | 100 | % |

## Electrical Specifications

### Humidity and Temperature Parameters

| Parameter | Conditions | Min | Typ | Max | Unit |
|-----------|------------|-----|-----|-----|------|
| Humidity range | | 0 | | 100 | % rH |
| Humidity accuracy | 20 to +80% rH | | ±3.5 | | % rH |
| Humidity resolution | | | 0.004 | | % rH/LSB |
| Temperature range | | -40 | | +120 | °C |
| Temperature accuracy | 15 to +40°C | | ±0.5 | | °C |
| Temperature resolution | | | 0.016 | | °C/LSB |

### Electrical Characteristics

| Parameter | Symbol | Conditions | Min | Typ | Max | Unit |
|-----------|--------|------------|-----|-----|-----|------|
| Supply voltage | VDD | | 1.7 | | 3.6 | V |
| Supply current | IDD | 1 Hz ODR | | 2 | | μA |
| Supply current | IDD | 12.5 Hz ODR | | 22.5 | | μA |
| Supply current (heater) | IDD_H | VDD = 3.3V | | 33 | | mA |
| Supply current (heater) | IDD_H | VDD = 1.8V | | 12 | | mA |

## Pin Configuration

| Pin | Name | Type | Description |
|-----|------|------|-------------|
| 1 | VDD | Power | Supply voltage |
| 2 | CS/SCL/SPC | Digital I/O | Chip select / I²C clock / SPI clock |
| 3 | DRDY | Digital Output | Data ready (optional) |
| 4 | SDA/SDI/SDO | Digital I/O | I²C data / SPI data input/output |
| 5 | GND | Ground | Ground |
| 6 | GND | Ground | Ground |

## Digital Interfaces

### I²C Interface (CS = HIGH or unconnected)

- **Address**: 0xBE (write), 0xBF (read)
- **Clock frequency**: Up to 400 kHz (fast mode)
- **Protocol**: Standard I²C with 7-bit addressing

#### I²C Timing Specifications

| Parameter | Standard Mode | Fast Mode | Unit |
|-----------|---------------|-----------|------|
| Clock frequency | 0-100 | 0-400 | kHz |
| Clock low time | 4.7 | 1.3 | μs |
| Clock high time | 4.0 | 0.6 | μs |
| Setup time | 250 | 100 | ns |
| Hold time | 0 | 0 | ns |

### SPI Interface (CS = LOW)

- **Mode**: 3-wire SPI (modes 0 and 3)
- **Clock frequency**: Up to 10 MHz
- **Data format**: MSB first

#### SPI Timing Specifications

| Parameter | Symbol | Min | Max | Unit |
|-----------|--------|-----|-----|------|
| Clock frequency | f(SPC) | | 10 | MHz |
| Clock cycle | tc(SPC) | 100 | | ns |
| CS setup time | tsu(CS) | 6 | | ns |
| CS hold time | th(CS) | 8 | | ns |
| Data setup time | tsu(SDI) | 5 | | ns |
| Data hold time | th(SDI) | 15 | | ns |

## Register Map

| Address | Name | Type | Default | Description |
|---------|------|------|---------|-------------|
| 0x0F | WHO_AM_I | R | 0xBC | Device identification |
| 0x10 | AV_CONF | R/W | 0x1B | Humidity/temperature average configuration |
| 0x20 | CTRL_REG1 | R/W | 0x00 | Control register 1 |
| 0x21 | CTRL_REG2 | R/W | 0x00 | Control register 2 |
| 0x22 | CTRL_REG3 | R/W | 0x00 | Control register 3 |
| 0x27 | STATUS_REG | R | 0x00 | Status register |
| 0x28 | HUMIDITY_OUT_L | R | 0x00 | Humidity output LSB |
| 0x29 | HUMIDITY_OUT_H | R | 0x00 | Humidity output MSB |
| 0x2A | TEMP_OUT_L | R | 0x00 | Temperature output LSB |
| 0x2B | TEMP_OUT_H | R | 0x00 | Temperature output MSB |
| 0x30-0x3F | CALIB_0..F | R | Various | Calibration coefficients |

## Key Register Descriptions

### CTRL_REG1 (0x20) - Control Register 1

| Bit | Name | Description |
|-----|------|-------------|
| 7 | PD | Power-down control (0: power-down, 1: active) |
| 6:3 | Reserved | Reserved |
| 2 | BDU | Block data update (0: continuous, 1: blocked until read) |
| 1:0 | ODR[1:0] | Output data rate selection |

#### Output Data Rate Configuration

| ODR1 | ODR0 | Humidity Rate | Temperature Rate |
|------|------|---------------|------------------|
| 0 | 0 | One-shot | One-shot |
| 0 | 1 | 1 Hz | 1 Hz |
| 1 | 0 | 7 Hz | 7 Hz |
| 1 | 1 | 12.5 Hz | 12.5 Hz |

### CTRL_REG2 (0x21) - Control Register 2

| Bit | Name | Description |
|-----|------|-------------|
| 7 | BOOT | Reboot memory content |
| 6:2 | Reserved | Reserved |
| 1 | Heater | Internal heater enable |
| 0 | ONE_SHOT | Start single conversion |

### AV_CONF (0x10) - Average Configuration

| Bit | Name | Description |
|-----|------|-------------|
| 7:6 | Reserved | Reserved |
| 5:3 | AVGT[2:0] | Temperature samples to average (2-256) |
| 2:0 | AVGH[2:0] | Humidity samples to average (4-512) |

#### Average Configuration Table

| AVGX[2:0] | Temperature Samples | Humidity Samples | Temp Noise (°C) | RH Noise (%) |
|-----------|-------------------|------------------|-----------------|--------------|
| 000 | 2 | 4 | 0.08 | 0.4 |
| 001 | 4 | 8 | 0.05 | 0.3 |
| 010 | 8 | 16 | 0.04 | 0.2 |
| 011 | 16 | 32 | 0.03 | 0.15 |
| 100 | 32 | 64 | 0.02 | 0.1 |
| 101 | 64 | 128 | 0.015 | 0.07 |
| 110 | 128 | 256 | 0.01 | 0.05 |
| 111 | 256 | 512 | 0.007 | 0.03 |

## Data Conversion Formulas

The HTS221 provides factory calibration coefficients stored in registers 0x30-0x3F. These coefficients are used for linear interpolation to convert raw ADC values to physical units.

### Temperature Conversion

1. **Read calibration coefficients**:
   - T0_degC_x8 (register 0x32)
   - T1_degC_x8 (register 0x33)
   - T0_OUT (registers 0x3C, 0x3D)
   - T1_OUT (registers 0x3E, 0x3F)
   - T1/T0 MSB (register 0x35, bits 1:0 for T0, bits 3:2 for T1)

2. **Calculate actual calibration temperatures**:
   ```
   T0_degC = (T1_T0_MSB[1:0] << 8 | T0_degC_x8) / 8
   T1_degC = (T1_T0_MSB[3:2] << 8 | T1_degC_x8) / 8
   ```

3. **Linear interpolation**:
   ```
   T_degC = T0_degC + (T_OUT - T0_OUT) × (T1_degC - T0_degC) / (T1_OUT - T0_OUT)
   ```

### Humidity Conversion

1. **Read calibration coefficients**:
   - H0_rH_x2 (register 0x30)
   - H1_rH_x2 (register 0x31)
   - H0_T0_OUT (registers 0x36, 0x37)
   - H1_T0_OUT (registers 0x3A, 0x3B)

2. **Calculate actual calibration humidity**:
   ```
   H0_rH = H0_rH_x2 / 2
   H1_rH = H1_rH_x2 / 2
   ```

3. **Linear interpolation**:
   ```
   H_rH = H0_rH + (H_OUT - H0_T0_OUT) × (H1_rH - H0_rH) / (H1_T0_OUT - H0_T0_OUT)
   ```

## Quick Start Guide

### Basic Initialization

1. **Power up the device**:
   - Set CTRL_REG1[7] (PD) = 1 to activate the device

2. **Configure averaging** (optional):
   - Write to AV_CONF register to set desired averaging

3. **Configure data rate**:
   - Set CTRL_REG1[1:0] (ODR) for continuous mode
   - Or use ONE_SHOT mode in CTRL_REG2[0]

4. **Enable block data update** (recommended):
   - Set CTRL_REG1[2] (BDU) = 1

### Reading Data

1. **Check data ready**:
   - Read STATUS_REG[1] for humidity ready
   - Read STATUS_REG[0] for temperature ready

2. **Read raw data**:
   - Read HUMIDITY_OUT_L and HUMIDITY_OUT_H
   - Read TEMP_OUT_L and TEMP_OUT_H

3. **Apply calibration**:
   - Use the conversion formulas with factory calibration coefficients

### One-Shot Mode Example

```c
// Power up and configure
write_register(CTRL_REG1, 0x84);  // PD=1, BDU=1, ODR=one-shot

// Start conversion
write_register(CTRL_REG2, 0x01);  // ONE_SHOT=1

// Wait for completion (ONE_SHOT bit returns to 0)
while(read_register(CTRL_REG2) & 0x01);

// Read data
uint16_t humidity_raw = read_register(HUMIDITY_OUT_L) | (read_register(HUMIDITY_OUT_H) << 8);
uint16_t temp_raw = read_register(TEMP_OUT_L) | (read_register(TEMP_OUT_H) << 8);

// Apply calibration (see conversion formulas above)
```

## Applications

- Air conditioning, heating and ventilation
- Air humidifiers and dehumidifiers
- Refrigerators and freezers
- Wearable devices
- Smart home automation
- Industrial automation
- Respiratory equipment
- Asset and goods tracking

## Package Information

- **Package**: HLGA-6L (Holed Land Grid Array)
- **Dimensions**: 2.0 × 2.0 × 0.9 mm
- **Land size**: 0.30 × 0.35 mm
- **Packing**: Tape and reel
- **ECOPACK**: Compliant

## Important Notes

1. **Condensation Recovery**: Use the internal heater (CTRL_REG2[1]) to speed up recovery from condensation
2. **Soldering**: After soldering, allow 3 days at 25°C/55% RH or 12 hours at 70% RH for re-hydration
3. **Replacement**: This device is not recommended for new designs; consider SHT40-AD1B as replacement
4. **Factory Calibration**: Device is factory calibrated - no user calibration required