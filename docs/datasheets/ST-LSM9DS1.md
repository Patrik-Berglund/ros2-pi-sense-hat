# LSM9DS1 iNEMO Inertial Module Datasheet

## Overview

The LSM9DS1 is a system-in-package featuring a 3D digital linear acceleration sensor, a 3D digital angular rate sensor, and a 3D digital magnetic sensor. This 9-axis inertial measurement unit (IMU) combines three sensors in a single compact package.

- **Package**: LGA-24L (3.5 × 3.0 × 1.0 mm)
- **Supply Voltage**: 1.9V to 3.6V
- **Temperature Range**: -40°C to +85°C
- **Digital Interfaces**: I²C and SPI
- **16-bit Data Output**: High resolution measurements

## Key Features

### Accelerometer
- **Full Scale Range**: ±2g/±4g/±8g/±16g selectable
- **16-bit data output**
- **Embedded temperature sensor**
- **Programmable interrupt generators**

### Gyroscope  
- **Full Scale Range**: ±245/±500/±2000 dps selectable
- **Low power mode available**
- **Programmable high-pass filter**
- **Sleep mode for power saving**

### Magnetometer
- **Full Scale Range**: ±4/±8/±12/±16 gauss selectable
- **Temperature compensation**
- **Offset cancellation**
- **Self-test capability**

### System Features
- **Embedded 32-level FIFO buffer**
- **I²C and SPI digital interfaces**
- **"Always-on" eco power mode down to 1.9 mA**
- **Position and motion detection functions**
- **Click/double-click recognition**
- **Factory calibrated (no user calibration required)**

## Absolute Maximum Ratings

| Parameter | Symbol | Min | Max | Unit |
|-----------|--------|-----|-----|------|
| Supply voltage | VDD | 1.9 | 3.6 | V |
| I/O supply voltage | VDD_IO | 1.71 | VDD+0.1 | V |
| Operating temperature | TOP | -40 | +85 | °C |
| Storage temperature | TSTG | -40 | +125 | °C |
| Magnetic disturbance field | - | - | 50 | gauss |

## Sensor Specifications

### Accelerometer Specifications
| Parameter | FS = ±2g | FS = ±4g | FS = ±8g | FS = ±16g | Unit |
|-----------|----------|----------|----------|-----------|------|
| **Sensitivity** | 0.061 | 0.122 | 0.244 | 0.732 | mg/LSB |
| **Zero-g offset** | ±90 | ±90 | ±90 | ±90 | mg |

### Gyroscope Specifications
| Parameter | FS = ±245dps | FS = ±500dps | FS = ±2000dps | Unit |
|-----------|--------------|--------------|---------------|------|
| **Sensitivity** | 8.75 | 17.50 | 70 | mdps/LSB |
| **Zero-rate offset** | ±30 | ±30 | ±30 | dps |

### Magnetometer Specifications
| Parameter | FS = ±4G | FS = ±8G | FS = ±12G | FS = ±16G | Unit |
|-----------|----------|----------|-----------|-----------|------|
| **Sensitivity** | 0.14 | 0.29 | 0.43 | 0.58 | mgauss/LSB |
| **Zero-gauss offset** | ±1 | ±1 | ±1 | ±1 | gauss |

## Electrical Characteristics

| Parameter | Symbol | Min | Typ | Max | Unit |
|-----------|--------|-----|-----|-----|------|
| Supply voltage | VDD | 1.9 | 2.2 | 3.6 | V |
| I/O supply voltage | VDD_IO | 1.71 | - | VDD+0.1 | V |
| Accelerometer + Magnetometer current | IDD_XM | - | 600 | - | μA |
| Gyroscope current (normal mode) | IDD_G | - | 4.0 | - | mA |
| Power supply rise time | Trise | 0.01 | - | 100 | ms |
| VDD_IO to VDD delay | Twait | 0 | - | 10 | ms |

## Pin Configuration

| Pin | Name | Type | Description |
|-----|------|------|-------------|
| 1 | VDDIO | Power | I/O supply voltage |
| 2 | SCL/SPC | I/O | I²C clock / SPI clock |
| 3 | VDDIO | Power | I/O supply voltage |
| 4 | SDA/SDI/SDO | I/O | I²C data / SPI data |
| 5 | SDO_A/G | Output | SPI data out (Accel/Gyro) |
| 6 | SDO_M | Output | SPI data out (Magnetometer) |
| 7 | CS_A/G | Input | Chip select (Accel/Gyro) |
| 8 | CS_M | Input | Chip select (Magnetometer) |
| 9 | DRDY_M | Output | Magnetometer data ready |
| 10 | INT_M | Output | Magnetometer interrupt |
| 11 | INT1_A/G | Output | Accel/Gyro interrupt 1 |
| 12 | INT2_A/G | Output | Accel/Gyro interrupt 2 |
| 13 | DEN_A/G | Input | Data enable |
| 14-18 | RES | - | Reserved (connect to GND) |
| 19-20 | GND | Power | Ground |
| 21 | CAP | - | Capacitor connection |
| 22-23 | VDD | Power | Supply voltage |
| 24 | C1 | - | 100nF capacitor connection |

## Digital Interfaces

### I²C Interface

**Slave Addresses:**
- **Accelerometer/Gyroscope**: `1101011xb` where x = SDO_A/G pin
  - SDO_A/G = 0: `0xD4` (write) / `0xD5` (read)  
  - SDO_A/G = 1: `0xD6` (write) / `0xD7` (read)
- **Magnetometer**: `0011110xb` where x = SDO_M pin
  - SDO_M = 0: `0x3C` (write) / `0x3D` (read)
  - SDO_M = 1: `0x3E` (write) / `0x3F` (read)

**Speed**: Supports Fast Mode (400 kHz) and Standard Mode (100 kHz)

**Multi-byte Access**: Set MSB of register address to 1 for auto-increment

### SPI Interface

**Configuration**: 4-wire SPI (3-wire mode available)
- **Clock Polarity**: CPOL = 1 (clock high when idle)
- **Clock Phase**: CPHA = 1 (data captured on falling edge)
- **Bit Order**: MSB first
- **Max Clock**: 10 MHz

**Read Operation**: Set MSB of address to 1
**Write Operation**: Set MSB of address to 0
**Multi-byte**: Set bit 6 of address to 1 for auto-increment

## Register Maps

### Accelerometer/Gyroscope Registers (I²C: 0xD4/0xD5)

| Address | Register Name | Access | Description |
|---------|---------------|--------|-------------|
| 0x04 | ACT_THS | R/W | Activity threshold |
| 0x05 | ACT_DUR | R/W | Activity duration |
| 0x06 | INT_GEN_CFG_XL | R/W | Accelerometer interrupt config |
| 0x07-09 | INT_GEN_THS_X/Y/Z_XL | R/W | Accelerometer interrupt thresholds |
| 0x0A | INT_GEN_DUR_XL | R/W | Accelerometer interrupt duration |
| 0x0B | REFERENCE_G | R/W | Gyroscope reference value |
| 0x0C | INT1_CTRL | R/W | INT1 pin control |
| 0x0D | INT2_CTRL | R/W | INT2 pin control |
| 0x0F | WHO_AM_I | R | Device identification (0x68) |
| 0x10 | CTRL_REG1_G | R/W | Gyroscope control 1 |
| 0x11 | CTRL_REG2_G | R/W | Gyroscope control 2 |
| 0x12 | CTRL_REG3_G | R/W | Gyroscope control 3 |
| 0x13 | ORIENT_CFG_G | R/W | Gyroscope orientation config |
| 0x14 | INT_GEN_SRC_G | R | Gyroscope interrupt source |
| 0x15-16 | OUT_TEMP_L/H | R | Temperature output |
| 0x17 | STATUS_REG | R | Status register |
| 0x18-1D | OUT_X/Y/Z_G | R | Gyroscope output data |
| 0x1E | CTRL_REG4 | R/W | Control register 4 |
| 0x1F | CTRL_REG5_XL | R/W | Accelerometer control 5 |
| 0x20 | CTRL_REG6_XL | R/W | Accelerometer control 6 |
| 0x21 | CTRL_REG7_XL | R/W | Accelerometer control 7 |
| 0x22 | CTRL_REG8 | R/W | Control register 8 |
| 0x23 | CTRL_REG9 | R/W | Control register 9 |
| 0x24 | CTRL_REG10 | R/W | Control register 10 |
| 0x26 | INT_GEN_SRC_XL | R | Accelerometer interrupt source |
| 0x27 | STATUS_REG | R | Status register |
| 0x28-2D | OUT_X/Y/Z_XL | R | Accelerometer output data |
| 0x2E | FIFO_CTRL | R/W | FIFO control |
| 0x2F | FIFO_SRC | R | FIFO status |
| 0x30 | INT_GEN_CFG_G | R/W | Gyroscope interrupt config |
| 0x31-36 | INT_GEN_THS_X/Y/Z_G | R/W | Gyroscope interrupt thresholds |
| 0x37 | INT_GEN_DUR_G | R/W | Gyroscope interrupt duration |

### Magnetometer Registers (I²C: 0x3C/0x3D)

| Address | Register Name | Access | Description |
|---------|---------------|--------|-------------|
| 0x05-0A | OFFSET_X/Y/Z_REG_L/H_M | R/W | Magnetometer offset registers |
| 0x0F | WHO_AM_I_M | R | Device identification (0x3D) |
| 0x20 | CTRL_REG1_M | R/W | Magnetometer control 1 |
| 0x21 | CTRL_REG2_M | R/W | Magnetometer control 2 |
| 0x22 | CTRL_REG3_M | R/W | Magnetometer control 3 |
| 0x23 | CTRL_REG4_M | R/W | Magnetometer control 4 |
| 0x24 | CTRL_REG5_M | R/W | Magnetometer control 5 |
| 0x27 | STATUS_REG_M | R | Magnetometer status |
| 0x28-2D | OUT_X/Y/Z_L/H_M | R | Magnetometer output data |
| 0x30 | INT_CFG_M | R/W | Magnetometer interrupt config |
| 0x31 | INT_SRC_M | R | Magnetometer interrupt source |
| 0x32-33 | INT_THS_L/H | R/W | Magnetometer interrupt threshold |

## Data Conversion Formulas

### Accelerometer Data Conversion
```
Acceleration [g] = (Raw_Data × Sensitivity) / 1000
```

**Sensitivity Values:**
- ±2g: 0.061 mg/LSB
- ±4g: 0.122 mg/LSB  
- ±8g: 0.244 mg/LSB
- ±16g: 0.732 mg/LSB

### Gyroscope Data Conversion
```
Angular_Rate [dps] = Raw_Data × Sensitivity
```

**Sensitivity Values:**
- ±245 dps: 8.75 mdps/LSB
- ±500 dps: 17.50 mdps/LSB
- ±2000 dps: 70 mdps/LSB

### Magnetometer Data Conversion
```
Magnetic_Field [gauss] = (Raw_Data × Sensitivity) / 1000
```

**Sensitivity Values:**
- ±4 gauss: 0.14 mgauss/LSB
- ±8 gauss: 0.29 mgauss/LSB
- ±12 gauss: 0.43 mgauss/LSB
- ±16 gauss: 0.58 mgauss/LSB

### Temperature Data Conversion
```
Temperature [°C] = (Raw_Data / 16) + 25
```

## FIFO Operation

The LSM9DS1 features a 32-level FIFO buffer for both accelerometer and gyroscope data.

### FIFO Modes

| Mode | FMODE[2:0] | Description |
|------|------------|-------------|
| **Bypass** | 000 | FIFO disabled, direct data access |
| **FIFO** | 001 | Collects data until full, then stops |
| **Continuous** | 110 | Continuously overwrites oldest data |
| **Continuous-to-FIFO** | 011 | Switches based on interrupt trigger |
| **Bypass-to-Continuous** | 100 | Switches based on interrupt trigger |

### FIFO Configuration
- **Enable**: Set FIFO_EN bit in CTRL_REG9 (0x23)
- **Threshold**: Set FTH[4:0] in FIFO_CTRL (0x2E)
- **Status**: Read FIFO_SRC (0x2F) for level and flags

## Quick Start Guide

### 1. Power-Up Sequence
1. Apply VDD_IO first
2. Wait Twait (max 10ms)
3. Apply VDD with rise time < 100ms
4. Wait for device ready (typ. 5ms)

### 2. Basic I²C Initialization
```c
// Device addresses
#define LSM9DS1_AG_ADDR  0xD4  // Accel/Gyro write address
#define LSM9DS1_M_ADDR   0x3C  // Magnetometer write address

// 1. Verify device IDs
uint8_t who_am_i_ag = i2c_read_reg(LSM9DS1_AG_ADDR, 0x0F); // Should be 0x68
uint8_t who_am_i_m = i2c_read_reg(LSM9DS1_M_ADDR, 0x0F);   // Should be 0x3D

// 2. Configure accelerometer (±2g, 119 Hz)
i2c_write_reg(LSM9DS1_AG_ADDR, 0x20, 0x60); // CTRL_REG6_XL

// 3. Configure gyroscope (±245 dps, 119 Hz)  
i2c_write_reg(LSM9DS1_AG_ADDR, 0x10, 0x60); // CTRL_REG1_G

// 4. Configure magnetometer (±4 gauss, 20 Hz)
i2c_write_reg(LSM9DS1_M_ADDR, 0x20, 0x1C);  // CTRL_REG1_M
i2c_write_reg(LSM9DS1_M_ADDR, 0x22, 0x00);  // CTRL_REG3_M (continuous mode)
```

### 3. Reading Sensor Data
```c
// Read accelerometer data
int16_t accel_x = i2c_read_reg16(LSM9DS1_AG_ADDR, 0x28); // OUT_X_XL
int16_t accel_y = i2c_read_reg16(LSM9DS1_AG_ADDR, 0x2A); // OUT_Y_XL  
int16_t accel_z = i2c_read_reg16(LSM9DS1_AG_ADDR, 0x2C); // OUT_Z_XL

// Read gyroscope data
int16_t gyro_x = i2c_read_reg16(LSM9DS1_AG_ADDR, 0x18); // OUT_X_G
int16_t gyro_y = i2c_read_reg16(LSM9DS1_AG_ADDR, 0x1A); // OUT_Y_G
int16_t gyro_z = i2c_read_reg16(LSM9DS1_AG_ADDR, 0x1C); // OUT_Z_G

// Read magnetometer data  
int16_t mag_x = i2c_read_reg16(LSM9DS1_M_ADDR, 0x28); // OUT_X_M
int16_t mag_y = i2c_read_reg16(LSM9DS1_M_ADDR, 0x2A); // OUT_Y_M
int16_t mag_z = i2c_read_reg16(LSM9DS1_M_ADDR, 0x2C); // OUT_Z_M
```

### 4. Data Ready Polling
```c
// Check if new accelerometer data is available
uint8_t status = i2c_read_reg(LSM9DS1_AG_ADDR, 0x27); // STATUS_REG
if (status & 0x01) {
    // New accelerometer data available
}

// Check if new magnetometer data is available  
uint8_t mag_status = i2c_read_reg(LSM9DS1_M_ADDR, 0x27); // STATUS_REG_M
if (mag_status & 0x08) {
    // New magnetometer data available
}
```

### 5. Power Management
```c
// Enable low-power mode for gyroscope
i2c_write_reg(LSM9DS1_AG_ADDR, 0x12, 0x08); // CTRL_REG3_G, LP_mode = 1

// Put magnetometer in power-down mode
i2c_write_reg(LSM9DS1_M_ADDR, 0x22, 0x03); // CTRL_REG3_M, MD[1:0] = 11
```

## Applications

- Indoor navigation and positioning
- Smart user interfaces and gesture recognition  
- Gaming and virtual reality input devices
- Display/map orientation and browsing
- Motion-activated functions
- Fall detection and activity monitoring
- Drone and robotics orientation control
- Augmented reality applications