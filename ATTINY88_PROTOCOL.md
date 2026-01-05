# ATTINY88 I2C Protocol Documentation

## Overview

The Sense HAT v2 uses an ATTINY88 microcontroller as an interface bridge between the Raspberry Pi and the LED matrix/joystick hardware. The ATTINY88 acts as an I2C slave device that the Pi can communicate with.

**Source:** [raspberrypi/rpi-sense](https://github.com/raspberrypi/rpi-sense)

## I2C Configuration

**I2C Slave Address:** `0x46` (70 decimal)

**I2C Bus:** `/dev/i2c-1` (I2C bus 1 on Raspberry Pi)

## GPIO Interrupt Lines

The ATTINY88 provides two interrupt signals to the Raspberry Pi:

| Pi GPIO | ATTINY Pin | Signal Name | Description |
|---------|------------|-------------|-------------|
| GPIO23  | PB6        | KEYS_INT    | Joystick button state change interrupt |
| GPIO24  | PB7        | FRAME_INT   | LED frame refresh complete interrupt |

### KEYS_INT (GPIO23)
- **Active:** HIGH when button state changes
- **Purpose:** Event-driven joystick reading
- **Usage:** Monitor for rising edge, then read register 0xF2 for button states
- **Benefit:** Eliminates need for polling joystick

### FRAME_INT (GPIO24)
- **Active:** Pulses on frame completion
- **Purpose:** Synchronize LED updates with refresh cycle
- **Usage:** Optional - wait for pulse before updating LED data
- **Benefit:** Prevents visual tearing during updates

## SPI Interface

**Note:** The Raspberry Pi's SPI bus is **NOT used during normal operation**.

The SPI signals visible on the schematic are only for **ISP (In-System Programming)** of the ATTINY88 firmware:
- `PB3_PROG_MOSI` - Programming only
- `PB4_PROG_MISO` - Programming only
- `PB5_PROG_SCK` - Programming only

**Internal SPI:** The ATTINY88 uses its own SPI interface to control the LED2472G driver chip (LED_SDI, LED_CLKR, LED_LE), but this is not accessible from the Raspberry Pi.

**For runtime communication:** Use I2C only (address 0x46).

## Register Map

| Address Range | Register Name | Access | Description |
|---------------|---------------|--------|-------------|
| 0x00 - 0xBF   | LED_PIXELS    | R/W    | LED matrix pixel data (192 bytes) |
| 0xF0          | I2C_WAI       | R      | "Who Am I" - Device identification |
| 0xF1          | I2C_VER       | R      | Firmware version |
| 0xF2          | I2C_KEYS      | R      | Joystick button states |
| 0xF3          | I2C_EE_WP     | R/W    | EEPROM write protect control |

## LED Matrix Control (Registers 0x00-0xBF)

### Format
- **Total bytes:** 192 (64 pixels × 3 color channels)
- **Pixel format:** RGB, 5-bit per channel (0-31)
- **Pixel order:** Row-major, left-to-right, top-to-bottom

### Memory Layout
```
Pixel 0:  [R0, G0, B0]   (registers 0x00, 0x01, 0x02)
Pixel 1:  [R1, G1, B1]   (registers 0x03, 0x04, 0x05)
...
Pixel 63: [R63, G63, B63] (registers 0xBD, 0xBE, 0xBF)
```

### 8x8 Matrix Layout
```
Row 0: Pixels 0-7   (registers 0x00-0x17)
Row 1: Pixels 8-15  (registers 0x18-0x2F)
Row 2: Pixels 16-23 (registers 0x30-0x47)
Row 3: Pixels 24-31 (registers 0x48-0x5F)
Row 4: Pixels 32-39 (registers 0x60-0x77)
Row 5: Pixels 40-47 (registers 0x78-0x8F)
Row 6: Pixels 48-55 (registers 0x90-0xA7)
Row 7: Pixels 56-63 (registers 0xA8-0xBF)
```

### PWM Brightness
- The ATTINY88 implements 6-bit PWM (64 levels: 0-63)
- Register values are 5-bit (0-31), internally scaled to PWM
- Higher values = brighter

### Writing LED Data

**Single Pixel:**
```c
// Set pixel at position (x, y) to color (r, g, b)
uint8_t pixel_index = y * 8 + x;
uint8_t reg_addr = pixel_index * 3;

i2c_write_byte(0x46, reg_addr + 0, r);  // Red (0-31)
i2c_write_byte(0x46, reg_addr + 1, g);  // Green (0-31)
i2c_write_byte(0x46, reg_addr + 2, b);  // Blue (0-31)
```

**Full Frame:**
```c
// Write all 192 bytes at once
uint8_t frame_buffer[192];
// ... fill frame_buffer with RGB data ...
i2c_write_block(0x46, 0x00, frame_buffer, 192);
```

## Joystick Control (Register 0xF2)

### Button Mapping
Reading register 0xF2 returns a single byte with button states:

| Bit | Button    | Description |
|-----|-----------|-------------|
| 0   | Button 0  | Joystick direction/button |
| 1   | Button 1  | Joystick direction/button |
| 2   | Button 2  | Joystick direction/button |
| 3   | Button 3  | Joystick direction/button |
| 4   | Button 4  | Joystick direction/button |
| 5-7 | Reserved  | Unused |

**Note:** The exact mapping of bits to physical buttons (up/down/left/right/center) needs to be determined through testing or kernel driver source analysis.

### Button State
- **Bit = 1:** Button is pressed
- **Bit = 0:** Button is released

### Reading Joystick
```c
uint8_t keys = i2c_read_byte(0x46, 0xF2);

bool button0_pressed = keys & 0x01;
bool button1_pressed = keys & 0x02;
bool button2_pressed = keys & 0x04;
bool button3_pressed = keys & 0x08;
bool button4_pressed = keys & 0x10;
```

### Interrupt Signal
- The ATTINY88 asserts KEYS_INT on GPIO23 (Pi) when button state changes
- **Active HIGH** - goes high when any button is pressed or released
- Can be used for event-driven joystick reading instead of polling
- Monitor GPIO23 for rising edge, then read register 0xF2

### Using GPIO Interrupt
```c
// Set up GPIO23 for interrupt detection
// Use Linux GPIO sysfs or gpiod library
// On interrupt: read register 0xF2 for current button states
```

## Device Identification (Register 0xF0)

**I2C_WAI (Who Am I):**
- Read register 0xF0
- Returns: `0x73` (ASCII 's' for "Sense HAT")
- Use this to verify the ATTINY88 is present and responding

```c
uint8_t device_id = i2c_read_byte(0x46, 0xF0);
if (device_id == 0x73) {
    // Sense HAT ATTINY88 detected
}
```

## Firmware Version (Register 0xF1)

**I2C_VER:**
- Read register 0xF1
- Returns: `0x00` (version 0)
- May vary with different firmware versions

## EEPROM Write Protect (Register 0xF3)

**I2C_EE_WP:**
- Controls write protection for the HAT EEPROM
- Not typically used during normal operation

## Implementation Notes

### Timing
- The ATTINY88 runs a continuous LED refresh loop at high speed
- I2C communication happens via interrupt (TWI_vect)
- No special timing requirements for I2C transactions

### Refresh Rate
- LED matrix is scanned continuously with 6-bit PWM
- Frame interrupt signal (FRAME_INT) on GPIO24 indicates frame completion
- Approximately 60-100 Hz refresh rate
- Can monitor GPIO24 to synchronize LED updates and prevent tearing

### Color Depth
- 5-bit per channel input (32 levels per R/G/B)
- 6-bit PWM output (64 brightness levels)
- Total: 32,768 colors (15-bit color depth)

### Alternative: Kernel Drivers

Instead of direct I2C communication, the Linux kernel provides:
- **Framebuffer:** `/dev/fb1` - RGB565 format, kernel handles I2C
- **Input Events:** `/dev/input/eventX` - Joystick events, kernel handles I2C

The kernel drivers communicate with the ATTINY88 using this same I2C protocol.

## Example: Full LED Matrix Update

```c
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

void set_led_matrix(int i2c_fd, uint8_t pixels[64][3]) {
    uint8_t buffer[192];
    
    // Convert 2D array to linear buffer
    for (int i = 0; i < 64; i++) {
        buffer[i * 3 + 0] = pixels[i][0] & 0x1F;  // Red (5-bit)
        buffer[i * 3 + 1] = pixels[i][1] & 0x1F;  // Green (5-bit)
        buffer[i * 3 + 2] = pixels[i][2] & 0x1F;  // Blue (5-bit)
    }
    
    // Set I2C slave address
    ioctl(i2c_fd, I2C_SLAVE, 0x46);
    
    // Write starting at register 0x00
    uint8_t write_buffer[193];
    write_buffer[0] = 0x00;  // Starting register
    memcpy(&write_buffer[1], buffer, 192);
    
    write(i2c_fd, write_buffer, 193);
}

int main() {
    int i2c_fd = open("/dev/i2c-1", O_RDWR);
    
    // Create a red cross pattern
    uint8_t pixels[64][3] = {0};
    for (int i = 0; i < 8; i++) {
        pixels[i * 8 + i][0] = 31;      // Diagonal: red
        pixels[i * 8 + (7-i)][0] = 31;  // Anti-diagonal: red
    }
    
    set_led_matrix(i2c_fd, pixels);
    
    close(i2c_fd);
    return 0;
}
```

## Example: Read Joystick

```c
uint8_t read_joystick(int i2c_fd) {
    uint8_t reg = 0xF2;
    uint8_t keys;
    
    // Set I2C slave address
    ioctl(i2c_fd, I2C_SLAVE, 0x46);
    
    // Write register address
    write(i2c_fd, &reg, 1);
    
    // Read button states
    read(i2c_fd, &keys, 1);
    
    return keys;
}
```

## References

- [ATTINY88 Firmware Source](https://github.com/raspberrypi/rpi-sense/blob/master/main.c)
- [Assembly LED Driver](https://github.com/raspberrypi/rpi-sense/blob/master/rpi-sense.S)
- [EEPROM Configuration](https://github.com/raspberrypi/rpi-sense/blob/master/eeprom/eeprom_settings.txt)
