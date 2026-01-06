# Disabling Sense HAT Kernel Drivers

This document explains how to disable the Raspberry Pi Sense HAT kernel drivers to enable direct I2C access to the ATTINY88 microcontroller.

## Why Disable Kernel Drivers?

By default, the Linux kernel loads drivers for the Sense HAT that claim the ATTINY88 I2C device (address 0x46). This prevents direct userspace access via I2C. The kernel drivers provide:

- `/dev/fb1` - Framebuffer for LED matrix
- `/dev/input/eventX` - Input events for joystick

To implement low-level I2C communication with the ATTINY88, these drivers must be disabled.

## Kernel Modules

The Sense HAT uses three kernel modules:

| Module | Purpose |
|--------|---------|
| `rpisense_core` | Core driver for ATTINY88 communication |
| `rpisense_fb` | Framebuffer driver for LED matrix |
| `rpisense_js` | Joystick input driver |

## Method 1: Permanent Blacklist (Recommended)

This method prevents the modules from loading at boot time.

### Step 1: Create Blacklist File

```bash
sudo nano /etc/modprobe.d/blacklist-sensehat.conf
```

Add the following content:

```
# Blacklist Raspberry Pi Sense HAT kernel drivers
# to allow direct I2C access
blacklist rpisense_fb
blacklist rpisense_js
blacklist rpisense_core
```

### Step 2: Update Initramfs

```bash
sudo update-initramfs -u
```

### Step 3: Reboot

```bash
sudo reboot
```

### Step 4: Verify

After reboot, check that the ATTINY88 is accessible:

```bash
i2cdetect -y 1
```

You should see `46` (not `UU`) at address 0x46, indicating the device is available for userspace access.

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- 1c -- -- -- 
20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- 46 -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- 5c -- -- 5f 
60: -- -- -- -- -- -- -- -- -- -- 6a -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

## Method 2: Temporary Unbind (Per-Boot)

This method unbinds the driver without blacklisting, requiring the command to be run after each boot.

### Unbind Command

```bash
# Check if driver is bound
ls /sys/bus/i2c/devices/1-0046/driver/

# Unbind the device
echo "1-0046" | sudo tee /sys/bus/i2c/devices/1-0046/driver/unbind
```

### Automatic Unbind on Boot

Create a systemd service to unbind automatically:

```bash
sudo nano /etc/systemd/system/sensehat-unbind.service
```

Add:

```ini
[Unit]
Description=Unbind Sense HAT kernel driver
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo "1-0046" > /sys/bus/i2c/devices/1-0046/driver/unbind'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable the service:

```bash
sudo systemctl enable sensehat-unbind.service
sudo systemctl start sensehat-unbind.service
```

## Re-enabling Kernel Drivers

If you want to restore the kernel drivers:

### Remove Blacklist

```bash
sudo rm /etc/modprobe.d/blacklist-sensehat.conf
sudo update-initramfs -u
sudo reboot
```

### Or Load Modules Manually (Temporary)

```bash
sudo modprobe rpisense_core
sudo modprobe rpisense_fb
sudo modprobe rpisense_js
```

## Verification

### Check Loaded Modules

```bash
lsmod | grep rpisense
```

**With drivers enabled:**
```
rpisense_fb            20480  1
rpisense_js            12288  0
rpisense_core          12288  2 rpisense_js,rpisense_fb
```

**With drivers disabled:**
```
(no output)
```

### Check I2C Device Status

```bash
i2cdetect -y 1
```

- `UU` at 0x46 = Kernel driver is using the device
- `46` at 0x46 = Device is available for userspace access

### Check Device Files

**With drivers enabled:**
```bash
ls /dev/fb1          # Framebuffer exists
ls /dev/input/event* # Joystick event device exists
```

**With drivers disabled:**
```bash
ls /dev/fb1          # File not found
# Joystick event device will not exist
```

## Impact on System Updates

### Ubuntu/Kernel Updates

- The blacklist file persists through system updates
- Kernel updates will not re-enable the drivers
- The blacklist applies to the module names, which are unlikely to change

### Reverting Changes

The blacklist can be removed at any time without affecting system stability. Simply delete the blacklist file and reboot.

## Troubleshooting

### Modules Still Loading

If modules are still loading after blacklist:

1. Verify blacklist file exists:
   ```bash
   cat /etc/modprobe.d/blacklist-sensehat.conf
   ```

2. Check for device tree overlay:
   ```bash
   grep sense /boot/firmware/config.txt
   ```
   
   If you see `dtoverlay=rpi-sense`, comment it out:
   ```bash
   sudo nano /boot/firmware/config.txt
   # Add # before: dtoverlay=rpi-sense
   ```

3. Rebuild initramfs and reboot:
   ```bash
   sudo update-initramfs -u
   sudo reboot
   ```

### Cannot Access I2C Device

If 0x46 still shows as `UU`:

1. Check what's using it:
   ```bash
   ls -l /sys/bus/i2c/devices/1-0046/driver
   ```

2. Manually unbind:
   ```bash
   echo "1-0046" | sudo tee /sys/bus/i2c/devices/1-0046/driver/unbind
   ```

### Permission Denied on I2C Access

Add your user to the i2c group:

```bash
sudo usermod -a -G i2c $USER
# Log out and back in for changes to take effect
```

Or use sudo for testing:

```bash
sudo i2cdetect -y 1
```

## References

- [Linux Kernel Module Blacklisting](https://wiki.debian.org/KernelModuleBlacklisting)
- [Raspberry Pi Sense HAT Kernel Driver](https://github.com/raspberrypi/linux/tree/rpi-6.1.y/drivers/mfd)
- [I2C Device Binding](https://www.kernel.org/doc/Documentation/i2c/instantiating-devices)
