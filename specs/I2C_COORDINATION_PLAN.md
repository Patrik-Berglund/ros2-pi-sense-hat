# Sense HAT I2C Coordination Problem

## Problem Statement

### Current Situation
- **LED Matrix Node**: Working individually - direct I2C access to ATTINY88 (0x46)
- **Joystick Node**: Working individually - direct I2C access to ATTINY88 (0x46) 
- **IMU Node**: Working individually - direct I2C access to LSM9DS1 (0x6A, 0x1C)
- More sensors to be added specified in README.md

### The Problem
**"It never worked good after we added the IMU"**

When running all three nodes simultaneously:
- I2C bus conflicts occur during runtime
- Multiple nodes trying to access I2C-1 bus at the same time
- Startup delays (3 seconds between nodes) are insufficient
- Async operations cause collisions:
  - IMU polling at 50Hz
  - LED matrix updates on demand
  - Joystick interrupts firing asynchronously

### Root Cause
**No coordination mechanism for shared I2C bus access during runtime**

- Each node opens `/dev/i2c-1` independently
- No mutual exclusion between I2C transactions
- Race conditions when multiple nodes attempt simultaneous I2C operations
- Linux I2C subsystem cannot handle concurrent userspace access safely

### Symptoms (to be confirmed)
- [ ] I2C read/write errors
- [ ] Corrupted sensor data  
- [ ] Node crashes
- [ ] Intermittent failures
- [ ] Performance degradation

## Current Architecture Analysis

### I2C Usage Patterns
1. **LED Matrix Node** (`ATTiny88Driver`)
   - Device: `/dev/i2c-1` at address `0x46`
   - Usage: On-demand writes (bulk 193-byte framebuffer updates)
   - Frequency: Event-driven (ROS2 service calls, image messages)
   - GPIO: Uses GPIO24 (FRAME_INT) for frame sync via `gpiod_line_event_wait()`

2. **Joystick Node** (`ATTiny88Driver`) 
   - Device: `/dev/i2c-1` at address `0x46` (same as LED matrix)
   - Usage: Event-driven I2C reads triggered by GPIO interrupts
   - Frequency: Interrupt-driven via GPIO23 (KEYS_INT)
   - GPIO: Uses GPIO23 (KEYS_INT) for button events via `gpiod_line_event_wait()`
   - Threading: Dedicated interrupt thread with 100ms timeout

3. **IMU Node** (`LSM9DS1Driver`)
   - Devices: `/dev/i2c-1` at addresses `0x6A` (accel/gyro) and `0x1C` (magnetometer)
   - Usage: Periodic burst reads (14-byte accel+gyro+temp data)
   - Frequency: Configurable, default 10Hz timer-based

### Key Findings
- **Same I2C Bus**: All nodes use `/dev/i2c-1` with different addresses
- **No Locking**: Each node opens its own file descriptor to `/dev/i2c-1`
- **Concurrent Access**: Multiple nodes can perform I2C transactions simultaneously
- **Startup Delays**: Current workaround uses 3-second delays between node starts
- **Shared Device**: LED Matrix and Joystick both use ATTINY88 at 0x46
- **GPIO Dependencies**: 
  - LED Matrix uses GPIO24 (FRAME_INT) for frame synchronization
  - Joystick uses GPIO23 (KEYS_INT) for interrupt-driven button events
  - Both use `libgpiod` with event waiting (not polling)
- **Threading**: Joystick runs dedicated interrupt thread with GPIO event waiting

### Root Cause Confirmation
Linux I2C subsystem allows multiple file descriptors to `/dev/i2c-1`, but concurrent transactions can cause:
- Bus arbitration conflicts
- Incomplete transactions
- Data corruption
- Device lockups

## Requirements

### Functional Requirements
1. **Mutual Exclusion**: Only one I2C transaction at a time on `/dev/i2c-1`
2. **Node Independence**: Sensor nodes maintain their own logic/calibration
3. **GPIO Interrupt Preservation**: Must maintain GPIO24/GPIO23 interrupt handling in their respective module
4. **Performance**: Minimal latency overhead for coordination
5. **Reliability**: Robust error handling and recovery
6. **Backward Compatibility**: Existing ROS2 interfaces should remain unchanged

### Non-Functional Requirements
1. **Latency**: < 5ms overhead for I2C coordination
2. **Throughput**: Support IMU at 10Hz + LED matrix updates + joystick polling
3. **Scalability**: Easy to add new sensor nodes
4. **Maintainability**: Clear separation of concerns