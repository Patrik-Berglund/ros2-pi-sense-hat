# Sense HAT I2C Coordination Problem

## Problem Statement

### Current Situation
- **LED Matrix Node**: Working individually - direct I2C access to ATTINY88 (0x46)
- **Joystick Node**: Working individually - direct I2C access to ATTINY88 (0x46) 
- **IMU Node**: Working individually - direct I2C access to LSM9DS1 (0x6A, 0x1C)

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
3. **GPIO Interrupt Preservation**: Must maintain GPIO24/GPIO23 interrupt handling
4. **Performance**: Minimal latency overhead for coordination
5. **Reliability**: Robust error handling and recovery
6. **Backward Compatibility**: Existing ROS2 interfaces should remain unchanged

### Non-Functional Requirements
1. **Latency**: < 5ms overhead for I2C coordination
2. **Throughput**: Support IMU at 10Hz + LED matrix updates + joystick polling
3. **Scalability**: Easy to add new sensor nodes
4. **Maintainability**: Clear separation of concerns

## Solution Options

### Option 1: Action-Based I2C Bridge (RECOMMENDED)
**Architecture**: Central I2C Bridge Node with ROS2 Action Server
- **Pros**: Standard ROS2, natural queuing, good error handling
- **Cons**: ~2-5ms latency overhead
- **Best For**: Current requirements (10Hz IMU + on-demand LED/joystick)

### Option 2: Service-Based I2C Bridge
**Architecture**: Central I2C Bridge Node with ROS2 Services
- **Pros**: Simple request/response, synchronous
- **Cons**: No queuing, potential blocking
- **Best For**: Low-frequency, synchronous operations

### Option 3: Shared Memory + Mutex
**Architecture**: Shared memory region with pthread mutex
- **Pros**: Lowest latency, direct hardware access
- **Cons**: Complex, not ROS2-native, harder debugging
- **Best For**: High-frequency applications (>100Hz)

### Option 4: Single Monolithic Node
**Architecture**: Combine all sensor logic into one node
- **Pros**: No coordination needed, simple
- **Cons**: Poor modularity, harder to maintain
- **Best For**: Simple applications

## Implementation Plan

### Phase 1: Action-Based I2C Bridge
1. **Create I2C Bridge Node**
   - ROS2 Action Server for I2C transactions
   - Single I2C device manager
   - Request queuing and serialization
   - **CRITICAL**: GPIO interrupt handling must remain in sensor nodes

2. **Define I2C Action Interface**
   - Goal: device address, register, data, read length
   - Result: read data, success/failure
   - Feedback: transaction progress

3. **Modify Existing Nodes**
   - Replace direct I2C access with Action Client calls
   - **PRESERVE**: GPIO interrupt threads and event handling
   - **PRESERVE**: Frame sync and joystick interrupt logic
   - Maintain existing ROS2 interfaces (services, topics)
   - Keep sensor-specific logic intact

4. **Testing & Validation**
   - Verify no I2C conflicts
   - Verify GPIO interrupts still work correctly
   - Test frame sync timing for LED matrix
   - Test joystick responsiveness
   - Measure latency overhead
   - Test concurrent operations

### Phase 2: Optimization (if needed)
- Profile performance bottlenecks
- Consider Zenoh integration for ultra-low latency
- Implement shared memory if Action overhead too high
