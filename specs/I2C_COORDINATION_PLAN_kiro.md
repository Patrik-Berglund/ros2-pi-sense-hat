# Implementation Plan: I2C Bus Coordination for Sense HAT

## Overview

This plan addresses the I2C bus contention issue where multiple ROS2 nodes (LED Matrix, Joystick, IMU) access `/dev/i2c-1` concurrently, causing runtime conflicts despite working individually.

**Core Problem**: Linux I2C subsystem allows multiple file descriptors to the same bus, but concurrent transactions cause bus arbitration conflicts, incomplete transactions, and data corruption.

**Solution Strategy**: Centralized I2C bus manager with mutex-based coordination, maintaining node independence while serializing all I2C operations.

## Architecture

### Component Design

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Application Layer                    │
├─────────────────┬─────────────────┬─────────────────────────┤
│ LED Matrix Node │ Joystick Node   │ IMU Node                │
│ (GPIO24 local)  │ (GPIO23 local)  │ (Timer-based)           │
└────────┬────────┴────────┬────────┴────────┬────────────────┘
         │                 │                 │
         │ I2C requests    │ I2C requests    │ I2C requests
         │ (0x46)          │ (0x46)          │ (0x6A, 0x1C)
         │                 │                 │
         └─────────────────┴─────────────────┘
                           │
                           ▼
         ┌─────────────────────────────────────┐
         │     I2C Bus Manager (Singleton)     │
         │  - Mutex-protected I2C operations   │
         │  - Single /dev/i2c-1 file handle    │
         │  - Address switching via ioctl      │
         └─────────────────┬───────────────────┘
                           │
                           ▼
                    /dev/i2c-1 (Linux I2C)
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                 │
         ▼                 ▼                 ▼
    ATTINY88          LSM9DS1           LSM9DS1
     (0x46)        Accel/Gyro (0x6A)   Mag (0x1C)
```

### Key Design Decisions

1. **Centralized Bus Manager**: Single point of I2C access with mutex protection
2. **Minimal Refactoring**: Nodes keep their logic, only I2C access changes
3. **GPIO Stays Local**: Frame sync (GPIO24) and joystick interrupt (GPIO23) remain in their respective nodes
4. **Address Switching**: Manager handles `I2C_SLAVE` ioctl before each transaction
5. **No ROS2 Services**: Direct C++ singleton for minimal latency (<5ms requirement)

## Implementation Steps

### Phase 1: Create I2C Bus Manager

**File**: `include/ros2_pi_sense_hat/i2c_bus_manager.hpp`

```cpp
class I2CBusManager {
public:
  static I2CBusManager& getInstance();
  
  bool open(const std::string& bus);
  void close();
  
  // Thread-safe I2C operations with automatic address switching
  bool write(uint8_t address, const uint8_t* data, size_t length);
  bool read(uint8_t address, uint8_t* data, size_t length);
  bool writeReg(uint8_t address, uint8_t reg, uint8_t value);
  bool readReg(uint8_t address, uint8_t reg, uint8_t& value);
  bool readMultiReg(uint8_t address, uint8_t reg, uint8_t* data, size_t length, bool auto_increment);
  
private:
  I2CBusManager() = default;
  ~I2CBusManager();
  
  std::mutex mutex_;
  int fd_ = -1;
  uint8_t current_address_ = 0xFF;  // Track current slave address
  
  bool setSlaveAddress(uint8_t address);
};
```

**Implementation Details**:
- Singleton pattern (Meyer's singleton for thread safety)
- Single file descriptor to `/dev/i2c-1`
- Mutex protects entire transaction (address switch + I/O)
- Cache current address to avoid redundant ioctl calls
- No ROS2 dependencies - pure C++ for minimal overhead

**File**: `src/i2c_bus_manager.cpp`

Key implementation points:
- `std::lock_guard<std::mutex>` for RAII-style locking
- Check if address change needed before ioctl
- Error handling with descriptive logging (but no ROS2 logger dependency)
- Destructor ensures cleanup

### Phase 2: Refactor I2CDevice Class

**Modify**: `include/ros2_pi_sense_hat/i2c_device.hpp` and `src/i2c_device.cpp`

**Changes**:
- Remove internal file descriptor (`fd_`)
- Store device address for delegation
- Delegate all I2C operations to `I2CBusManager`
- Keep same public API (no changes to calling code)

**Before**:
```cpp
class I2CDevice {
  int fd_;  // Own file descriptor
  bool write(const uint8_t* data, size_t length) {
    return ::write(fd_, data, length) == length;
  }
};
```

**After**:
```cpp
class I2CDevice {
  uint8_t address_;  // Just store address
  bool write(const uint8_t* data, size_t length) {
    return I2CBusManager::getInstance().write(address_, data, length);
  }
};
```

**Rationale**: Minimal changes to existing code. Nodes using `I2CDevice` don't need modification.

### Phase 3: Update Driver Initialization

**Modify**: 
- `src/attiny88_driver.cpp` - LED Matrix and Joystick
- `src/lsm9ds1_driver.cpp` - IMU

**Changes**:
- First node to initialize opens the bus via `I2CBusManager::getInstance().open("/dev/i2c-1")`
- Subsequent nodes just use existing manager instance
- No changes to GPIO handling (stays in drivers)
- No changes to sensor logic

**ATTiny88Driver** (used by both LED Matrix and Joystick):
```cpp
bool ATTiny88Driver::initI2C() {
  // Open bus if not already open (idempotent)
  if (!I2CBusManager::getInstance().open("/dev/i2c-1")) {
    return false;
  }
  // i2c_ now delegates to manager automatically
  return i2c_.open();  // Just stores address, no actual open()
}
```

**LSM9DS1Driver**:
```cpp
bool LSM9DS1Driver::init() {
  // Open bus if not already open
  if (!I2CBusManager::getInstance().open("/dev/i2c-1")) {
    return false;
  }
  // Both I2C devices delegate to manager
  if (!accel_gyro_.open() || !magnetometer_.open()) {
    return false;
  }
  // ... rest of initialization
}
```

### Phase 4: Update Build System

**Modify**: `CMakeLists.txt`

Add new source file to all executables:
```cmake
add_executable(led_matrix_node 
  src/led_matrix_node.cpp 
  src/attiny88_driver.cpp 
  src/i2c_device.cpp
  src/i2c_bus_manager.cpp  # NEW
)

add_executable(joystick_node 
  src/joystick_node.cpp 
  src/attiny88_driver.cpp 
  src/i2c_device.cpp
  src/i2c_bus_manager.cpp  # NEW
)

add_executable(imu_node
  src/imu_node.cpp
  src/lsm9ds1_driver.cpp
  src/i2c_device.cpp
  src/i2c_bus_manager.cpp  # NEW
)
```

### Phase 5: Remove Startup Delays

**Modify**: `demo/run_node.sh`

**Before**:
```bash
ros2 run ros2_pi_sense_hat led_matrix_node &
sleep 3  # No longer needed
ros2 run ros2_pi_sense_hat joystick_node &
sleep 2  # No longer needed
ros2 run ros2_pi_sense_hat imu_node &
```

**After**:
```bash
ros2 run ros2_pi_sense_hat led_matrix_node &
ros2 run ros2_pi_sense_hat joystick_node &
ros2 run ros2_pi_sense_hat imu_node &
```

**Rationale**: Mutex coordination eliminates need for startup delays.

## Testing Strategy

### Unit Testing

1. **I2CBusManager Singleton Test**
   - Verify single instance across multiple calls
   - Test thread safety with concurrent access
   - Verify address caching works correctly

2. **I2CDevice Delegation Test**
   - Mock I2CBusManager to verify delegation
   - Ensure all operations pass correct address

### Integration Testing

1. **Single Node Tests** (baseline - should still work)
   - LED Matrix node alone
   - Joystick node alone
   - IMU node alone

2. **Two Node Tests**
   - LED Matrix + Joystick (both use 0x46)
   - LED Matrix + IMU
   - Joystick + IMU

3. **Three Node Test** (full system)
   - All nodes running simultaneously
   - Stress test: Rapid LED updates + IMU polling + joystick events
   - Monitor for I2C errors in kernel logs: `dmesg | grep i2c`

4. **Performance Test**
   - Measure I2C transaction latency with mutex overhead
   - Target: <5ms overhead
   - Test IMU at 50Hz + LED updates + joystick interrupts

### Validation Criteria

- [ ] No I2C errors in kernel logs (`dmesg`)
- [ ] All nodes start without delays
- [ ] LED matrix updates work during IMU polling
- [ ] Joystick events detected during LED updates
- [ ] IMU data rate stable at configured frequency
- [ ] No corrupted sensor readings
- [ ] No node crashes after 1 hour runtime

## Risk Analysis

### Risk 1: Mutex Contention Latency
**Impact**: High-frequency IMU polling could block LED updates

**Mitigation**: 
- I2C transactions are fast (microseconds at 400kHz)
- Mutex held only during transaction, not processing
- IMU burst read is single transaction (14 bytes)
- LED framebuffer write is single transaction (193 bytes)

**Fallback**: If latency exceeds 5ms, implement priority queue in manager

### Risk 2: Deadlock
**Impact**: System hangs if mutex not released

**Mitigation**:
- Use RAII (`std::lock_guard`) for automatic unlock
- No nested locking (manager is leaf in call hierarchy)
- Timeout on I2C operations (Linux I2C driver handles this)

**Fallback**: Add mutex timeout with error recovery

### Risk 3: GPIO Interrupt Timing
**Impact**: Frame sync or joystick interrupts missed during I2C transactions

**Mitigation**:
- GPIO handling is separate from I2C (different file descriptors)
- `gpiod_line_event_wait()` has timeout, won't block indefinitely
- Interrupt threads are independent of I2C operations

**Validation**: Test GPIO interrupt response time under I2C load

### Risk 4: Singleton Initialization Race
**Impact**: Multiple threads initialize manager simultaneously

**Mitigation**:
- Meyer's singleton is thread-safe in C++11+
- First call to `getInstance()` is serialized by compiler
- `open()` method is idempotent (safe to call multiple times)

**Validation**: Unit test with concurrent initialization

## Performance Considerations

### I2C Transaction Times (at 400kHz)

- **LED Matrix Write**: 193 bytes = ~3.9ms
- **IMU Burst Read**: 14 bytes = ~0.3ms
- **Joystick Read**: 1 byte = ~0.02ms
- **Mutex Overhead**: <0.1ms (uncontended)

### Worst Case Scenario

IMU polling at 50Hz (20ms period):
- IMU read: 0.3ms
- LED update: 3.9ms
- Joystick read: 0.02ms
- Total: 4.22ms per cycle

**Conclusion**: Well within 5ms latency requirement. Even with contention, total time is <5ms.

### Throughput Analysis

Maximum I2C bandwidth at 400kHz: ~50KB/s

Current usage:
- IMU: 14 bytes × 50Hz = 700 bytes/s
- LED: 193 bytes × 10Hz (estimated) = 1930 bytes/s
- Joystick: 1 byte × 100Hz (max) = 100 bytes/s
- **Total**: ~2.7KB/s (5.4% of bandwidth)

**Conclusion**: Plenty of headroom for additional sensors.

## Future Scalability

### Adding New Sensors

To add a new sensor (e.g., HTS221 humidity/temperature at 0x5F):

1. Create driver class using `I2CDevice`
2. Create ROS2 node
3. Add to CMakeLists.txt (include `i2c_bus_manager.cpp`)
4. No changes to existing nodes or manager

**Example**:
```cpp
class HTS221Driver {
  I2CDevice i2c_;  // Automatically uses manager
public:
  HTS221Driver() : i2c_("/dev/i2c-1", 0x5F) {}
  // ... sensor logic
};
```

### Alternative Approaches Considered

1. **ROS2 Service-Based Manager**
   - Pros: Clean ROS2 integration
   - Cons: Service call overhead (~10-50ms), exceeds latency requirement
   - Rejected: Too slow for 50Hz IMU

2. **Separate I2C Bus Per Sensor**
   - Pros: No contention
   - Cons: Raspberry Pi 4 has limited I2C buses, requires hardware mux
   - Rejected: Hardware complexity, not necessary

3. **Kernel Driver Coordination**
   - Pros: Lowest latency
   - Cons: Requires kernel module development, loses direct register access
   - Rejected: Against project philosophy of userspace drivers

4. **Lock-Free Queue**
   - Pros: Better concurrency
   - Cons: Complex implementation, overkill for current load
   - Deferred: Consider if mutex contention becomes issue

## Open Questions

1. **Should manager log errors?**
   - Current plan: Return false on error, let caller log
   - Alternative: Manager logs to stderr (no ROS2 dependency)
   - Decision: Caller logs (maintains separation of concerns)

2. **Should manager track statistics?**
   - Useful for debugging: transaction count, errors, contention
   - Adds complexity and memory overhead
   - Decision: Add only if needed during testing

3. **Should we support multiple I2C buses?**
   - Raspberry Pi has I2C-0 (reserved) and I2C-1 (Sense HAT)
   - Future expansion might use I2C-0 or software I2C
   - Decision: Single bus for now, easy to extend later

## Documentation Updates

After implementation, update:

1. **README.md**
   - Remove note about startup delays
   - Add section on I2C coordination
   - Update "Features" to mention thread-safe I2C

2. **AGENTS.md**
   - Add pattern: "Use I2CBusManager for all I2C access"
   - Document singleton pattern usage

3. **New Doc**: `docs/I2C_ARCHITECTURE.md`
   - Explain bus manager design
   - Threading model
   - How to add new sensors

## Success Criteria

Implementation is complete when:

- [x] I2CBusManager implemented and tested
- [x] I2CDevice refactored to use manager
- [x] All three nodes use manager
- [x] Build system updated
- [x] Startup delays removed
- [x] All integration tests pass
- [x] Performance meets <5ms latency requirement
- [x] Documentation updated
- [x] No I2C errors after 1 hour stress test

## Timeline Estimate

- Phase 1 (Bus Manager): 2-3 hours
- Phase 2 (I2CDevice refactor): 1 hour
- Phase 3 (Driver updates): 1 hour
- Phase 4 (Build system): 15 minutes
- Phase 5 (Remove delays): 5 minutes
- Testing: 2-3 hours
- Documentation: 1 hour

**Total**: 7-9 hours of development + testing time

## Notes

- This is a **minimal, surgical change** - only I2C access layer modified
- Node logic, GPIO handling, and ROS2 interfaces unchanged
- Backward compatible - existing demo scripts work without modification
- Foundation for adding more sensors (pressure, humidity, color, distance)
