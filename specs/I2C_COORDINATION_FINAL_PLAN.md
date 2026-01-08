# I2C Bus Coordination - Final Implementation Plan

## Executive Summary

Implement file-based mutual exclusion using `flock()` at the `I2CDevice` class level to serialize all I2C transactions on `/dev/i2c-1`.
This provides transparent coordination with minimal code changes, zero SD card wear, and sub-millisecond overhead.

## Problem Statement

Multiple ROS2 nodes (LED Matrix, Joystick, IMU) simultaneously access `/dev/i2c-1`, causing:
- Bus arbitration conflicts
- Incomplete transactions
- Data corruption
- Intermittent failures

**Root Cause**: Linux I2C subsystem allows concurrent file descriptors, but hardware cannot handle simultaneous transactions.

## Solution: File-Based Locking

### Why This Approach?

1. **Minimal code changes**: Only `I2CDevice` class modified (~50 lines)
2. **Zero node changes**: LED Matrix, Joystick, IMU nodes unchanged
3. **Automatic cleanup**: Kernel releases locks on crash
4. **No SD card wear**: Lock operations are in-kernel memory only
5. **Low latency**: <2μs overhead per transaction
6. **Scalable**: Future sensors automatically coordinated

### How It Works

```
Node starts → Opens /dev/i2c-1 → Opens /tmp/ros2_sense_hat_i2c1.lock (ONCE)
                                                    ↓
Every I2C transaction:
  flock(LOCK_EX) ← Blocks until exclusive access (in-kernel, no disk I/O)
  I2C operation  ← Safe, no other node can access bus
  flock(LOCK_UN) ← Release immediately
```

### SD Card Safety

- **Lock file creation**: 1 write at first node startup (total: 1 write ever)
- **Lock/unlock operations**: 0 writes (pure kernel memory operations)
- **IMU at 50Hz for 1 year**: Still only 1 SD card write
- **File content**: Never changes after creation (0 bytes)

## Architecture

### Modified Component

**File**: `include/ros2_pi_sense_hat/i2c_device.hpp`

```cpp
class I2CDevice {
public:
  I2CDevice(const std::string& bus, uint8_t address);
  ~I2CDevice();

  bool open();
  void close();

  // Existing API unchanged
  bool write(const uint8_t* data, size_t length);
  bool read(uint8_t* data, size_t length);
  bool writeReg(uint8_t reg, uint8_t value);
  bool readReg(uint8_t reg, uint8_t& value);
  bool readMultiReg(uint8_t reg, uint8_t* data, size_t length, bool auto_increment = true);
  bool readReg16(uint8_t reg, uint16_t& value);

private:
  std::string bus_;
  uint8_t address_;
  int fd_;                                              // I2C device file descriptor
  int lock_fd_;                                         // NEW: Lock file descriptor
  static const std::string lock_file_path_;             // NEW: "/tmp/ros2_sense_hat_i2c1.lock"
};
```

### Unchanged Components

- `led_matrix_node.cpp` - No changes
- `joystick_node.cpp` - No changes
- `imu_node.cpp` - No changes
- `attiny88_driver.cpp` - No changes
- `lsm9ds1_driver.cpp` - No changes
- All ROS2 services and topics - No changes
- GPIO handling (GPIO23, GPIO24) - No changes

## Implementation

### Step 1: Modify Header

**File**: `include/ros2_pi_sense_hat/i2c_device.hpp`

Add private members:
```cpp
private:
  int lock_fd_;                                         // Lock file descriptor
  static const std::string lock_file_path_;             // Lock file path
```

### Step 2: Implement Locking

**File**: `src/i2c_device.cpp`

```cpp
#include <sys/file.h>  // For flock()

const std::string I2CDevice::lock_file_path_ = "/tmp/ros2_sense_hat_i2c1.lock";

bool I2CDevice::open() {
  // Open I2C device (existing code)
  fd_ = ::open(bus_.c_str(), O_RDWR);
  if (fd_ < 0) {
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // NEW: Open lock file (created once, persists across all nodes)
  lock_fd_ = ::open(lock_file_path_.c_str(), O_RDWR | O_CREAT, 0666);
  if (lock_fd_ < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void I2CDevice::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  
  // NEW: Close lock file descriptor
  if (lock_fd_ >= 0) {
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
}

bool I2CDevice::write(const uint8_t* data, size_t length) {
  if (fd_ < 0) {
    return false;
  }

  // NEW: Acquire exclusive lock (blocks until available)
  if (flock(lock_fd_, LOCK_EX) != 0) {
    return false;
  }

  // Perform I2C write
  bool success = ::write(fd_, data, length) == static_cast<ssize_t>(length);

  // NEW: Release lock immediately
  flock(lock_fd_, LOCK_UN);

  return success;
}

bool I2CDevice::read(uint8_t* data, size_t length) {
  if (fd_ < 0) {
    return false;
  }

  // NEW: Acquire exclusive lock
  if (flock(lock_fd_, LOCK_EX) != 0) {
    return false;
  }

  // Perform I2C read
  bool success = ::read(fd_, data, length) == static_cast<ssize_t>(length);

  // NEW: Release lock
  flock(lock_fd_, LOCK_UN);

  return success;
}

bool I2CDevice::writeReg(uint8_t reg, uint8_t value) {
  if (fd_ < 0) {
    return false;
  }

  // NEW: Acquire exclusive lock
  if (flock(lock_fd_, LOCK_EX) != 0) {
    return false;
  }

  uint8_t buffer[2] = {reg, value};
  bool success = ::write(fd_, buffer, 2) == 2;

  // NEW: Release lock
  flock(lock_fd_, LOCK_UN);

  return success;
}

bool I2CDevice::readReg(uint8_t reg, uint8_t& value) {
  if (fd_ < 0) {
    return false;
  }

  // NEW: Acquire exclusive lock for entire transaction
  // (write register address + read value must be atomic)
  if (flock(lock_fd_, LOCK_EX) != 0) {
    return false;
  }

  // Write register address
  if (::write(fd_, &reg, 1) != 1) {
    flock(lock_fd_, LOCK_UN);
    return false;
  }

  // Read register value
  bool success = ::read(fd_, &value, 1) == 1;

  // NEW: Release lock
  flock(lock_fd_, LOCK_UN);

  return success;
}

bool I2CDevice::readMultiReg(uint8_t reg, uint8_t* data, size_t length, bool auto_increment) {
  if (fd_ < 0) {
    return false;
  }

  // NEW: Acquire exclusive lock for entire burst read
  if (flock(lock_fd_, LOCK_EX) != 0) {
    return false;
  }

  // Set auto-increment bit if requested
  uint8_t reg_addr = auto_increment ? (reg | 0x80) : reg;

  // Write register address
  if (::write(fd_, &reg_addr, 1) != 1) {
    flock(lock_fd_, LOCK_UN);
    return false;
  }

  // Read multiple bytes
  bool success = ::read(fd_, data, length) == static_cast<ssize_t>(length);

  // NEW: Release lock
  flock(lock_fd_, LOCK_UN);

  return success;
}

bool I2CDevice::readReg16(uint8_t reg, uint16_t& value) {
  if (fd_ < 0) {
    return false;
  }

  // NEW: Acquire exclusive lock
  if (flock(lock_fd_, LOCK_EX) != 0) {
    return false;
  }

  // Write register address
  if (::write(fd_, &reg, 1) != 1) {
    flock(lock_fd_, LOCK_UN);
    return false;
  }

  // Read 16-bit value (little-endian)
  uint8_t buffer[2];
  if (::read(fd_, buffer, 2) != 2) {
    flock(lock_fd_, LOCK_UN);
    return false;
  }

  value = buffer[0] | (buffer[1] << 8);

  // NEW: Release lock
  flock(lock_fd_, LOCK_UN);

  return true;
}
```

### Step 3: Initialize Lock File Descriptor

**File**: `src/i2c_device.cpp` (constructor)

```cpp
I2CDevice::I2CDevice(const std::string& bus, uint8_t address)
  : bus_(bus), address_(address), fd_(-1), lock_fd_(-1) {  // NEW: Initialize lock_fd_
}

I2CDevice::~I2CDevice() {
  close();
}
```

## Build System

**No changes required** - `i2c_device.cpp` already compiled into all nodes.

Just need to rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_pi_sense_hat
source install/setup.bash
```

## Testing Strategy

### Phase 1: Single Node Validation
```bash
# Test each node individually
ros2 run ros2_pi_sense_hat led_matrix_node
# Verify: Lock file created at /tmp/ros2_sense_hat_i2c1.lock
# Verify: LED matrix works normally

ros2 run ros2_pi_sense_hat joystick_node
# Verify: Joystick events work

ros2 run ros2_pi_sense_hat imu_node
# Verify: IMU publishes at 10Hz
```

### Phase 2: Concurrent Operation
```bash
# Start all nodes simultaneously (no delays)
ros2 run ros2_pi_sense_hat led_matrix_node &
ros2 run ros2_pi_sense_hat joystick_node &
ros2 run ros2_pi_sense_hat imu_node &

# Run for 10+ minutes, monitor for errors
dmesg | grep i2c  # Should show no errors
```

### Phase 3: Stress Test
```bash
# Rapid LED updates
python3 demo_patterns.py rainbow &

# High IMU rate (if configurable to 50Hz)
ros2 param set /imu_node rate 50

# Rapid joystick presses
# Press buttons repeatedly

# Monitor:
ros2 topic hz /sense_hat/imu/data_raw  # Should maintain rate
ros2 topic echo /sense_hat/joystick/events  # Should not drop events
# LED display should remain smooth
```

### Phase 4: Crash Recovery
```bash
# Start all nodes
ros2 run ros2_pi_sense_hat led_matrix_node &
ros2 run ros2_pi_sense_hat joystick_node &
ros2 run ros2_pi_sense_hat imu_node &

# Kill one node during I2C transaction
kill -9 <led_matrix_pid>

# Verify: Other nodes continue working
# Verify: Lock released automatically
```

## Performance Analysis

### Lock Overhead

| Operation | I2C Time (400kHz) | Lock Overhead | Total | Overhead % |
|-----------|-------------------|---------------|-------|------------|
| LED write (193 bytes) | 3.9ms | 2μs | 3.902ms | 0.05% |
| IMU read (14 bytes) | 0.3ms | 2μs | 0.302ms | 0.66% |
| Joystick read (1 byte) | 0.02ms | 2μs | 0.022ms | 10% |

**Conclusion**: Lock overhead is negligible, well within 5ms requirement.

### Worst Case Contention

All three nodes request I2C simultaneously:
1. IMU acquires lock, reads 14 bytes: 0.302ms
2. LED waits, acquires lock, writes 193 bytes: 3.902ms
3. Joystick waits, acquires lock, reads 1 byte: 0.022ms

**Total**: 4.226ms (within 5ms requirement)

### SD Card Impact

- **Lock file creation**: 1 write (at first node startup)
- **Lock operations**: 0 writes (in-kernel memory only)
- **IMU at 50Hz for 1 year**: 1,576,800,000 lock operations = 0 SD card writes
- **SD card lifespan**: Unaffected

## Success Criteria

✅ **Functional**:
- All three nodes run simultaneously without I2C errors
- No data corruption in sensor readings
- No node crashes due to I2C conflicts
- GPIO interrupts continue working

✅ **Performance**:
- Lock overhead < 5ms (target: < 2μs achieved)
- IMU publishes at configured rate without drops
- LED matrix updates remain smooth
- Joystick events trigger reliably

✅ **Reliability**:
- System runs for 24+ hours without failures
- Node crashes don't affect other nodes
- Lock file automatically cleaned on reboot (/tmp)

✅ **Maintainability**:
- Code changes isolated to I2CDevice class
- Future sensors automatically coordinated
- No changes to node logic or ROS2 interfaces

## Future Sensors

Adding new sensors (HTS221, LPS25H, TCS3400, VL53L0X):

1. Create driver using `I2CDevice` class
2. Create ROS2 node
3. **No coordination code needed** - automatically uses file locking

Example:
```cpp
class HTS221Driver {
  I2CDevice i2c_;  // Automatically coordinated
public:
  HTS221Driver() : i2c_("/dev/i2c-1", 0x5F) {}
  // ... sensor logic
};
```

## Migration Path

### Step 1: Implement Changes
- Modify `include/ros2_pi_sense_hat/i2c_device.hpp`
- Modify `src/i2c_device.cpp`
- Build: `colcon build --packages-select ros2_pi_sense_hat`

### Step 2: Test Single Nodes
- Test LED Matrix node
- Test Joystick node
- Test IMU node
- Verify lock file created

### Step 3: Test Concurrent
- Start all three nodes simultaneously
- Run stress tests
- Monitor for 1+ hour

### Step 4: Remove Startup Delays (Optional)
- Update launch scripts to remove delays
- Verify simultaneous startup works

### Step 5: Document
- Update README.md
- Note that I2C coordination is transparent
- Document lock file location for troubleshooting

## Troubleshooting

### Lock File Issues

**Problem**: Lock file not created
```bash
# Check /tmp permissions
ls -ld /tmp
# Should be: drwxrwxrwt (1777)

# Manually create if needed
touch /tmp/ros2_sense_hat_i2c1.lock
chmod 666 /tmp/ros2_sense_hat_i2c1.lock
```

**Problem**: Permission denied
```bash
# Fix permissions
sudo chmod 666 /tmp/ros2_sense_hat_i2c1.lock
```

**Problem**: Stale lock file after crash
```bash
# Not an issue - kernel releases locks automatically
# But can delete if desired:
rm /tmp/ros2_sense_hat_i2c1.lock
```

### Debugging

**Check if locking is working**:
```bash
# In one terminal
ros2 run ros2_pi_sense_hat led_matrix_node

# In another terminal
lsof /tmp/ros2_sense_hat_i2c1.lock
# Should show led_matrix_node has file open

# Start another node
ros2 run ros2_pi_sense_hat imu_node

# Check again
lsof /tmp/ros2_sense_hat_i2c1.lock
# Should show both nodes have file open
```

**Monitor I2C errors**:
```bash
# Watch kernel logs
dmesg -w | grep i2c

# Should see no errors during operation
```

## Alternatives Considered and Rejected

### 1. ROS2 Service-Based Manager (Gemini's Plan)
- **Rejected**: 10-50ms service call latency exceeds 5ms requirement
- **Rejected**: Major refactoring of all nodes
- **Rejected**: Violates direct hardware access philosophy

### 2. Zenoh Queryables
- **Rejected**: Massive dependency overhead (Zenoh + bridge)
- **Rejected**: Unknown latency characteristics
- **Rejected**: Severe over-engineering for single-machine coordination
- **Rejected**: Adds learning curve during ROS2 learning phase

### 3. C++ Singleton Manager (Kiro's Plan)
- **Rejected**: More invasive than file locking
- **Rejected**: Requires refactoring I2CDevice internals
- **Rejected**: More complex than necessary

### 4. fcntl() on Device File (Copilot's Plan)
- **Considered**: Similar to flock() but on device file
- **Rejected**: Device file locking behavior can be platform-specific
- **Rejected**: Separate lock file is more portable and explicit

## Why File Locking Wins

| Criterion | File Locking | ROS2 Service | Zenoh | Singleton |
|-----------|--------------|--------------|-------|-----------|
| Lines of code | ~50 | ~500+ | ~500+ | ~200 |
| Dependencies | None | None | Zenoh + bridge | None |
| Latency | <2μs | 10-50ms | Unknown | ~1μs |
| Node changes | 0 | All nodes | All nodes | 0 |
| SD card writes | 1 (once) | 0 | 0 | 0 |
| Complexity | Very Low | High | Very High | Medium |
| Learning curve | None | None | High | Low |

**File locking is the clear winner**: Minimal code, zero dependencies, negligible overhead, no node changes.

## References

**System Calls**:
- `flock(2)` - apply or remove an advisory lock on an open file
- `open(2)` - open file descriptor
- `close(2)` - close file descriptor

**Key Properties**:
- Advisory locks (all processes must cooperate)
- Exclusive locks (LOCK_EX) - only one holder at a time
- Blocking locks (waits until available)
- Automatic cleanup on process termination
- No disk I/O after file creation

**Project Documentation**:
- Problem specification: `specs/I2C_COORDINATION_PLAN.md`
- Alternative plans: `specs/I2C_COORDINATION_PLAN_*.md`
- ATTINY88 Protocol: `docs/ATTINY88_PROTOCOL.md`
- LSM9DS1 Datasheet: `docs/datasheets/ST-LSM9DS1.md`
