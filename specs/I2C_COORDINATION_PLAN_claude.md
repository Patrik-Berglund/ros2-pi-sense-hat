# I2C Bus Coordination Implementation Plan

## Executive Summary

This plan addresses the I2C bus contention issue where multiple ROS2 nodes (LED Matrix, Joystick, IMU) simultaneously access `/dev/i2c-1`, causing race conditions and unreliable operation.
The solution uses **file-based mutual exclusion** at the `I2CDevice` class level to serialize all I2C transactions while maintaining node independence and preserving GPIO interrupt functionality.

## Approach

### High-Level Strategy

Implement transparent I2C bus coordination by adding a file-based lock mechanism to the existing `I2CDevice` class.
Each I2C transaction will acquire an exclusive lock before accessing the bus and release it immediately after completion.
This provides mutual exclusion without changing node architecture or ROS2 interfaces.

### Why File-Based Locking?

**File-based locking with flock() is the optimal solution** because:

1. **Automatic cleanup on crash**: If a process crashes while holding the lock, the kernel automatically releases it
2. **Low latency**: Lock acquisition is microseconds (well under 5ms requirement)
3. **Standard POSIX API**: No external dependencies beyond standard C library
4. **Process-safe**: Works across multiple processes (unlike pthread mutexes which are thread-safe only)
5. **Simple implementation**: Minimal code changes to existing architecture
6. **Transparent to nodes**: No changes to node code, service interfaces, or GPIO handling

### Rejected Alternatives

**Centralized I2C Manager Node**: Rejected due to high ROS2 service call latency (10-50ms typical) and violation of node independence requirement.

**Shared Memory Mutex**: Rejected due to complexity of cleanup on crashes (requires careful handling of robust mutexes, shared memory lifecycle) and no significant performance benefit over file locks.

**Component Composition**: Rejected because it couples all sensors into one process, defeating the architectural goal of independent sensor nodes.

## Architecture

### Modified Components

#### 1. I2CDevice Class Enhancement

**File**: `include/ros2_pi_sense_hat/i2c_device.hpp` and `src/i2c_device.cpp`

**Changes**:
- Add private member: `int lock_fd_` for lock file descriptor
- Add private member: `static std::string lock_file_path_` for lock file location
- Modify `open()`: Create/open lock file `/tmp/ros2_sense_hat_i2c1.lock`
- Modify all I2C operations (`write()`, `read()`, `writeReg()`, `readReg()`, `readMultiReg()`, `readReg16()`):
  - Acquire exclusive lock with `flock(lock_fd_, LOCK_EX)` before I2C operation
  - Perform I2C transaction
  - Release lock with `flock(lock_fd_, LOCK_UN)` after I2C operation
- Modify `close()`: Close lock file descriptor
- Add error handling for lock acquisition failures

**Lock Scope**: Each individual I2C transaction is atomic (write register, read register, burst read, etc.).
Complex operations like "write register address then read data" in `readReg()` are locked as a single unit to prevent interleaving.

#### 2. Node Architecture (Unchanged)

**No changes required** to:
- `led_matrix_node.cpp` - ROS2 service callbacks unchanged
- `joystick_node.cpp` - GPIO interrupt thread unchanged
- `imu_node.cpp` - Timer-based polling unchanged
- `attiny88_driver.cpp` - GPIO handling unchanged
- `lsm9ds1_driver.cpp` - Sensor logic unchanged

Nodes continue to operate independently, unaware of the coordination happening at the I2C layer.

### Data Flow

```
Node Layer (unchanged):
  LED Matrix Node → ATTiny88Driver → I2CDevice
  Joystick Node   → ATTiny88Driver → I2CDevice
  IMU Node        → LSM9DS1Driver  → I2CDevice
                                         ↓
I2C Device Layer (modified):
  I2CDevice::write(data)
    ├─ flock(lock_fd, LOCK_EX)  ← Acquire exclusive lock
    ├─ ::write(fd_, data)        ← Perform I2C write
    └─ flock(lock_fd, LOCK_UN)  ← Release lock
                                         ↓
Kernel Layer:
  /dev/i2c-1 ← Only one process can access at a time
```

### Lock File Semantics

**Lock File**: `/tmp/ros2_sense_hat_i2c1.lock`

**Lifecycle**:
1. Each I2CDevice instance opens the lock file in `open()` (file persists across all instances)
2. Before each I2C operation: `flock(lock_fd_, LOCK_EX)` blocks until lock available
3. After each I2C operation: `flock(lock_fd_, LOCK_UN)` releases lock immediately
4. On node shutdown: Lock file descriptor closed automatically
5. On crash: Kernel releases lock automatically

**Properties**:
- Lock is **advisory** (all processes must cooperate by using flock)
- Lock is **exclusive** (LOCK_EX) - only one holder at a time
- Lock is **blocking** (no timeout) - simpler than non-blocking with retries
- Lock granularity is **per-transaction** (not per-node session)

## Key Decisions

### 1. Lock Granularity: Per-Transaction vs. Per-Operation-Sequence

**Decision**: Lock each atomic I2C transaction separately (write, read, writeReg, readReg, etc.)

**Rationale**:
- Minimizes lock hold time (typically 100-500 microseconds for I2C transaction at 400kHz)
- Maximizes concurrency between nodes
- IMU burst reads (14 bytes) complete in ~400μs, well within latency budget

**Consideration**: Complex operations like "write register address, then read data" must be locked as one transaction to prevent another node from interleaving I2C operations between them.
This is already the case in `readReg()` which calls `write()` then `read()` - both will be locked as one unit.

### 2. Lock Type: Blocking vs. Non-Blocking

**Decision**: Use blocking locks (`LOCK_EX` without `LOCK_NB`)

**Rationale**:
- Simpler implementation (no retry logic)
- Natural backpressure (slow consumers don't need timeouts)
- Lock hold times are microseconds, blocking is negligible
- Deadlock impossible (single lock, no lock ordering issues)

**Trade-off**: A misbehaving node that holds the lock indefinitely would block all I2C access.
However, this is unlikely since:
- I2C operations are short (hardware timeout ~100ms)
- Kernel releases lock on process termination
- This failure mode is no worse than current state (I2C bus lockup)

### 3. Error Handling Strategy

**Decision**: Log errors but don't retry automatically

**Approach**:
- If `flock()` fails: Log error, return false from I2C operation
- If I2C transaction fails while holding lock: Release lock, return false
- Node-level error handling unchanged (already logs warnings/errors)

**Rationale**: Retry logic at this level could cause cascading delays.
Better to fail fast and let nodes handle retries if needed.

### 4. GPIO Interrupt Preservation

**Decision**: No changes to GPIO handling - interrupts remain in node code

**Rationale**:
- GPIO operations (libgpiod) don't use I2C bus
- GPIO interrupts trigger I2C reads, but I2C reads are now mutex-protected
- LED Matrix: GPIO24 frame sync remains in `attiny88_driver.cpp`
- Joystick: GPIO23 interrupt thread remains in `joystick_node.cpp`
- Clean separation: GPIO events trigger application logic, I2C coordination is transparent infrastructure

### 5. Backward Compatibility

**Decision**: Zero changes to ROS2 interfaces and node logic

**What stays the same**:
- All ROS2 topics, services, and message types
- Node startup order (can still use delays if desired, but no longer required)
- Driver APIs (ATTiny88Driver, LSM9DS1Driver)
- GPIO interrupt handling

**What changes**:
- Only internal implementation of `I2CDevice` class
- Lock file created in `/tmp/` (cleanup on reboot)

## Implementation Details

### Code Changes

#### Modified: `include/ros2_pi_sense_hat/i2c_device.hpp`

```cpp
class I2CDevice {
public:
  I2CDevice(const std::string& bus, uint8_t address);
  ~I2CDevice();

  bool open();
  void close();

  // ... existing public API unchanged ...

private:
  std::string bus_;
  uint8_t address_;
  int fd_;          // I2C device file descriptor
  int lock_fd_;     // Lock file descriptor (NEW)

  static const std::string lock_file_path_;  // "/tmp/ros2_sense_hat_i2c1.lock" (NEW)

  bool acquireLock();    // NEW
  void releaseLock();    // NEW
};
```

#### Modified: `src/i2c_device.cpp`

**Key modifications**:

```cpp
const std::string I2CDevice::lock_file_path_ = "/tmp/ros2_sense_hat_i2c1.lock";

bool I2CDevice::open() {
  // Open I2C device (existing code)
  fd_ = ::open(bus_.c_str(), O_RDWR);
  if (fd_ < 0) return false;

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Open lock file (NEW)
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
  if (lock_fd_ >= 0) {  // NEW
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
}

bool I2CDevice::write(const uint8_t* data, size_t length) {
  if (fd_ < 0) return false;

  // Acquire exclusive lock (NEW)
  if (flock(lock_fd_, LOCK_EX) != 0) {
    return false;
  }

  // Perform I2C write
  bool success = ::write(fd_, data, length) == static_cast<ssize_t>(length);

  // Release lock (NEW)
  flock(lock_fd_, LOCK_UN);

  return success;
}

// Similar pattern for read(), writeReg(), readReg(), readMultiReg(), readReg16()
```

**Lock pattern applied to all I2C operations**:
1. `write()` - bulk write (LED matrix framebuffer, etc.)
2. `read()` - bulk read
3. `writeReg()` - single register write
4. `readReg()` - write register address + read value (locked as one unit)
5. `readMultiReg()` - burst read (IMU sensor data)
6. `readReg16()` - 16-bit register read

### Testing Strategy

#### Phase 1: Unit Testing (Single Node)
1. Test each node individually with new I2CDevice implementation
2. Verify lock file created in `/tmp/`
3. Verify functional behavior unchanged

#### Phase 2: Concurrent Testing (Multiple Nodes)
1. Start all three nodes simultaneously (no startup delays)
2. Run for extended period (1+ hours)
3. Monitor for I2C errors, data corruption, crashes
4. Verify performance:
   - IMU publishes at 10Hz consistently
   - LED matrix updates remain smooth
   - Joystick events trigger reliably

#### Phase 3: Stress Testing
1. Add rapid LED matrix updates (image streaming)
2. Increase IMU rate to 50Hz
3. Rapid joystick button presses
4. Verify no dropped messages or corrupted data

#### Phase 4: Failure Testing
1. Kill node while I2C transaction in progress (verify lock released)
2. SIGKILL node (verify lock released)
3. Verify other nodes continue operating

### Performance Analysis

**Lock Overhead Estimation**:
- flock() syscall: ~1-2 microseconds
- I2C write 193 bytes at 400kHz: ~4ms
- I2C read 14 bytes at 400kHz: ~400μs
- Lock overhead: < 0.1% of transaction time

**Worst Case Latency** (all three nodes contending):
1. IMU requests lock, acquires immediately, reads 14 bytes (400μs)
2. LED Matrix requests lock, waits for IMU, acquires, writes 193 bytes (4ms)
3. Joystick requests lock, waits for LED, acquires, reads 1 byte (50μs)

Total time: ~4.5ms (within 5ms requirement)

**Typical Case**: Nodes operate asynchronously, lock contention rare.
Most transactions acquire lock immediately.

## Risks & Trade-offs

### Risks

**Risk 1: Lock File Permissions**
- **Issue**: If lock file created with wrong permissions, other nodes can't access
- **Mitigation**: Use mode 0666 (world read/write) when creating lock file
- **Fallback**: Document manual cleanup if needed (`rm /tmp/ros2_sense_hat_i2c1.lock`)

**Risk 2: /tmp Filesystem Full**
- **Issue**: Lock file creation could fail if `/tmp` is full
- **Mitigation**: Lock file is 0 bytes, extremely unlikely to fail
- **Detection**: `open()` will fail and be logged

**Risk 3: Misbehaving Node Holds Lock Indefinitely**
- **Issue**: Bug in node or I2C driver could hang while holding lock
- **Mitigation**: I2C hardware timeout (~100ms) prevents true indefinite hang
- **Detection**: Other nodes will log "waiting for I2C lock" if this occurs
- **Recovery**: Kill misbehaving node, lock is released automatically

**Risk 4: Performance Regression on Single Node**
- **Issue**: Lock overhead might slow down single-node operation
- **Mitigation**: Overhead is negligible (~1μs per transaction)
- **Measurement**: Benchmark before/after on IMU publish rate

### Trade-offs

**Trade-off 1: Lock Granularity**
- **Finer-grained (per-transaction)**: Better concurrency, more lock operations
- **Coarser-grained (per-node-operation)**: Fewer lock operations, more blocking
- **Choice**: Per-transaction (prioritize concurrency)

**Trade-off 2: Lock File Location**
- **Option A**: `/tmp/` - cleaned on reboot, standard temporary location
- **Option B**: `/var/run/` - standard for runtime state, cleaned on reboot
- **Option C**: `/var/lock/` - standard for lock files, persists across reboots
- **Choice**: `/tmp/` (simpler, automatic cleanup, no permissions issues)

**Trade-off 3: Error Handling**
- **Option A**: Retry on lock failure
- **Option B**: Fail immediately
- **Choice**: Fail immediately (simpler, lets nodes decide retry policy)

## Open Questions

### Q1: Should we add lock acquisition timeout?

**Context**: Currently using blocking locks (wait forever).
Could use `LOCK_NB` (non-blocking) with retry loop and timeout.

**Recommendation**: Start with blocking locks.
Only add timeout if testing reveals a need.
Blocking is simpler and sufficient given short lock hold times.

### Q2: Should we add lock diagnostics/monitoring?

**Context**: Could add logging of lock wait times, lock hold times, contention metrics.

**Recommendation**: Add basic logging:
- Log warning if lock acquisition takes > 10ms (indicates contention)
- Log error if lock acquisition fails
- Don't log on normal fast path (would spam logs)

**Implementation**: Optional compile-time debug flag for detailed lock statistics.

### Q3: What about future sensors (pressure, humidity, color)?

**Context**: More sensors will be added, all using I2C bus 1.

**Impact**: Zero additional work - they'll use `I2CDevice` class and automatically get coordination.

**Validation**: This is the whole point of the design - easy scalability.

### Q4: Should startup delays be removed?

**Context**: Current launch sequence has 3-second delays between nodes to reduce conflicts.

**Recommendation**: Keep delays initially during testing, remove once coordination is validated.
Delays are harmless and provide margin during development.

**Final State**: No delays required, all nodes start simultaneously.

### Q5: Should we add I2C bus health monitoring?

**Context**: Could detect bus lockups, transaction failures, etc.

**Recommendation**: Out of scope for coordination implementation.
Node-level error handling already exists.
Could be future enhancement if issues arise.

## Migration Path

### Step 1: Implement I2CDevice Changes
- Modify `i2c_device.hpp` and `i2c_device.cpp`
- Add lock file handling to `open()` and `close()`
- Add lock acquire/release to all I2C operations
- Build and verify compilation

### Step 2: Test Single Node
- Test LED Matrix node in isolation
- Verify lock file created
- Verify functional behavior unchanged
- Repeat for Joystick and IMU nodes

### Step 3: Test Multi-Node Sequential
- Start LED Matrix node
- Start Joystick node (verify both work)
- Start IMU node (verify all three work)
- Observe for 10+ minutes

### Step 4: Test Multi-Node Concurrent
- Kill all nodes
- Start all three simultaneously
- Run stress tests (rapid LED updates, high IMU rate, etc.)
- Verify stable operation

### Step 5: Remove Startup Delays (Optional)
- Update launch files to start nodes in parallel
- Verify no issues with simultaneous startup

### Step 6: Add Future Sensors
- Implement additional sensors using I2CDevice class
- Verify coordination works with 4+ nodes

## Success Criteria

✅ **Functional Requirements Met**:
1. All three nodes run simultaneously without I2C errors
2. No data corruption in sensor readings
3. No node crashes due to I2C conflicts
4. GPIO interrupts (GPIO23, GPIO24) continue working correctly
5. ROS2 interfaces unchanged (backward compatible)

✅ **Performance Requirements Met**:
1. Lock overhead < 5ms per transaction (target: < 1ms)
2. IMU publishes at 10Hz without drops
3. LED matrix updates remain smooth
4. Joystick events trigger with < 100ms latency

✅ **Reliability Requirements Met**:
1. System runs for 24+ hours without failures
2. Node crashes don't affect other nodes (lock released)
3. Lock file automatically cleaned up on reboot

✅ **Maintainability Requirements Met**:
1. Code changes isolated to `I2CDevice` class
2. Clear separation between I2C coordination and node logic
3. Easy to add new sensors (no additional coordination code)
4. Debugging straightforward (lock contention logged)

## Future Enhancements

**Not in scope for initial implementation**:

1. **Priority-based locking**: Give certain nodes priority (e.g., joystick interrupts)
2. **Lock statistics**: Track contention, wait times, throughput
3. **Dynamic lock timeout**: Adjust timeout based on transaction type
4. **I2C bus error recovery**: Detect and recover from bus lockup
5. **Multi-bus support**: If future hardware has multiple I2C buses

These can be added later if profiling reveals they're needed.

## References

**System Calls**:
- `flock(2)` - apply or remove an advisory lock on an open file
- `open(2)` - open file descriptor
- `close(2)` - close file descriptor

**I2C Documentation**:
- Linux I2C subsystem: `/Documentation/i2c/dev-interface.rst`
- i2c-dev kernel module documentation

**ROS2 Patterns**:
- Component-based nodes: https://docs.ros.org/en/rolling/Tutorials/Composition.html
- Lifecycle nodes: https://design.ros2.org/articles/node_lifecycle.html

**Project Documentation**:
- ATTINY88 Protocol: `docs/ATTINY88_PROTOCOL.md`
- LSM9DS1 Datasheet: `docs/datasheets/ST-LSM9DS1.md`
- Specification: `specs/I2C_COORDINATION_PLAN.md`
