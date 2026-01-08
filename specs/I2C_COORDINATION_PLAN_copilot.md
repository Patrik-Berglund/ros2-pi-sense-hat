# Implementation Plan: I2C Coordination

## Approach
- Introduce cross-process mutual exclusion for `/dev/i2c-1` using POSIX advisory file locks (fcntl) applied on the I2C device file descriptor inside I2CDevice.
- Keep all existing node/public ROS2 interfaces unchanged; only touch the shared I2CDevice and its composite register operations to serialize transactions.
- Use a small RAII BusLock helper to acquire a blocking write lock per transaction; wrap multi-step register ops (write address + read data) in a single critical section to avoid interleaving.

## Architecture
- Centralize bus arbitration in `I2CDevice` (src/i2c_device.cpp, include/ros2_pi_sense_hat/i2c_device.hpp):
  - Add private methods: `bool acquire_bus_lock(); void release_bus_lock();` implemented with `fcntl(F_SETLKW)` on `fd_` locking the whole file.
  - Add `class BusLockGuard` (RAII) to handle lock/unlock safely, even on early returns.
  - Primitive ops (`read`, `write`) take a lock internally for single-call transactions.
  - Composite ops (`readReg`, `readMultiReg`, `readReg16`) take one outer lock spanning both phases (register write + data read).
- No changes to ATTiny88Driver/LSM9DS1Driver APIs or nodes (led_matrix_node, joystick_node, imu_node) beyond benefiting from serialized access.
- Error handling: on `EINTR`/`EAGAIN` retry lock and I/O; on I/O failure close/reopen fd and re-issue `I2C_SLAVE` selection, then retry once.

## Key Decisions
- Use fcntl advisory locks on the device inode to coordinate across processes (simplest, zero extra daemons, minimal latency).
- Always take an exclusive (write) lock for any transaction to keep ordering simple and safe across mixed read/write users.
- Block while waiting for the bus (F_SETLKW) to avoid busy-wait and CPU spin; keep transactions small to respect latency goals.

## Risks & Trade-offs
- Advisory locks require all participants to cooperate: verified that all I2C access goes through `I2CDevice` (grep shows only i2c_device.* in use). If any future code bypasses it, conflicts could reappear.
- Device-node fcntl locks should work cross-process on the same inode; if platform/kernel specifics break this, fallback is a well-known lock file (e.g., `/tmp/ros2_pi_sense_hat.i2c-1.lock`) using fcntl on that file.
- Slight added latency (microseconds to a few ms) when contending; acceptable per requirement (<5ms).

## Implementation Steps
1. Add BusLock to I2CDevice:
   - Update header with forward declaration and (private) helpers; keep public API unchanged.
   - Implement `acquire_bus_lock`/`release_bus_lock` with `struct flock` (type=F_WRLCK, whence=SEEK_SET, start=0, len=0) and F_SETLKW; handle EINTR.
2. Wrap operations:
   - `write`/`read`: take RAII lock internally around the syscall.
   - `readReg`, `readMultiReg`, `readReg16`: take one outer RAII lock for the entire sequence; call internal non-locking primitives or raw syscalls to avoid nested locks.
3. Robustness:
   - On failed I/O, retry once after `usleep(1000)`; on persistent failure, return false.
   - Ensure `close()` always releases any held lock via RAII scope.
4. Build and run on target (Raspberry Pi 4):
   - `colcon build --packages-select ros2_pi_sense_hat && source install/setup.bash`.
   - Launch led_matrix_node, joystick_node, imu_node together; verify no I2C errors.
5. Stress test:
   - Hammer LED updates (demo/test_pixels_fast.py), hold joystick, and run IMU at >=10Hz; monitor for errors and throughput.
6. Telemetry & logs:
   - Add minimal RCLCPP_DEBUG logs on lock wait > 2ms and on retries (guarded by log level) for diagnostics.

## Validation Criteria
- No I2C bus errors across 10-minute stress run with all three nodes active.
- LED updates remain visually smooth; joystick events not dropped; IMU publishes at configured rate (10Hz default).
- Measured additional latency per transaction < 5ms and typically < 1ms under contention.

## Alternatives Considered
- I2C Manager Node (broker): central daemon exposing services for read/write. Rejected for now due to larger refactor and interface changes.
- POSIX named semaphore (`sem_open`) for cross-process lock: viable, but fcntl on the device file avoids extra global namespace and failure modes.

## Open Questions
- Confirm fcntl advisory locks on `/dev/i2c-1` behave consistently on Ubuntu 24.04 on Pi; if not, switch to dedicated lock file approach.
- Do any demos/tests access I2C directly outside `I2CDevice`? If so, migrate them or document restriction.
