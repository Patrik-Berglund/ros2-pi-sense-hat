# Implementation Plan: I2C Coordination Manager

Based on the specification in `specs/I2C_COORDINATION_PLAN.md`.

## Approach
We will implement a centralized ROS2 node, named `i2c_manager`, which will act as the sole gatekeeper for the `/dev/i2c-1` bus. This approach resolves the core problem of concurrent, uncoordinated access from multiple nodes.

Existing hardware drivers (`LSM9DS1Driver`, `ATTiny88Driver`) will be refactored from direct hardware controllers into clients of the `i2c_manager`. All I2C read/write operations will be requested via a new, batched ROS2 service. This ensures that all I2C transactions are serialized, preventing bus conflicts, while minimizing the performance overhead of ROS2 communication.

## Architecture

### 1. Central I2C Manager Node (`i2c_manager_node`)
- **Type**: It will be implemented as a `rclcpp_lifecycle::LifecycleNode`. This is crucial for robustly managing the I2C bus file descriptor, ensuring it's opened only upon activation and gracefully closed on deactivation or shutdown.
- **Responsibility**: This node will be the only component in the system that directly accesses `/dev/i2c-1` using the existing `I2CDevice` helper class.
- **Concurrency**: Inside the node, a `std::mutex` will protect the I2C bus. The service callback will lock this mutex before executing a transaction batch and unlock it upon completion, guaranteeing atomic execution.
- **Service Provided**: It will offer a single ROS2 service, e.g., `perform_transactions`, for executing a sequence of I2C operations.

### 2. New ROS2 Service and Message Types
To facilitate communication with the manager, the following new interface files will be created in the `srv/` and `msg/` directories:

- **`msg/I2COperation.msg`**: Describes a single atomic I2C operation.
  ```ros2
  # Type of operation
  uint8 OP_READ=0
  uint8 OP_WRITE=1
  uint8 OP_WRITE_READ=2
  uint8 op_type
  
  # I2C device address
  uint8 address
  
  # Data for WRITE and WRITE_READ operations
  uint8[] write_data
  
  # Number of bytes for READ and WRITE_READ operations
  uint8 read_length
  ```

- **`msg/I2CResult.msg`**: Contains the outcome of a single read operation.
  ```ros2
  # Data read from the I2C bus
  uint8[] read_data
  ```

- **`srv/I2CTransaction.srv`**: Defines the batch operation service.
  ```ros2
  # Request: A list of operations to perform atomically
  msg/I2COperation[] operations
  ---
  # Response: Success status and results from read operations
  bool success
  string error_message
  msg/I2CResult[] results
  ```

### 3. Driver Refactoring
The existing drivers will be modified to act as clients:
- **Remove `I2CDevice`**: The `I2CDevice` member and direct hardware access code will be removed from `lsm9ds1_driver` and `attiny88_driver`.
- **Add Service Client**: A `rclcpp::Client<srv::I2CTransaction>::SharedPtr` will be added to each driver. The drivers' constructors or init methods will be updated to accept a node handle to create this client.
- **Update Driver Methods**: Functions like `LSM9DS1Driver::readAllSensors` will be rewritten. Instead of performing multiple I2C calls, they will construct a single request containing an array of `I2COperation` messages and make one call to the `perform_transactions` service. The response will contain all the required data, which is then parsed.

### 4. GPIO Interrupt Handling
As required by the specification, GPIO interrupt handling will remain in the respective drivers (e.g., `ATTiny88Driver` for joystick events). When a GPIO interrupt is detected, the driver's handler will now trigger a call to the `i2c_manager` service to read the relevant data, rather than accessing the I2C bus directly.

## Key Decisions
- **Batched Transactions**: The choice of a batched service interface is the cornerstone of this plan. It avoids the high latency of making multiple, separate service calls for complex sensor reads, addressing the performance requirements.
- **Lifecycle Node for Manager**: Using a `LifecycleNode` is a best practice for managing critical, exclusive resources like a hardware bus. It makes the system more robust and predictable.
- **Custom Interface Definitions**: Creating specific `.srv` and `.msg` files provides a clear, stable, and reusable API for all hardware interactions within the ROS2 ecosystem.

## Risks & Trade-offs
- **Implementation Complexity**: This architecture introduces new nodes and interfaces, which adds initial development complexity compared to a simpler (but incorrect) locking mechanism. This is a necessary trade-off for a robust, scalable, and correct solution.
- **Single Point of Failure**: The `i2c_manager` is a central component. Its failure would disable all I2C-based hardware. This risk is mitigated by using a Lifecycle Node for controlled startup/shutdown and implementing thorough error handling.
- **Refactoring Effort**: Modifying the existing drivers is a significant task that will require careful implementation and unit testing to ensure correctness.

## Open Questions
- **Interface Naming**: The final names for the node, services, and messages can be refined. The proposed names (`i2c_manager`, `I2CTransaction`, etc.) are clear and will be used as a starting point.
- **Error Propagation**: The service response includes a `success` flag and an `error_message` string. This is sufficient for initial implementation. In the future, this could be expanded to include structured error codes (e.g., for NACK, timeout) if finer-grained error handling is required by the client nodes.
