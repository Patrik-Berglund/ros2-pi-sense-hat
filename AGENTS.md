# AI Agent Guidelines for ROS2 Projects

## Development Environment

**Current Environment**: WSL (Windows Subsystem for Linux) - for code editing and planning

**Target System**: Raspberry Pi 4 (4GB RAM, 16GB SD card)
- OS: Ubuntu 24.04 Server (no GUI/X11)
- ROS2: Kilted
- When running on target: Can build, run, and test code directly
- Hardware access: I2C, GPIO, sensors available on target only
- No graphical display - terminal/SSH only

**Workflow**:
- Edit code in WSL
- Build and test on Raspberry Pi target system
- Agent can run on either environment depending on task

**Team Context**:
- Currently in learning phase for ROS2 and C++
- Explanations should be clear and educational
- Include rationale for technical decisions

## Communication Style

- Maintain 85-90% honesty factor: Be direct about technical issues without unnecessary harshness
- Use straightforward, developer-to-developer language
- Don't automatically agree with user suggestions - assess approaches critically
- When users propose problematic solutions:
  - State issues clearly: "That approach has significant issues"
  - Explain why it's problematic
  - Steer toward better solutions
- Be direct about technical trade-offs and downsides
- Skip flattery - respond directly to technical questions
- Never say things like "That's a great idea!" or "Excellent suggestion!" unless it genuinely is
- Evaluate proposals on technical merit, not to be agreeable

## Development Workflow

### Specification-Driven Development

We follow a structured process:

1. **Specification**: User creates a spec (like a user story) in Markdown in the `specs/` folder
2. **Investigation**: Agent investigates the codebase - read existing code, check patterns, verify dependencies
3. **Planning**: Agent drafts an implementation plan in Markdown based on the spec
4. **Review & Iterate**: Review and iterate the plan until it's solid
5. **Implementation**: Only then write code

**Planning Guidelines**:
- Plans should be high-level, not code-detailed (unless developer requests it)
- Focus on approach, architecture, and key decisions
- Identify risks, trade-offs, and open questions
- Keep it concise - details emerge during implementation
- Save plans in `specs/` folder alongside the spec
- **Use web search during planning** when uncertain about:
  - API specifications or conventions
  - Hardware capabilities or limitations
  - Best practices for unfamiliar technologies
  - Current versions or compatibility
  - Technical specifications that may have changed
- Focus on approach, architecture, and key decisions
- Identify risks, trade-offs, and open questions
- Keep it concise - details emerge during implementation
- Save plans in `specs/` folder alongside the spec

**Example Plan Structure**:
```markdown
# Implementation Plan: [Feature Name]

## Approach
- High-level strategy

## Architecture
- Components involved
- Key interfaces

## Key Decisions
- Important technical choices

## Risks & Trade-offs
- What could go wrong
- Alternative approaches considered

## Open Questions
- Things to clarify before coding
```

### Research First - Never Assume

During investigation phase:

1. **Read existing implementations**: Find similar code as a template
2. **Check patterns**: Review existing code in the same directory
3. **Verify dependencies**: Understand what the code depends on
4. **Read documentation**: Check project docs, datasheets, and specs
5. **Search when needed**: Use web search for ROS2 API details, hardware specs, or unfamiliar concepts

**When to use web search:**
- ROS2 API details, message types, or conventions
- Hardware specifications, datasheets, register addresses
- Unfamiliar libraries or frameworks
- Current best practices or recent changes
- Verifying technical specifications

**Never assume:**
- Function signatures or method names
- ROS2 message types or service definitions
- Existing class structures
- Hardware protocols or register addresses

**Always verify by reading actual code or searching documentation.**

## Production Code

**CRITICAL**: ALL production code MUST be C++. Python is ONLY for demos and tests.

## Project Structure

**Directory Organization:**
- `scripts/` - Production tools (calibration, node startup scripts)
- `demo/` - Example and test scripts
- `specs/` - Specifications and implementation plans
- `src/` - C++ source code
- `include/` - C++ headers
- `config/` - Configuration files

## ROS2 Development Tools

**CRITICAL: Use the `cli-runner` subagent for ALL command-line operations.**

This includes:
- Building packages (`colcon build`)
- Running nodes (`ros2 run`)
- Running Python scripts
- Any bash commands
- File operations that involve execution

**Why:** Keeps main context clean and focused on code/planning.

**How:** Use `use_subagent` tool with `agent_name: "cli-runner"` for any CLI operation.

### Workspace Setup (First Time Only)
```bash
# Create workspace and link package
cd /home/patrik
# This directory is already the workspace

# No additional setup needed
```

### Build and Run
```bash
# Source ROS2 environment
source /opt/ros/kilted/setup.bash

# Build workspace

colcon build --packages-select ros2_pi_sense_hat
source install/setup.bash

# Build with verbose output
colcon build --packages-select ros2_pi_sense_hat --event-handlers console_direct+

# Clean build
colcon build --packages-select ros2_pi_sense_hat --cmake-clean-cache

# Run nodes (available: led_matrix_node, joystick_node, imu_node)
ros2 run ros2_pi_sense_hat led_matrix_node
ros2 run ros2_pi_sense_hat joystick_node
ros2 run ros2_pi_sense_hat imu_node

# Launch files
ros2 launch ros2_pi_sense_hat <launch_file>
```

### Testing and Debugging
```bash
# Run tests
colcon test --packages-select <package_name>
colcon test-result --verbose

# List topics
ros2 topic list
ros2 topic echo /topic_name
ros2 topic info /topic_name

# Services
ros2 service list
ros2 service call /service_name <srv_type> "request"

# Nodes
ros2 node list
ros2 node info /node_name

# Parameters
ros2 param list /node_name
ros2 param get /node_name parameter_name
ros2 param set /node_name parameter_name value
```

## Code Style Guidelines

Follow the ROS2 Code Style Guide: https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html

### C++ (ALL Production Code)

**Standard**: C++17

**Style**: Google C++ Style Guide with ROS2 modifications

**Key Rules**:
- 100 character line length
- `.hpp` for headers, `.cpp` for implementation
- `snake_case` for functions/methods (ROS2 convention)
- `CamelCase` for classes
- `g_snake_case` for global variables
- Open braces for functions/classes, cuddled for if/while/for
- `char * c` pointer syntax
- `///` and `/** */` for documentation, `//` for code comments
- Always use braces for if/else/while/for
- No Boost unless absolutely required
- Exceptions allowed (but avoid in C-wrapped APIs)

**Example**:
```cpp
class SensorDriver
{
public:
  SensorDriver();
  
  /// Read sensor data
  bool read_data(uint8_t * buffer, size_t length);

private:
  int fd_;
  uint8_t address_;
};

void process_data()
{
  if (condition) {
    do_something();
  } else {
    do_other_thing();
  }
}
```

**Linters**: Use ament linting tools
- `ament_clang_format` - Code formatting
- `ament_cpplint` - Style checking
- `ament_uncrustify` - Code formatting
- `ament_cppcheck` - Static analysis

### Python (Demos and Tests ONLY)

**Standard**: Python 3

**Style**: PEP8 with ROS2 modifications

**Key Rules**:
- 100 character line length
- Single quotes preferred
- One import per line
- Hanging indents for continuation lines

**Linters**: Use ament linting tools
- `ament_pycodestyle` - PEP8 checking
- `ament_flake8` - Style and error checking

### CMake

**Key Rules**:
- Lowercase command names: `find_package`, not `FIND_PACKAGE`
- `snake_case` identifiers
- 2-space indentation
- No whitespace before `(`

**Linters**: Use ament linting tools
- `ament_cmake_lint` - CMake style checking

### Markdown Documentation

**Key Rules**:
- Each sentence on a new line (better diffs)
- ATX-style headers (`#`, `##`, `###`)
- Code blocks with syntax highlighting
- Empty line before/after code blocks

## Testing with ament

**Use ROS2 ament testing framework** - see `ament_lint_auto` for automated style checking.

### C++ Tests

Use `ament_cmake_gtest` for unit tests:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_node test/test_node.cpp)
  target_link_libraries(test_node ${PROJECT_NAME})
endif()
```

### Integration Tests

Use `ament_cmake_pytest` for integration tests:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(integration_test test/integration_test.py)
endif()
```

### Automated Linting

Use `ament_lint_auto` to run all linters:

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

Add linters to `package.xml`:
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

## ROS2 Architecture Patterns

### Node Design

- **One node per sensor/device**: Each sensor should be its own node
- Keeps concerns separated and allows independent lifecycle management
- Enables flexible composition and deployment

### Hardware Access Philosophy

- **Direct I2C/GPIO access preferred**: Avoid kernel drivers for basic I2C devices
- Use direct register-level access for sensors and peripherals
- Provides better control and understanding of hardware behavior
- May require blacklisting kernel drivers to prevent conflicts

### Component-Based Nodes

- Use ROS2 component architecture for composability
- Implement `rclcpp::Node` or `rclcpp_lifecycle::LifecycleNode`
- Register components with `RCLCPP_COMPONENTS_REGISTER_NODE`

### Message Types

- Use standard ROS2 messages when possible:
  - `sensor_msgs/msg/*` for sensor data
  - `geometry_msgs/msg/*` for poses, transforms
  - `std_msgs/msg/*` for simple types
  - `nav_msgs/msg/*` for navigation
- Create custom messages only when necessary

### Service Patterns

- Use `std_srvs/srv/Trigger` for simple actions
- Use `std_srvs/srv/SetBool` for enable/disable
- Create custom services for complex operations
- Return success/failure with meaningful error messages

### Lifecycle Nodes

- Use lifecycle nodes for managed startup/shutdown
- Implement state transitions: unconfigured → inactive → active
- Handle cleanup in appropriate lifecycle callbacks

## Common Pitfalls

### Build Issues
- **Problem**: Header not found
  - **Check**: `ament_target_dependencies` includes required packages
  - **Check**: Package listed in `package.xml` dependencies
  - **Check**: Include paths correct in CMakeLists.txt

### Runtime Issues
- **Problem**: Node not publishing
  - **Check**: Topic name correct? `ros2 topic list`
  - **Check**: Message type matches? `ros2 topic info /topic_name`
  - **Check**: Node running? `ros2 node list`
  - **Check**: QoS settings compatible?

### Linting Issues
- **Problem**: Style check failures
  - **Fix**: Run `ament_uncrustify --reformat` or `ament_clang_format --reformat`
  - **Check**: Verify with `colcon test --packages-select <package_name>`

## Package Structure

```
package_name/
├── include/
│   └── package_name/       # Public C++ headers
├── src/                    # C++ implementation
├── test/                   # C++ tests (gtest)
├── launch/                 # Launch files (Python)
├── config/                 # Configuration files (YAML)
├── msg/                    # Custom message definitions
├── srv/                    # Custom service definitions
├── action/                 # Custom action definitions
├── CMakeLists.txt
├── package.xml
└── README.md
```

## References

- ROS2 Documentation: https://docs.ros.org/
- ROS2 Code Style Guide: https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html
- Google C++ Style Guide: https://google.github.io/styleguide/cppguide.html
- ament_lint Documentation: https://github.com/ament/ament_lint
