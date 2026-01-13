# Camera Visual-Inertial Odometry Specification

## Hardware Constraints

**Fixed Platform - No Additional Hardware:**
- Raspberry Pi 4 (4GB RAM, 16GB SD card)
- Raspberry Pi Sense HAT v2
- Raspberry Pi Camera Module v2.1 (8MP, Sony IMX219)
  - RGB variant (with IR filter) OR
  - NoIR variant (without IR filter)
  - Single camera only - physical swap required to change

**No plans for:**
- Additional cameras
- Stereo camera rigs
- Compute modules
- External sensors
- Additional processing hardware

**All implementations must work within these hardware limits.**

## Overview

Add Raspberry Pi Camera v2.1 support for Visual-Inertial Odometry (VIO) to complement the existing IMU-based orientation tracking with position estimation.

## Hardware

**Available Cameras:**
- Raspberry Pi Camera Module (RGB with IR filter)
- Raspberry Pi Camera Module NoIR (no IR filter)

**Connection:**
- CSI ribbon cable to Pi 4 camera port
- **Single camera only** - Pi 4 has one CSI port

**Camera Selection:**
- **Primary use:** RGB camera for visual odometry and navigation
- **Alternative use:** NoIR camera for night vision or IR experiments
- **Note:** Cameras must be physically swapped - no multi-camera support on Pi 4

**Recommendation:** Start with RGB camera for better feature detection in visual odometry.

## Objectives

### Primary Goal
Implement Visual-Inertial Odometry to provide full 6DOF pose estimation (position + orientation) by fusing camera and IMU data.

### Learning Goals
- Understand monocular visual odometry
- Learn camera calibration
- Explore sensor fusion with heterogeneous sensors
- Implement standard robotics navigation stack

## Current System State

**Existing Capabilities:**
- ✅ Calibrated IMU (LSM9DS1) publishing at 10 Hz
- ✅ Madgwick AHRS filter providing orientation
- ✅ EKF providing orientation-only pose estimation
- ✅ TF tree for coordinate transforms

**Limitations:**
- ❌ No position tracking (only orientation)
- ❌ IMU drift in position (accelerometer integration)
- ❌ No visual feedback for navigation

## Proposed Architecture

```
Camera → Camera Node → Image Processing → Visual Odometry
   ↓                                            ↓
IMU → IMU Node → Madgwick → EKF ← Visual-Inertial Fusion
                              ↓
                    /odometry/filtered (6DOF pose)
```

## Implementation Options

### Option 1: RTAB-Map (Recommended for Learning)
**Package:** `rtabmap_ros`

**Pros:**
- Complete SLAM solution
- Good documentation and tutorials
- Visual loop closure
- 3D mapping
- Works well with monocular + IMU

**Cons:**
- Heavier computation
- More complex configuration

**Use Case:** Indoor navigation, mapping, exploration

### Option 2: ORB-SLAM3
**Package:** `orb_slam3_ros`

**Pros:**
- State-of-the-art accuracy
- Monocular-Inertial mode
- Robust to fast motion

**Cons:**
- Complex setup
- Requires good calibration
- Higher computational cost

**Use Case:** Research-grade SLAM

### Option 3: Simple Visual Odometry
**Package:** `viso2_ros` or custom

**Pros:**
- Lightweight
- Easy to understand
- Good for learning basics

**Cons:**
- No loop closure
- Drift over time
- Less robust

**Use Case:** Learning, simple navigation

## Required Components

### 1. Camera Driver Node
**Package:** `v4l2_camera` or `camera_ros`

**Responsibilities:**
- Capture images from Pi Camera
- Publish to `/camera/image_raw`
- Publish camera info to `/camera/camera_info`

**Configuration:**
- Resolution: 640x480 (balance speed/quality)
- Frame rate: 30 Hz
- Format: RGB8 or MONO8

### 2. Camera Calibration
**Tool:** `camera_calibration` (ROS2 package)

**Process:**
- Print checkerboard pattern
- Capture 20-30 images at different angles
- Calculate intrinsic parameters (focal length, distortion)
- Save to `camera_info.yaml`

**Parameters to calibrate:**
- Camera matrix (fx, fy, cx, cy)
- Distortion coefficients (k1, k2, p1, p2, k3)

### 3. IMU-Camera Extrinsic Calibration
**Tool:** `kalibr` or manual measurement

**Process:**
- Determine spatial relationship between IMU and camera
- Rotation matrix and translation vector
- Critical for accurate VIO

**Parameters:**
- Transform from IMU frame to camera frame
- Time synchronization offset

### 4. Visual-Inertial Odometry Node
**Package:** TBD (rtabmap_ros, orb_slam3, or custom)

**Inputs:**
- `/camera/image_raw` - Camera images
- `/imu/data` - Fused IMU with orientation
- `/camera/camera_info` - Camera calibration

**Outputs:**
- `/vo/odometry` - Visual odometry estimate
- `/vo/pose` - Camera pose
- (Optional) `/vo/map` - 3D point cloud

### 5. Updated EKF Configuration
**Modification:** `config/ekf_vio.yaml`

**Changes:**
- Add visual odometry as input source
- Configure covariance matrices
- Tune filter parameters for camera + IMU fusion

## Topics and Data Flow

**New Topics:**
- `/camera/image_raw` (sensor_msgs/Image) - Raw camera images
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Calibration data
- `/vo/odometry` (nav_msgs/Odometry) - Visual odometry output
- `/vo/pose` (geometry_msgs/PoseStamped) - Camera pose

**Modified Topics:**
- `/odometry/filtered` - Now includes position from VIO (not just orientation)

**TF Frames:**
- `base_link` → `camera_link` (static transform)
- `base_link` → `imu_link` (existing)
- `odom` → `base_link` (from EKF with VIO)

## Parameters

### Camera Node
- `frame_rate` (default: 30 Hz)
- `image_width` (default: 640)
- `image_height` (default: 480)
- `camera_frame_id` (default: "camera_link")

### VIO Node (example for RTAB-Map)
- `frame_id` (default: "base_link")
- `odom_frame_id` (default: "odom")
- `subscribe_depth` (default: false - monocular)
- `subscribe_rgb` (default: true)
- `subscribe_odom_info` (default: true)
- `approx_sync` (default: false)

## Calibration Workflow

### 1. Camera Intrinsic Calibration
```bash
# Install calibration package
sudo apt install ros-kilted-camera-calibration

# Run calibration
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/camera/image_raw

# Save results to config/camera_info.yaml
```

### 2. IMU-Camera Extrinsic Calibration
```bash
# Option A: Manual measurement
# Measure physical offset and orientation between IMU and camera
# Create static transform in launch file

# Option B: Kalibr (more accurate)
# Record synchronized IMU + camera data
# Run Kalibr calibration tool
# Extract extrinsic parameters
```

### 3. Validation
```bash
# Verify camera publishes
ros2 topic echo /camera/image_raw

# Verify camera info
ros2 topic echo /camera/camera_info

# Check TF tree
ros2 run tf2_tools view_frames
```

## Testing Strategy

### Phase 1: Camera Setup
- [ ] Enable camera interface
- [ ] Test with `libcamera-hello`
- [ ] Launch camera node
- [ ] Verify image publishing

### Phase 2: Calibration
- [ ] Perform intrinsic calibration
- [ ] Measure/calculate extrinsic calibration
- [ ] Validate calibration quality

### Phase 3: Visual Odometry
- [ ] Launch VIO node with camera + IMU
- [ ] Test stationary (should show no drift)
- [ ] Test translation (forward/backward)
- [ ] Test rotation (yaw/pitch/roll)

### Phase 4: Integration
- [ ] Update EKF to fuse VIO
- [ ] Verify `/odometry/filtered` includes position
- [ ] Test complete navigation stack
- [ ] Validate against ground truth

## Performance Targets

**Accuracy:**
- Position drift: < 1% of distance traveled
- Orientation: ± 2-5° (already achieved with IMU)
- Update rate: 10-30 Hz

**Computational:**
- CPU usage: < 50% on Pi 4
- Memory: < 1 GB
- Latency: < 100 ms

## Known Challenges

### 1. Scale Ambiguity
**Problem:** Monocular camera can't determine absolute scale
**Solution:** IMU accelerometer provides scale reference

### 2. Feature-Poor Environments
**Problem:** Visual odometry fails in textureless areas
**Solution:** IMU continues tracking, camera resumes when features return

### 3. Lighting Conditions
**Problem:** Poor lighting affects feature detection
**Solution:** 
- Use NoIR camera + IR illumination for night
- Adjust exposure settings
- Consider switching cameras based on TCS3400 light sensor

### 4. Synchronization
**Problem:** Camera and IMU have different rates and latencies
**Solution:** 
- Use approximate time synchronization
- Calibrate time offset
- Buffer messages appropriately

### 5. Computational Load
**Problem:** VIO is computationally intensive
**Solution:**
- Reduce image resolution
- Lower frame rate
- Use efficient algorithms (ORB features)
- Consider hardware acceleration

## Future Enhancements

### Short Term
- [ ] Basic monocular visual odometry
- [ ] Camera-IMU fusion in EKF
- [ ] Simple obstacle detection

### Medium Term
- [ ] Loop closure for drift correction
- [ ] 3D mapping (sparse point cloud)
- [ ] Path planning integration

### Long Term
- [ ] Dense mapping
- [ ] Object detection/recognition
- [ ] Semantic SLAM

**Note:** Multi-camera support is out of scope - Pi 4 hardware limitation and no plans for additional hardware.

## References

**ROS2 Packages:**
- `v4l2_camera` - Camera driver
- `camera_calibration` - Calibration tool
- `rtabmap_ros` - SLAM with VIO
- `orb_slam3_ros` - Advanced SLAM
- `robot_localization` - EKF (already using)

**Documentation:**
- [ROS2 Camera Calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- [RTAB-Map ROS2](https://github.com/introlab/rtabmap_ros)
- [Kalibr Calibration](https://github.com/ethz-asl/kalibr)

**Learning Resources:**
- Visual SLAM tutorial series
- Monocular-Inertial odometry papers
- ROS2 navigation stack documentation

## Success Criteria

**Minimum Viable Product:**
- ✅ Camera publishing calibrated images
- ✅ Visual odometry providing position estimates
- ✅ EKF fusing camera + IMU data
- ✅ Position tracking with < 5% drift over 10m

**Stretch Goals:**
- Loop closure working
- 3D map visualization
- Real-time obstacle detection
- Autonomous navigation demo

## Timeline Estimate

**Phase 1 - Camera Setup:** 1-2 hours
**Phase 2 - Calibration:** 2-3 hours
**Phase 3 - VIO Integration:** 4-6 hours
**Phase 4 - Testing & Tuning:** 2-4 hours

**Total:** 10-15 hours for basic VIO system

## Notes

- Start with RGB camera (better for feature detection)
- Keep NoIR camera for future night vision experiments
- VIO significantly improves navigation capabilities
- Complements existing IMU-only orientation tracking
- Standard approach in modern robotics (drones, robots, AR/VR)
