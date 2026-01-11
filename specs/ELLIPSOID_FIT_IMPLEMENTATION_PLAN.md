# Ellipsoid Fitting Implementation Plan

## Investigation Summary

**Current Implementation:**

### C++ Side (`src/imu_calibration.cpp`, `include/ros2_pi_sense_hat/imu_calibration.hpp`)
- `CalibrationData` struct: Simple offset + scale per axis (6 floats each for accel/mag)
- `correctIMUData()`: Simple per-axis correction: `(raw - offset) * scale`
- YAML loading: Basic string parsing, no matrix support
- **No matrix multiplication capability**

### Python Side (`scripts/calibrate_imu.py`)
- Gyro: Proper bias calculation ✅
- Accel: 6-point min/max method (claims nothing, is what it is)
- Mag: Min/max method with misleading "ellipsoid fitting" docstring ❌
- **No actual ellipsoid fitting algorithm**

### Current YAML Format
```yaml
accel_offset_x: -0.003707
accel_scale_x: 0.999930
# ... simple per-axis values
```

## Problem Statement

**Both accelerometer and magnetometer use oversimplified calibration:**
1. Only corrects offset and scale per axis independently
2. Ignores cross-axis coupling (non-orthogonal axes)
3. Magnetometer docstring lies about using ellipsoid fitting
4. Results in suboptimal accuracy

**Verification test shows:** After calibration, `sqrt(ax² + ay² + az²)` should equal 9.80665 m/s² but likely varies by ±0.1 m/s² or more.

## Solution: True Ellipsoid Fitting with Matrix Correction

### Mathematical Approach

**Goal:** Transform ellipsoid measurements → sphere of known radius

**Ellipsoid equation (general form):**
```
Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
```

**Solve for 10 parameters, then extract:**
- Center offset: [x₀, y₀, z₀]
- Transformation matrix M (3x3): Combines rotation + scaling

**Correction formula:**
```
corrected = M * (raw - offset)
```

## Implementation Plan

### Phase 1: Core Ellipsoid Fitting Algorithm

**Create:** `demo/ellipsoid_fit.py`

```python
import numpy as np
from numpy.linalg import svd, eig, lstsq

def fit_ellipsoid(points):
    """
    Fit ellipsoid to 3D points using algebraic fit.
    
    Args:
        points: Nx3 numpy array of measurements
        
    Returns:
        offset: [x0, y0, z0] center
        matrix: 3x3 transformation matrix
        radii: [a, b, c] semi-axes (for validation)
        residual: RMS fit error
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # Design matrix for ellipsoid equation
    # Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
    D = np.column_stack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        np.ones_like(x)
    ])
    
    # Solve using SVD (find null space)
    _, _, V = svd(D)
    params = V[-1, :]
    
    # Extract parameters
    A, B, C, D_xy, E_xz, F_yz, G, H, I, J = params
    
    # Build quadratic form matrix (3x3)
    Q3 = np.array([
        [A, D_xy, E_xz],
        [D_xy, B, F_yz],
        [E_xz, F_yz, C]
    ])
    
    # Linear terms
    u = np.array([G, H, I])
    
    # Solve for center: Q3 * center = -u
    offset = -np.linalg.solve(Q3, u)
    
    # Translate to center
    Q3_centered = Q3 / (-J - offset @ Q3 @ offset)
    
    # Eigendecomposition
    eigenvalues, eigenvectors = eig(Q3_centered)
    radii = 1.0 / np.sqrt(np.abs(eigenvalues))
    
    # Build transformation matrix
    # M = R * diag(1/radii) * R^T
    scale_inv = np.diag(1.0 / radii)
    matrix = eigenvectors @ scale_inv @ eigenvectors.T
    
    # Calculate residual
    corrected = (points - offset) @ matrix.T
    magnitudes = np.linalg.norm(corrected, axis=1)
    residual = np.std(magnitudes)
    
    return offset, matrix, radii, residual


def validate_calibration(points, offset, matrix, expected_radius):
    """
    Validate calibration quality.
    
    Returns:
        rms_error: RMS deviation from expected radius
        max_error: Maximum deviation
        mean_radius: Mean radius after correction
    """
    corrected = (points - offset) @ matrix.T
    magnitudes = np.linalg.norm(corrected, axis=1)
    
    errors = magnitudes - expected_radius
    rms_error = np.sqrt(np.mean(errors**2))
    max_error = np.max(np.abs(errors))
    mean_radius = np.mean(magnitudes)
    
    return rms_error, max_error, mean_radius
```

**Test this algorithm first** before integrating!

### Phase 2: Update Python Calibration Script

**Modify:** `scripts/calibrate_imu.py`

**Changes needed:**

1. **Import ellipsoid fitting:**
```python
from ellipsoid_fit import fit_ellipsoid, validate_calibration
```

2. **Replace `calibrate_accelerometer()` method:**
```python
def calibrate_accelerometer(self):
    """Calibrate accelerometer using ellipsoid fitting"""
    print("\n🎯 Starting accelerometer calibration...")
    print("📋 You will position the IMU in 6 orientations:")
    print("   (More positions = better accuracy, but 6 is minimum)")
    
    orientations = [
        ("+X up", "X-axis pointing up"),
        ("-X up", "X-axis pointing down"), 
        ("+Y up", "Y-axis pointing up"),
        ("-Y up", "Y-axis pointing down"),
        ("+Z up", "Z-axis pointing up"),
        ("-Z up", "Z-axis pointing down")
    ]
    
    measurements = []
    
    for i, (name, desc) in enumerate(orientations):
        print(f"\n📐 Position {i+1}/6: {name} ({desc})")
        input("   Press Enter when positioned...")
        
        samples = self.collect_samples(5)
        
        if len(samples) < 10:
            print(f"❌ Failed to collect samples for {name}")
            return None
        
        accel_data = np.array([s['accel'] for s in samples])
        avg = np.mean(accel_data, axis=0)
        measurements.append(avg)
        
        print(f"✅ {name}: [{avg[0]:.3f}, {avg[1]:.3f}, {avg[2]:.3f}] m/s²")
    
    measurements = np.array(measurements)
    
    # Fit ellipsoid
    g = 9.80665
    offset, matrix, radii, residual = fit_ellipsoid(measurements)
    
    # Validate
    rms_error, max_error, mean_radius = validate_calibration(
        measurements, offset, matrix, g
    )
    
    print(f"\n✅ Accelerometer calibration complete!")
    print(f"   Offset: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}] m/s²")
    print(f"   Radii: [{radii[0]:.3f}, {radii[1]:.3f}, {radii[2]:.3f}] m/s²")
    print(f"   Expected: {g:.5f} m/s²")
    print(f"   Mean radius: {mean_radius:.5f} m/s²")
    print(f"   RMS error: {rms_error:.6f} m/s² ({rms_error/g*100:.3f}%)")
    print(f"   Max error: {max_error:.6f} m/s² ({max_error/g*100:.3f}%)")
    
    # Check if calibration is acceptable
    if rms_error > 0.1:  # 1% threshold
        print(f"⚠️  WARNING: RMS error is high. Consider recalibrating.")
    
    return {
        'accel_offset_x': float(offset[0]),
        'accel_offset_y': float(offset[1]),
        'accel_offset_z': float(offset[2]),
        'accel_matrix': matrix.flatten().tolist(),  # 9 elements, row-major
        'accel_calibrated': True
    }
```

3. **Replace `calibrate_magnetometer()` method:**
```python
def calibrate_magnetometer(self):
    """Calibrate magnetometer using TRUE ellipsoid fitting"""
    print("\n🧭 Starting magnetometer calibration...")
    print("🔄 Rotate the IMU in ALL directions!")
    print("   Goal: Cover a complete sphere of orientations")
    print("   Duration: 60 seconds (need 200+ samples)")
    
    input("Press Enter when ready to start rotating...")
    
    # Collect samples
    start_time = time.time()
    mag_samples = []
    
    print("🔄 ROTATE NOW! Move in all directions...")
    
    while time.time() - start_time < 60:
        if len(self.mag_data) > 0:
            mag_samples.append(self.mag_data[-1]['mag'])
        
        elapsed = time.time() - start_time
        progress = int((elapsed / 60) * 20)
        bar = "█" * progress + "░" * (20 - progress)
        print(f"\r🔄 [{bar}] {elapsed:.1f}s/60s - Keep rotating!", end="", flush=True)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.1)
    
    print(f"\n✅ Collected {len(mag_samples)} magnetometer samples")
    
    if len(mag_samples) < 200:
        print(f"❌ Need 200+ samples, got {len(mag_samples)}")
        return None
    
    mag_data = np.array(mag_samples)
    
    # Fit ellipsoid
    offset, matrix, radii, residual = fit_ellipsoid(mag_data)
    
    # Expected field strength is mean of radii
    expected_radius = np.mean(radii)
    
    # Validate
    rms_error, max_error, mean_radius = validate_calibration(
        mag_data, offset, matrix, expected_radius
    )
    
    print(f"\n✅ Magnetometer calibration complete!")
    print(f"   Offset: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}] T")
    print(f"   Radii: [{radii[0]:.6f}, {radii[1]:.6f}, {radii[2]:.6f}] T")
    print(f"   Field strength: {expected_radius:.6f} T")
    print(f"   RMS error: {rms_error:.6f} T ({rms_error/expected_radius*100:.2f}%)")
    print(f"   Max error: {max_error:.6f} T ({max_error/expected_radius*100:.2f}%)")
    
    if rms_error/expected_radius > 0.02:  # 2% threshold
        print(f"⚠️  WARNING: RMS error is high. Consider recalibrating.")
    
    return {
        'mag_offset_x': float(offset[0]),
        'mag_offset_y': float(offset[1]),
        'mag_offset_z': float(offset[2]),
        'mag_matrix': matrix.flatten().tolist(),  # 9 elements, row-major
        'mag_calibrated': True,
        'mag_field_strength': float(expected_radius)
    }
```

### Phase 3: Update C++ Data Structures

**Modify:** `include/ros2_pi_sense_hat/imu_calibration.hpp`

```cpp
struct CalibrationData {
    // Gyroscope bias (rad/s) - unchanged
    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;
    
    // Accelerometer - NEW: matrix-based correction
    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;
    float accel_matrix[9] = {1.0f, 0.0f, 0.0f,  // Identity matrix default
                             0.0f, 1.0f, 0.0f,
                             0.0f, 0.0f, 1.0f};
    
    // Legacy scale factors for backward compatibility
    float accel_scale_x = 1.0f;
    float accel_scale_y = 1.0f;
    float accel_scale_z = 1.0f;
    bool accel_use_matrix = false;  // Flag: true = use matrix, false = use scale
    
    // Magnetometer - NEW: matrix-based correction
    float mag_offset_x = 0.0f;
    float mag_offset_y = 0.0f;
    float mag_offset_z = 0.0f;
    float mag_matrix[9] = {1.0f, 0.0f, 0.0f,
                           0.0f, 1.0f, 0.0f,
                           0.0f, 0.0f, 1.0f};
    
    // Legacy scale factors for backward compatibility
    float mag_scale_x = 1.0f;
    float mag_scale_y = 1.0f;
    float mag_scale_z = 1.0f;
    bool mag_use_matrix = false;  // Flag: true = use matrix, false = use scale
    
    // Calibration status
    bool gyro_calibrated = false;
    bool accel_calibrated = false;
    bool mag_calibrated = false;
    
    // Timestamp
    uint64_t timestamp = 0;
};
```

### Phase 4: Update C++ Correction Logic

**Modify:** `src/imu_calibration.cpp`

```cpp
void IMUCalibration::correctIMUData(IMUData& data) const {
    // Gyroscope - unchanged
    if (cal_data_.gyro_calibrated) {
        data.gyro_x -= cal_data_.gyro_bias_x;
        data.gyro_y -= cal_data_.gyro_bias_y;
        data.gyro_z -= cal_data_.gyro_bias_z;
    }
    
    // Accelerometer - matrix or legacy scale
    if (cal_data_.accel_calibrated) {
        // Remove offset
        float ax = data.accel_x - cal_data_.accel_offset_x;
        float ay = data.accel_y - cal_data_.accel_offset_y;
        float az = data.accel_z - cal_data_.accel_offset_z;
        
        if (cal_data_.accel_use_matrix) {
            // Matrix correction (new method)
            const float* M = cal_data_.accel_matrix;
            data.accel_x = M[0]*ax + M[1]*ay + M[2]*az;
            data.accel_y = M[3]*ax + M[4]*ay + M[5]*az;
            data.accel_z = M[6]*ax + M[7]*ay + M[8]*az;
        } else {
            // Legacy scale correction (backward compatible)
            data.accel_x = ax * cal_data_.accel_scale_x;
            data.accel_y = ay * cal_data_.accel_scale_y;
            data.accel_z = az * cal_data_.accel_scale_z;
        }
    }
    
    // Magnetometer - matrix or legacy scale
    if (cal_data_.mag_calibrated) {
        // Remove offset
        float mx = data.mag_x - cal_data_.mag_offset_x;
        float my = data.mag_y - cal_data_.mag_offset_y;
        float mz = data.mag_z - cal_data_.mag_offset_z;
        
        if (cal_data_.mag_use_matrix) {
            // Matrix correction (new method)
            const float* M = cal_data_.mag_matrix;
            data.mag_x = M[0]*mx + M[1]*my + M[2]*mz;
            data.mag_y = M[3]*mx + M[4]*my + M[5]*mz;
            data.mag_z = M[6]*mx + M[7]*my + M[8]*mz;
        } else {
            // Legacy scale correction (backward compatible)
            data.mag_x = mx * cal_data_.mag_scale_x;
            data.mag_y = my * cal_data_.mag_scale_y;
            data.mag_z = mz * cal_data_.mag_scale_z;
        }
    }
}
```

### Phase 5: Update YAML Loading/Saving

**Modify:** `src/imu_calibration.cpp`

**Add matrix parsing to `loadCalibration()`:**
```cpp
bool IMUCalibration::loadCalibration(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    std::vector<float> accel_matrix_values;
    std::vector<float> mag_matrix_values;
    
    while (std::getline(file, line)) {
        // ... existing parsing ...
        
        // NEW: Parse matrix values
        if (line.find("accel_matrix:") != std::string::npos) {
            std::string matrix_str = line.substr(line.find("[") + 1);
            matrix_str = matrix_str.substr(0, matrix_str.find("]"));
            
            std::stringstream ss(matrix_str);
            std::string value;
            while (std::getline(ss, value, ',')) {
                accel_matrix_values.push_back(std::stof(value));
            }
        }
        
        if (line.find("mag_matrix:") != std::string::npos) {
            std::string matrix_str = line.substr(line.find("[") + 1);
            matrix_str = matrix_str.substr(0, matrix_str.find("]"));
            
            std::stringstream ss(matrix_str);
            std::string value;
            while (std::getline(ss, value, ',')) {
                mag_matrix_values.push_back(std::stof(value));
            }
        }
    }
    
    // Load matrix if present
    if (accel_matrix_values.size() == 9) {
        for (int i = 0; i < 9; i++) {
            cal_data_.accel_matrix[i] = accel_matrix_values[i];
        }
        cal_data_.accel_use_matrix = true;
    }
    
    if (mag_matrix_values.size() == 9) {
        for (int i = 0; i < 9; i++) {
            cal_data_.mag_matrix[i] = mag_matrix_values[i];
        }
        cal_data_.mag_use_matrix = true;
    }
    
    return true;
}
```

**Update `saveCalibration()` to write matrices:**
```cpp
bool IMUCalibration::saveCalibration(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file << "# IMU Calibration Data\n";
    
    // Gyroscope
    file << "gyro_bias_x: " << cal_data_.gyro_bias_x << "\n";
    file << "gyro_bias_y: " << cal_data_.gyro_bias_y << "\n";
    file << "gyro_bias_z: " << cal_data_.gyro_bias_z << "\n";
    file << "gyro_calibrated: " << (cal_data_.gyro_calibrated ? "true" : "false") << "\n";
    
    // Accelerometer
    file << "accel_offset_x: " << cal_data_.accel_offset_x << "\n";
    file << "accel_offset_y: " << cal_data_.accel_offset_y << "\n";
    file << "accel_offset_z: " << cal_data_.accel_offset_z << "\n";
    
    if (cal_data_.accel_use_matrix) {
        file << "accel_matrix: [";
        for (int i = 0; i < 9; i++) {
            file << cal_data_.accel_matrix[i];
            if (i < 8) file << ", ";
        }
        file << "]\n";
    } else {
        // Legacy format
        file << "accel_scale_x: " << cal_data_.accel_scale_x << "\n";
        file << "accel_scale_y: " << cal_data_.accel_scale_y << "\n";
        file << "accel_scale_z: " << cal_data_.accel_scale_z << "\n";
    }
    file << "accel_calibrated: " << (cal_data_.accel_calibrated ? "true" : "false") << "\n";
    
    // Magnetometer (similar structure)
    // ...
    
    return true;
}
```

### Phase 6: Create Verification Tool

**Create:** `demo/verify_calibration.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np

class CalibrationVerifier(Node):
    def __init__(self):
        super().__init__('calibration_verifier')
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, '/imu/mag', self.mag_callback, 10)
        
        self.accel_errors = []
        self.mag_samples = []
        
    def imu_callback(self, msg):
        acc = msg.linear_acceleration
        norm = np.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
        error = abs(norm - 9.80665)
        
        self.accel_errors.append(error)
        
        if len(self.accel_errors) % 10 == 0:
            rms = np.sqrt(np.mean(np.array(self.accel_errors)**2))
            max_err = np.max(self.accel_errors)
            print(f"Accel - Samples: {len(self.accel_errors):4d} | "
                  f"RMS: {rms:.4f} m/s² ({rms/9.80665*100:.2f}%) | "
                  f"Max: {max_err:.4f} m/s²")
    
    def mag_callback(self, msg):
        mag = msg.magnetic_field
        self.mag_samples.append([mag.x, mag.y, mag.z])
        
        if len(self.mag_samples) >= 100:
            mag_data = np.array(self.mag_samples)
            norms = np.linalg.norm(mag_data, axis=1)
            mean_norm = np.mean(norms)
            std_norm = np.std(norms)
            
            print(f"Mag - Samples: {len(self.mag_samples):4d} | "
                  f"Mean: {mean_norm:.6f} T | "
                  f"Std: {std_norm:.6f} T ({std_norm/mean_norm*100:.2f}%)")
            
            self.mag_samples = []

def main():
    rclpy.init()
    node = CalibrationVerifier()
    print("Verifying IMU calibration...")
    print("Rotate IMU through different orientations\n")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Implementation Checklist

### Phase 1: Algorithm Development
- [ ] Create `demo/ellipsoid_fit.py`
- [ ] Write unit tests with synthetic data
- [ ] Verify algorithm with known ellipsoid
- [ ] Test with real sensor data (offline)

### Phase 2: Python Integration
- [ ] Update `calibrate_imu.py` imports
- [ ] Replace `calibrate_accelerometer()` method
- [ ] Replace `calibrate_magnetometer()` method
- [ ] Update `save_calibration()` to write matrices
- [ ] Test Python calibration end-to-end

### Phase 3: C++ Data Structures
- [ ] Update `CalibrationData` struct in header
- [ ] Add backward compatibility flags
- [ ] Update default initialization

### Phase 4: C++ Correction Logic
- [ ] Update `correctIMUData()` with matrix path
- [ ] Keep legacy scale path for compatibility
- [ ] Test both paths work correctly

### Phase 5: C++ YAML Handling
- [ ] Update `loadCalibration()` to parse matrices
- [ ] Update `saveCalibration()` to write matrices
- [ ] Handle both old and new formats
- [ ] Test loading old calibration files

### Phase 6: Verification
- [ ] Create `verify_calibration.py`
- [ ] Test with old calibration
- [ ] Test with new calibration
- [ ] Compare accuracy improvements

### Phase 7: Documentation
- [ ] Update README with new calibration procedure
- [ ] Document YAML format changes
- [ ] Add troubleshooting guide
- [ ] Update specs with results

## Testing Strategy

### Unit Tests
1. **Ellipsoid fitting algorithm:**
   - Test with perfect sphere → should return identity matrix
   - Test with known ellipsoid → verify transformation
   - Test with noisy data → check robustness

2. **C++ matrix multiplication:**
   - Test identity matrix → no change
   - Test known matrix → verify correct result
   - Test backward compatibility with scale factors

### Integration Tests
1. **Python calibration:**
   - Run full calibration sequence
   - Verify YAML output format
   - Check matrix values are reasonable

2. **C++ loading:**
   - Load new format YAML
   - Load old format YAML (backward compat)
   - Verify correct correction applied

3. **End-to-end:**
   - Calibrate with new method
   - Restart IMU node
   - Run verification script
   - Measure RMS error < 0.05 m/s² for accel

### Validation Metrics

**Accelerometer:**
- Target: RMS error < 0.05 m/s² (0.5%)
- Current: ~0.1 m/s² (1%)
- Expected: 2x improvement

**Magnetometer:**
- Target: Std dev < 1% of mean field
- Current: ~2-5%
- Expected: 2-5x improvement

## Expected Outcomes

### Accuracy Improvements
- **Accelerometer:** ±0.01g → ±0.003-0.005g (2-3x better)
- **Magnetometer:** ±2-5° heading → ±0.5-1° heading (2-5x better)

### User Experience
- Same calibration procedure (6 positions for accel, rotation for mag)
- Better feedback (RMS error, validation metrics)
- Automatic quality checking

### System Benefits
- Better sensor fusion performance
- More accurate odometry
- Improved heading estimation
- Foundation for advanced calibration (temperature compensation, etc.)

## Risks and Mitigations

### Risk: Algorithm complexity
- **Mitigation:** Test thoroughly with synthetic data first
- **Fallback:** Keep legacy method available

### Risk: Numerical instability
- **Mitigation:** Use SVD (more stable than direct inversion)
- **Mitigation:** Add condition number checking

### Risk: Breaking existing calibrations
- **Mitigation:** Backward compatibility with old YAML format
- **Mitigation:** Auto-detect format and use appropriate correction

### Risk: User confusion
- **Mitigation:** Clear documentation
- **Mitigation:** Verification tool shows improvement

## Timeline Estimate

- **Phase 1 (Algorithm):** 2-3 hours
- **Phase 2 (Python):** 2-3 hours
- **Phase 3-5 (C++):** 3-4 hours
- **Phase 6 (Verification):** 1-2 hours
- **Testing & Debug:** 2-3 hours

**Total:** 10-15 hours of focused work

## Open Questions

1. **Minimum sample count for magnetometer?**
   - Current: 200+ samples (60s at ~3Hz)
   - May need more for good ellipsoid fit
   - Test with different counts

2. **Accelerometer: 6 positions sufficient?**
   - Mathematically: 6 points, 6 DOF (offset + 3 radii)
   - But matrix has 9 DOF
   - May need 12 positions for full matrix
   - Or: Use 6 positions with constrained fit

3. **Matrix conditioning:**
   - Check condition number of transformation matrix
   - Reject if poorly conditioned (>100?)
   - Warn user to recalibrate

4. **Temperature effects:**
   - Defer to future work
   - Would require temperature-dependent calibration
   - Significant additional complexity
