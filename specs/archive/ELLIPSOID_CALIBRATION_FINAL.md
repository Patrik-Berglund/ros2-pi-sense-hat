# Ellipsoid Calibration - Final Implementation Plan

## Summary

Fix accelerometer and magnetometer calibration using proper ellipsoid fitting with Gauss-Newton optimization.

**User procedure:** Rotate device for 60 seconds (same for both sensors)

**Algorithm:** Gauss-Newton ellipsoid fit → 3x3 transformation matrix

**Result:** 2-5x better accuracy

## Calibration Procedure (User Experience)

**Both sensors use same simple procedure:**

```bash
python3 scripts/calibrate_imu.py accel
# "Slowly rotate IMU in all directions for 60 seconds"

python3 scripts/calibrate_imu.py mag
# "Slowly rotate IMU in all directions for 60 seconds"
```

No precise positioning needed - just wave it around covering all orientations.

## Implementation

### Phase 1: Ellipsoid Fitting Algorithm

**Create:** `scripts/ellipsoid_fit.py`

```python
import numpy as np
from scipy.optimize import least_squares

def fit_ellipsoid(points):
    """
    Fit ellipsoid to 3D points using Gauss-Newton optimization.
    
    Args:
        points: Nx3 numpy array of measurements
        
    Returns:
        offset: [x0, y0, z0] center
        matrix: 3x3 transformation matrix
        radii: [a, b, c] semi-axes
        residual: RMS fit error
    """
    # Step 1: Initial guess using algebraic method (SVD)
    offset_init, matrix_init = fit_ellipsoid_algebraic(points)
    
    # Step 2: Refine using Gauss-Newton
    def residuals(params):
        offset = params[0:3]
        M = params[3:12].reshape(3, 3)
        
        corrected = (points - offset) @ M.T
        norms = np.linalg.norm(corrected, axis=1)
        return norms - 1.0  # Target: unit sphere
    
    x0 = np.concatenate([offset_init, matrix_init.flatten()])
    result = least_squares(residuals, x0, method='lm')
    
    offset = result.x[0:3]
    matrix = result.x[3:12].reshape(3, 3)
    
    # Calculate radii from matrix
    eigenvalues = np.linalg.eigvals(matrix.T @ matrix)
    radii = 1.0 / np.sqrt(np.abs(eigenvalues))
    
    # Calculate residual
    corrected = (points - offset) @ matrix.T
    norms = np.linalg.norm(corrected, axis=1)
    residual = np.std(norms)
    
    return offset, matrix, radii, residual


def fit_ellipsoid_algebraic(points):
    """
    Initial algebraic fit using SVD (fast but less accurate).
    Used as starting point for Gauss-Newton.
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    
    # Design matrix
    D = np.column_stack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        np.ones_like(x)
    ])
    
    # SVD solution
    _, _, V = np.linalg.svd(D)
    params = V[-1, :]
    
    A, B, C, D_xy, E_xz, F_yz, G, H, I, J = params
    
    # Extract center and matrix
    Q3 = np.array([[A, D_xy, E_xz],
                   [D_xy, B, F_yz],
                   [E_xz, F_yz, C]])
    u = np.array([G, H, I])
    
    offset = -np.linalg.solve(Q3, u)
    Q3_centered = Q3 / (-J - offset @ Q3 @ offset)
    
    eigenvalues, eigenvectors = np.linalg.eig(Q3_centered)
    radii = 1.0 / np.sqrt(np.abs(eigenvalues))
    
    scale_inv = np.diag(1.0 / radii)
    matrix = eigenvectors @ scale_inv @ eigenvectors.T
    
    return offset, matrix


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

### Phase 2: Update Python Calibration

**Modify:** `scripts/calibrate_imu.py`

**Add import:**
```python
from ellipsoid_fit import fit_ellipsoid, validate_calibration
```

**Replace accelerometer calibration:**
```python
def calibrate_accelerometer(self):
    """Calibrate accelerometer using rotation method"""
    print("\n🎯 Accelerometer Calibration")
    print("🔄 Slowly rotate IMU in ALL directions for 60 seconds")
    input("Press Enter to start...")
    
    # Collect samples
    samples = []
    start_time = time.time()
    
    while time.time() - start_time < 60:
        if len(self.imu_data) > 0:
            samples.append(self.imu_data[-1]['accel'])
        
        elapsed = time.time() - start_time
        progress = int((elapsed / 60) * 20)
        bar = "█" * progress + "░" * (20 - progress)
        print(f"\r🔄 [{bar}] {elapsed:.1f}s/60s", end="", flush=True)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.1)
    
    print(f"\n✅ Collected {len(samples)} samples")
    
    if len(samples) < 200:
        print(f"❌ Need 200+ samples, got {len(samples)}")
        return None
    
    # Fit ellipsoid
    data = np.array(samples)
    g = 9.80665
    offset, matrix, radii, residual = fit_ellipsoid(data)
    
    # Validate
    rms_error, max_error, mean_radius = validate_calibration(data, offset, matrix, g)
    
    print(f"\n✅ Calibration complete!")
    print(f"   RMS error: {rms_error:.4f} m/s² ({rms_error/g*100:.2f}%)")
    print(f"   Max error: {max_error:.4f} m/s²")
    
    if rms_error > 0.1:
        print(f"⚠️  High error - consider recalibrating")
    
    return {
        'accel_offset_x': float(offset[0]),
        'accel_offset_y': float(offset[1]),
        'accel_offset_z': float(offset[2]),
        'accel_matrix': matrix.flatten().tolist(),
        'accel_calibrated': True
    }
```

**Replace magnetometer calibration:**
```python
def calibrate_magnetometer(self):
    """Calibrate magnetometer using rotation method"""
    print("\n🧭 Magnetometer Calibration")
    print("🔄 Slowly rotate IMU in ALL directions for 60 seconds")
    input("Press Enter to start...")
    
    # Collect samples
    samples = []
    start_time = time.time()
    
    while time.time() - start_time < 60:
        if len(self.mag_data) > 0:
            samples.append(self.mag_data[-1]['mag'])
        
        elapsed = time.time() - start_time
        progress = int((elapsed / 60) * 20)
        bar = "█" * progress + "░" * (20 - progress)
        print(f"\r🔄 [{bar}] {elapsed:.1f}s/60s", end="", flush=True)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.1)
    
    print(f"\n✅ Collected {len(samples)} samples")
    
    if len(samples) < 200:
        print(f"❌ Need 200+ samples, got {len(samples)}")
        return None
    
    # Fit ellipsoid
    data = np.array(samples)
    offset, matrix, radii, residual = fit_ellipsoid(data)
    
    # For mag, estimate expected field from fit
    expected_radius = np.mean(radii)
    
    # Validate
    rms_error, max_error, mean_radius = validate_calibration(data, offset, matrix, expected_radius)
    
    print(f"\n✅ Calibration complete!")
    print(f"   Field strength: {expected_radius:.6f} T")
    print(f"   RMS error: {rms_error:.6f} T ({rms_error/expected_radius*100:.2f}%)")
    
    if rms_error/expected_radius > 0.02:
        print(f"⚠️  High error - consider recalibrating")
    
    return {
        'mag_offset_x': float(offset[0]),
        'mag_offset_y': float(offset[1]),
        'mag_offset_z': float(offset[2]),
        'mag_matrix': matrix.flatten().tolist(),
        'mag_calibrated': True,
        'mag_field_strength': float(expected_radius)
    }
```

### Phase 3: Update C++ (No Changes to Correction Logic)

**Only update data structures and YAML parsing:**

**`include/ros2_pi_sense_hat/imu_calibration.hpp`:**
```cpp
struct CalibrationData {
    // Gyroscope (unchanged)
    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;
    bool gyro_calibrated = false;
    
    // Accelerometer
    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;
    float accel_matrix[9] = {1,0,0, 0,1,0, 0,0,1};  // Identity default
    bool accel_calibrated = false;
    
    // Magnetometer
    float mag_offset_x = 0.0f;
    float mag_offset_y = 0.0f;
    float mag_offset_z = 0.0f;
    float mag_matrix[9] = {1,0,0, 0,1,0, 0,0,1};
    bool mag_calibrated = false;
};
```

**`src/imu_calibration.cpp` - correction logic:**
```cpp
void IMUCalibration::correctIMUData(IMUData& data) const {
    // Gyroscope
    if (cal_data_.gyro_calibrated) {
        data.gyro_x -= cal_data_.gyro_bias_x;
        data.gyro_y -= cal_data_.gyro_bias_y;
        data.gyro_z -= cal_data_.gyro_bias_z;
    }
    
    // Accelerometer
    if (cal_data_.accel_calibrated) {
        float ax = data.accel_x - cal_data_.accel_offset_x;
        float ay = data.accel_y - cal_data_.accel_offset_y;
        float az = data.accel_z - cal_data_.accel_offset_z;
        
        const float* M = cal_data_.accel_matrix;
        data.accel_x = M[0]*ax + M[1]*ay + M[2]*az;
        data.accel_y = M[3]*ax + M[4]*ay + M[5]*az;
        data.accel_z = M[6]*ax + M[7]*ay + M[8]*az;
    }
    
    // Magnetometer
    if (cal_data_.mag_calibrated) {
        float mx = data.mag_x - cal_data_.mag_offset_x;
        float my = data.mag_y - cal_data_.mag_offset_y;
        float mz = data.mag_z - cal_data_.mag_offset_z;
        
        const float* M = cal_data_.mag_matrix;
        data.mag_x = M[0]*mx + M[1]*my + M[2]*mz;
        data.mag_y = M[3]*mx + M[4]*my + M[5]*mz;
        data.mag_z = M[6]*mx + M[7]*my + M[8]*mz;
    }
}
```

**YAML parsing - add matrix loading:**
```cpp
// In loadCalibration(), add:
if (line.find("accel_matrix:") != std::string::npos) {
    // Parse [1.0, 0.0, ...] format
    std::string matrix_str = line.substr(line.find("[") + 1);
    matrix_str = matrix_str.substr(0, matrix_str.find("]"));
    
    std::stringstream ss(matrix_str);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 9) {
        cal_data_.accel_matrix[i++] = std::stof(value);
    }
}
// Similar for mag_matrix
```

### Phase 4: Verification Tool

**Create:** `scripts/verify_calibration.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np

class CalibrationVerifier(Node):
    def __init__(self):
        super().__init__('calibration_verifier')
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/imu/mag', self.mag_cb, 10)
        self.accel_errors = []
        self.mag_samples = []
        
    def imu_cb(self, msg):
        acc = msg.linear_acceleration
        norm = np.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
        error = abs(norm - 9.80665)
        self.accel_errors.append(error)
        
        if len(self.accel_errors) % 10 == 0:
            rms = np.sqrt(np.mean(np.array(self.accel_errors)**2))
            print(f"Accel: {len(self.accel_errors):4d} samples | "
                  f"RMS: {rms:.4f} m/s² ({rms/9.80665*100:.2f}%)")
    
    def mag_cb(self, msg):
        mag = msg.magnetic_field
        self.mag_samples.append([mag.x, mag.y, mag.z])
        
        if len(self.mag_samples) >= 100:
            data = np.array(self.mag_samples)
            norms = np.linalg.norm(data, axis=1)
            mean = np.mean(norms)
            std = np.std(norms)
            print(f"Mag: {len(self.mag_samples):4d} samples | "
                  f"Mean: {mean:.6f} T | Std: {std:.6f} T ({std/mean*100:.2f}%)")
            self.mag_samples = []

def main():
    rclpy.init()
    node = CalibrationVerifier()
    print("Verifying calibration - rotate IMU through orientations\n")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Implementation Checklist

- [ ] Create `scripts/ellipsoid_fit.py` with Gauss-Newton algorithm
- [ ] Test ellipsoid fitting with synthetic data
- [ ] Update `scripts/calibrate_imu.py` - accelerometer rotation method
- [ ] Update `scripts/calibrate_imu.py` - magnetometer rotation method
- [ ] Update C++ `CalibrationData` struct with matrix arrays
- [ ] Update C++ `correctIMUData()` with matrix multiplication
- [ ] Update C++ YAML parsing to load matrices
- [ ] Update C++ YAML saving to write matrices
- [ ] Create `scripts/verify_calibration.py`
- [ ] Test full calibration workflow
- [ ] Verify accuracy improvement

## Expected Results

**Accelerometer:**
- Current: RMS error ~0.1 m/s² (1%)
- Target: RMS error <0.05 m/s² (0.5%)
- Improvement: 2x better

**Magnetometer:**
- Current: Std dev ~2-5% of field
- Target: Std dev <1% of field
- Improvement: 2-5x better

## Timeline

- Phase 1 (Algorithm): 2-3 hours
- Phase 2 (Python): 2 hours
- Phase 3 (C++): 3 hours
- Phase 4 (Verification): 1 hour
- Testing: 2 hours

**Total: ~10 hours**
