# Ellipsoid Fitting Calibration Plan

## Problem Statement

**Current implementation is misleading:**
- Magnetometer function claims "ellipsoid fitting" but uses simple min/max
- Accelerometer uses simple 6-point averaging
- Both use same flawed approach: independent axis offset + scale
- Neither corrects cross-axis coupling

**Result:**
- Accelerometer: ±0.01g accuracy (insufficient for precision)
- Magnetometer: ±2-5° heading error (should be <1°)
- Both miss non-orthogonality and coupling effects

## Root Cause

Both sensors should measure points on a **sphere**:
- **Accelerometer**: Sphere of radius g = 9.80665 m/s²
- **Magnetometer**: Sphere of radius = local magnetic field strength (~0.5 gauss)

Due to sensor imperfections, measurements form an **ellipsoid** instead:
- Offset (hard-iron for mag, bias for accel)
- Non-uniform scaling per axis
- **Cross-axis coupling** (axes not orthogonal)

**Current method only fixes first two, ignores coupling.**

## Solution: Proper Ellipsoid Fitting

### Mathematical Model

**Ellipsoid equation (general form):**
```
Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
```

**10 parameters to solve:**
- A, B, C: Quadratic terms (scaling)
- D, E, F: Cross terms (coupling)
- G, H, I: Linear terms (offset)
- J: Constant

**Goal:** Find transformation that maps ellipsoid → sphere

### Algorithm: Least-Squares Ellipsoid Fit

**Input:** N measurement points (x, y, z) where N >> 10

**Output:** 
- Offset vector: [x₀, y₀, z₀]
- Transformation matrix M (3x3)

**Steps:**
1. Collect many measurements (100+ for mag, 12+ for accel)
2. Solve for ellipsoid parameters using least-squares
3. Decompose ellipsoid into center + radii + rotation
4. Compute transformation: M = R * S * R^T
   - R: Rotation matrix (aligns ellipsoid axes)
   - S: Scale matrix (normalizes radii)

**Correction formula:**
```
corrected = M * (raw - offset)
```

## Implementation Plan

### Phase 1: Core Ellipsoid Fitting Algorithm

**Create shared utility** (`demo/ellipsoid_fit.py`):

```python
import numpy as np
from numpy.linalg import lstsq, eig

def fit_ellipsoid(points):
    """
    Fit ellipsoid to 3D points using least-squares.
    
    Args:
        points: Nx3 array of measurements
        
    Returns:
        offset: [x0, y0, z0] center of ellipsoid
        matrix: 3x3 transformation matrix
        radii: [a, b, c] semi-axes lengths (for validation)
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # Build design matrix for ellipsoid equation
    # Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
    D = np.array([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        np.ones_like(x)
    ]).T
    
    # Solve least-squares: D * params = 0
    # Use constraint: 4*A*B*C + D² + E² + F² - A*F² - B*E² - C*D² = 1
    # (Ensures unique solution)
    
    # Simplified: Use SVD to find null space
    _, _, V = np.linalg.svd(D)
    params = V[-1, :]  # Last singular vector
    
    # Extract parameters
    A, B, C, D, E, F, G, H, I, J = params
    
    # Compute center (offset)
    center_matrix = np.array([
        [A, D, E],
        [D, B, F],
        [E, F, C]
    ])
    center_vector = np.array([G, H, I])
    offset = -np.linalg.solve(center_matrix, center_vector)
    
    # Translate ellipsoid to origin
    T = np.eye(4)
    T[3, 0:3] = offset
    
    # Build quadratic form matrix
    Q = np.array([
        [A, D, E, G],
        [D, B, F, H],
        [E, F, C, I],
        [G, H, I, J]
    ])
    
    # Apply translation
    Q_centered = T.T @ Q @ T
    
    # Extract 3x3 matrix
    Q3 = Q_centered[0:3, 0:3] / -Q_centered[3, 3]
    
    # Eigendecomposition to get radii and rotation
    eigenvalues, eigenvectors = eig(Q3)
    radii = 1.0 / np.sqrt(eigenvalues)
    
    # Build transformation matrix
    # M = R * S where S scales radii to 1
    scale_matrix = np.diag(1.0 / radii)
    rotation_matrix = eigenvectors
    
    transformation = rotation_matrix @ scale_matrix @ rotation_matrix.T
    
    return offset, transformation, radii


def validate_calibration(points, offset, matrix, expected_radius):
    """
    Validate calibration by checking corrected points lie on sphere.
    
    Returns:
        rms_error: RMS deviation from expected radius
        max_error: Maximum deviation
    """
    corrected = (points - offset) @ matrix.T
    magnitudes = np.linalg.norm(corrected, axis=1)
    errors = magnitudes - expected_radius
    
    rms_error = np.sqrt(np.mean(errors**2))
    max_error = np.max(np.abs(errors))
    
    return rms_error, max_error
```

### Phase 2: Update Magnetometer Calibration

**Modify** `scripts/calibrate_imu.py`:

```python
from ellipsoid_fit import fit_ellipsoid, validate_calibration

def calibrate_magnetometer(self):
    """Calibrate magnetometer using TRUE ellipsoid fitting"""
    print("\n🧭 Starting magnetometer calibration...")
    print("🔄 Rotate the IMU in ALL directions!")
    print("   Goal: Cover a complete sphere of orientations")
    print("   Duration: 60 seconds (need 200+ samples)")
    
    input("Press Enter when ready to start rotating...")
    
    # Collect samples
    mag_samples = self.collect_rotation_samples(duration=60)
    
    if len(mag_samples) < 200:
        print(f"❌ Need 200+ samples, got {len(mag_samples)}")
        return None
    
    mag_data = np.array(mag_samples)
    
    # Fit ellipsoid
    print("🔄 Fitting ellipsoid to data...")
    offset, matrix, radii = fit_ellipsoid(mag_data)
    
    # Estimate expected field strength from data
    expected_radius = np.mean(radii)
    
    # Validate
    rms_error, max_error = validate_calibration(
        mag_data, offset, matrix, expected_radius
    )
    
    print(f"\n✅ Magnetometer calibration complete!")
    print(f"   Offset: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}] T")
    print(f"   Radii: [{radii[0]:.6f}, {radii[1]:.6f}, {radii[2]:.6f}] T")
    print(f"   Expected field: {expected_radius:.6f} T")
    print(f"   RMS error: {rms_error:.6f} T ({rms_error/expected_radius*100:.2f}%)")
    print(f"   Max error: {max_error:.6f} T ({max_error/expected_radius*100:.2f}%)")
    
    # Flatten matrix for storage
    matrix_flat = matrix.flatten().tolist()
    
    return {
        'mag_offset_x': float(offset[0]),
        'mag_offset_y': float(offset[1]),
        'mag_offset_z': float(offset[2]),
        'mag_matrix': matrix_flat,  # 9 elements, row-major
        'mag_calibrated': True,
        'mag_field_strength': float(expected_radius)
    }
```

### Phase 3: Update Accelerometer Calibration

**Two options:**

**Option A: 12-position static (more accurate)**
```python
def calibrate_accelerometer_12point(self):
    """12-position calibration with ellipsoid fitting"""
    positions = [
        # 6 faces
        ("+X up", "X-axis pointing up"),
        ("-X up", "X-axis pointing down"),
        ("+Y up", "Y-axis pointing up"),
        ("-Y up", "Y-axis pointing down"),
        ("+Z up", "Z-axis pointing up"),
        ("-Z up", "Z-axis pointing down"),
        
        # 6 edges (45° angles)
        ("+X+Y edge", "Between +X and +Y"),
        ("+X-Y edge", "Between +X and -Y"),
        ("+Y+Z edge", "Between +Y and +Z"),
        ("+Y-Z edge", "Between +Y and -Z"),
        ("+Z+X edge", "Between +Z and +X"),
        ("-Z+X edge", "Between -Z and +X")
    ]
    
    measurements = []
    for name, desc in positions:
        print(f"\n📐 Position: {name} ({desc})")
        input("   Press Enter when positioned...")
        samples = self.collect_samples(5)
        accel_data = np.array([s['accel'] for s in samples])
        measurements.append(np.mean(accel_data, axis=0))
    
    accel_data = np.array(measurements)
    
    # Fit ellipsoid
    g = 9.80665
    offset, matrix, radii = fit_ellipsoid(accel_data)
    
    # Validate
    rms_error, max_error = validate_calibration(accel_data, offset, matrix, g)
    
    print(f"\n✅ Accelerometer calibration complete!")
    print(f"   RMS error: {rms_error:.6f} m/s² ({rms_error/g*100:.3f}%)")
    
    return {
        'accel_offset_x': float(offset[0]),
        'accel_offset_y': float(offset[1]),
        'accel_offset_z': float(offset[2]),
        'accel_matrix': matrix.flatten().tolist(),
        'accel_calibrated': True
    }
```

**Option B: Rotation method (easier for user)**
```python
def calibrate_accelerometer_rotation(self):
    """Rotation-based calibration (like magnetometer)"""
    print("\n🎯 Starting accelerometer calibration...")
    print("🔄 Slowly rotate IMU through all orientations")
    print("   Duration: 60 seconds")
    
    input("Press Enter when ready...")
    
    samples = self.collect_rotation_samples(duration=60)
    accel_data = np.array([s['accel'] for s in samples])
    
    g = 9.80665
    offset, matrix, radii = fit_ellipsoid(accel_data)
    
    # Validate
    rms_error, max_error = validate_calibration(accel_data, offset, matrix, g)
    
    print(f"\n✅ Accelerometer calibration complete!")
    print(f"   RMS error: {rms_error:.6f} m/s² ({rms_error/g*100:.3f}%)")
    
    return {
        'accel_offset_x': float(offset[0]),
        'accel_offset_y': float(offset[1]),
        'accel_offset_z': float(offset[2]),
        'accel_matrix': matrix.flatten().tolist(),
        'accel_calibrated': True
    }
```

### Phase 4: Update C++ Real-Time Correction

**Modify** `src/imu_calibration.cpp`:

```cpp
struct CalibrationData {
    // Accelerometer
    float accel_offset_x, accel_offset_y, accel_offset_z;
    float accel_matrix[9];  // 3x3 row-major
    bool accel_calibrated;
    
    // Magnetometer
    float mag_offset_x, mag_offset_y, mag_offset_z;
    float mag_matrix[9];  // 3x3 row-major
    bool mag_calibrated;
    
    // Gyroscope (unchanged)
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;
    bool gyro_calibrated;
};

void IMUCalibration::correctIMUData(IMUData& data) const {
    // Gyroscope (unchanged)
    if (cal_data_.gyro_calibrated) {
        data.gyro_x -= cal_data_.gyro_bias_x;
        data.gyro_y -= cal_data_.gyro_bias_y;
        data.gyro_z -= cal_data_.gyro_bias_z;
    }
    
    // Accelerometer with matrix
    if (cal_data_.accel_calibrated) {
        float ax = data.accel_x - cal_data_.accel_offset_x;
        float ay = data.accel_y - cal_data_.accel_offset_y;
        float az = data.accel_z - cal_data_.accel_offset_z;
        
        const float* M = cal_data_.accel_matrix;
        data.accel_x = M[0]*ax + M[1]*ay + M[2]*az;
        data.accel_y = M[3]*ax + M[4]*ay + M[5]*az;
        data.accel_z = M[6]*ax + M[7]*ay + M[8]*az;
    }
    
    // Magnetometer with matrix
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

### Phase 5: Update YAML Format

```yaml
# IMU Calibration Data - Ellipsoid Fitting

# Gyroscope (unchanged)
gyro_bias_x: -0.001234
gyro_bias_y: 0.002345
gyro_bias_z: -0.000567
gyro_calibrated: true

# Accelerometer (ellipsoid fit)
accel_offset_x: 0.123456
accel_offset_y: -0.234567
accel_offset_z: 0.345678
accel_matrix: [1.001234, 0.002345, -0.001234,
               0.001234, 0.998765, 0.003456,
               -0.002345, 0.001234, 1.002345]
accel_calibrated: true

# Magnetometer (ellipsoid fit)
mag_offset_x: 0.000012
mag_offset_y: -0.000023
mag_offset_z: 0.000034
mag_matrix: [1.123456, 0.012345, -0.023456,
             0.012345, 0.987654, 0.034567,
             -0.023456, 0.034567, 1.045678]
mag_calibrated: true
mag_field_strength: 0.000485  # Local field strength in Tesla
```

## Expected Improvements

### Accelerometer
- **Current**: ±0.01g (simple 6-point)
- **After**: ±0.001-0.003g (ellipsoid fit)
- **Improvement**: 3-10x better accuracy
- **Benefit**: Corrects cross-axis coupling

### Magnetometer
- **Current**: ±2-5° heading error (min/max method)
- **After**: ±0.5-1° heading error (ellipsoid fit)
- **Improvement**: 2-5x better accuracy
- **Benefit**: True hard/soft-iron correction

## Implementation Checklist

- [ ] Create `demo/ellipsoid_fit.py` with core algorithm
- [ ] Add unit tests for ellipsoid fitting
- [ ] Update magnetometer calibration to use ellipsoid fit
- [ ] Choose accelerometer method (12-point vs rotation)
- [ ] Update accelerometer calibration
- [ ] Modify C++ calibration data structure
- [ ] Update C++ correction to use matrices
- [ ] Update YAML loading/saving for matrices
- [ ] Add calibration validation reporting
- [ ] Test both sensors with new calibration
- [ ] Update documentation

## Testing Strategy

### Validation Metrics

**Accelerometer:**
- RMS error from g = 9.80665 m/s²
- Target: <0.05 m/s² (<0.5%)

**Magnetometer:**
- RMS error from expected field strength
- Target: <1% of field strength
- Heading accuracy test: rotate 360°, measure drift

### Test Procedure

1. Run old calibration, measure errors
2. Run new calibration, measure errors
3. Compare before/after
4. Verify matrix is well-conditioned (condition number < 100)
5. Check radii are reasonable (not too elliptical)

## Open Questions

1. **Accelerometer method**: 12-point static or rotation?
   - 12-point: More accurate, more user effort
   - Rotation: Easier, but need to move slowly

2. **Minimum sample count**:
   - Magnetometer: 200+ samples (60s at 10Hz)
   - Accelerometer: 12 positions or 200+ samples?

3. **Backward compatibility**:
   - Support old simple calibration format?
   - Auto-detect format in C++ loader?

4. **Calibration quality threshold**:
   - Reject calibration if RMS error too high?
   - What's acceptable error threshold?

## References

- [Ellipsoid Fitting Paper](https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit)
- [Magnetometer Calibration](https://teslabs.com/articles/magnetometer-calibration/)
- [Accelerometer Calibration](https://www.nxp.com/docs/en/application-note/AN4399.pdf)
