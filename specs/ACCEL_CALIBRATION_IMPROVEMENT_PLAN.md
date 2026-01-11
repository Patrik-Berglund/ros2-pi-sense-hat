# Accelerometer Calibration Improvement Plan

## Problem Statement

The current 6-point calibration method only corrects for:
- **Offset (bias)** on each axis
- **Scale factor** on each axis

**What it misses:**
- **Cross-axis misalignment** - axes not perfectly orthogonal
- **Non-linearity** - sensor response varies across range
- **Temperature effects** - calibration drifts with temperature

**Result:** Accuracy limited to ±0.01g, insufficient for precision applications.

## Proposed Improvement: Multi-Position Calibration with Full Matrix Correction

### Method Overview

**12-position calibration** with full 3x3 transformation matrix:
- Corrects offset, scale, AND cross-axis misalignment
- Uses least-squares fitting to sphere surface
- Achieves ±0.001-0.005g accuracy

### Mathematical Model

**Current (simple) correction:**
```
accel_corrected = (accel_raw - offset) * scale
```

**Improved (full matrix) correction:**
```
accel_corrected = M * (accel_raw - offset)

where M is 3x3 transformation matrix:
M = [m11  m12  m13]
    [m21  m22  m23]
    [m31  m32  m33]
```

**What M corrects:**
- Diagonal elements (m11, m22, m33): Scale factors
- Off-diagonal elements: Cross-axis misalignment (non-orthogonality)

### Calibration Procedure

**12 static positions** (cube faces + edges):

1. **6 face positions** (same as current):
   - +X up, -X up
   - +Y up, -Y up
   - +Z up, -Z up

2. **6 additional edge positions** (NEW):
   - +X+Y edge (45° between X and Y)
   - +X-Y edge
   - +Y+Z edge
   - +Y-Z edge
   - +Z+X edge
   - +Z-X edge

**Why 12 positions?**
- 6 positions: 6 measurements, 6 unknowns (3 offsets + 3 scales) → just enough
- 12 positions: 12 measurements, 12 unknowns (3 offsets + 9 matrix elements) → overdetermined system, better accuracy

### Algorithm: Ellipsoid Fitting

**Concept:** Accelerometer measurements should lie on a sphere of radius g (9.80665 m/s²).
Due to sensor imperfections, they lie on an ellipsoid instead.

**Steps:**
1. Collect 12+ measurements at known orientations
2. Fit ellipsoid to measurement points
3. Calculate transformation matrix M that maps ellipsoid → sphere
4. Extract offset and matrix from ellipsoid parameters

**Ellipsoid equation:**
```
(x-x0)²/a² + (y-y0)²/b² + (z-z0)²/c² = 1

where:
- (x0, y0, z0) = center offset
- (a, b, c) = semi-axes (related to scale)
- Rotation angles = cross-axis misalignment
```

### Implementation Architecture

**Python calibration script** (`scripts/calibrate_imu.py`):
```python
def calibrate_accelerometer_advanced(self):
    """12-position calibration with full matrix correction"""
    
    positions = [
        # 6 face positions
        ("+X", [1, 0, 0]),
        ("-X", [-1, 0, 0]),
        ("+Y", [0, 1, 0]),
        ("-Y", [0, -1, 0]),
        ("+Z", [0, 0, 1]),
        ("-Z", [0, 0, -1]),
        
        # 6 edge positions (normalized)
        ("+X+Y", [0.707, 0.707, 0]),
        ("+X-Y", [0.707, -0.707, 0]),
        ("+Y+Z", [0, 0.707, 0.707]),
        ("+Y-Z", [0, 0.707, -0.707]),
        ("+Z+X", [0.707, 0, 0.707]),
        ("+Z-X", [-0.707, 0, 0.707])
    ]
    
    measurements = []
    for name, expected_g in positions:
        # Guide user to position
        # Collect samples
        # Store measurement
        pass
    
    # Fit ellipsoid using least-squares
    offset, matrix = fit_ellipsoid_to_sphere(measurements)
    
    return {
        'accel_offset': offset,  # [x, y, z]
        'accel_matrix': matrix,  # 3x3 flattened to 9 elements
        'accel_calibrated': True
    }

def fit_ellipsoid_to_sphere(measurements):
    """Fit ellipsoid and compute transformation to sphere"""
    # Use scipy.optimize or numpy least-squares
    # Return offset vector and 3x3 transformation matrix
    pass
```

**C++ real-time correction** (`src/imu_calibration.cpp`):
```cpp
struct CalibrationData {
    // Offset
    float accel_offset_x, accel_offset_y, accel_offset_z;
    
    // 3x3 transformation matrix (row-major)
    float accel_matrix[9];  // [m11, m12, m13, m21, m22, m23, m31, m32, m33]
    
    bool accel_calibrated;
};

void IMUCalibration::correctIMUData(IMUData& data) const {
    if (cal_data_.accel_calibrated) {
        // Step 1: Remove offset
        float ax = data.accel_x - cal_data_.accel_offset_x;
        float ay = data.accel_y - cal_data_.accel_offset_y;
        float az = data.accel_z - cal_data_.accel_offset_z;
        
        // Step 2: Apply transformation matrix
        data.accel_x = cal_data_.accel_matrix[0] * ax + 
                       cal_data_.accel_matrix[1] * ay + 
                       cal_data_.accel_matrix[2] * az;
        
        data.accel_y = cal_data_.accel_matrix[3] * ax + 
                       cal_data_.accel_matrix[4] * ay + 
                       cal_data_.accel_matrix[5] * az;
        
        data.accel_z = cal_data_.accel_matrix[6] * ax + 
                       cal_data_.accel_matrix[7] * ay + 
                       cal_data_.accel_matrix[8] * az;
    }
}
```

## Alternative: Simplified 6-Position with Least-Squares

If 12 positions are too cumbersome, improve the current 6-position method:

**Current issue:** Simple averaging doesn't account for measurement noise optimally.

**Improvement:** Use least-squares optimization on 6 positions:
```python
def calibrate_accelerometer_improved_6point(measurements):
    """
    Optimize offset and scale to minimize error from g = 9.80665
    """
    from scipy.optimize import least_squares
    
    def residuals(params):
        offset = params[0:3]
        scale = params[3:6]
        
        errors = []
        for meas in measurements:
            corrected = (meas - offset) * scale
            magnitude = np.linalg.norm(corrected)
            errors.append(magnitude - 9.80665)
        
        return errors
    
    # Initial guess
    x0 = [0, 0, 0, 1, 1, 1]
    
    result = least_squares(residuals, x0)
    offset = result.x[0:3]
    scale = result.x[3:6]
    
    return offset, scale
```

**Benefit:** Better accuracy from same 6 positions, no extra user effort.

## Comparison of Methods

| Method | Positions | Corrects | Accuracy | User Effort | Computation |
|--------|-----------|----------|----------|-------------|-------------|
| Current 6-point | 6 | Offset + Scale | ±0.01g | Low | Minimal |
| Improved 6-point LS | 6 | Offset + Scale (optimized) | ±0.005g | Low | Low |
| 12-point Matrix | 12 | Offset + Scale + Misalignment | ±0.001-0.005g | Medium | Medium |

## Recommendation

**Phase 1:** Implement improved 6-point with least-squares
- Quick win: Better accuracy with no extra user effort
- Drop-in replacement for current method
- Validates optimization approach

**Phase 2:** Add optional 12-point matrix calibration
- For users needing maximum accuracy
- Separate calibration mode: `python3 scripts/calibrate_imu.py accel_advanced`
- Backward compatible with simple 6-point

## Implementation Tasks

### Phase 1: Improved 6-Point (Quick Win)

1. **Modify Python calibration** (`scripts/calibrate_imu.py`):
   - Add `scipy.optimize.least_squares` import
   - Replace simple averaging with optimization
   - Keep same 6-position user flow

2. **Test accuracy improvement**:
   - Compare before/after calibration error
   - Measure magnitude deviation from 9.80665 m/s²

### Phase 2: Full 12-Point Matrix (Maximum Accuracy)

1. **Add 12-position calibration** (`scripts/calibrate_imu.py`):
   - New function `calibrate_accelerometer_advanced()`
   - Implement ellipsoid fitting algorithm
   - Output 3x3 matrix to YAML

2. **Update C++ correction** (`src/imu_calibration.cpp`):
   - Add matrix storage to `CalibrationData`
   - Implement matrix multiplication correction
   - Maintain backward compatibility with simple scale

3. **Update YAML format**:
   ```yaml
   # Simple calibration (backward compatible)
   accel_offset_x: 0.123
   accel_offset_y: -0.234
   accel_offset_z: 0.345
   accel_scale_x: 1.001
   accel_scale_y: 0.998
   accel_scale_z: 1.002
   
   # Advanced calibration (optional)
   accel_matrix: [1.001, 0.002, -0.001,
                  0.001, 0.998, 0.003,
                  -0.002, 0.001, 1.002]
   accel_calibrated: true
   accel_advanced: true  # Flag for matrix vs simple
   ```

4. **Add calibration validation**:
   - Compute RMS error after calibration
   - Report accuracy to user
   - Suggest recalibration if error > threshold

## Expected Outcomes

**Phase 1 (Improved 6-point):**
- Accuracy: ±0.01g → ±0.005g (2x improvement)
- Implementation time: 1-2 hours
- No change to user experience

**Phase 2 (12-point matrix):**
- Accuracy: ±0.005g → ±0.001-0.003g (2-5x improvement)
- Implementation time: 4-6 hours
- Slightly more user effort (12 vs 6 positions)
- Corrects cross-axis misalignment

## References

- [Accelerometer Calibration: A Practical Approach](https://www.nxp.com/docs/en/application-note/AN4399.pdf)
- [Ellipsoid Fitting for Sensor Calibration](https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit)
- [IMU Calibration Best Practices](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imucal)

## Open Questions

1. **Which phase to implement first?** 
   - Recommend Phase 1 for quick improvement
   - Phase 2 only if ±0.005g insufficient

2. **Temperature compensation needed?**
   - Requires temperature-dependent calibration
   - Significant additional complexity
   - Defer unless critical

3. **Validation method?**
   - How to measure actual accuracy improvement?
   - Need reference accelerometer or known gravity vector
   - Can use magnitude error as proxy

4. **User experience for 12 positions?**
   - Visual guide for edge positions?
   - Tolerance for positioning accuracy?
   - Automatic position detection?
