#!/usr/bin/env python3
"""
Ellipsoid fitting for IMU calibration using Gauss-Newton optimization.
"""

import numpy as np
from scipy.optimize import least_squares


def fit_ellipsoid(points):
    """
    Fit ellipsoid to 3D points using Gauss-Newton optimization.
    
    Args:
        points: Nx3 numpy array of measurements
        
    Returns:
        offset: [x0, y0, z0] center of ellipsoid
        matrix: 3x3 transformation matrix
        radii: [a, b, c] semi-axes lengths
        residual: RMS fit error
    """
    # Step 1: Get initial guess using algebraic method
    offset_init, matrix_init = _fit_ellipsoid_algebraic(points)
    
    # Step 2: Refine using Gauss-Newton (Levenberg-Marquardt)
    def residuals(params):
        offset = params[0:3]
        M = params[3:12].reshape(3, 3)
        
        corrected = (points - offset) @ M.T
        norms = np.linalg.norm(corrected, axis=1)
        return norms - 1.0  # Target: unit sphere
    
    # Initial parameters
    x0 = np.concatenate([offset_init, matrix_init.flatten()])
    
    # Optimize
    result = least_squares(residuals, x0, method='lm')
    
    # Extract results
    offset = result.x[0:3]
    matrix = result.x[3:12].reshape(3, 3)
    
    # Calculate expected radius from data
    centered = points - offset
    expected_radius = np.mean(np.linalg.norm(centered, axis=1))
    
    # Scale matrix to preserve expected radius
    matrix = matrix * expected_radius
    
    # Calculate radii from matrix
    eigenvalues = np.linalg.eigvals(matrix.T @ matrix)
    radii = 1.0 / np.sqrt(np.abs(eigenvalues)) * expected_radius
    
    # Calculate residual
    corrected = (points - offset) @ matrix.T
    norms = np.linalg.norm(corrected, axis=1)
    residual = np.std(norms)
    
    return offset, matrix, radii, residual


def _fit_ellipsoid_algebraic(points):
    """
    Initial algebraic fit using SVD.
    Fast but less accurate - used as starting point for Gauss-Newton.
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # Design matrix for ellipsoid equation:
    # Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
    D = np.column_stack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        np.ones_like(x)
    ])
    
    # Solve using SVD
    _, _, V = np.linalg.svd(D)
    params = V[-1, :]
    
    # Extract parameters
    A, B, C, D_xy, E_xz, F_yz, G, H, I, J = params
    
    # Build quadratic form matrix
    Q3 = np.array([
        [A, D_xy, E_xz],
        [D_xy, B, F_yz],
        [E_xz, F_yz, C]
    ])
    
    # Linear terms
    u = np.array([G, H, I])
    
    # Solve for center
    offset = -np.linalg.solve(Q3, u)
    
    # Translate to center
    Q3_centered = Q3 / (-J - offset @ Q3 @ offset)
    
    # Eigendecomposition
    eigenvalues, eigenvectors = np.linalg.eig(Q3_centered)
    radii = 1.0 / np.sqrt(np.abs(eigenvalues))
    
    # Build transformation matrix
    scale_inv = np.diag(1.0 / radii)
    matrix = eigenvectors @ scale_inv @ eigenvectors.T
    
    return offset, matrix


def validate_calibration(points, offset, matrix, expected_radius):
    """
    Validate calibration quality.
    
    Args:
        points: Nx3 numpy array of measurements
        offset: [x0, y0, z0] center offset
        matrix: 3x3 transformation matrix
        expected_radius: Expected radius after correction
        
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
