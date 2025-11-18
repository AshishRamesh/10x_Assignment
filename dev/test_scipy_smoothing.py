#!/usr/bin/env python3
"""
Test script for SciPy-based path smoothing
"""

import numpy as np
import math
from scipy.interpolate import CubicSpline, splprep, splev
from typing import List, Tuple
import matplotlib.pyplot as plt

def test_scipy_smoothing():
    """Test the SciPy smoothing implementation"""
    
    # Test waypoints (same as in the node)
    waypoints = [
        (0.0, 0.0),
        (1.0, 0.5),
        (2.0, 1.0),
        (2.5, 0.5),
        (3.0, 0.0)
    ]
    
    num_points = 150
    
    # Extract coordinates
    x_points = np.array([wp[0] for wp in waypoints])
    y_points = np.array([wp[1] for wp in waypoints])
    
    print(f"Original waypoints: {len(waypoints)} points")
    print("Testing parametric spline interpolation...")
    
    try:
        # Method 1: Parametric spline
        tck, u = splprep([x_points, y_points], s=0, k=min(3, len(waypoints)-1))
        u_new = np.linspace(0, 1, num_points)
        smooth_points = splev(u_new, tck)
        smooth_x, smooth_y = smooth_points
        
        print(f"✅ Parametric spline successful: {len(smooth_x)} points generated")
        
        # Test distance-based method as well
        print("Testing distance-based spline interpolation...")
        
        distances = [0.0]
        for i in range(1, len(waypoints)):
            dx = x_points[i] - x_points[i-1]
            dy = y_points[i] - y_points[i-1]
            dist = math.sqrt(dx*dx + dy*dy)
            distances.append(distances[-1] + dist)
        
        distances = np.array(distances)
        
        spline_x = CubicSpline(distances, x_points, bc_type='natural')
        spline_y = CubicSpline(distances, y_points, bc_type='natural')
        
        total_distance = distances[-1]
        sample_distances = np.linspace(0, total_distance, num_points)
        
        smooth_x_dist = spline_x(sample_distances)
        smooth_y_dist = spline_y(sample_distances)
        
        print(f"✅ Distance-based spline successful: {len(smooth_x_dist)} points generated")
        
        # Compare methods
        print(f"Total path length (parametric): {total_distance:.2f} units")
        
        return True
        
    except Exception as e:
        print(f"❌ Error in SciPy smoothing: {e}")
        return False

if __name__ == "__main__":
    success = test_scipy_smoothing()
    if success:
        print("\n🎉 All SciPy smoothing tests passed!")
    else:
        print("\n⚠️  Some tests failed!")