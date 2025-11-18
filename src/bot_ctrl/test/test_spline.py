#!/usr/bin/env python3
"""
Simple test script to verify the cubic spline implementation
"""

import sys
import os

# Add the bot_ctrl package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'bot_ctrl'))

from path_smoother import CubicSpline
import numpy as np


def test_cubic_spline():
    """Test the cubic spline implementation with simple data"""
    print("Testing Cubic Spline Implementation...")
    
    # Test with simple waypoints
    waypoints = [
        (0.0, 0.0),
        (1.0, 0.5),
        (2.0, 1.0),
        (2.5, 0.5),
        (3.0, 0.0)
    ]
    
    x_points = [wp[0] for wp in waypoints]
    y_points = [wp[1] for wp in waypoints]
    
    print(f"Original waypoints: {waypoints}")
    
    # Create spline
    spline_x = CubicSpline([0, 1, 2, 3, 4], x_points)  # Simple parameterization
    spline_y = CubicSpline([0, 1, 2, 3, 4], y_points)
    
    # Test interpolation at original points
    print("\nTesting interpolation at original parameter values:")
    for i in range(5):
        x_interp = spline_x.interpolate(i)
        y_interp = spline_y.interpolate(i)
        print(f"t={i}: Original=({x_points[i]}, {y_points[i]}), Interpolated=({x_interp:.6f}, {y_interp:.6f})")
    
    # Test interpolation at intermediate points
    print("\nTesting interpolation at intermediate points:")
    test_params = [0.5, 1.5, 2.5, 3.5]
    for t in test_params:
        x_interp = spline_x.interpolate(t)
        y_interp = spline_y.interpolate(t)
        print(f"t={t}: Interpolated=({x_interp:.6f}, {y_interp:.6f})")
    
    print("\nCubic Spline test completed successfully!")


if __name__ == '__main__':
    test_cubic_spline()