import numpy as np
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from file_io import read_points_from_file

def test_basic_math_operations():
    print("=== Testing Basic Math Operations ===")
    
    # Test 1: Point3D operations
    p1 = Point3D(1, 2, 3)
    p2 = Point3D(4, 5, 6)
    print(f"Point p1: ({p1.x}, {p1.y}, {p1.z})")
    print(f"Point as array: {p1.to_array()}")
    
    # Test 2: Rotation - 90 degrees around Z-axis
    R_z_90 = np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]
    ])
    rotation = Rotation(R_z_90)
    p_rotated = rotation @ p1
    print(f"Point rotated 90° around Z: ({p_rotated.x:.2f}, {p_rotated.y:.2f}, {p_rotated.z:.2f})")
    # Expected: (-2, 1, 3)
    
    # Test 3: Frame transformation
    translation = Point3D(10, 20, 30)
    frame = Frame(rotation, translation)
    p_transformed = frame.transform_point(p1)
    print(f"Point after frame transformation: ({p_transformed.x:.2f}, {p_transformed.y:.2f}, {p_transformed.z:.2f})")
    # Expected: (8, 21, 33) because: rotate (1,2,3) -> (-2,1,3) then translate + (10,20,30) -> (8,21,33)
    
    # Test 4: Frame inverse
    frame_inv = frame.inverse()
    p_restored = frame_inv.transform_point(p_transformed)
    print(f"Point after inverse transformation: ({p_restored.x:.2f}, {p_restored.y:.2f}, {p_restored.z:.2f})")
    # Should return to original: (1, 2, 3)
    
    print("Basic math tests completed!\n")