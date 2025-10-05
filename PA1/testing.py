import numpy as np
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from file_io import read_points_from_file

def test_basic_math_operations():    
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
    
    print("math tests completed\n")

def test_rigid_transform_algorithm():        
    # Test 1: Pure translation
    print("Test 1: Pure Translation")
    points_A = [Point3D(1, 1, 1), Point3D(2, 1, 1), Point3D(1, 2, 1)]
    points_B = [Point3D(4, 4, 4), Point3D(5, 4, 4), Point3D(4, 5, 4)]  # Translated by (3,3,3)
    
    frame = find_rigid_transform(points_A, points_B)
    
    # Test the transformation
    test_point = points_A[0]
    transformed = frame.transform_point(test_point)
    expected = points_B[0]
    
    print(f"Original point A: ({test_point.x}, {test_point.y}, {test_point.z})")
    print(f"Transformed point: ({transformed.x:.2f}, {transformed.y:.2f}, {transformed.z:.2f})")
    print(f"Expected point B: ({expected.x}, {expected.y}, {expected.z})")
    print(f"Error: {np.linalg.norm(transformed.to_array() - expected.to_array()):.6f}")
    
    # Test 2: Rotation only
    print("\nTest 2: 90° Rotation around Z")
    points_A = [Point3D(1, 0, 0), Point3D(0, 1, 0), Point3D(0, 0, 1)]
    points_B = [Point3D(0, 1, 0), Point3D(-1, 0, 0), Point3D(0, 0, 1)]  # Rotated 90° around Z
    
    frame = find_rigid_transform(points_A, points_B)
    
    test_point = points_A[0]
    transformed = frame.transform_point(test_point)
    expected = points_B[0]
    
    print(f"Original point A: ({test_point.x}, {test_point.y}, {test_point.z})")
    print(f"Transformed point: ({transformed.x:.2f}, {transformed.y:.2f}, {transformed.z:.2f})")
    print(f"Expected point B: ({expected.x}, {expected.y}, {expected.z})")
    print(f"Error: {np.linalg.norm(transformed.to_array() - expected.to_array()):.6f}")
    
    print("3D set point registration algorithm tests done\n")