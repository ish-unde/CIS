import numpy as np
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from file_io import read_points_from_file
from math_utils_cart import PivotCalibration, calibrate

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

def test_calibrate_function():
    #Test #1: Testing known pivot point and tip position
    cal = PivotCalibration()
    known_tip = np.array([0.1, 0.2, 0.3])
    known_pivot = np.array([1.0, 2.0, 3.0])

    poses = []
    for i in range(10): 
        axis = np.random.randn(3)
        axis = axis / np.linalg.norm(axis)
        angle = i * (np.pi / 10) 

        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ]) 

        
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        p = known_pivot - R @ known_tip
        poses.append((R, p))
    
    tip_pos, pivot_pt = cal.calibrate(poses)
    assert np.allclose(tip_pos, known_tip, atol=1e-3)
    assert np.allclose(pivot_pt, known_pivot, atol=1e-3)
    assert cal.residual_error < 1e-3
    print("known pivot point and tip position test passed")

    #Test #2: Testing edge case
    cal = PivotCalibration()
    poses = []
    R_almost_1 = np.array([
        [1.0000001, 0, 0],
        [0, 0.9999999, 0],
        [0, 0, 1.0]
    ])
    poses.append((np.eye(3), np.array([0, 0, 0])))
    poses.append((R_almost_1, np.array([1, 1, 1])))
    
    tip_pos, pivot_pt = cal.calibrate(poses)
    assert tip_pos is not None
    print("Edge cases test passed")

    #Test #3: Testing to make sure output shape is (3,)
    cal = PivotCalibration()
    poses = []
    for i in range(4):
        R = np.eye(3)
        p = np.random.randn(3)
        poses.append((R, p))
    
    tip_pos, pivot_pt = cal.calibrate(poses)
    assert tip_pos.shape == (3,)
    assert pivot_pt.shape == (3,)
    assert isinstance(cal.residual_error, float)
    print("Output is shape (3, ) test, ie passed")

    