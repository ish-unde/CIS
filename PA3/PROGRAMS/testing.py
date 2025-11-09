import numpy as np
from geometry import closest_point_triange, closest_point_mesh, compute_transform_from_markers
from registration import find_rigid_transform

def rmse(a, b):
    """Compute root mean squared error between two arrays"""
    return np.sqrt(np.mean(np.sum((a - b)**2, axis=-1)))

def test_closest_point_triangle():
    """Unit test for closest_point_triangle with numerical evidence"""
    print("Testing closest_point_triangle")
    
    # traingle vertices
    A = np.array([0.0, 0.0, 0.0])
    B = np.array([1.0, 0.0, 0.0])
    C = np.array([0.0, 1.0, 0.0])

    # test cases, format: (test_point, expected_closest_point)
    test_cases = [
        (np.array([0.0, 0.0, 0.0]), A),          # vertex A
        (np.array([1.0, 0.0, 0.0]), B),          # vertex B
        (np.array([0.0, 1.0, 0.0]), C),          # vertex C
        (np.array([0.3, 0.3, 0.0]), np.array([0.3, 0.3, 0.0])),  # inside
        (np.array([0.5, 0.0, 0.0]), np.array([0.5, 0.0, 0.0])),  # edge AB
        (np.array([0.3, 0.3, 1.0]), np.array([0.3, 0.3, 0.0]))   # above triangle
    ]

    distances = []
    for i, (P, expected) in enumerate(test_cases, 1):
        closest, dist = closest_point_triange(P, A, B, C)
        distances.append(dist)
        correct = np.allclose(closest, expected)
        print(f"Test {i}: Passed={correct}, Distance={dist:.6f}")

    #(RMSE)
    rmse_val = np.sqrt(np.mean(np.array(distances)**2))
    print(f"RMSE across all triangle tests: {rmse_val:.6e}")

def test_closest_point_mesh():
    """Unit test for closest_point_mesh with detailed failure reporting and numerical evidence"""
    print("\n Testing closest_point_mesh")
    
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    triangles = np.array([[0, 1, 2]])

    # test case format: (test_point, expected_closest_point)
    test_cases = [
        (np.array([0.3, 0.3, 0.0]), np.array([0.3, 0.3, 0.0])),
        (np.array([0.3, 0.3, 1.0]), np.array([0.3, 0.3, 0.0]))
    ]

    distances = []
    failed_tests = []
    
    for i, (P, expected) in enumerate(test_cases, 1):
        closest, dist = closest_point_mesh(P, vertices, triangles)
        distances.append(dist)

        # tolerance 
        correct = np.allclose(closest, expected, atol=1e-6)

        if not correct:
            failed_tests.append({
                "Test": i,
                "Input": P,
                "Expected": expected,
                "Got": closest,
                "Error Vector": closest - expected
            })

        print(f"Mesh Test {i}: Passed={correct}, Distance={dist:.6f}")

    # RMSE
    rmse_val = np.sqrt(np.mean(np.array(distances) ** 2))
    print(f"RMSE across all mesh tests: {rmse_val:.6e}")

def test_find_rigid_transform_directly():
    """Unit test for find_rigid_transform with numerical evidence"""
    print("\n Testing find_rigid_transform")
    
    body_markers = np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]])
    R_true = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    t_true = np.array([10,5,2])
    tracker_markers = (R_true @ body_markers.T).T + t_true

    R_computed, t_computed = find_rigid_transform(body_markers, tracker_markers)
    
    rotation_error = np.linalg.norm(R_computed - R_true)
    translation_error = np.linalg.norm(t_computed - t_true)
    
    print(f"Rotation error (Frobenius norm): {rotation_error:.6e}")
    print(f"Translation error (Euclidean norm): {translation_error:.6e}")
    print("Rotation correct" if np.isclose(rotation_error, 0) else "Rotation incorrect")
    print("Translation correct" if np.isclose(translation_error, 0) else "Translation incorrect")

def run_all_tests():
    test_closest_point_triangle()
    test_closest_point_mesh()
    test_find_rigid_transform_directly()
    print("\nAll tests completed!")
    
if __name__ == "__main__":
    run_all_tests()
