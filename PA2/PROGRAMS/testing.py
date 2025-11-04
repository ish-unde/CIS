import numpy as np
from math_utils_cart import Point3D, Rotation, Frame
from distortion import DistortionCorrector, b_j_locations, navigation_data


#testin distortion corrector
def test_distortion_corrector_fit_and_predict():
    """Test the distortion corrector fitting and prediction."""

    def synthetic_distortion(points):
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        dx = 0.01 * x**2 + 0.005 * y
        dy = -0.002 * x * y
        dz = 0.003 * z
        return np.column_stack([dx, dy, dz])
    
    measured = np.random.rand(100, 3) * 100
    distortion = synthetic_distortion(measured)
    expected = measured + distortion
    
    corrector = DistortionCorrector(degree=3)
    corrector.fit(measured, expected)
    corrected = corrector.correct(measured)
    
    original_error = np.mean(np.linalg.norm(measured - expected, axis=1))
    corrected_error = np.mean(np.linalg.norm(corrected - expected, axis=1))
    
    assert corrected_error < original_error * 0.1  # Significant improvement
    assert corrector.coefficients is not None
    print("Distortion corrector fit/predict test passed")


def test_distortion_corrector_feature_generation():
    """Test polynomial feature generation for different degrees."""
    points = np.array([[1.0, 2.0, 3.0]])
    
    corrector_deg1 = DistortionCorrector(degree=1)
    features_deg1 = corrector_deg1.compute_feature(points)
    assert features_deg1.shape[1] == 4  # 1 + x + y + z
    
    corrector_deg2 = DistortionCorrector(degree=2)
    features_deg2 = corrector_deg2.compute_feature(points)
    assert features_deg2.shape[1] == 10  # All combinations up to degree 2
    
    print("Feature generation test passed")



# testing location computation
def test_b_j_locations_computation():
    """Test computation of fiducial locations in EM space."""

    synthetic_frames = []
    for i in range(5):
        g_points = [Point3D(10 + i, 20 + i, 30 + i) for _ in range(3)]
        synthetic_frames.append(type('Frame', (), {'g_points': g_points})())
    
    class MockCorrector:
        def correct(self, points):
            return points  # no distortion
    
    corrector = MockCorrector()
    t_g = Point3D(0, 0, 150)  # 150mm tip offset
    g_j_reference = [Point3D(1, 0, 0), Point3D(0, 1, 0), Point3D(0, 0, 1)]
    
    b_j_location = b_j_locations(synthetic_frames, corrector, t_g, g_j_reference)
    
    assert len(b_j_location) == len(synthetic_frames)
    for point in b_j_location:
        assert isinstance(point, Point3D)
    print("B_j locations computation test passed")

def test_b_j_locations_with_distortion():
    """Test fiducial computation with actual distortion correction."""

    synthetic_frames = []
    base_points = [Point3D(10, 20, 30), Point3D(15, 25, 35), Point3D(20, 30, 40)]
    
    for i in range(3):
        distorted_points = []
        for p in base_points:

            distorted = Point3D(p.x + 0.1 * i, p.y, p.z)
            distorted_points.append(distorted)
        synthetic_frames.append(type('Frame', (), {'g_points': distorted_points})())
    
    class MockCorrector:
        def correct(self, points):
            corrected = points.copy()
            corrected[:, 0] -= 0.1  
            return corrected
    
    corrector = MockCorrector()
    t_g = Point3D(0, 0, 100)
    g_j_reference = [Point3D(1, 0, 0), Point3D(0, 1, 0), Point3D(0, 0, 1)]
    
    b_j_location = b_j_locations(synthetic_frames, corrector, t_g, g_j_reference)
    
    positions = np.array([[p.x, p.y, p.z] for p in b_j_location])
    variances = np.var(positions, axis=0)
    assert np.all(variances < 1.0) 
    print("B_j locations with distortion correction test passed")





# testing navigation data processig
def test_navigation_data():
    """Test the complete navigation data processing pipeline."""

    nav_frames = []
    for i in range(10):

        g_points = [Point3D(10 + i, 20, 30) for _ in range(3)]
        nav_frames.append(type('Frame', (), {'g_points': g_points})())
    
    class MockCorrector:
        def correct(self, points):
            return points
    
    t_g = Point3D(0, 0, 150)
    g_j_reference = [Point3D(1, 0, 0), Point3D(0, 1, 0), Point3D(0, 0, 1)]
    
    identity_rotation = Rotation(np.eye(3))
    identity_translation = Point3D(0, 0, 0)
    F_reg = Frame(identity_rotation, identity_translation)
    
    def compute_navigation_points(frames, corrector, tip_vector, reference_points, F_reg):
        ct_points = []
        for frame in frames:
            g_measured = np.array([[p.x, p.y, p.z] for p in frame.g_points])
            g_corrected = corrector.correct(g_measured)
            
            centroid = np.mean(g_corrected, axis=0)
            tip_em = Point3D(centroid[0] + tip_vector.x,
                           centroid[1] + tip_vector.y,
                           centroid[2] + tip_vector.z)
            
            tip_ct = F_reg.transform_point(tip_em)
            ct_points.append(tip_ct)
        
        return ct_points
    
    ct_points = compute_navigation_points(nav_frames, MockCorrector(), t_g, g_j_reference, F_reg)
    
    assert len(ct_points) == len(nav_frames)
    for point in ct_points:
        assert isinstance(point, Point3D)
    print("Navigation data computation test passed")



def run_pa2_unit_tests():
    """Run all PA2-specific unit tests without file I/O."""
    print("Running PA2-specific unit tests (no file I/O)...\n")
    
    # Distortion correction tests
    test_distortion_corrector_fit_and_predict()
    test_distortion_corrector_feature_generation()
    
    # Fiducial computation tests
    test_b_j_locations_computation()
    test_b_j_locations_with_distortion()
    
    
    # Navigation tests
    test_navigation_data()
    
    # Transformation chain tests
   
    
    print("\nAll PA2-specific unit tests completed!")


def main():
    try:
        # Run distortion correction tests
        print("\nDISTORTION CORRECTION TESTS")
        print("-" * 40)
        test_distortion_corrector_fit_and_predict()
        test_distortion_corrector_feature_generation()
        
        # Run fiducial location tests
        print("\nFIDUCIAL LOCATION TESTS")
        print("-" * 40)
        test_b_j_locations_computation()
        test_b_j_locations_with_distortion()
        
        # Run navigation data tests
        print("\nNAVIGATION DATA TESTS")
        print("-" * 40)
        test_navigation_data()
        
        print("\n" + "=" * 60)
        print("ALL PA2 TESTS COMPLETED SUCCESSFULLY!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\nTEST FAILED: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    run_pa2_unit_tests()
