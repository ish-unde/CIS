import numpy as np
from scipy import linalg
from file_io import read_calreadings, read_empivot, read_em_fiducials, read_calbody, read_ct_fiducials
from math_utils_cart import compute_expected_C, find_fa, find_fd, em_tracking, PivotCalibration, find_rigid_transform, Point3D

class DistortionCorrector:

    def __init__(self, degree=3):
        self.degree = degree
        self.coefficients = None
        self.features = None
        self.feature_naming()

    def feature_naming(self):
        """Generate names for polynomial features based on degree."""
        self.naming = ['1']
        for total_degree in range(1, self.degree + 1):
            for i in range(total_degree + 1):
                for j in range(total_degree + 1 - i):
                    k = total_degree - i - j

                    if k > 0:
                        term_parts = []

                        if i > 0:
                            term_parts.append(f"x^{i}" if i > 1 else "x")
                        
                        if j > 0:
                            term_parts.append(f"y^{j}" if j > 1 else "y")
                        
                        if k > 0:
                            term_parts.append(f"z^{k}" if k > 1 else "z")
                        
                        self.naming.append(''.join(term_parts))


    def get_polynomial_terms(self, x, y, z):
        """
        Generate polynomial terms for a single 3D point.
        
        Args:
            x (float): x-coordinate
            y (float): y-coordinate  
            z (float): z-coordinate
            
        Returns:
            list: Polynomial terms [1, x, y, z, x², xy, xz, y², yz, z², ...]
        """
        terms = [1.0]

        for total_degree in range(1, self.degree + 1):
            for i in range(total_degree + 1):
                for j in range(total_degree + 1 - i):
                    k = total_degree - i - j

                    if k >= 0:

                        term_value = (x ** i) * (y ** j) * (z ** k)
                        terms.append(term_value)
        return terms
    

    def compute_feature(self, points):
        """
        Compute polynomial features for a set of 3D points.
        """

        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("Input points must be a 2D array with shape (N, 3)")
        
        features = []

        for point in points:
            x, y, z = point
            terms = self.get_polynomial_terms(x, y, z)
            features.append(terms)

        return np.array(features)
    
    def fit(self, meausured_points, expected_points):
        
        measured_points = np.array(meausured_points)
        expected_points = np.array(expected_points)

        if measured_points.shape != expected_points.shape:
            raise ValueError("Measured points and expected points must have the same shape.")
        
        if measured_points.shape[1] != 3:
            raise ValueError("Input points must be 3D coordinates with shape (N, 3).")
        
        n_points = measured_points.shape[0]

        if n_points < self.get_number_of_features():
            raise ValueError("Not enough points to fit the distortion model.")


        distorition_vectors = expected_points - measured_points
        x = self.compute_feature(measured_points)

        self.coefficients = []
        for dim in range(3):
            coeffs, residuals, rank, s = linalg.lstsq(x, distorition_vectors[:, dim])
            self.coefficients.append(coeffs)

            explained_variance = 1 - residuals / np.sum((distorition_vectors[:, dim] - np.mean(distorition_vectors[:, dim]))**2)


        return self

    def get_number_of_features(self):
        return len(self.get_polynomial_terms(0, 0, 0))  

    def predict_distortion(self, points):
        if self.coefficients is None:
            raise ValueError("Model has not been fitted yet. Please call 'fit' before prediction")
      
        x = self.compute_feature(points)
        distortion = np.zeros_like(points)

        for dim in range(3):
            distortion[:, dim] = x @ (self.coefficients[dim])       

        return distortion 

    def correct(self, points):
        distortion = self.predict_distortion(points)
        return points + distortion     
    

#question 4 
         
def b_j_locations(em_fiducials_data, distortion_corrector, pivot_tip_G, g_j_reference):
    """
    Question 4: Compute fiducial locations b_j with distortion correction
    """
    b_j_em = []
    
    # Convert g_j_reference (numpy array) to Point3D objects
    if isinstance(g_j_reference, np.ndarray):
        g_j_ref_points = [Point3D(p[0], p[1], p[2]) for p in g_j_reference]
    else:
        g_j_ref_points = g_j_reference
    
    for frame in em_fiducials_data:
        # Extract EM marker measurements
        g_measured_points = frame.g_points
        
        # Convert Point3D objects to numpy array
        g_measured = np.array([[p.x, p.y, p.z] for p in g_measured_points])
        
        # Apply distortion correction
        g_corrected = distortion_corrector.correct(g_measured)
        
        # Convert back to Point3D for rigid transform
        corrected_points = [Point3D(p[0], p[1], p[2]) for p in g_corrected]
        
        # Compute probe transform - use converted Point3D objects
        F_G = find_rigid_transform(g_j_ref_points, corrected_points)
        
        # Compute tip position in EM coordinates
        tip_position_em = F_G.transform_point(pivot_tip_G)
        b_j_em.append([tip_position_em.x, tip_position_em.y, tip_position_em.z])
    
    return np.array(b_j_em)

#question 5 
def find_f_reg(ct_fiducials_data, b_j_em):
    """
    Question 5: Compute registration frame F_reg that maps EM coordinates to CT coordinates

    """
    # Convert b_j_em numpy array to Point3D objects
    points_em = [Point3D(p[0], p[1], p[2]) for p in b_j_em]
    
    # ct_fiducials_data should already be Point3D objects
    points_ct = ct_fiducials_data
    
    # Compute registration: find F such that points_ct = F • points_em
    F_reg = find_rigid_transform(points_em, points_ct)
    
    return F_reg

# func might not be needed 
def compute_registration_error(F_reg, b_j_em, ct_fiducials_file):
    """
    Compute RMS error of the registration
    """
    ct_fiducials = read_ct_fiducials(ct_fiducials_file)
    
    errors = []
    for b_em, b_ct in zip(b_j_em, ct_fiducials):
        # Transform EM point to CT coordinates
        b_em_transformed = F_reg.transform_point(Point3D(b_em[0], b_em[1], b_em[2]))
        
        # Compute error
        error = np.sqrt((b_em_transformed.x - b_ct.x)**2 + 
                       (b_em_transformed.y - b_ct.y)**2 + 
                       (b_em_transformed.z - b_ct.z)**2)
        errors.append(error)
    
    return np.mean(errors)

#question 6 

def compute_probe_transform(current_points, reference_points):
    """
    Compute the transform from probe coordinates to tracker coordinates
    """
    # Convert reference_points (numpy array) to Point3D objects if needed
    if isinstance(reference_points, np.ndarray):
        ref_points_p3d = [Point3D(p[0], p[1], p[2]) for p in reference_points]
    else:
        ref_points_p3d = reference_points
    
    # Use your existing rigid transform function
    F_G = find_rigid_transform(ref_points_p3d, current_points)
    return F_G

def navigation_data(em_nav_data, distortion_corrector, t_g_corrected, g_j_reference, F_reg, output_file):
    """
    Question 6: Process navigation data and output CT coordinates
    """
    tip_positions_ct = []
    
    for frame in em_nav_data:
        # Use attribute access for EmPivotFrame objects
        g_measured_points = frame.g_points  # Get the list of Point3D objects
        
        # Convert Point3D objects to numpy array
        g_measured = np.array([[p.x, p.y, p.z] for p in g_measured_points])
        
        # Apply distortion correction
        g_corrected = distortion_corrector.correct(g_measured)
        
        # Convert back to Point3D for your existing functions
        corrected_points = [Point3D(p[0], p[1], p[2]) for p in g_corrected]
        
        # Compute probe transform
        F_G = compute_probe_transform(corrected_points, g_j_reference)
        
        # Compute tip in EM coordinates
        tip_em = F_G.transform_point(t_g_corrected)
        
        # Transform to CT coordinates
        tip_ct = F_reg.transform_point(tip_em)
        tip_positions_ct.append([tip_ct.x, tip_ct.y, tip_ct.z])
    
    # Write output file
    write_output2_file(output_file, tip_positions_ct)
    
    return np.array(tip_positions_ct)

def write_output2_file(output_file, tip_positions_ct):
    N_frames = len(tip_positions_ct)
    
    with open(output_file, 'w') as file:
        file.write(f"{N_frames}, {output_file}\n")
        for tip_pos in tip_positions_ct:
            file.write(f"{tip_pos[0]:8.2f}, {tip_pos[1]:8.2f}, {tip_pos[2]:8.2f}\n")

