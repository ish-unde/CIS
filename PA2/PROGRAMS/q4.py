import numpy as np
from scipy import linalg
from file_io import read_calreadings, read_empivot, read_em_fiducials
from math_utils_cart import compute_expected_C, find_fa, find_fd, read_calbody, em_tracking, PivotCalibration, find_rigid_transform, Point3D

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
         
def b_j_locations(calbody_file, calreadings_file, em_fiducials_file, empivot_file):
    # Step 1-2: Fit distortion model from scratch
    frames = read_calreadings(calreadings_file)
    empivot_data = read_empivot(empivot_file)
    em_fiducials_data = read_em_fiducials(em_fiducials_file)

    # Extract measurements
    measured_C = np.array([[[p.x, p.y, p.z] for p in frame.C_j] for frame in frames])

    # Get reference points (from calbody file)
    d_ref, a_ref, c_ref = read_calbody(calbody_file)  # You need this!

    # Compute transformations
    all_fa = find_fa(frames, a_ref) 
    f_d = find_fd(frames, d_ref)    

    # Compute expected C
    expected_C = compute_expected_C(frames, f_d, all_fa, c_ref)  

    # Flatten for distortion fitting
    measured_C_flat = measured_C.reshape(-1, 3)        
    expected_C_flat = np.array([[[p.x, p.y, p.z] for frame in expected_C for p in frame]]).reshape(-1, 3)

    # Fit distortion model
    distortion_corrector = DistortionCorrector(degree=3)
    distortion_corrector.fit(measured_C_flat, expected_C_flat)

    #Step 3: Improved pivot calibration using distortion correction
    def improved_pivot_calibration(empivot_data, corrector):
        # Use first frame (corrected) to define probe coords
        first_frame_corrected = corrector.correct(empivot_data[0])
        g_j_reference = em_tracking(first_frame_corrected)
        
        # Process all frames with correction
        transforms = []
        for frame in empivot_data:
            corrected_frame = corrector.correct(frame)
            F_G = em_tracking(corrected_frame, g_j_reference)
            transforms.append(F_G)
        
        return PivotCalibration.calibrate()(transforms), g_j_reference
    
    pivot_tip_improved, g_j_reference = improved_pivot_calibration(
        empivot_data, distortion_corrector
    )

    b_j_em = []
    for frame in em_fiducials_data:
        corrected_frame = distortion_corrector.correct(frame)
        F_G = em_tracking(corrected_frame, g_j_reference)
        tip_position = F_G.apply(pivot_tip_improved)
        b_j_em.append(tip_position)
    
    return np.array(b_j_em)

#question 5 
def read_ct_fiducials(filename):
    """
    Read CT fiducials from NAME-CT-FIDUCIALS.TXT
    """
    points = []
    
    with open(filename, 'r') as file:
        # First line: N_B, filename
        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]
        
        N_B = int(header_parts[0])
        # filename = header_parts[1] 
        
        # Next N_B lines: b_x,i, b_y,i, b_z,i
        for _ in range(N_B):
            line = file.readline().strip()
            coords = [coord.strip() for coord in line.split(',')]
            x, y, z = map(float, coords)
            points.append(Point3D(x, y, z))
    
    return points
    
def find_f_reg(ct_fiducials_file, b_j_em):
    """
    Question 5: Compute registration frame F_reg
    """
    F_reg = find_rigid_transform(ct_fiducials_file, b_j_em)
    
    # Optional: Compute registration error
    registration_error = compute_registration_error(F_reg, b_j_em, ct_fiducials_file)
    print(f"Registration error: {registration_error:.4f} mm")
    
    return F_reg

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

def solve_question_6(em_nav_file, distortion_corrector, pivot_tip_G, g_j_reference, F_reg, output_file):
    """
    Question 6: Process navigation data and output CT coordinates

    """
    # Read navigation data
    nav_frames = read_empivot(em_nav_file)
    
    tip_positions_ct = []
    
    for frame in nav_frames:
        # Step 1: Convert frame to numpy and apply distortion correction
        frame_points = []
        for point3d in frame.g_points:
            frame_points.append([point3d.x, point3d.y, point3d.z])
        frame_np = np.array(frame_points)
        
        corrected_frame_np = distortion_corrector.correct(frame_np)
        
        # Step 2: Convert back to Point3D and compute probe transform
        corrected_points = [Point3D(p[0], p[1], p[2]) for p in corrected_frame_np]
        F_G = find_rigid_transform(g_j_reference, corrected_points)
        
        # Step 3: Compute tip position in EM tracker coordinates
        tip_position_em = F_G.transform_point(pivot_tip_G)
        
        # Step 4: Transform to CT coordinates using registration
        tip_position_ct = F_reg.transform_point(tip_position_em)
        
        tip_positions_ct.append([tip_position_ct.x, tip_position_ct.y, tip_position_ct.z])
    
    # Write output file
    write_output2_file(output_file, tip_positions_ct)
    
    return np.array(tip_positions_ct)

def write_output2_file(output_file, tip_positions_ct):
    N_frames = len(tip_positions_ct)
    
    with open(output_file, 'w') as file:
        file.write(f"{N_frames}, {output_file}\n")
        for tip_pos in tip_positions_ct:
            file.write(f"{tip_pos[0]:.2f}, {tip_pos[1]:.2f}, {tip_pos[2]:.2f}\n")

