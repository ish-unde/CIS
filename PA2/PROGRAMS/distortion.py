import numpy as np
from scipy import linalg

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
         
