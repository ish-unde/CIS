import numpy as np
from scipy import linalg


class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def to_array(self):
        return np.array([self.x, self.y, self.z])
    
    @staticmethod
    def from_array(arr):
        return Point3D(arr[0], arr[1], arr[2])

class Rotation:
    def __init__(self, matrix):
        # 3x3 rotation matrix
        self.matrix = np.array(matrix)
    
    def __matmul__(self, other):
        # Matrix multiplication for rotations
        if isinstance(other, Rotation):
            return Rotation(self.matrix @ other.matrix)
        elif isinstance(other, Point3D):
            rotated = self.matrix @ other.to_array()
            return Point3D.from_array(rotated)
        return NotImplemented

class Frame:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation  # Point3D
    
    def transform_point(self, point):
        # F • p = R * p + t
        rotated = self.rotation @ point
        return Point3D(rotated.x + self.translation.x,
                      rotated.y + self.translation.y, 
                      rotated.z + self.translation.z)
    
    def inverse(self):
        # F^{-1} = (R^T, -R^T * t)
        R_inv = Rotation(self.rotation.matrix.T)  # Transpose
        t_inv = R_inv @ Point3D(-self.translation.x, 
                               -self.translation.y, 
                               -self.translation.z)
        return Frame(R_inv, t_inv)
    
    def compose(self, other):
        # F1 • F2
        new_rotation = self.rotation @ other.rotation
        new_translation = self.transform_point(other.translation)
        return Frame(new_rotation, new_translation)
    
def find_rigid_transform(points_A, points_B):
    """
    Find the rigid transformation that best aligns points_A to points_B
    using slides 4 from CIS Class and Direct Technique to solve for R from 
    K.Arun et al. IEEE PAMI, Vol 9 no 5, September 1987.

    """
    # Convert to numpy 
    A = np.array([p.to_array() for p in points_A]).T  
    B = np.array([p.to_array() for p in points_B]).T  
    
    # Center the points - Step 1 
    centroid_A = np.mean(A, axis=1, keepdims=True)
    centroid_B = np.mean(B, axis=1, keepdims=True)
    
    A_centered = A - centroid_A
    B_centered = B - centroid_B
    
    # Compute covariance matrix - Step 2 
    H = A_centered @ B_centered.T
    
    # SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Calculate rotation
    R = Vt.T @ U.T
    
    # Handle reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    #Verifying that Det(R) = 1 
    assert np.isclose(np.linalg.det(R), 1.0), "Det(R) is not 1"

    # Calculate translation - Step 3 
    t = centroid_B - R @ centroid_A
    
    # Create Frame object - Step 4 
    rotation = Rotation(R)
    translation = Point3D(t[0, 0], t[1, 0], t[2, 0])
    
    return Frame(rotation, translation)

#Calculate centroid of a set of points
def calculate_centroid(points):
    if not points:
        return Point3D(0, 0, 0)
    
    sum_x = sum(p.x for p in points)
    sum_y = sum(p.y for p in points)
    sum_z = sum(p.z for p in points)
    n = len(points)
    
    return Point3D(sum_x/n, sum_y/n, sum_z/n)

#Apply frame transformation to a list of points
def transform_points(frame, points):
    return [frame.transform_point(p) for p in points]

#pivot calibration method   
class PivotCalibration:

    def __init__(self):
        self.tip_position = None
        self.pivot_point = None
        self.residual_error = None

    def calibrate(self, poses):
        """
        Perform pivot calibration using parameter estimation 

        Method taken from class slides "Calibration"
            
        Returns:
        tip_position: estimated tip position in tool coordinates (3,)
        pivot_point: estimated pivot point in world coordinates (3,)
        """
        if len(poses) < 2:
            raise ValueError("Need at least 2 poses for pivot calibration")
    
        n_poses = len(poses)
        
        # Settuping the matricies for the poses based on calibrations 
        A = np.zeros((3 * n_poses, 6))
        b = np.zeros(3 * n_poses)
        
        for i, (R, p) in enumerate(poses):
            start_idx = 3 * i
            end_idx = 3 * i + 3
            
            # Fill A matrix
            A[start_idx:end_idx, 0:3] = R  # R_j for p_t
            A[start_idx:end_idx, 3:6] = -np.eye(3)  # -I for p_pivot
            
            # Fill b vector
            b[start_idx:end_idx] = -p
        
        # Solve using least squares
        x, residuals, rank, s = linalg.lstsq(A, b)

        # Check solution quality
        if rank < 6:
            print(f"System is underdetermined, rank is < 6: {rank}")

        #Check if its a valid rotation by checking if Det(R) = 1
        if np.isclose(np.linalg.det(R), 1):
            print("Valid rotation matrix with Det(R) = 1")

        # update points and error 
        self.tip_position = x[0:3]
        self.pivot_point = x[3:6]
        self.residual_error = np.sqrt(np.sum(residuals) / n_poses) if len(residuals) > 0 else 0
        
        return self.tip_position, self.pivot_point

